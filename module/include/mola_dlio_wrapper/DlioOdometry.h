/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: wraps the MIT-licensed DLIO algorithm
 (third_party/dlio_core), copyright Kenny J. Chen, Ryan Nemiroff, Brett T.
 Lopez (VECTR Lab, UCLA).
*/

/**
 * @file   DlioOdometry.h
 * @brief  MOLA front-end module wrapping the DLIO algorithm
 */
#pragma once

// Only the light, PCL/nano_gicp-free plain-data header: keeps consumers of
// this public header from needing dlio_core's internal include dirs. The
// full dlio_core/DlioCore.h is only ever included from DlioCoreBridge.cpp,
// never here nor from DlioOdometry.cpp -- see DlioCoreBridge.h for why
// (vendored nano_gicp's pinned nanoflann vs. mrpt's own system nanoflann,
// both unguarded against each other in the same translation unit).
#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <atomic>
#include <dlio_core/Types.hpp>
#include <memory>
#include <mutex>
#include <regex>
#include <vector>

namespace mola
{
class DlioCoreBridge;
/** MOLA online front-end wrapping the DLIO (Direct LiDAR-Inertial Odometry)
 * algorithm (see `dlio_core`, in `third_party/`).
 *
 * Consumes `mrpt::obs::CObservationPointCloud` (LiDAR) and
 * `mrpt::obs::CObservationIMU` (IMU) observations and publishes live
 * localization + map updates via `LocalizationSourceBase`/`MapSourceBase`.
 *
 * For the interactive MOLA GUI, this class additionally drives the
 * `VizInterface` directly, drawing the current pose, the estimated
 * trajectory and the current registration **submap** (DLIO's own local map,
 * built from a keyframe convex/concave-hull kNN selection -- not a
 * voxel/octree structure) in the 3-D scene, plus a small status
 * sub-window.
 *
 * LiDAR scans are processed on a single-thread worker pool with the
 * `POLICY_DROP_OLD` policy, exactly as `FastLio2Odometry` does; the offline
 * CLI (`mola-dlio-cli`) achieves the loss-free guarantee by feeding one
 * scan at a time and busy-waiting on `isBusy()`.
 *
 * \ingroup mola_dlio_wrapper_grp
 */
class DlioOdometry : public FrontEndBase, public LocalizationSourceBase, public MapSourceBase
{
  DEFINE_MRPT_OBJECT(DlioOdometry, mola)

public:
  DlioOdometry();
  ~DlioOdometry() override;

  // ExecutableBase
  void spinOnce() override;
  void onQuit() override;

  // RawDataConsumer
  void onNewObservation(const mrpt::obs::CObservation::ConstPtr & o) override;

  /** True while the LiDAR worker still has a scan queued or running. Polled
   * by `mola-dlio-cli` to guarantee no scan is ever dropped. */
  bool isBusy() const;

  /** Trajectory accumulated so far (`map` -> `lidar`, i.e. the frame
   * `gt-tum.txt` uses for Oxford Spires -- see the common plan). */
  mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

protected:
  void initialize_frontend(const Yaml & cfg) override;

private:
  void onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void onImuObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void processLidarScan(mrpt::obs::CObservation::ConstPtr o);
  void drainAndPublish();
  void publishOutput(const dlio_core::OdometryOutput & out);

  /// Drains `worker_lidar_` and, if requested, writes the accumulated
  /// trajectory to disk. Called from both `onQuit()` (while every other
  /// module is still alive -- see `mola::LidarOdometry::shutdownCleanup()`
  /// for why that matters) and the destructor; idempotent via
  /// `shutdown_cleanup_done_` so the second call is a no-op.
  void shutdownCleanup();

  // --- GUI / 3D scene visualization (mola_viz / mola_viz_imgui) ---
  void updateVisualization(
    const mrpt::poses::CPose3D & lidarPose, const dlio_core::OdometryOutput & out);
  void updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose);
  void updateVisualizationSubmap();
  void internalBuildGUI();
  void updateVisualizationTextLabels(const dlio_core::OdometryOutput & out);

  std::regex lidar_sensor_label_regex_{"lidar"};
  std::regex imu_sensor_label_regex_{"imu"};

  /// Pose of the IMU frame wrt the LiDAR frame ("x y z yaw_deg pitch_deg
  /// roll_deg"), same convention/value as `fixed_sensor_pose` for the IMU
  /// sensor in the dataset YAML when the LiDAR is base_link. Used both to
  /// fill `core_cfg_.baselink2imu` (see initialize_frontend()) and, here in
  /// the module, to recover the LiDAR pose from the core's IMU-frame output
  /// -- mirrors `FastLio2Odometry::imu_pose_in_lidar_` exactly.
  mrpt::poses::CPose3D imu_pose_in_lidar_;

  /// Used only as a fallback when an incoming point cloud has no per-point
  /// timestamp channel (synthesizes one from azimuth angle).
  double fallback_scan_period_ = 0.1;

  /// Minimum point range [m]; see dlio_core::Config::crop_box_size doc.
  double blind_ = 0.0;

  dlio_core::Config core_cfg_;
  std::unique_ptr<DlioCoreBridge> core_;

  mrpt::WorkerThreadsPool worker_lidar_{1, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "dlio_lidar"};
  std::atomic_int tasks_in_flight_{0};

  mutable std::mutex trajectory_mtx_;
  mrpt::poses::CPose3DInterpolator trajectory_;

  std::atomic_bool shutdown_cleanup_done_{false};

  /// If true, `shutdownCleanup()` dumps `trajectory_` to
  /// `trajectory_output_file_` in TUM format -- mirrors
  /// `mola::LidarOdometry`'s `estimated_trajectory` pipeline block, needed
  /// so `mola-cli` (online mode) can export a trajectory the same way the
  /// offline `mola-dlio-cli` app already does via `estimatedTrajectory()` +
  /// `--output-tum-path`.
  bool save_trajectory_to_file_ = false;
  std::string trajectory_output_file_;

  struct VisualizationParams
  {
    std::atomic_bool show_trajectory{true};
    std::atomic_bool show_submap{true};
    std::atomic_bool camera_follows_vehicle{true};

    float current_pose_corner_size = 1.0f;
    float submap_point_size = 2.0f;

    /// How many processed scans to wait between submap viz refreshes.
    int map_update_decimation = 10;
  } visualization_params_;

  mrpt::opengl::CSetOfLines::Ptr gl_estimated_path_;
  int map_viz_update_counter_ = 0;
  bool gui_created_ = false;

  struct Gui
  {
    mola::gui::LiveString::Ptr lbStatus;
  } gui_;
};

}  // namespace mola
