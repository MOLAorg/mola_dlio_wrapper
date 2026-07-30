// SPDX-License-Identifier: GPL-3.0-only
// mola_dlio_wrapper: original code (not derived from DLIO), the plain data
// types used at the dlio_core boundary. See THIRD_PARTY_NOTICES.md at the
// repository root for the licensing situation of the vendored DLIO /
// nano_gicp / nanoflann code this library wraps.
#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <vector>

namespace dlio_core
{
/// Default standard gravity [m/s^2] (same constant DLIO's `cfg/params.yaml`
/// ships as its default `odom.gravity`).
inline constexpr double DEFAULT_GRAVITY = 9.80665;

/// One input LiDAR point. `t_offset_sec` is relative to the scan's start
/// time (`t_beg` passed to `DlioCore::addLidarScan()`), seconds, always
/// >= 0 -- the same convention every other wrapper in this benchmark suite
/// uses. Internally, `DlioCore` turns this into an absolute per-point time
/// (`t_beg + t_offset_sec`) before running its own deskewing math; see the
/// file comment in `include/dlio/dlio.h` for why upstream's per-sensor
/// timestamp-format branching was dropped.
struct LidarPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
  double t_offset_sec = 0;
};
using LidarPointVector = std::vector<LidarPoint>;

/// One output (deskewed-scan/submap) point: XYZI only, no per-point timing.
struct XYZIPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
};
using XYZIPointVector = std::vector<XYZIPoint>;

/// A single IMU sample, decoupled from any ROS message type.
struct ImuSample
{
  double t = 0;                                    //!< seconds, same clock as scan timestamps
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();   //!< m/s^2, sensor (IMU) frame
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  //!< rad/s, sensor (IMU) frame
};

/// Algorithm parameters. Field names/nesting mirror DLIO's own
/// `cfg/dlio.yaml` + `cfg/params.yaml` schema (see the comment above each
/// group) so a pipeline YAML author can cross-reference upstream docs.
struct Config
{
  /// dlio.yaml: `dlio.adaptive` -- scale `keyframe.threshD` and the GICP max
  /// correspondence distance from the scan's spaciousness/density metrics.
  bool adaptive = true;

  /// dlio.yaml: `dlio.pointcloud.deskew` / `dlio.pointcloud.voxelize`.
  bool pointcloud_deskew = true;
  bool pointcloud_voxelize = true;

  /// dlio.yaml: `dlio.imu.calibration` -- whether the state's initial
  /// gyro/accel bias comes from the startup calibration window (true) or
  /// from `imu_prior_*_bias`/`imu_accel_sm` below (false). Distinct from
  /// `imu_calibrate_gyro`/`imu_calibrate_accel` below, which select which
  /// of the two biases the calibration window actually estimates.
  bool imu_calibrate = true;
  Eigen::Vector3d imu_prior_accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_prior_gyro_bias = Eigen::Vector3d::Zero();
  /// dlio.yaml: `dlio.imu.intrinsics.accel.sm` (scale/misalignment matrix).
  Eigen::Matrix3d imu_accel_sm = Eigen::Matrix3d::Identity();

  struct Extrinsic
  {
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  };
  /// dlio.yaml: `dlio.extrinsics.baselink2imu` -- pose of the IMU frame wrt
  /// base_link: p_baselink = R * p_imu + t.
  Extrinsic baselink2imu;
  /// dlio.yaml: `dlio.extrinsics.baselink2lidar` -- pose of the LiDAR frame
  /// wrt base_link. Identity when the LiDAR itself is base_link (the Oxford
  /// Spires convention; see pipelines/dlio-oxford-spires.yaml).
  Extrinsic baselink2lidar;

  /// params.yaml: `dlio.map.waitUntilMove` / `dlio.map.dense.filtered`.
  bool wait_until_move = true;
  bool dense_map_filtered = false;
  /// params.yaml: `dlio.map.sparse.leafSize`. Not read by DlioCore itself
  /// (it fed the separate map-accumulation node, dropped per the common
  /// plan -- MOLA's MapSourceBase/viz covers that role); kept here for YAML
  /// schema parity and available to the MOLA module if it wants to
  /// decimate its own accumulated-map visualization at the same resolution.
  double map_sparse_leaf_size = 0.25;

  /// params.yaml: `dlio.odom.gravity`.
  double gravity = DEFAULT_GRAVITY;

  /// params.yaml: `dlio.odom.computeTimeOffset`. In upstream this corrects
  /// for LiDAR/host clock disagreement by re-basing each scan's per-point
  /// absolute time onto the ROS message header stamp. This port's MRPT
  /// adapter already re-bases every scan's points onto the *observation's
  /// own* timestamp before they reach DlioCore (see LidarPoint's doc
  /// comment), which serves the same purpose, so this flag is a no-op here
  /// -- kept only for YAML schema parity with upstream.
  bool compute_time_offset = true;

  /// params.yaml: `dlio.odom.imu.approximateGravity` -- estimate the initial
  /// gravity-aligned orientation from the calibration window's average
  /// accelerometer reading (true), or assume the sensor starts already
  /// level (false).
  bool imu_approximate_gravity = true;
  /// params.yaml: `dlio.odom.imu.calibration.{gyro,accel,time}`.
  bool imu_calibrate_gyro = true;
  bool imu_calibrate_accel = true;
  double imu_calib_time_sec = 3.0;
  /// params.yaml: `dlio.odom.imu.bufferSize`.
  int imu_buffer_size = 5000;

  /// params.yaml: `dlio.odom.preprocessing.cropBoxFilter.size`. DLIO's own
  /// near-field cut: a symmetric box of *half*-extent `crop_box_size`
  /// centered on base_link (i.e. full side length `2*crop_box_size`) is
  /// removed -- confirmed by reading `crop.setMin/setMax(+-crop_box_size)`
  /// with `crop.setNegative(true)` in upstream's `OdomNode` constructor.
  /// This is not a spherical radius cut (it over-trims a little at the box
  /// corners relative to a strict `crop_box_size`-metre sphere), but the
  /// upstream default of 1.0 m is already the same order of magnitude as
  /// the ~1 m handheld-carrier cut the other three wrappers apply via a
  /// `blind`/`min_range` radius, so it is kept unchanged as the Oxford
  /// Spires default.
  double crop_box_size = 1.0;
  /// params.yaml: `dlio.odom.preprocessing.voxelFilter.res`.
  double voxel_filter_res = 0.25;

  /// params.yaml: `dlio.odom.keyframe.threshD` [m] / `threshR` [deg].
  double keyframe_thresh_dist = 1.0;
  double keyframe_thresh_rot = 45.0;

  /// params.yaml: `dlio.odom.submap.keyframe.{knn,kcv,kcc}`.
  int submap_knn = 10;
  int submap_kcv = 10;
  int submap_kcc = 10;

  /// params.yaml: `dlio.odom.gicp.*`.
  int gicp_min_num_points = 64;
  int gicp_k_correspondences = 16;
  double gicp_max_corr_dist = 0.5;
  int gicp_max_iterations = 32;
  double gicp_transformation_epsilon = 0.01;
  double gicp_rotation_epsilon = 0.01;
  double gicp_init_lambda_factor = 1e-9;

  /// params.yaml: `dlio.odom.geo.*` -- the geometric observer's constant
  /// gains and bias clamp limits.
  double geo_Kp = 4.5;
  double geo_Kv = 11.25;
  double geo_Kq = 4.0;
  double geo_Kab = 2.25;
  double geo_Kgb = 1.0;
  double geo_abias_max = 5.0;
  double geo_gbias_max = 0.5;
};

/// Per-scan instrumentation payload (the research-facing part of
/// `OdometryOutput`): DLIO has no filter/factor-graph state to dump, so this
/// exposes the pieces the benchmark cares about instead -- see the common
/// plan's "Instrumentation" section.
struct Analytics
{
  std::size_t num_keyframes = 0;
  std::size_t submap_size = 0;

  /// GICP convergence summary for this scan. `num_correspondences` and
  /// `final_error` are NanoGICP's own public members/accessor
  /// (`num_correspondences`, `getFinalError()`); the per-iteration count is
  /// not exposed by NanoGICP's public API (it lives in a `protected`
  /// `pcl::Registration` member) and vendored `nano_gicp` code is not
  /// modified to add a getter for it, so it is omitted here rather than
  /// approximated -- see the plan's Open items.
  std::size_t num_gicp_correspondences = 0;
  double gicp_final_error = 0;
  bool gicp_converged = false;

  /// DLIO's own adaptive-parameter inputs (see `setAdaptiveParams()`):
  /// the low-pass-filtered median point range (`spaciousness`) and the
  /// GICP source point density (`density`), both directly relevant to the
  /// "preprocessing changes the effective forcing" hypothesis.
  float spaciousness = 0;
  float density = 0;

  /// The keyframe distance threshold actually in force this scan (equal to
  /// `Config::keyframe_thresh_dist` unless `Config::adaptive` rescaled it
  /// from `spaciousness`). DLIO does not adapt the rotation threshold.
  double keyframe_thresh_dist_active = 0;

  /// The geometric observer's current bias estimate (same values as
  /// `OdometryOutput::bias_gyro`/`bias_acc`, repeated here for symmetry with
  /// the other wrappers' `Analytics`).
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

/// Output of processing one LiDAR scan.
struct OdometryOutput
{
  /// DLIO's own notion of "the" time of a scan: the *median* per-point
  /// time, not the scan's start or end (see `DlioCore`'s deskewing code --
  /// this is where upstream computes `this->scan_stamp` and also where it
  /// evaluates the IMU-integrated pose used as the registration prior).
  /// Differs from fast_lio2_core's convention (scan end time); documented
  /// here since it affects how outputs from different method wrappers line
  /// up in time.
  double timestamp = 0;

  /// world_from_imu: the pose of the IMU frame in the (arbitrary,
  /// gravity-aligned-at-startup) odometry world frame. DLIO's native state
  /// is world_from_baselink; this is that pose composed with
  /// `Config::baselink2imu`.
  Eigen::Isometry3d pose_imu = Eigen::Isometry3d::Identity();

  Eigen::Vector3d vel = Eigen::Vector3d::Zero();  //!< world frame, base_link origin
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();

  /// DLIO has no explicit gravity filter state (unlike an IESKF's
  /// `state.grav`): this is *reconstructed* every scan from the current
  /// orientation estimate as `R^T * [0, 0, -gravity]`, i.e. gravity
  /// expressed in the current body frame. See DlioCore.h and README.md for
  /// why this, and the one-off calibration-window estimate, are both
  /// dumped by `MOLA_DLIO_DUMP_ESTIMATED_GRAV`.
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

  /// Deskewed, downsampled scan in LiDAR body frame (at the scan's
  /// median-time pose).
  std::shared_ptr<const XYZIPointVector> deskewed_points_body;

  Analytics analytics;
};

}  // namespace dlio_core
