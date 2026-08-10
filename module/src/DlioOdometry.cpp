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
 * @file   DlioOdometry.cpp
 * @brief  MOLA front-end module wrapping the DLIO algorithm
 */
#include "DlioCoreBridge.h"
#include "DlioOdometry_Adapters.h"

// Deliberately NOT including <dlio_core/DlioCore.h> here -- see
// DlioCoreBridge.h for why (vendored nano_gicp's pinned nanoflann vs.
// mrpt's own system nanoflann, both pulled into this file's translation
// unit -- via DlioOdometry_Adapters.h's mrpt/maps/CPointsMap.h -- would
// collide).
#include <mola_dlio_wrapper/DlioOdometry.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>

#include <sstream>

namespace mola
{

IMPLEMENTS_MRPT_OBJECT(DlioOdometry, FrontEndBase, mola)

namespace
{
/// Parses "x y z yaw_deg pitch_deg roll_deg".
mrpt::poses::CPose3D parsePose6D(const std::string & s)
{
  std::istringstream iss(s);
  double x, y, z, yaw, pitch, roll;
  if (!(iss >> x >> y >> z >> yaw >> pitch >> roll)) {
    THROW_EXCEPTION_FMT(
      "Could not parse pose string '%s' (expected: 'x y z yaw_deg pitch_deg roll_deg')", s.c_str());
  }
  return {x, y, z, mrpt::DEG2RAD(yaw), mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll)};
}

mrpt::poses::CPose3D isometryToPose(const Eigen::Isometry3d & iso)
{
  mrpt::math::CMatrixDouble44 m;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      m(r, c) = iso.matrix()(r, c);
    }
  }
  return mrpt::poses::CPose3D(m);
}

/// Fills a `dlio_core::Config::Extrinsic` (a standard point-transform,
/// p_parent = R*p_child + t) from a parsed "x y z yaw pitch roll" pose.
void poseToExtrinsic(const mrpt::poses::CPose3D & pose, dlio_core::Config::Extrinsic & ext)
{
  ext.R = pose.getRotationMatrix().asEigen();
  ext.t = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
}

}  // namespace

DlioOdometry::DlioOdometry() = default;
DlioOdometry::~DlioOdometry() { shutdownCleanup(); }

void DlioOdometry::onQuit() { shutdownCleanup(); }

void DlioOdometry::shutdownCleanup()
{
  if (shutdown_cleanup_done_.exchange(true)) {
    return;
  }

  worker_lidar_.clear();

  if (save_trajectory_to_file_ && !trajectory_output_file_.empty()) {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    MRPT_LOG_INFO_STREAM(
      "Saving estimated trajectory with " << trajectory_.size() << " keyframes to file '"
                                           << trajectory_output_file_ << "' in TUM format...");
    trajectory_.saveToTextFile_TUM(trajectory_output_file_);
    MRPT_LOG_INFO("Final trajectory saved.");
  }
}

void DlioOdometry::initialize_frontend(const Yaml & cfg)
{
  MRPT_TRY_START

  std::string lidar_sensor_label = "lidar", imu_sensor_label = "imu";
  YAML_LOAD_OPT(lidar_sensor_label, std::string);
  YAML_LOAD_OPT(imu_sensor_label, std::string);
  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);
  imu_sensor_label_regex_ = std::regex(imu_sensor_label);

  std::string imu_pose_in_lidar_str = "0 0 0 0 0 0";
  YAML_LOAD_OPT(imu_pose_in_lidar_str, std::string);
  imu_pose_in_lidar_ = parsePose6D(imu_pose_in_lidar_str);

  std::string baselink2lidar_pose_str = "0 0 0 0 0 0";
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);

  // Loaded into non-underscore locals first, then assigned to the
  // (trailing-underscore) members: YAML_LOAD_OPT's key is the literal C++
  // identifier text, so passing the member name directly would look for a
  // YAML key with a trailing underscore too, which pipeline YAMLs don't use.
  double fallback_scan_period = fallback_scan_period_;
  YAML_LOAD_OPT(fallback_scan_period, double);
  fallback_scan_period_ = fallback_scan_period;

  double blind = blind_;
  YAML_LOAD_OPT(blind, double);
  blind_ = blind;

  YAML_LOAD_OPT3(core_cfg_, adaptive, bool);
  YAML_LOAD_OPT3(core_cfg_, pointcloud_deskew, bool);
  YAML_LOAD_OPT3(core_cfg_, pointcloud_voxelize, bool);

  YAML_LOAD_OPT3(core_cfg_, imu_calibrate, bool);

  YAML_LOAD_OPT3(core_cfg_, wait_until_move, bool);
  YAML_LOAD_OPT3(core_cfg_, dense_map_filtered, bool);
  YAML_LOAD_OPT3(core_cfg_, map_sparse_leaf_size, double);

  YAML_LOAD_OPT3(core_cfg_, gravity, double);
  YAML_LOAD_OPT3(core_cfg_, compute_time_offset, bool);

  YAML_LOAD_OPT3(core_cfg_, imu_approximate_gravity, bool);
  YAML_LOAD_OPT3(core_cfg_, imu_calibrate_gyro, bool);
  YAML_LOAD_OPT3(core_cfg_, imu_calibrate_accel, bool);
  YAML_LOAD_OPT3(core_cfg_, imu_calib_time_sec, double);
  YAML_LOAD_OPT3(core_cfg_, imu_buffer_size, int);

  YAML_LOAD_OPT3(core_cfg_, crop_box_size, double);
  YAML_LOAD_OPT3(core_cfg_, voxel_filter_res, double);

  YAML_LOAD_OPT3(core_cfg_, keyframe_thresh_dist, double);
  YAML_LOAD_OPT3(core_cfg_, keyframe_thresh_rot, double);

  YAML_LOAD_OPT3(core_cfg_, submap_knn, int);
  YAML_LOAD_OPT3(core_cfg_, submap_kcv, int);
  YAML_LOAD_OPT3(core_cfg_, submap_kcc, int);

  YAML_LOAD_OPT3(core_cfg_, gicp_min_num_points, int);
  YAML_LOAD_OPT3(core_cfg_, gicp_k_correspondences, int);
  YAML_LOAD_OPT3(core_cfg_, gicp_max_corr_dist, double);
  YAML_LOAD_OPT3(core_cfg_, gicp_max_iterations, int);
  YAML_LOAD_OPT3(core_cfg_, gicp_transformation_epsilon, double);
  YAML_LOAD_OPT3(core_cfg_, gicp_rotation_epsilon, double);
  YAML_LOAD_OPT3(core_cfg_, gicp_init_lambda_factor, double);

  YAML_LOAD_OPT3(core_cfg_, geo_Kp, double);
  YAML_LOAD_OPT3(core_cfg_, geo_Kv, double);
  YAML_LOAD_OPT3(core_cfg_, geo_Kq, double);
  YAML_LOAD_OPT3(core_cfg_, geo_Kab, double);
  YAML_LOAD_OPT3(core_cfg_, geo_Kgb, double);
  YAML_LOAD_OPT3(core_cfg_, geo_abias_max, double);
  YAML_LOAD_OPT3(core_cfg_, geo_gbias_max, double);

  YAML_LOAD_OPT3(visualization_params_, show_trajectory, bool);
  YAML_LOAD_OPT3(visualization_params_, show_submap, bool);
  YAML_LOAD_OPT3(visualization_params_, camera_follows_vehicle, bool);
  YAML_LOAD_OPT3(visualization_params_, current_pose_corner_size, float);
  YAML_LOAD_OPT3(visualization_params_, submap_point_size, float);
  YAML_LOAD_OPT3(visualization_params_, map_update_decimation, int);

  YAML_LOAD_MEMBER_OPT(save_trajectory_to_file, bool);
  YAML_LOAD_MEMBER_OPT(trajectory_output_file, std::string);

  // DLIO's own extrinsics (used internally by dlio_core, independent of
  // this module's own imu_pose_in_lidar_ used below for publishing): both
  // are standard point-transforms p_baselink = R*p_child + t, so the parsed
  // pose is used directly, unlike FastLio2Odometry's extrinsic_R/T (which
  // needs the inverse for Fast-LIO2's own different convention).
  poseToExtrinsic(imu_pose_in_lidar_, core_cfg_.baselink2imu);
  poseToExtrinsic(parsePose6D(baselink2lidar_pose_str), core_cfg_.baselink2lidar);

  // imu_prior_{accel,gyro}_bias / imu_accel_sm are not exposed via YAML yet
  // (left at Config defaults: zero bias, identity scale/misalignment) --
  // only relevant when imu_calibrate=false, not used by the Oxford Spires
  // pipeline. See the plan's Open items.

  core_ = std::make_unique<DlioCoreBridge>(core_cfg_);

  MRPT_LOG_INFO_STREAM(
    "DlioOdometry initialized. lidar_sensor_label='" << lidar_sensor_label << "' imu_sensor_label='"
                                                     << imu_sensor_label << "'");

  MRPT_TRY_END
}

void DlioOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  ASSERT_(o);

  if (std::regex_match(o->sensorLabel, imu_sensor_label_regex_)) {
    onImuObservation(o);
  } else if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidarObservation(o);
  }
  MRPT_TRY_END
}

void DlioOdometry::onImuObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
  if (!imu || !core_) {
    return;
  }

  // Cheap: run synchronously. IMU is the deskew clock and the geometric
  // observer's propagation clock, must never be dropped.
  core_->feedImu(toDlioImu(*imu));
  drainAndPublish();
}

void DlioOdometry::onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!core_) {
    return;
  }

  tasks_in_flight_++;
  auto fut = worker_lidar_.enqueue(&DlioOdometry::processLidarScan, this, o);
  (void)fut;
}

void DlioOdometry::processLidarScan(mrpt::obs::CObservation::ConstPtr o)
{
  MRPT_TRY_START
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (pc && core_) {
    double t_beg, t_end;
    auto cloud = toDlioCloud(*pc, fallback_scan_period_, t_beg, t_end, blind_);
    core_->addLidarScan(std::move(cloud), t_beg, t_end);
    drainAndPublish();
  }
  tasks_in_flight_--;
  MRPT_TRY_END
}

void DlioOdometry::drainAndPublish()
{
  if (!core_) {
    return;
  }
  for (const auto & out : core_->drainReady()) {
    publishOutput(out);
  }
}

void DlioOdometry::publishOutput(const dlio_core::OdometryOutput & out)
{
  // DLIO estimates the IMU (base_link) pose; report the LiDAR pose instead,
  // matching every other wrapper's `map -> lidar` convention (see the
  // common plan's Oxford Spires section: `gt-tum.txt` is in the raw LiDAR
  // frame). Same "+" composition FastLio2Odometry uses with the identical
  // imu_pose_in_lidar_ value; see DlioOdometry.h's doc comment.
  const mrpt::poses::CPose3D imuPose = isometryToPose(out.pose_imu);
  const mrpt::poses::CPose3D lidarPose = imuPose + imu_pose_in_lidar_;

  const auto timestamp = mrpt::Clock::fromDouble(out.timestamp);

  {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    trajectory_.insert(timestamp, lidarPose);
  }

  if (anyUpdateLocalizationSubscriber()) {
    LocalizationUpdate lu;
    lu.timestamp = timestamp;
    lu.reference_frame = "map";
    lu.child_frame = "lidar";
    lu.method = "dlio";
    lu.pose = lidarPose.asTPose();
    advertiseUpdatedLocalization(lu);
  }

  if (anyUpdateMapSubscriber()) {
    const auto submap = core_->currentSubmap();
    if (submap) {
      MapUpdate mu;
      mu.timestamp = timestamp;
      mu.reference_frame = "map";
      mu.method = "dlio";
      mu.map_name = "local_map";
      mu.map = toMrptPointsMap(*submap);
      mu.keep_last_one_only = true;
      advertiseUpdatedMap(mu);
    }
  }

  updateVisualization(lidarPose, out);
}

void DlioOdometry::updateVisualization(
  const mrpt::poses::CPose3D & lidarPose, const dlio_core::OdometryOutput & out)
{
  if (!visualizer_) {
    return;
  }

  if (visualization_params_.current_pose_corner_size > 0) {
    auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
    glVehicle->insert(
      mrpt::opengl::stock_objects::CornerXYZ(visualization_params_.current_pose_corner_size));
    glVehicle->setPose(lidarPose);
    visualizer_->update_3d_object("dlio/vehicle", glVehicle);
  }

  updateVisualizationPath(lidarPose);
  updateVisualizationSubmap();

  if (visualization_params_.camera_follows_vehicle) {
    const mrpt::math::TPoint3Df lookAt(lidarPose.x(), lidarPose.y(), lidarPose.z());
    visualizer_->update_viewport_look_at(lookAt);
  }

  if (!gui_created_) {
    internalBuildGUI();
    gui_created_ = true;
  }
  updateVisualizationTextLabels(out);
}

void DlioOdometry::updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose)
{
  if (!visualization_params_.show_trajectory) {
    visualizer_->update_3d_object("dlio/path", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (!gl_estimated_path_) {
    gl_estimated_path_ = mrpt::opengl::CSetOfLines::Create();
    gl_estimated_path_->setColor_u8(0x20, 0xa0, 0x20, 0xff);
  }

  const auto t = lidarPose.translation();
  if (gl_estimated_path_->empty()) {
    gl_estimated_path_->appendLine(t, t);
  } else {
    gl_estimated_path_->appendLineStrip(t);
  }

  auto pathGrp = mrpt::opengl::CSetOfObjects::Create();
  pathGrp->insert(mrpt::opengl::CSetOfLines::Create(*gl_estimated_path_));
  visualizer_->update_3d_object("dlio/path", pathGrp);
}

void DlioOdometry::updateVisualizationSubmap()
{
  if (!visualization_params_.show_submap) {
    visualizer_->update_3d_object("dlio/submap", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (++map_viz_update_counter_ < visualization_params_.map_update_decimation) {
    return;
  }
  map_viz_update_counter_ = 0;

  const auto submapPts = core_->currentSubmap();
  if (!submapPts || submapPts->empty()) {
    return;
  }

  const auto pointsMap = toMrptPointsMap(*submapPts);

  auto glCloud = mrpt::opengl::CPointCloudColoured::Create();
  glCloud->loadFromPointsMap(pointsMap.get());
  glCloud->setPointSize(visualization_params_.submap_point_size);

  const auto bbox = pointsMap->boundingBox();
  glCloud->recolorizeByCoordinate(bbox.min.z, bbox.max.z, 2 /*Z*/, mrpt::img::TColormap::cmJET);

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->insert(glCloud);
  visualizer_->update_3d_object("dlio/submap", grp);
}

void DlioOdometry::internalBuildGUI()
{
  using namespace mola::gui;

  gui_.lbStatus = std::make_shared<LiveString>(" ");

  Tab tab;
  tab.title = "Status";
  tab.widgets.emplace_back(Label{gui_.lbStatus});
  tab.widgets.emplace_back(CheckBox{
    "Show trajectory", visualization_params_.show_trajectory,
    [this](bool checked) { visualization_params_.show_trajectory = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Show submap", visualization_params_.show_submap,
    [this](bool checked) { visualization_params_.show_submap = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Camera follows vehicle", visualization_params_.camera_follows_vehicle,
    [this](bool checked) { visualization_params_.camera_follows_vehicle = checked; }});

  WindowDescription desc;
  desc.title = "DLIO";
  desc.position = {5, 700};
  desc.size = {320, 0};
  desc.tabs.emplace_back(std::move(tab));

  visualizer_->create_subwindow_from_description(desc);
}

void DlioOdometry::updateVisualizationTextLabels(const dlio_core::OdometryOutput & out)
{
  if (!gui_.lbStatus) {
    return;
  }

  gui_.lbStatus->set(mrpt::format(
    "t=%.03f | keyframes: %zu | submap pts: %zu | gicp corr: %zu err: %.4f conv: %s | "
    "spaciousness: %.2f density: %.2f | busy: %s",
    out.timestamp, out.analytics.num_keyframes, out.analytics.submap_size,
    out.analytics.num_gicp_correspondences, out.analytics.gicp_final_error,
    out.analytics.gicp_converged ? "yes" : "no", out.analytics.spaciousness, out.analytics.density,
    isBusy() ? "yes" : "no"));
}

void DlioOdometry::spinOnce() {}

bool DlioOdometry::isBusy() const
{
  return tasks_in_flight_.load() > 0 || worker_lidar_.pendingTasks() > 0;
}

mrpt::poses::CPose3DInterpolator DlioOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
