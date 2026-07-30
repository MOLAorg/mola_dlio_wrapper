// SPDX-License-Identifier: MIT
//
// Derived from DLIO's dlio::OdomNode (include/dlio/odom.h,
// src/dlio/odom.cc), MIT License, Copyright (c) Kenny J. Chen,
// Ryan Nemiroff, Brett T. Lopez (VECTR Lab, UCLA). See
// THIRD_PARTY_NOTICES.md at the repository root.
//
// Every ROS type/call (node handle, subscribers, publishers, timers,
// ros::Time, tf broadcasting, the terminal debug printout and its
// /proc/cpuinfo /proc/self/stat CPU-usage helpers) was removed and replaced
// with a push-based API (`feedImu()`/`addLidarScan()`/`drainReady()`,
// mirroring fast_lio2_core's own core-library shape). The per-scan
// processing steps are otherwise the same private methods upstream's
// `OdomNode` had, transplanted into this stateful, ROS-free class.
//
// Deviations beyond mechanical de-ROS-ing (see THIRD_PARTY_NOTICES.md for
// the full list; summarized at each spot below):
//  - `imuMeasFromTimeRange()` no longer blocks on a condition variable
//    waiting for IMU data to catch up to a scan's end time (that relied on
//    the IMU and LiDAR ROS callbacks running on independent threads pumped
//    by `ros::spin()`). It now returns immediately; a scan whose IMU
//    coverage isn't ready yet is queued and retried from a later
//    `feedImu()` call, exactly like fast_lio2_core's `addLidarScan()`/
//    `drainReady()` contract -- see the class doc comment below.
//  - Three places in upstream used a C++ function-local `static` variable
//    to carry state across calls (the low-pass filter state in
//    `computeSpaciousness()`/`computeDensity()`, the running average
//    accumulators in the IMU calibration window, and the previous
//    sample/angular-velocity in `transformImu()`). A function-local static
//    is process-wide, not per-instance, so two `DlioCore` objects in the
//    same process (e.g. this library's own gtest) would silently share and
//    corrupt each other's state. All three are now ordinary member
//    variables; behavior for a single instance is unchanged.
#pragma once

#include <dlio/dlio.h>
#include <nano_gicp/nano_gicp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <deque>
#include <dlio_core/Types.hpp>
#include <future>
#include <mutex>
#include <utility>
#include <vector>

namespace dlio_core
{
using PointCloudXYZI = pcl::PointCloud<PointType>;

/// DLIO's algorithm core: GICP scan-to-submap registration with IMU motion
/// correction (continuous-time deskewing) and a nonlinear geometric-observer
/// state estimator. No ROS, no MOLA dependency.
///
/// Usage: call `feedImu()` for every IMU sample and `addLidarScan()` for
/// every LiDAR scan, in timestamp order (matches fast_lio2_core's own
/// contract). A scan is only processed once IMU data covering its full time
/// span has arrived; if `addLidarScan()` is called before that, it buffers
/// the scan and returns -- this is picked up automatically by a later
/// `feedImu()` call once coverage completes, and is not a dropped scan.
/// Call `drainReady()` to collect newly-completed results.
///
/// Internally keeps the two pieces of concurrency that are part of DLIO's
/// real-time design rather than being ROS plumbing: keyframe/submap
/// rebuilding runs on a detached `std::async` task so the next scan can be
/// registered against the previous submap while a new one is being built
/// (`buildKeyframesAndSubmap()`/`pauseSubmapBuildIfNeeded()`, unchanged from
/// upstream).
class DlioCore
{
public:
  explicit DlioCore(const Config & cfg);

  /// Buffers a (bias/extrinsic-corrected) IMU sample and, during the
  /// startup window, feeds the static gyro/accel bias and gravity-alignment
  /// calibration. If a pending scan's IMU coverage is now complete,
  /// processes it (result collected via `drainReady()`).
  void feedImu(const ImuSample & s);

  /// Buffers a LiDAR scan (points in LiDAR body frame) and processes it
  /// immediately if enough IMU data is already available.
  void addLidarScan(LidarPointVector points, double t_beg, double t_end);

  /// Returns (and clears) every `OdometryOutput` produced since the last
  /// call. Usually 0 or 1 entries.
  std::vector<OdometryOutput> drainReady();

  /// True once DLIO has left its IMU calibration/gravity-alignment startup
  /// window and processed at least one scan (mirrors upstream's
  /// `dlio_initialized` && `first_valid_scan`).
  bool isInitialized() const { return dlio_initialized_ && first_valid_scan_; }

  /// Snapshot of the current registration submap (world frame). May be
  /// expensive (flattens keyframe clouds); call sparingly.
  std::shared_ptr<const XYZIPointVector> currentSubmap() const;

  std::size_t numKeyframes() const;

private:
  struct ImuMeas
  {
    double stamp = 0;
    double dt = 0;  //!< difference vs. the previous measurement
    Eigen::Vector3f ang_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
  };

  struct ImuBias
  {
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel = Eigen::Vector3f::Zero();
  };

  struct Frames
  {
    Eigen::Vector3f b = Eigen::Vector3f::Zero();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();
  };

  struct Velocity
  {
    Frames lin;
    Frames ang;
  };

  struct State
  {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();  //!< position in world frame
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Velocity v;
    ImuBias b;  //!< IMU biases in body frame
  };

  struct Pose
  {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
  };

  struct Geo
  {
    bool first_opt_done = false;
    std::mutex mtx;
    Eigen::Vector3f prev_p = Eigen::Vector3f::Zero();
    Eigen::Quaternionf prev_q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f prev_vel = Eigen::Vector3f::Zero();
  };

  struct PendingScan
  {
    LidarPointVector points;
    double t_beg = 0;
    double t_end = 0;
  };

  using KeyframeCloudPtr = PointCloudXYZI::ConstPtr;

  // --- Setup helpers ---
  void computeExtrinsicMatrices();

  // --- IMU path ---
  ImuMeas transformImu(const ImuSample & raw);
  void runImuCalibrationWindow(const ImuMeas & m, double t_since_first_imu);
  bool imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator & begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator & end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> integrateImu(
    double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double> & sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double> & sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void propagateState();  //!< geometric observer: IMU-rate state propagation
  void updateState();     //!< geometric observer: GICP-rate correction

  /// Outcome of attempting to process one pending scan. Distinguishes two
  /// very different "not produced yet" reasons, which upstream's blocking
  /// design never had to (see the class doc comment's deviation list):
  /// `kDeferred` means the scan is still a valid candidate and should be
  /// retried once more IMU data arrives; `kDropped` means the scan can
  /// *never* succeed no matter how much more IMU data arrives (e.g. it
  /// predates the oldest sample still in `imu_buffer_`, which only grows
  /// older-bound over time) and must be discarded instead of retried
  /// forever -- upstream's ROS callback achieves the same effect implicitly,
  /// by simply never re-invoking the callback for a message it already
  /// processed and rejected.
  enum class ScanOutcome
  {
    kReady,
    kDeferred,
    kDropped
  };

  // --- Scan path ---
  void tryProcessPending();
  ScanOutcome processOneScan(const PendingScan & scan);
  void initializeDLIO();
  ScanOutcome preprocessAndDeskew(
    const PendingScan & scan, PointCloudXYZI::Ptr & out_deskewed,
    PointCloudXYZI::Ptr & out_current);
  void getNextPose();
  void propagateGICP();

  void computeMetrics();
  void computeSpaciousness(const PointCloudXYZI & original_scan);
  void computeDensity();
  void setAdaptiveParams();

  void updateKeyframes(const PointCloudXYZI::ConstPtr & current_scan);
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(const std::vector<float> & dists, int k, const std::vector<int> & frames);
  void buildSubmap(const State & vehicle_state);
  void buildKeyframesAndSubmap(State vehicle_state);
  void pauseSubmapBuildIfNeeded();

  OdometryOutput makeOutput(const PointCloudXYZI::ConstPtr & current_scan_body);

  // --- Config ---
  Config cfg_;
  int num_threads_ = 1;

  // --- Flags ---
  bool dlio_initialized_ = false;
  bool first_valid_scan_ = false;
  bool first_imu_received_ = false;
  bool imu_calibrated_ = false;
  bool submap_has_changed_ = true;
  bool gicp_has_converged_ = false;
  int deskew_size_ = 0;

  // --- Preprocessing filters ---
  pcl::CropBox<PointType> crop_;
  pcl::VoxelGrid<PointType> voxel_;

  // --- Point clouds ---
  PointCloudXYZI::ConstPtr original_scan_ = std::make_shared<const PointCloudXYZI>();
  PointCloudXYZI::ConstPtr deskewed_scan_ = std::make_shared<const PointCloudXYZI>();
  PointCloudXYZI::ConstPtr current_scan_ = std::make_shared<const PointCloudXYZI>();

  // --- Keyframes ---
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, KeyframeCloudPtr>>
    keyframes_;
  std::vector<double> keyframe_timestamps_;
  std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations_;
  std::mutex keyframes_mutex_;
  int num_processed_keyframes_ = 0;

  pcl::ConvexHull<PointType> convex_hull_;
  pcl::ConcaveHull<PointType> concave_hull_;
  std::vector<int> keyframe_convex_;
  std::vector<int> keyframe_concave_;

  // --- Submap ---
  PointCloudXYZI::ConstPtr submap_cloud_ = std::make_shared<const PointCloudXYZI>();
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals_;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree_;
  std::vector<int> submap_kf_idx_curr_;
  std::vector<int> submap_kf_idx_prev_;

  bool new_submap_is_ready_ = false;
  std::future<void> submap_future_;
  std::condition_variable submap_build_cv_;
  bool main_loop_running_ = false;
  std::mutex main_loop_running_mutex_;

  // --- Timestamps ---
  double scan_stamp_ = 0;  //!< median-point time of the current scan
  double prev_scan_stamp_ = 0;
  double first_scan_stamp_ = 0;
  double elapsed_time_ = 0;

  // --- GICP ---
  nano_gicp::NanoGICP<PointType, PointType> gicp_;
  nano_gicp::NanoGICP<PointType, PointType> gicp_temp_;

  // --- Transformations ---
  Eigen::Matrix4f T_ = Eigen::Matrix4f::Identity();  //!< world_from_baselink
  Eigen::Matrix4f T_prior_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_corr_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f baselink2imu_T_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f baselink2lidar_T_ = Eigen::Matrix4f::Identity();

  // --- IMU ---
  double first_imu_stamp_ = 0;
  double prev_imu_stamp_ = 0;
  ImuMeas imu_meas_;

  boost::circular_buffer<ImuMeas> imu_buffer_{2000};
  std::mutex mtx_imu_;

  // IMU calibration window accumulators (member, not `static`: see the
  // class-level deviation note).
  int imu_calib_num_samples_ = 0;
  Eigen::Vector3f imu_calib_gyro_avg_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f imu_calib_accel_avg_ = Eigen::Vector3f::Zero();

  // `transformImu()`'s previous-sample state (member, not `static`).
  bool imu_transform_has_prev_ = false;
  double imu_transform_prev_stamp_ = 0;
  Eigen::Vector3f imu_transform_prev_ang_vel_cg_ = Eigen::Vector3f::Zero();

  // --- Geometric observer ---
  Geo geo_;

  // --- State ---
  State state_;
  Pose lidar_pose_;

  // --- Metrics (member, not `static`: see the class-level deviation note) ---
  bool metrics_initialized_ = false;
  float spaciousness_ = 0;
  float spaciousness_prev_ = 0;
  float density_ = 0;
  float density_prev_ = 0;

  // One-off gravity estimate from the IMU calibration window, and the
  // sample count it used -- dumped once by MOLA_DLIO_DUMP_ESTIMATED_GRAV,
  // see makeOutput()/DlioCore.cpp.
  Eigen::Vector3f initial_gravity_estimate_ = Eigen::Vector3f::Zero();
  int initial_gravity_estimate_nsamples_ = 0;

  // --- Adaptive parameters ---
  double keyframe_thresh_dist_active_ = 1.0;

  // --- Pending scans / results (drainReady contract) ---
  std::deque<PendingScan> pending_scans_;
  std::vector<OdometryOutput> ready_outputs_;
  std::mutex pending_mutex_;
};

}  // namespace dlio_core
