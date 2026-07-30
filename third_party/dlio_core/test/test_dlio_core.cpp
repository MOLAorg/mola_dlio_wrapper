// SPDX-License-Identifier: GPL-3.0-only
// mola_dlio_wrapper: original test code (not derived from DLIO).
#include <dlio_core/DlioCore.h>
#include <gtest/gtest.h>

namespace
{
using dlio_core::Config;
using dlio_core::DEFAULT_GRAVITY;
using dlio_core::DlioCore;
using dlio_core::ImuSample;
using dlio_core::LidarPoint;
using dlio_core::LidarPointVector;
using dlio_core::OdometryOutput;

// A flat 13x13 grid of points on a horizontal plane 1 m below the sensor,
// simulating a LiDAR looking down at a floor. `t_scan_period` gives each
// point a synthetic (linearly spaced) per-point relative timestamp.
LidarPointVector makeFloorScan(double t_scan_period)
{
  LidarPointVector cloud;
  int idx = 0;
  for (double x = -3.0; x <= 3.0; x += 0.5) {
    for (double y = -3.0; y <= 3.0; y += 0.5) {
      LidarPoint p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      // Tiny per-point jitter avoids a perfectly flat z (degenerate bounding
      // box along one axis), which is a known trigger for a heap-buffer
      // overflow inside the system libpcl_filters VoxelGrid on this
      // machine -- see fast_lio2_core's own test for the same guard.
      p.z = -1.0f + 0.001f * static_cast<float>(idx % 7);
      p.intensity = 0.5f;
      p.t_offset_sec = t_scan_period * (idx % 100) / 100.0;
      cloud.push_back(p);
      idx++;
    }
  }
  return cloud;
}

}  // namespace

TEST(DlioCore, StationaryRunConvergesAndBuildsSubmap)
{
  Config cfg;
  // Short calibration window to keep the test fast; still exercises the
  // real gravity-alignment/bias-estimation code path (D4).
  cfg.imu_calibrate = true;
  cfg.imu_calib_time_sec = 0.5;
  cfg.imu_calibrate_gyro = true;
  cfg.imu_calibrate_accel = true;
  cfg.imu_approximate_gravity = true;

  cfg.gicp_min_num_points = 10;
  cfg.gicp_max_iterations = 5;
  cfg.crop_box_size = 0.0;  // don't cut anything from this small synthetic scan
  cfg.pointcloud_voxelize = false;
  cfg.keyframe_thresh_dist = 1.0;
  cfg.keyframe_thresh_rot = 45.0;
  cfg.submap_knn = cfg.submap_kcv = cfg.submap_kcc = 5;

  DlioCore core(cfg);

  constexpr double kImuRateHz = 200.0;
  constexpr double kScanRateHz = 10.0;
  constexpr double kImuDt = 1.0 / kImuRateHz;
  constexpr double kScanDt = 1.0 / kScanRateHz;
  constexpr int kNumScans = 8;

  double t = 0.0;

  // Feed a stationary IMU covering the calibration window plus some margin
  // before the first scan.
  while (t < cfg.imu_calib_time_sec + 0.2) {
    ImuSample s;
    s.t = t;
    s.acc = Eigen::Vector3d(0, 0, DEFAULT_GRAVITY);
    s.gyro = Eigen::Vector3d::Zero();
    core.feedImu(s);
    t += kImuDt;
  }

  double t_scan = t;
  bool got_any_output = false;
  OdometryOutput last_output;

  for (int scan_i = 0; scan_i < kNumScans; scan_i++) {
    const double t_scan_end = t_scan + kScanDt;

    while (t <= t_scan_end + kImuDt) {
      ImuSample s;
      s.t = t;
      s.acc = Eigen::Vector3d(0, 0, DEFAULT_GRAVITY);
      s.gyro = Eigen::Vector3d::Zero();
      core.feedImu(s);
      t += kImuDt;
    }

    core.addLidarScan(makeFloorScan(kScanDt), t_scan, t_scan_end);

    for (const auto & out : core.drainReady()) {
      got_any_output = true;
      last_output = out;
      ASSERT_TRUE(out.pose_imu.matrix().allFinite());
      ASSERT_TRUE(out.vel.allFinite());
      ASSERT_TRUE(out.gravity.allFinite());
    }

    t_scan = t_scan_end;
  }

  EXPECT_TRUE(got_any_output) << "expected at least one OdometryOutput after " << kNumScans
                              << " scans";
  if (got_any_output) {
    // Purely stationary run: position should stay close to the origin.
    EXPECT_LT(last_output.pose_imu.translation().norm(), 2.0);
    // Reconstructed gravity should point down, close to -G in Z (body frame
    // stays close to the gravity-aligned world frame for a stationary run).
    EXPECT_NEAR(last_output.gravity.z(), -DEFAULT_GRAVITY, 2.0);
  }

  EXPECT_GT(core.numKeyframes(), 0u);

  auto submap = core.currentSubmap();
  ASSERT_TRUE(static_cast<bool>(submap));
  EXPECT_GT(submap->size(), 0u);
}
