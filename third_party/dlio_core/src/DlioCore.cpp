// SPDX-License-Identifier: MIT
//
// Derived from DLIO's dlio::OdomNode (include/dlio/odom.h,
// src/dlio/odom.cc). See THIRD_PARTY_NOTICES.md and the doc comment at the
// top of include/dlio_core/DlioCore.h for what changed and why.
#include <dlio_core/DlioCore.h>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <queue>

namespace dlio_core
{
namespace
{
/// `<time> <gx> <gy> <gz>`, same format every wrapper in this benchmark uses
/// (see the common plan's "Instrumentation" section). Off by default, zero
/// cost when off.
bool dumpEstimatedGravEnabled()
{
  static const bool enabled = ::getenv("MOLA_DLIO_DUMP_ESTIMATED_GRAV") != nullptr;
  return enabled;
}

std::ofstream & gravDumpFile()
{
  static std::ofstream f("grav.txt");
  return f;
}

}  // namespace

DlioCore::DlioCore(const Config & cfg) : cfg_(cfg)
{
  num_threads_ = omp_get_max_threads();
  imu_calibrated_ = !cfg_.imu_calibrate;
  imu_buffer_.set_capacity(static_cast<std::size_t>(cfg_.imu_buffer_size));

  computeExtrinsicMatrices();

  convex_hull_.setDimension(3);
  concave_hull_.setDimension(3);
  concave_hull_.setAlpha(cfg_.keyframe_thresh_dist);
  concave_hull_.setKeepInformation(true);

  for (auto * g : {&gicp_, &gicp_temp_}) {
#ifdef DLIO_DETERMINISTIC
    // The GICP Hessian and error accumulators are summed per thread under a
    // dynamic schedule, so their rounding depends on how iterations were
    // partitioned; one thread restores a fixed summation order.
    g->setNumThreads(1);
#endif
    g->setCorrespondenceRandomness(cfg_.gicp_k_correspondences);
    g->setMaxCorrespondenceDistance(cfg_.gicp_max_corr_dist);
    g->setMaximumIterations(cfg_.gicp_max_iterations);
    g->setTransformationEpsilon(cfg_.gicp_transformation_epsilon);
    g->setRotationEpsilon(cfg_.gicp_rotation_epsilon);
    g->setInitialLambdaFactor(cfg_.gicp_init_lambda_factor);
  }

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  gicp_.setSearchMethodSource(temp, true);
  gicp_.setSearchMethodTarget(temp, true);
  gicp_temp_.setSearchMethodSource(temp, true);
  gicp_temp_.setSearchMethodTarget(temp, true);

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  crop_.setNegative(true);
  const float cs = static_cast<float>(cfg_.crop_box_size);
  crop_.setMin(Eigen::Vector4f(-cs, -cs, -cs, 1.0f));
  crop_.setMax(Eigen::Vector4f(cs, cs, cs, 1.0f));

  const float vf = static_cast<float>(cfg_.voxel_filter_res);
  voxel_.setLeafSize(vf, vf, vf);

  keyframe_thresh_dist_active_ = cfg_.keyframe_thresh_dist;

  if (!cfg_.imu_calibrate) {
    state_.b.accel = cfg_.imu_prior_accel_bias.cast<float>();
    state_.b.gyro = cfg_.imu_prior_gyro_bias.cast<float>();
  }
}

void DlioCore::computeExtrinsicMatrices()
{
  baselink2imu_T_ = Eigen::Matrix4f::Identity();
  baselink2imu_T_.block<3, 1>(0, 3) = cfg_.baselink2imu.t.cast<float>();
  baselink2imu_T_.block<3, 3>(0, 0) = cfg_.baselink2imu.R.cast<float>();

  baselink2lidar_T_ = Eigen::Matrix4f::Identity();
  baselink2lidar_T_.block<3, 1>(0, 3) = cfg_.baselink2lidar.t.cast<float>();
  baselink2lidar_T_.block<3, 3>(0, 0) = cfg_.baselink2lidar.R.cast<float>();
}

// ---------------------------------------------------------------------------
//  IMU path
// ---------------------------------------------------------------------------

DlioCore::ImuMeas DlioCore::transformImu(const ImuSample & raw)
{
  ImuMeas out;
  out.stamp = raw.t;

  double dt = imu_transform_has_prev_ ? (raw.t - imu_transform_prev_stamp_) : 0.0;
  imu_transform_prev_stamp_ = raw.t;
  if (dt == 0) {
    dt = 1.0 / 200.0;
  }

  const Eigen::Vector3f ang_vel = raw.gyro.cast<float>();
  const Eigen::Vector3f ang_vel_cg = cfg_.baselink2imu.R.cast<float>() * ang_vel;

  if (!imu_transform_has_prev_) {
    imu_transform_prev_ang_vel_cg_ = ang_vel_cg;
    imu_transform_has_prev_ = true;
  }

  const Eigen::Vector3f lin_accel = raw.acc.cast<float>();
  Eigen::Vector3f lin_accel_cg = cfg_.baselink2imu.R.cast<float>() * lin_accel;

  // Lever-arm correction: transports the acceleration measured at the IMU
  // origin to base_link, `-t` being the IMU's position wrt base_link
  // negated (see Config::Extrinsic doc).
  const Eigen::Vector3f lever = -cfg_.baselink2imu.t.cast<float>();
  lin_accel_cg +=
    ((ang_vel_cg - imu_transform_prev_ang_vel_cg_) / static_cast<float>(dt)).cross(lever) +
    ang_vel_cg.cross(ang_vel_cg.cross(lever));

  imu_transform_prev_ang_vel_cg_ = ang_vel_cg;

  out.ang_vel = ang_vel_cg;
  out.lin_accel = lin_accel_cg;
  return out;
}

void DlioCore::runImuCalibrationWindow(const ImuMeas & m, double t_since_first_imu)
{
  if (t_since_first_imu < cfg_.imu_calib_time_sec) {
    imu_calib_num_samples_++;
    imu_calib_gyro_avg_ += m.ang_vel;
    imu_calib_accel_avg_ += m.lin_accel;
    return;
  }

  if (imu_calib_num_samples_ <= 0) {
    // No samples collected within the window (imu_calib_time_sec too small
    // or IMU started right at/after the window boundary): keep whatever
    // prior bias the Config specified and mark calibration done rather than
    // dividing by zero.
    imu_calibrated_ = true;
    return;
  }

  imu_calib_gyro_avg_ /= static_cast<float>(imu_calib_num_samples_);
  imu_calib_accel_avg_ /= static_cast<float>(imu_calib_num_samples_);

  Eigen::Vector3f grav_vec(0.f, 0.f, static_cast<float>(cfg_.gravity));

  if (cfg_.imu_approximate_gravity) {
    grav_vec = (imu_calib_accel_avg_ - state_.b.accel).normalized() *
               std::abs(static_cast<float>(cfg_.gravity));
    const Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(
      grav_vec, Eigen::Vector3f(0.f, 0.f, static_cast<float>(cfg_.gravity)));

    state_.q = grav_q;
    T_.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    lidar_pose_.q = state_.q;

    initial_gravity_estimate_ = grav_vec;
    initial_gravity_estimate_nsamples_ = imu_calib_num_samples_;

    if (dumpEstimatedGravEnabled()) {
      auto & f = gravDumpFile();
      f << "# initial gravity estimate from " << initial_gravity_estimate_nsamples_
        << " calibration samples\n";
      f << m.stamp << " " << grav_vec.x() << " " << grav_vec.y() << " " << grav_vec.z() << "\n";
      f.flush();
    }
  }

  if (cfg_.imu_calibrate_accel) {
    state_.b.accel = imu_calib_accel_avg_ - grav_vec;
  }
  if (cfg_.imu_calibrate_gyro) {
    state_.b.gyro = imu_calib_gyro_avg_;
  }

  imu_calibrated_ = true;
}

bool DlioCore::imuMeasFromTimeRange(
  double start_time, double end_time,
  boost::circular_buffer<ImuMeas>::reverse_iterator & begin_imu_it,
  boost::circular_buffer<ImuMeas>::reverse_iterator & end_imu_it)
{
  std::lock_guard<std::mutex> lock(mtx_imu_);

  if (imu_buffer_.empty() || imu_buffer_.front().stamp < end_time) {
    // Not enough IMU data yet. Upstream blocks here on a condition variable
    // (IMU and LiDAR ROS callbacks run on independent threads); this port
    // returns immediately instead and lets the caller defer the scan for a
    // later retry -- see the DlioCore.h class doc comment.
    return false;
  }

  auto imu_it = imu_buffer_.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != imu_buffer_.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != imu_buffer_.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == imu_buffer_.end()) {
    return false;
  }
  imu_it++;

  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);
  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> DlioCore::integrateImu(
  double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
  const std::vector<double> & sorted_timestamps)
{
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  if (!imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it)) {
    return empty;
  }

  // Backwards integration to find pose at first IMU sample.
  const ImuMeas & f1 = *begin_imu_it;
  const ImuMeas & f2 = *(begin_imu_it + 1);

  const double dt = f2.dt;
  const double idt = start_time - f1.stamp;

  const Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  const Eigen::Vector3f alpha = alpha_dt / static_cast<float>(dt);

  const Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5f * alpha * static_cast<float>(idt));

  q_init = Eigen::Quaternionf(
    q_init.w() -
      0.5 * (q_init.x() * omega_i[0] + q_init.y() * omega_i[1] + q_init.z() * omega_i[2]) * idt,
    q_init.x() +
      0.5 * (q_init.w() * omega_i[0] - q_init.z() * omega_i[1] + q_init.y() * omega_i[2]) * idt,
    q_init.y() +
      0.5 * (q_init.z() * omega_i[0] + q_init.w() * omega_i[1] - q_init.x() * omega_i[2]) * idt,
    q_init.z() +
      0.5 * (q_init.x() * omega_i[1] - q_init.y() * omega_i[0] + q_init.w() * omega_i[2]) * idt);
  q_init.normalize();

  const Eigen::Vector3f omega = f1.ang_vel + 0.5f * alpha_dt;

  Eigen::Quaternionf q2(
    q_init.w() - 0.5 * (q_init.x() * omega[0] + q_init.y() * omega[1] + q_init.z() * omega[2]) * dt,
    q_init.x() + 0.5 * (q_init.w() * omega[0] - q_init.z() * omega[1] + q_init.y() * omega[2]) * dt,
    q_init.y() + 0.5 * (q_init.z() * omega[0] + q_init.w() * omega[1] - q_init.x() * omega[2]) * dt,
    q_init.z() +
      0.5 * (q_init.x() * omega[1] - q_init.y() * omega[0] + q_init.w() * omega[2]) * dt);
  q2.normalize();

  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= static_cast<float>(cfg_.gravity);

  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= static_cast<float>(cfg_.gravity);

  const Eigen::Vector3f j = (a2 - a1) / static_cast<float>(dt);

  v_init -= a1 * static_cast<float>(idt) + 0.5f * j * static_cast<float>(idt * idt);
  p_init -= v_init * static_cast<float>(idt) + 0.5f * a1 * static_cast<float>(idt * idt) +
            (1.f / 6.f) * j * static_cast<float>(idt * idt * idt);

  return integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
DlioCore::integrateImuInternal(
  Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
  const std::vector<double> & sorted_timestamps,
  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it)
{
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= static_cast<float>(cfg_.gravity);

  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {
    const ImuMeas & f0 = *prev_imu_it;
    const ImuMeas & f = *imu_it;

    const double dt = f.dt;

    const Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    const Eigen::Vector3f alpha = alpha_dt / static_cast<float>(dt);

    const Eigen::Vector3f omega = f0.ang_vel + 0.5f * alpha_dt;

    q = Eigen::Quaternionf(
      q.w() - 0.5 * (q.x() * omega[0] + q.y() * omega[1] + q.z() * omega[2]) * dt,
      q.x() + 0.5 * (q.w() * omega[0] - q.z() * omega[1] + q.y() * omega[2]) * dt,
      q.y() + 0.5 * (q.z() * omega[0] + q.w() * omega[1] - q.x() * omega[2]) * dt,
      q.z() + 0.5 * (q.x() * omega[1] - q.y() * omega[0] + q.w() * omega[2]) * dt);
    q.normalize();

    const Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= static_cast<float>(cfg_.gravity);

    const Eigen::Vector3f j_dt = a - a0;
    const Eigen::Vector3f j = j_dt / static_cast<float>(dt);

    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      const double idt = *stamp_it - f0.stamp;

      const Eigen::Vector3f omega_i = f0.ang_vel + 0.5f * alpha * static_cast<float>(idt);

      Eigen::Quaternionf q_i(
        q.w() - 0.5 * (q.x() * omega_i[0] + q.y() * omega_i[1] + q.z() * omega_i[2]) * idt,
        q.x() + 0.5 * (q.w() * omega_i[0] - q.z() * omega_i[1] + q.y() * omega_i[2]) * idt,
        q.y() + 0.5 * (q.z() * omega_i[0] + q.w() * omega_i[1] - q.x() * omega_i[2]) * idt,
        q.z() + 0.5 * (q.x() * omega_i[1] - q.y() * omega_i[0] + q.w() * omega_i[2]) * idt);
      q_i.normalize();

      const Eigen::Vector3f p_i = p + v * static_cast<float>(idt) +
                                  0.5f * a0 * static_cast<float>(idt * idt) +
                                  (1.f / 6.f) * j * static_cast<float>(idt * idt * idt);

      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block<3, 3>(0, 0) = q_i.toRotationMatrix();
      T.block<3, 1>(0, 3) = p_i;
      imu_se3.push_back(T);

      stamp_it++;
    }

    p += v * static_cast<float>(dt) + 0.5f * a0 * static_cast<float>(dt * dt);
    v += a0 * static_cast<float>(dt) + 0.5f * j_dt * static_cast<float>(dt);

    prev_imu_it = imu_it;
  }

  return imu_se3;
}

void DlioCore::propagateGICP()
{
  lidar_pose_.p << T_(0, 3), T_(1, 3), T_(2, 3);

  Eigen::Matrix3f rot;
  // clang-format off
  rot << T_(0, 0), T_(0, 1), T_(0, 2),
         T_(1, 0), T_(1, 1), T_(1, 2),
         T_(2, 0), T_(2, 1), T_(2, 2);
  // clang-format on

  Eigen::Quaternionf q(rot);
  q.normalize();
  lidar_pose_.q = q;
}

void DlioCore::propagateState()
{
  std::lock_guard<std::mutex> lock(geo_.mtx);

  const double dt = imu_meas_.dt;

  const Eigen::Quaternionf qhat = state_.q;
  const Eigen::Vector3f world_accel = qhat._transformVector(imu_meas_.lin_accel);

  state_.p[0] += state_.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  state_.p[1] += state_.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  state_.p[2] += state_.v.lin.w[2] * dt + 0.5 * dt * dt * (world_accel[2] - cfg_.gravity);

  state_.v.lin.w[0] += world_accel[0] * dt;
  state_.v.lin.w[1] += world_accel[1] * dt;
  state_.v.lin.w[2] += (world_accel[2] - cfg_.gravity) * dt;
  state_.v.lin.b = state_.q.toRotationMatrix().inverse() * state_.v.lin.w;

  Eigen::Quaternionf omega;
  omega.w() = 0;
  omega.vec() = imu_meas_.ang_vel;
  const Eigen::Quaternionf tmp = qhat * omega;
  state_.q.w() += 0.5 * dt * tmp.w();
  state_.q.vec() += 0.5 * dt * tmp.vec();
  state_.q.normalize();

  state_.v.ang.b = imu_meas_.ang_vel;
  state_.v.ang.w = state_.q.toRotationMatrix() * state_.v.ang.b;
}

void DlioCore::updateState()
{
  std::lock_guard<std::mutex> lock(geo_.mtx);

  const Eigen::Vector3f pin = lidar_pose_.p;
  const Eigen::Quaternionf qin = lidar_pose_.q;
  const double dt = scan_stamp_ - prev_scan_stamp_;

  const Eigen::Quaternionf qhat = state_.q;
  Eigen::Quaternionf qe = qhat.conjugate() * qin;

  const double sgn = (qe.w() < 0) ? -1.0 : 1.0;

  Eigen::Quaternionf qcorr;
  qcorr.w() = 1 - std::abs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  const Eigen::Vector3f err = pin - state_.p;
  const Eigen::Vector3f err_body = qhat.conjugate()._transformVector(err);

  const double abias_max = cfg_.geo_abias_max;
  const double gbias_max = cfg_.geo_gbias_max;

  state_.b.accel -= dt * cfg_.geo_Kab * err_body;
  state_.b.accel = state_.b.accel.array().min(abias_max).max(-abias_max);

  state_.b.gyro[0] -= dt * cfg_.geo_Kgb * qe.w() * qe.x();
  state_.b.gyro[1] -= dt * cfg_.geo_Kgb * qe.w() * qe.y();
  state_.b.gyro[2] -= dt * cfg_.geo_Kgb * qe.w() * qe.z();
  state_.b.gyro = state_.b.gyro.array().min(gbias_max).max(-gbias_max);

  state_.p += dt * cfg_.geo_Kp * err;
  state_.v.lin.w += dt * cfg_.geo_Kv * err;

  state_.q.w() += dt * cfg_.geo_Kq * qcorr.w();
  state_.q.x() += dt * cfg_.geo_Kq * qcorr.x();
  state_.q.y() += dt * cfg_.geo_Kq * qcorr.y();
  state_.q.z() += dt * cfg_.geo_Kq * qcorr.z();
  state_.q.normalize();

  geo_.prev_p = state_.p;
  geo_.prev_q = state_.q;
  geo_.prev_vel = state_.v.lin.w;
}

// ---------------------------------------------------------------------------
//  Scan path
// ---------------------------------------------------------------------------

void DlioCore::initializeDLIO()
{
  if (!first_imu_received_ || !imu_calibrated_) {
    return;
  }
  dlio_initialized_ = true;
}

void DlioCore::feedImu(const ImuSample & raw)
{
  first_imu_received_ = true;

  const ImuMeas transformed = transformImu(raw);

  if (first_imu_stamp_ == 0.0) {
    first_imu_stamp_ = transformed.stamp;
  }

  if (!imu_calibrated_) {
    runImuCalibrationWindow(transformed, transformed.stamp - first_imu_stamp_);
  } else {
    double dt = transformed.stamp - prev_imu_stamp_;
    if (dt == 0) {
      dt = 1.0 / 200.0;
    }
    prev_imu_stamp_ = transformed.stamp;

    ImuMeas m;
    m.stamp = transformed.stamp;
    m.dt = dt;
    m.lin_accel = (cfg_.imu_accel_sm.cast<float>() * transformed.lin_accel) - state_.b.accel;
    m.ang_vel = transformed.ang_vel - state_.b.gyro;

    imu_meas_ = m;

    {
      std::lock_guard<std::mutex> lock(mtx_imu_);
      imu_buffer_.push_front(m);
    }

    if (geo_.first_opt_done) {
      propagateState();
    }
  }

  std::lock_guard<std::mutex> lock(pending_mutex_);
  tryProcessPending();
}

void DlioCore::addLidarScan(LidarPointVector points, double t_beg, double t_end)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  PendingScan scan;
  scan.points = std::move(points);
  scan.t_beg = t_beg;
  scan.t_end = t_end;
  pending_scans_.push_back(std::move(scan));
  tryProcessPending();
}

void DlioCore::tryProcessPending()
{
  // Caller holds pending_mutex_. Scans must resolve strictly in feeding
  // order: DLIO's state (T_, keyframes_, gicp_, ...) is inherently
  // sequential, so a later scan can never usefully process ahead of an
  // earlier still-pending one.
  while (!pending_scans_.empty()) {
    if (!dlio_initialized_) {
      initializeDLIO();
      if (!dlio_initialized_) {
        break;
      }
    }

    const ScanOutcome outcome = processOneScan(pending_scans_.front());
    if (outcome == ScanOutcome::kDeferred) {
      break;
    }
    // kReady or kDropped: this scan is resolved one way or another, move on.
    pending_scans_.pop_front();
  }
}

std::vector<OdometryOutput> DlioCore::drainReady()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  std::vector<OdometryOutput> out;
  out.swap(ready_outputs_);
  return out;
}

DlioCore::ScanOutcome DlioCore::processOneScan(const PendingScan & scan)
{
  {
    std::lock_guard<std::mutex> lock(main_loop_running_mutex_);
    main_loop_running_ = true;
  }

  if (first_scan_stamp_ == 0.0) {
    first_scan_stamp_ = scan.t_beg;
  }

  // Build the DLIO-native point cloud and apply the near-field crop-box cut.
  PointCloudXYZI::Ptr original(new PointCloudXYZI());
  original->points.reserve(scan.points.size());
  for (const auto & p : scan.points) {
    PointType pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    pt.intensity = p.intensity;
    pt.timestamp = scan.t_beg + p.t_offset_sec;
    original->points.push_back(pt);
  }
  original->width = original->points.size();
  original->height = 1;
  original->is_dense = true;

  crop_.setInputCloud(original);
  PointCloudXYZI::Ptr cropped(new PointCloudXYZI());
  crop_.filter(*cropped);
  original_scan_ = cropped;

  PointCloudXYZI::Ptr deskewed;
  PointCloudXYZI::Ptr current;
  const ScanOutcome preprocess_outcome = preprocessAndDeskew(scan, deskewed, current);
  if (preprocess_outcome != ScanOutcome::kReady) {
    std::lock_guard<std::mutex> lock(main_loop_running_mutex_);
    main_loop_running_ = false;
    return preprocess_outcome;
  }

  deskewed_scan_ = deskewed;
  current_scan_ = current;

  if (static_cast<int>(current_scan_->points.size()) <= cfg_.gicp_min_num_points) {
    // Too few points to register (matches upstream's ROS_FATAL("Low number
    // of points in the cloud!")): drop this scan, nothing to retry.
    std::lock_guard<std::mutex> lock(main_loop_running_mutex_);
    main_loop_running_ = false;
    return ScanOutcome::kDropped;
  }

  computeMetrics();
  if (cfg_.adaptive) {
    setAdaptiveParams();
  }

  gicp_.setInputSource(current_scan_);
  gicp_.calculateSourceCovariances();

  if (keyframes_.empty()) {
    prev_scan_stamp_ = scan_stamp_;
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      keyframes_.push_back({{lidar_pose_.p, lidar_pose_.q}, current_scan_});
      keyframe_timestamps_.push_back(scan_stamp_);
      keyframe_normals_.push_back(gicp_.getSourceCovariances());
      keyframe_transformations_.push_back(T_corr_);
    }
    {
      std::lock_guard<std::mutex> lock(main_loop_running_mutex_);
      main_loop_running_ = false;
    }
    submap_future_ =
      std::async(std::launch::async, &DlioCore::buildKeyframesAndSubmap, this, state_);
    submap_future_.wait();

    ready_outputs_.push_back(makeOutput(current_scan_));
    return ScanOutcome::kReady;
  }

  getNextPose();
  updateKeyframes(current_scan_);

  {
    std::unique_lock<std::mutex> lock(main_loop_running_mutex_);
    if (new_submap_is_ready_) {
      main_loop_running_ = false;
      lock.unlock();
      submap_future_ =
        std::async(std::launch::async, &DlioCore::buildKeyframesAndSubmap, this, state_);
#ifdef DLIO_DETERMINISTIC
      // Wait for the rebuild to finish before the next scan, so the choice of
      // registration target no longer depends on thread timing.
      submap_future_.wait();
#endif
    } else {
      main_loop_running_ = false;
      lock.unlock();
      submap_build_cv_.notify_one();
    }
  }

  prev_scan_stamp_ = scan_stamp_;
  gicp_has_converged_ = gicp_.hasConverged();
  geo_.first_opt_done = true;

  ready_outputs_.push_back(makeOutput(current_scan_));
  return ScanOutcome::kReady;
}

DlioCore::ScanOutcome DlioCore::preprocessAndDeskew(
  const PendingScan & scan, PointCloudXYZI::Ptr & out_deskewed, PointCloudXYZI::Ptr & out_current)
{
  PointCloudXYZI::Ptr deskewed;

  if (cfg_.pointcloud_deskew) {
    deskewed = PointCloudXYZI::Ptr(new PointCloudXYZI());
    deskewed->points.resize(original_scan_->points.size());

    std::vector<const PointType *> sorted;
    sorted.reserve(original_scan_->points.size());
    for (const auto & p : original_scan_->points) {
      sorted.push_back(&p);
    }
    std::sort(sorted.begin(), sorted.end(), [](const PointType * a, const PointType * b) {
      return a->timestamp < b->timestamp;
    });
    for (std::size_t i = 0; i < sorted.size(); i++) {
      deskewed->points[i] = *sorted[i];
    }

    std::vector<double> timestamps;
    std::vector<int> unique_time_indices;
    for (std::size_t i = 0; i < deskewed->points.size(); i++) {
      if (i == 0 || deskewed->points[i].timestamp != deskewed->points[i - 1].timestamp) {
        timestamps.push_back(deskewed->points[i].timestamp);
        unique_time_indices.push_back(static_cast<int>(i));
      }
    }
    unique_time_indices.push_back(static_cast<int>(deskewed->points.size()));

    if (timestamps.empty()) {
      // Degenerate scan (no valid per-point timestamps at all): this is
      // intrinsic to the scan's own data, not a timing issue, so retrying
      // later can never help -- drop it.
      return ScanOutcome::kDropped;
    }

    const std::size_t median_pt_index = timestamps.size() / 2;
    scan_stamp_ = timestamps[median_pt_index];

    if (!first_valid_scan_) {
      if (imu_buffer_.empty()) {
        return ScanOutcome::kDeferred;  // IMU calibration not done yet: retry once it is
      }
      if (scan_stamp_ <= imu_buffer_.back().stamp) {
        // This scan predates the oldest sample still held in imu_buffer_.
        // That bound only grows forward over time (samples are only ever
        // evicted from the back as newer ones arrive), so this scan can
        // *never* pass this check on a later retry -- drop it rather than
        // block every later scan behind an unresolvable one. Matches
        // upstream's real-world behavior: scans that arrive before the
        // startup calibration window completes are simply never revisited
        // by a later ROS callback either. See the class doc comment.
        return ScanOutcome::kDropped;
      }
      first_valid_scan_ = true;
      T_prior_ = T_;  // assume no motion for the first scan
      pcl::transformPointCloud(*deskewed, *deskewed, T_prior_ * baselink2lidar_T_);
    } else {
      auto frames =
        integrateImu(prev_scan_stamp_, lidar_pose_.q, lidar_pose_.p, geo_.prev_vel, timestamps);
      deskew_size_ = static_cast<int>(frames.size());

      if (frames.size() != timestamps.size()) {
        // IMU coverage not ready for this scan's full span yet: defer and
        // retry (deviation from upstream's blocking wait; see the class doc
        // comment in DlioCore.h).
        return ScanOutcome::kDeferred;
      }

      T_prior_ = frames[median_pt_index];

#pragma omp parallel for num_threads(num_threads_)
      for (std::size_t i = 0; i < timestamps.size(); i++) {
        const Eigen::Matrix4f T = frames[i] * baselink2lidar_T_;
        for (int k = unique_time_indices[i]; k < unique_time_indices[i + 1]; k++) {
          auto & pt = deskewed->points[k];
          Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.f);
          v = T * v;
          pt.x = v.x();
          pt.y = v.y();
          pt.z = v.z();
        }
      }
    }
  } else {
    scan_stamp_ = scan.t_end;

    if (!first_valid_scan_) {
      if (imu_buffer_.empty()) {
        return ScanOutcome::kDeferred;
      }
      if (scan_stamp_ <= imu_buffer_.back().stamp) {
        // See the identical check/comment in the deskew branch above.
        return ScanOutcome::kDropped;
      }
      first_valid_scan_ = true;
      T_prior_ = T_;
    } else {
      auto frames =
        integrateImu(prev_scan_stamp_, lidar_pose_.q, lidar_pose_.p, geo_.prev_vel, {scan_stamp_});
      T_prior_ = frames.empty() ? T_ : frames.back();
    }

    deskewed = PointCloudXYZI::Ptr(new PointCloudXYZI());
    pcl::transformPointCloud(*original_scan_, *deskewed, T_prior_ * baselink2lidar_T_);
  }

  out_deskewed = deskewed;

  if (cfg_.pointcloud_voxelize) {
    PointCloudXYZI::Ptr current(new PointCloudXYZI(*deskewed));
    voxel_.setInputCloud(current);
    voxel_.filter(*current);
    out_current = current;
  } else {
    out_current = deskewed;
  }

  return ScanOutcome::kReady;
}

void DlioCore::getNextPose()
{
  new_submap_is_ready_ =
    (submap_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready);

  if (new_submap_is_ready_ && submap_has_changed_) {
    gicp_.registerInputTarget(submap_cloud_);
    gicp_.target_kdtree_ = submap_kdtree_;
    gicp_.setTargetCovariances(submap_normals_);
    submap_has_changed_ = false;
  }

  // Both source and target are already in the (IMU-prior-estimated) world
  // frame -- see the class doc comment -- so no explicit initial guess is
  // needed; align() defaults to identity, and T_corr_ below is a small
  // refinement, not a full pose.
  PointCloudXYZI::Ptr aligned(new PointCloudXYZI());
  gicp_.align(*aligned);

  T_corr_ = gicp_.getFinalTransformation();
  T_ = T_corr_ * T_prior_;

  propagateGICP();
  updateState();
}

void DlioCore::computeMetrics()
{
  computeSpaciousness(*original_scan_);
  computeDensity();
  metrics_initialized_ = true;
}

void DlioCore::computeSpaciousness(const PointCloudXYZI & original_scan)
{
  std::vector<float> ds;
  ds.reserve(original_scan.points.size());
  for (const auto & p : original_scan.points) {
    ds.push_back(std::sqrt(p.x * p.x + p.y * p.y));
  }

  float median_curr = spaciousness_prev_;
  if (!ds.empty()) {
    std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
    median_curr = ds[ds.size() / 2];
  }

  const float prev = metrics_initialized_ ? spaciousness_prev_ : median_curr;
  const float median_lpf = 0.95f * prev + 0.05f * median_curr;
  spaciousness_prev_ = median_lpf;
  spaciousness_ = median_lpf;
}

void DlioCore::computeDensity()
{
  const float density = geo_.first_opt_done ? gicp_.source_density_ : 0.f;

  const float prev = metrics_initialized_ ? density_prev_ : density;
  const float density_lpf = 0.95f * prev + 0.05f * density;
  density_prev_ = density_lpf;
  density_ = density_lpf;
}

void DlioCore::setAdaptiveParams()
{
  float sp = spaciousness_;
  sp = std::max(0.5f, std::min(5.0f, sp));
  keyframe_thresh_dist_active_ = sp;

  // Faithful port of upstream's `setAdaptiveParams()`: the density-derived
  // clamp below is fully overwritten by the spaciousness-derived branch
  // right after (unless sp is exactly 5.0), which looks redundant but is
  // upstream's real, unmodified behavior -- kept as-is per the "byte
  // identical beyond mechanical de-ROSing" rule.
  float den = density_;
  const float lo = static_cast<float>(0.5 * cfg_.gicp_max_corr_dist);
  const float hi = static_cast<float>(2.0 * cfg_.gicp_max_corr_dist);
  den = std::max(lo, std::min(hi, den));

  if (sp < 5.0f) {
    den = lo;
  }
  if (sp > 5.0f) {
    den = hi;
  }

  gicp_.setMaxCorrespondenceDistance(den);
  concave_hull_.setAlpha(keyframe_thresh_dist_active_);
}

void DlioCore::pushSubmapIndices(
  const std::vector<float> & dists, int k, const std::vector<int> & frames)
{
  if (dists.empty()) {
    return;
  }

  std::priority_queue<float> pq;
  for (auto d : dists) {
    if (static_cast<int>(pq.size()) >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (static_cast<int>(pq.size()) < k) {
      pq.push(d);
    }
  }

  const float kth_element = pq.top();
  for (std::size_t i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element) {
      submap_kf_idx_curr_.push_back(frames[i]);
    }
  }
}

void DlioCore::computeConvexHull()
{
  if (num_processed_keyframes_ < 4) {
    return;
  }

  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    for (int i = 0; i < num_processed_keyframes_; i++) {
      PointType pt;
      pt.x = keyframes_[i].first.first[0];
      pt.y = keyframes_[i].first.first[1];
      pt.z = keyframes_[i].first.first[2];
      cloud->push_back(pt);
    }
  }

  convex_hull_.setInputCloud(cloud);
  PointCloudXYZI::Ptr convex_points(new PointCloudXYZI());
  convex_hull_.reconstruct(*convex_points);

  pcl::PointIndices::Ptr idx(new pcl::PointIndices());
  convex_hull_.getHullPointIndices(*idx);

  keyframe_convex_.clear();
  for (const int i : idx->indices) {
    keyframe_convex_.push_back(i);
  }
}

void DlioCore::computeConcaveHull()
{
  if (num_processed_keyframes_ < 5) {
    return;
  }

  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    for (int i = 0; i < num_processed_keyframes_; i++) {
      PointType pt;
      pt.x = keyframes_[i].first.first[0];
      pt.y = keyframes_[i].first.first[1];
      pt.z = keyframes_[i].first.first[2];
      cloud->push_back(pt);
    }
  }

  concave_hull_.setInputCloud(cloud);
  PointCloudXYZI::Ptr concave_points(new PointCloudXYZI());
  concave_hull_.reconstruct(*concave_points);

  pcl::PointIndices::Ptr idx(new pcl::PointIndices());
  concave_hull_.getHullPointIndices(*idx);

  keyframe_concave_.clear();
  for (const int i : idx->indices) {
    keyframe_concave_.push_back(i);
  }
}

void DlioCore::updateKeyframes(const PointCloudXYZI::ConstPtr & current_scan)
{
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = 0;
  int keyframes_idx = 0;
  int num_nearby = 0;

  for (const auto & k : keyframes_) {
    const float delta_d = std::sqrt(
      std::pow(state_.p[0] - k.first.first[0], 2) + std::pow(state_.p[1] - k.first.first[1], 2) +
      std::pow(state_.p[2] - k.first.first[2], 2));

    if (delta_d <= keyframe_thresh_dist_active_ * 1.5) {
      ++num_nearby;
    }
    if (delta_d < closest_d) {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }
    keyframes_idx++;
  }

  const Eigen::Vector3f closest_pose = keyframes_[closest_idx].first.first;
  const Eigen::Quaternionf closest_pose_r = keyframes_[closest_idx].first.second;

  const float dd = std::sqrt(
    std::pow(state_.p[0] - closest_pose[0], 2) + std::pow(state_.p[1] - closest_pose[1], 2) +
    std::pow(state_.p[2] - closest_pose[2], 2));

  Eigen::Quaternionf dq;
  if (state_.q.dot(closest_pose_r) < 0.) {
    Eigen::Quaternionf lq = closest_pose_r;
    lq.w() *= -1.;
    lq.x() *= -1.;
    lq.y() *= -1.;
    lq.z() *= -1.;
    dq = state_.q * lq.inverse();
  } else {
    dq = state_.q * closest_pose_r.inverse();
  }

  const double theta_rad =
    2. *
    std::atan2(std::sqrt(std::pow(dq.x(), 2) + std::pow(dq.y(), 2) + std::pow(dq.z(), 2)), dq.w());
  const double theta_deg = theta_rad * (180.0 / M_PI);

  bool new_keyframe = false;
  if (
    std::abs(dd) > keyframe_thresh_dist_active_ || std::abs(theta_deg) > cfg_.keyframe_thresh_rot) {
    new_keyframe = true;
  }
  if (std::abs(dd) <= keyframe_thresh_dist_active_) {
    new_keyframe = false;
  }
  if (
    std::abs(dd) <= keyframe_thresh_dist_active_ &&
    std::abs(theta_deg) > cfg_.keyframe_thresh_rot && num_nearby <= 1) {
    new_keyframe = true;
  }

  if (new_keyframe) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframes_.push_back({{lidar_pose_.p, lidar_pose_.q}, current_scan});
    keyframe_timestamps_.push_back(scan_stamp_);
    keyframe_normals_.push_back(gicp_.getSourceCovariances());
    keyframe_transformations_.push_back(T_corr_);
  }
}

void DlioCore::buildSubmap(const State & vehicle_state)
{
  submap_kf_idx_curr_.clear();

  std::vector<float> ds;
  std::vector<int> keyframe_nn;
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    for (int i = 0; i < num_processed_keyframes_; i++) {
      const float d = std::sqrt(
        std::pow(vehicle_state.p[0] - keyframes_[i].first.first[0], 2) +
        std::pow(vehicle_state.p[1] - keyframes_[i].first.first[1], 2) +
        std::pow(vehicle_state.p[2] - keyframes_[i].first.first[2], 2));
      ds.push_back(d);
      keyframe_nn.push_back(i);
    }
  }

  pushSubmapIndices(ds, cfg_.submap_knn, keyframe_nn);

  computeConvexHull();
  std::vector<float> convex_ds;
  for (const auto & c : keyframe_convex_) {
    convex_ds.push_back(ds[c]);
  }
  pushSubmapIndices(convex_ds, cfg_.submap_kcv, keyframe_convex_);

  computeConcaveHull();
  std::vector<float> concave_ds;
  for (const auto & c : keyframe_concave_) {
    concave_ds.push_back(ds[c]);
  }
  pushSubmapIndices(concave_ds, cfg_.submap_kcc, keyframe_concave_);

  std::sort(submap_kf_idx_curr_.begin(), submap_kf_idx_curr_.end());
  std::sort(submap_kf_idx_prev_.begin(), submap_kf_idx_prev_.end());

  auto last = std::unique(submap_kf_idx_curr_.begin(), submap_kf_idx_curr_.end());
  submap_kf_idx_curr_.erase(last, submap_kf_idx_curr_.end());

  if (submap_kf_idx_curr_ != submap_kf_idx_prev_) {
    submap_has_changed_ = true;

    pauseSubmapBuildIfNeeded();

    PointCloudXYZI::Ptr submap_cloud(new PointCloudXYZI());
    std::shared_ptr<nano_gicp::CovarianceList> submap_normals(new nano_gicp::CovarianceList());

    for (const int k : submap_kf_idx_curr_) {
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        *submap_cloud += *keyframes_[k].second;
      }
      submap_normals->insert(
        submap_normals->end(), keyframe_normals_[k]->begin(), keyframe_normals_[k]->end());
    }

    submap_cloud_ = submap_cloud;
    submap_normals_ = submap_normals;

    pauseSubmapBuildIfNeeded();

    gicp_temp_.setInputTarget(submap_cloud_);
    submap_kdtree_ = gicp_temp_.target_kdtree_;

    submap_kf_idx_prev_ = submap_kf_idx_curr_;
  }
}

void DlioCore::buildKeyframesAndSubmap(State vehicle_state)
{
  {
    std::unique_lock<std::mutex> lock(keyframes_mutex_);
    for (int i = num_processed_keyframes_; i < static_cast<int>(keyframes_.size()); i++) {
      KeyframeCloudPtr raw_keyframe = keyframes_[i].second;
      std::shared_ptr<const nano_gicp::CovarianceList> raw_covariances = keyframe_normals_[i];
      Eigen::Matrix4f T = keyframe_transformations_[i];
      lock.unlock();

      const Eigen::Matrix4d Td = T.cast<double>();

      PointCloudXYZI::Ptr transformed_keyframe(new PointCloudXYZI());
      pcl::transformPointCloud(*raw_keyframe, *transformed_keyframe, T);

      std::shared_ptr<nano_gicp::CovarianceList> transformed_covariances(
        new nano_gicp::CovarianceList(raw_covariances->size()));
      std::transform(
        raw_covariances->begin(), raw_covariances->end(), transformed_covariances->begin(),
        [&Td](const Eigen::Matrix4d & cov) { return Td * cov * Td.transpose(); });

      ++num_processed_keyframes_;

      lock.lock();
      keyframes_[i].second = transformed_keyframe;
      keyframe_normals_[i] = transformed_covariances;
    }
  }

  pauseSubmapBuildIfNeeded();

  buildSubmap(vehicle_state);
}

void DlioCore::pauseSubmapBuildIfNeeded()
{
  std::unique_lock<std::mutex> lock(main_loop_running_mutex_);
  submap_build_cv_.wait(lock, [this] { return !main_loop_running_; });
}

// ---------------------------------------------------------------------------
//  Output
// ---------------------------------------------------------------------------

OdometryOutput DlioCore::makeOutput(const PointCloudXYZI::ConstPtr & current_scan_body_world)
{
  OdometryOutput out;
  out.timestamp = scan_stamp_;

  const Eigen::Matrix4f world_from_baselink = T_;
  const Eigen::Matrix4f world_from_imu = world_from_baselink * baselink2imu_T_;
  out.pose_imu.matrix() = world_from_imu.cast<double>();

  out.vel = state_.v.lin.w.cast<double>();
  out.bias_gyro = state_.b.gyro.cast<double>();
  out.bias_acc = state_.b.accel.cast<double>();

  // Reconstructed per-scan gravity (no explicit filter state to read, unlike
  // an IESKF): current orientation's inverse rotation applied to the
  // world-frame gravity vector. See Types.hpp's OdometryOutput doc.
  const Eigen::Vector3f world_gravity(0.f, 0.f, static_cast<float>(-cfg_.gravity));
  out.gravity = (state_.q.conjugate() * world_gravity).cast<double>();

  if (dumpEstimatedGravEnabled()) {
    auto & f = gravDumpFile();
    f << out.timestamp << " " << out.gravity.x() << " " << out.gravity.y() << " " << out.gravity.z()
      << "\n";
    f.flush();
  }

  // current_scan_body_world is stored in world frame (see the getNextPose()
  // comment on why GICP's source cloud is pre-transformed there); undo the
  // world_from_lidar transform to report it in LiDAR body frame, matching
  // every other wrapper's OdometryOutput::deskewed_points_body convention.
  auto body = std::make_shared<XYZIPointVector>();
  const Eigen::Matrix4f world_from_lidar = T_ * baselink2lidar_T_;
  const Eigen::Matrix4f lidar_from_world = world_from_lidar.inverse();
  body->reserve(current_scan_body_world->points.size());
  for (const auto & p : current_scan_body_world->points) {
    const Eigen::Vector4f pw(p.x, p.y, p.z, 1.f);
    const Eigen::Vector4f pb = lidar_from_world * pw;
    XYZIPoint q;
    q.x = pb.x();
    q.y = pb.y();
    q.z = pb.z();
    q.intensity = p.intensity;
    body->push_back(q);
  }
  out.deskewed_points_body = body;

  out.analytics.num_keyframes = keyframes_.size();
  out.analytics.submap_size = submap_cloud_ ? submap_cloud_->size() : std::size_t{0};
  out.analytics.num_gicp_correspondences =
    static_cast<std::size_t>(std::max(0, gicp_.num_correspondences));
  out.analytics.gicp_final_error = gicp_.getFinalError();
  out.analytics.gicp_converged = gicp_has_converged_;
  out.analytics.spaciousness = spaciousness_;
  out.analytics.density = density_;
  out.analytics.keyframe_thresh_dist_active = keyframe_thresh_dist_active_;
  out.analytics.bias_gyro = out.bias_gyro;
  out.analytics.bias_accel = out.bias_acc;

  return out;
}

std::shared_ptr<const XYZIPointVector> DlioCore::currentSubmap() const
{
  auto out = std::make_shared<XYZIPointVector>();
  if (!submap_cloud_) {
    return out;
  }
  out->reserve(submap_cloud_->points.size());
  for (const auto & p : submap_cloud_->points) {
    XYZIPoint q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.intensity;
    out->push_back(q);
  }
  return out;
}

std::size_t DlioCore::numKeyframes() const { return keyframes_.size(); }

}  // namespace dlio_core
