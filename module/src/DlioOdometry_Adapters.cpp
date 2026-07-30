/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   DlioOdometry_Adapters.cpp
 * @brief  Conversions between MRPT/MOLA and dlio_core plain types
 */
#include "DlioOdometry_Adapters.h"

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <algorithm>
#include <cmath>

namespace mola
{
using dlio_core::LidarPoint;
using dlio_core::LidarPointVector;
using dlio_core::XYZIPoint;
using dlio_core::XYZIPointVector;

/** Converts an incoming LiDAR observation into DLIO points, and computes the
 * scan's absolute [t_beg, t_end] time span. Identical strategy to
 * `mola_fast_lio2_wrapper`'s `toFastLio2Cloud()` (see its doc comment for
 * the full rationale): re-normalizes any per-point timestamp channel
 * relative to this scan's own min/max, falling back to an azimuth-based
 * synthetic offset when there is none.
 *
 * `dlio_core::LidarPoint::t_offset_sec` is always relative to `t_beg` (see
 * dlio_core/Types.hpp and include/dlio/dlio.h), so this is the single place
 * in this wrapper responsible for the timestamp convention DlioCore relies
 * on.
 */
LidarPointVector toDlioCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind)
{
  LidarPointVector out;
  const double blind2 = blind * blind;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    t_beg = t_end = mrpt::Clock::toDouble(obs.timestamp);
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();

  const auto * is =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  out.resize(n);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  if (ts && ts->size() == n && n > 0) {
    float tMin = (*ts)[0], tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }
    t_beg = obsTime + tMin;
    t_end = obsTime + tMax;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i] - tMin);
    }
    out.resize(k);
  } else {
    // Fallback: assume `obsTime` is the scan's end (forward-facing point),
    // and synthesize each point's offset from its azimuth angle.
    t_beg = obsTime - fallback_scan_period;
    t_end = obsTime;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;

      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t_offset_sec = azimuth01 * fallback_scan_period;
    }
    out.resize(k);
  }

  return out;
}

dlio_core::ImuSample toDlioImu(const mrpt::obs::CObservationIMU & obs)
{
  dlio_core::ImuSample s;
  s.t = mrpt::Clock::toDouble(obs.timestamp);
  s.acc = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_X_ACC), obs.get(mrpt::obs::IMU_Y_ACC), obs.get(mrpt::obs::IMU_Z_ACC));
  s.gyro = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_WX), obs.get(mrpt::obs::IMU_WY), obs.get(mrpt::obs::IMU_WZ));
  return s;
}

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const XYZIPointVector & points)
{
  auto out = mrpt::maps::CSimplePointsMap::Create();
  out->reserve(points.size());
  for (const auto & p : points) {
    out->insertPointFast(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace mola
