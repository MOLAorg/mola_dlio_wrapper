/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/
//
// De-ROS'd port for mola_dlio_wrapper. See THIRD_PARTY_NOTICES.md at the
// repository root. Original file: include/dlio/dlio.h. Everything ROS
// (headers, message types, cpuid/proc-stat helpers used only by the terminal
// debug printout) was removed; the `dlio::Point` PCL point type is kept.
//
// One deliberate behavioral simplification vs. upstream: `dlio::Point` no
// longer carries the Ouster (uint32 ns)/Velodyne (float sec)/Hesai+Livox
// (double abs.) three-way timestamp union with runtime `SensorType`
// detection, because mola_dlio_wrapper's MRPT adapter (see
// DlioOdometry_Adapters.cpp) already normalizes every incoming point's time
// to a single convention -- an absolute per-point time, same clock as
// `t_beg`/`t_end` -- before it reaches this library, regardless of the
// upstream sensor family. This makes every scan take the code path upstream
// used for its HESAI sensor type (the only one of the four that already used
// a plain absolute per-point timestamp with no per-scan offset arithmetic).
// See dlio_core/Types.hpp and DlioCore.h for where this matters.
#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Upstream's dlio.h pulled this in for its own use; kept here (unlike the
// rest of upstream's ROS/proc-stat includes) because the verbatim-vendored
// nano_gicp/src/lsq_registration.cc's debug-print path (off by default,
// `setDebugPrint(true)`) uses `boost::format` and only includes this header
// transitively via `#include "dlio/dlio.h"`.
#include <boost/format.hpp>

namespace dlio
{
struct Point
{
  Point() : data{0.f, 0.f, 0.f, 1.f} {}

  PCL_ADD_POINT4D;
  float intensity = 0;   // intensity
  double timestamp = 0;  // absolute per-point time [s], see the file comment above
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace dlio

POINT_CLOUD_REGISTER_POINT_STRUCT(
  dlio::Point,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))

// Matches upstream's un-namespaced `typedef dlio::Point PointType;`: kept at
// global scope (not `dlio_core::`) so the verbatim-vendored nano_gicp/*.cc
// translation units, which `#include "dlio/dlio.h"` and explicitly
// instantiate `nano_gicp::NanoGICP<PointType, PointType>` at global scope,
// compile completely unmodified.
using PointType = dlio::Point;
