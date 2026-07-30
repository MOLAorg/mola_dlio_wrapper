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
 * @file   DlioOdometry_Adapters.h
 * @brief  Conversions between MRPT/MOLA and dlio_core plain types (private
 *         to this module, not installed).
 */
#pragma once

#include <dlio_core/Types.hpp>

// dlio_core/Types.hpp does not pull in PCL, unlike fast_lio2_core's; DLIO's
// own PCL usage is fully contained inside dlio_core's own translation
// units. mrpt/maps/CPointsMap.h is included plainly here.
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>

namespace mola
{
/** See the .cpp file for the full documentation of the conversion logic. */
dlio_core::LidarPointVector toDlioCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind = 0.0);

dlio_core::ImuSample toDlioImu(const mrpt::obs::CObservationIMU & obs);

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const dlio_core::XYZIPointVector & points);

}  // namespace mola
