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
 * @file   DlioCoreBridge.h
 * @brief  Thin, PCL/nanoflann-free wrapper around dlio_core::DlioCore.
 *
 * Why this exists: `dlio_core/DlioCore.h` transitively includes the
 * vendored `nano_gicp/nanoflann.h` (an old nanoflann version, pinned
 * because nano_gicp's API depends on it -- see THIRD_PARTY_NOTICES.md).
 * `DlioOdometry.cpp` also needs `mrpt/maps/CPointsMap.h` (via
 * DlioOdometry_Adapters.h), which pulls in a *different*, newer system
 * `nanoflann.hpp` through `mrpt::math::KDTreeCapable`. Both define the same
 * `nanoflann::` symbols with no shared include guard, so including both in
 * one translation unit is a hard redefinition error -- an unavoidable
 * consequence of vendoring nano_gicp's own pinned nanoflann rather than the
 * system one (which the plan explicitly requires, since swapping it would
 * change the method).
 *
 * The fix: isolate the two in separate translation units. This header (and
 * `DlioOdometry.h`, which forward-declares `DlioCoreBridge` only) never
 * includes `dlio_core/DlioCore.h`; only `DlioCoreBridge.cpp` does, and it
 * never includes any mrpt header. `dlio_core/Types.hpp` -- plain structs,
 * no PCL/nanoflann -- is safe to include from anywhere and is used for
 * every parameter/return type here.
 */
#pragma once

#include <cstddef>
#include <dlio_core/Types.hpp>
#include <memory>
#include <vector>

namespace dlio_core
{
class DlioCore;
}

namespace mola
{
class DlioCoreBridge
{
public:
  explicit DlioCoreBridge(const dlio_core::Config & cfg);
  ~DlioCoreBridge();

  void feedImu(const dlio_core::ImuSample & s);
  void addLidarScan(dlio_core::LidarPointVector points, double t_beg, double t_end);
  std::vector<dlio_core::OdometryOutput> drainReady();

  std::shared_ptr<const dlio_core::XYZIPointVector> currentSubmap() const;
  std::size_t numKeyframes() const;

private:
  std::unique_ptr<dlio_core::DlioCore> core_;
};

}  // namespace mola
