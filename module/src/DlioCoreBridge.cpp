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
 * @file   DlioCoreBridge.cpp
 * @brief  See DlioCoreBridge.h for why this translation unit exists and
 *         why it must never include any mrpt header.
 */
#include "DlioCoreBridge.h"

#include <dlio_core/DlioCore.h>

namespace mola
{

DlioCoreBridge::DlioCoreBridge(const dlio_core::Config & cfg)
: core_(std::make_unique<dlio_core::DlioCore>(cfg))
{
}

DlioCoreBridge::~DlioCoreBridge() = default;

void DlioCoreBridge::feedImu(const dlio_core::ImuSample & s) { core_->feedImu(s); }

void DlioCoreBridge::addLidarScan(dlio_core::LidarPointVector points, double t_beg, double t_end)
{
  core_->addLidarScan(std::move(points), t_beg, t_end);
}

std::vector<dlio_core::OdometryOutput> DlioCoreBridge::drainReady() { return core_->drainReady(); }

std::shared_ptr<const dlio_core::XYZIPointVector> DlioCoreBridge::currentSubmap() const
{
  return core_->currentSubmap();
}

std::size_t DlioCoreBridge::numKeyframes() const { return core_->numKeyframes(); }

}  // namespace mola
