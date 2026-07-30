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
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 */

/** \defgroup mola_dlio_wrapper_grp mola_dlio_wrapper
 * MOLA module: DLIO (Direct LiDAR-Inertial Odometry) wrapper.
 */

#include <mola_dlio_wrapper/DlioOdometry.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

MRPT_INITIALIZER(do_register_mola_dlio_wrapper)  // NOLINT
{
  using mrpt::rtti::registerClass;

  MOLA_REGISTER_MODULE(mola::DlioOdometry);
}
