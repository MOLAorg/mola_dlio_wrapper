# Third-party notices

This repository is a [MOLA](https://github.com/MOLAorg/mola) wrapper that integrates
**DLIO** (Direct LiDAR-Inertial Odometry), developed by the VECTR Lab, UCLA. The code
under `third_party/` is derived from, or vendored verbatim from, the following upstream
projects. None of it is original work of this repository's author except where
explicitly noted.

## DLIO

- Upstream: https://github.com/vectr-ucla/direct_lidar_inertial_odometry, `master`
  branch, commit **`fc8d183f18cdcfb9bb4fc754c6d373cedc4cbd04`** (2024-11-08).
- License: **MIT** (confirmed from the repository's `LICENSE` file).
- Copyright: Kenny J. Chen, Ryan Nemiroff, and Brett T. Lopez (The Verifiable &
  Control-Theoretic Robotics (VECTR) Lab, University of California, Los Angeles).
- K. Chen, R. Nemiroff, B. T. Lopez, "Direct LiDAR-Inertial Odometry: Lightweight LIO
  with Continuous-Time Motion Correction," ICRA 2023.
- Files derived from it in this repository: `third_party/dlio_core/include/dlio_core/DlioCore.h`
  and `.../src/DlioCore.cpp` (ported from `include/dlio/odom.h` and `src/dlio/odom.cc`:
  every ROS type/call removed and replaced with a push-based
  `feedImu()`/`addLidarScan()`/`drainReady()` API), and
  `third_party/dlio_core/include/dlio/dlio.h` (ported from `include/dlio/dlio.h`: the
  `dlio::Point` PCL point type is kept, `SensorType` and the ROS/proc-stat includes are
  removed -- see the deviation list below).
- The map-accumulation node (`src/dlio/map.cc`, `map_node.cc`, `include/dlio/map.h`) and
  the `srv/save_pcd.srv` service were **dropped**, per the common plan: MOLA's own
  `MapSourceBase`/GUI map view covers that role.

## nano_gicp

- Bundled inside the DLIO repository at `include/nano_gicp/`, `src/nano_gicp/`.
- License: **BSD 3-Clause** (per the license header reproduced verbatim at the top of
  every `nano_gicp/*.h`/`*.cc` file in this repository).
- Copyright (c) 2020, SMRT-AIST (exactly as the header states; the file headers do not
  name an individual author). nano_gicp is DLIO's own fork/adaptation of
  [`fast_gicp`](https://github.com/SMRT-AIST/fast_gicp) (also SMRT-AIST, BSD-3), adapted
  by the DLIO authors (VECTR Lab, UCLA) -- both the SMRT-AIST and VECTR Lab copyright
  headers are present verbatim at the top of each vendored file.
- Vendored **verbatim** (byte-for-byte, no edits) at
  `third_party/dlio_core/include/nano_gicp/` and `third_party/dlio_core/src/nano_gicp/`:
  `nano_gicp.h`/`.cc`, `lsq_registration.h`/`.cc`, `nanoflann_adaptor.h`. These files
  `#include "dlio/dlio.h"` only to name the `PointType` template parameter for their
  explicit template instantiations (`template class nano_gicp::NanoGICP<PointType,
  PointType>;`); this repository's ROS-free `dlio/dlio.h` (see above) satisfies that
  include unmodified, so no edits to `nano_gicp` were needed at all.

## nanoflann (bundled inside nano_gicp)

- File: `third_party/dlio_core/include/nano_gicp/nanoflann.h` (and its small
  `src/nano_gicp/nanoflann.cc` explicit-instantiation file), vendored verbatim as part
  of the `nano_gicp` copy above -- **do not** replace with the system nanoflann package,
  since DLIO's `nano_gicp` API depends on this specific bundled version/interface.
- License: **BSD 2-Clause** (per the license header at the top of the file: two
  redistribution conditions, no "endorse or promote" clause).
- Copyright 2008-2009 Marius Muja, David G. Lowe; Copyright 2011-2022 Jose Luis Blanco.

## Deviations from upstream beyond mechanical de-ROS-ing

Per the common plan's "keep the algorithm logic byte-identical" rule, every behavioral
change beyond stripping ROS types/calls is listed here (see also
`~/plans/lio-wrappers-dlio.md`'s work log for the reasoning behind each):

1. **Blocking IMU wait replaced with a non-blocking retry queue.** Upstream's
   `imuMeasFromTimeRange()` blocks on a condition variable until enough IMU data has
   arrived, which relies on the IMU and LiDAR ROS callbacks running on independent
   threads pumped by `ros::spin()`. This port's offline CLI and online module drive
   `feedImu()`/`addLidarScan()` from a caller-controlled thread pair where blocking would
   risk a deadlock (the thread that would need to deliver the awaited IMU sample is the
   same one blocked waiting for `isBusy()==false` in the offline CLI's no-drop loop). The
   check now returns immediately; the caller (`DlioCore::tryProcessPending()`) defers the
   scan and retries it on the next `feedImu()`/`addLidarScan()` call, mirroring
   `fast_lio2_core`'s own `addLidarScan()`/`drainReady()` contract in this repository's
   sibling package. The estimation math itself (`integrateImu`, `integrateImuInternal`,
   the geometric observer) is unchanged.
2. **Three function-local `static` variables converted to instance members.** Upstream
   used a C++ function-local `static` to carry state across calls in three places: the
   low-pass filter state in `computeSpaciousness()`/`computeDensity()`, the running-average
   accumulators in the IMU calibration window (`callbackImu()`), and the previous
   sample/angular-velocity in `transformImu()`. A function-local `static` is process-wide,
   not per-instance, so two `DlioCore` objects constructed in the same process (e.g. this
   repository's own `test_dlio_core` gtest) would silently share and corrupt each other's
   state. All three now live as ordinary `DlioCore` member variables; behavior for a
   single instance (the only configuration upstream ever ran) is unchanged.
3. **Per-sensor point-timestamp format detection dropped.** Upstream's `dlio::Point`
   carries a three-way union (`t`: Ouster, `uint32` ns; `time`: Velodyne, `float` s;
   `timestamp`: Hesai/Livox, `double` s) with runtime `SensorType` detection driving
   `deskewPointcloud()`'s per-vendor timestamp arithmetic. This wrapper's MRPT adapter
   (`DlioOdometry_Adapters.cpp`, identical in spirit to `fast_lio2_core`'s own adapter)
   already normalizes every incoming point's time to a single convention -- an absolute
   per-point time on the observation's own clock -- before it reaches `dlio_core`,
   regardless of the upstream sensor family. `dlio::Point` was simplified to a single
   `double timestamp` field and every scan now takes the code path upstream used for its
   HESAI sensor type (the only one of the four that already used a plain absolute
   per-point timestamp with no additional per-scan offset arithmetic). See the file
   comment in `include/dlio/dlio.h` and `dlio_core/Types.hpp`'s `LidarPoint` doc.
4. **`odom.gravity`/`compute_time_offset` interaction.** Upstream's `computeTimeOffset`
   corrects for LiDAR/host clock disagreement by re-basing a scan's per-point absolute
   time onto the ROS message header stamp. Because of deviation 3 above, every scan's
   points are already re-based onto the observation's own timestamp by the MRPT adapter,
   so `Config::compute_time_offset` is a no-op in this port -- kept only for YAML schema
   parity with upstream's `cfg/params.yaml`. Documented in `Types.hpp`.
5. **DLIO does not carry gravity as an explicit filter state.** Unlike an IESKF's
   `state.grav`, DLIO's geometric observer only ever uses gravity implicitly (subtracting
   it from world-frame `z` acceleration in `propagateState()`). `MOLA_DLIO_DUMP_ESTIMATED_GRAV`
   therefore dumps two different things instead of one filter-state read: the one-off
   calibration-window estimate, and a per-scan *reconstruction* (`R^T * [0,0,-g]`) rather
   than a state read. Not a change to the estimation math; documented in
   `dlio_core/DlioCore.h`, `Types.hpp`, and `README.md` since it is exactly the kind of
   cross-method difference this benchmark suite exists to surface.

No other algorithmic step (GICP setup, deskewing math, the geometric observer's
propagation/update equations, keyframe selection, submap construction via convex/concave
hull + kNN, adaptive parameter scaling) was changed beyond the ROS-type/API-shape
substitution described in DLIO's own section above.

## Licensing note: MIT + BSD + GPLv3

DLIO (MIT), `nano_gicp` (BSD-3), and `nanoflann` (BSD-2) are all permissive licenses,
fully compatible with this repository's own GPL-3.0-only code and with MOLA's other
GPL-3.0 dependencies -- unlike `mola_fast_lio2_wrapper` (GPLv2-only Fast-LIO2/ikd-Tree)
and `mola_ig_lio_wrapper` (GPLv2-only iG-LIO), **this package has no GPLv2/GPLv3
incompatibility to resolve** before any future public release. The MIT/BSD notices above
must still be reproduced in any redistribution, per their terms.
