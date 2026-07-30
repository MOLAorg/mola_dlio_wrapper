# mola_dlio_wrapper

A [MOLA](https://github.com/MOLAorg/mola) wrapper around
[DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) (Direct
LiDAR-Inertial Odometry), in the same spirit as
[`mola_fast_lio2_wrapper`](https://github.com/MOLAorg/mola_fast_lio2_wrapper)
and [`mola_ellipse_lio_wrapper`](https://github.com/MOLAorg/mola_ellipse_lio_wrapper),
so DLIO can be run on MOLA-supported datasets for comparison against other
LiDAR-inertial odometry methods (see
[`mola-lo`](https://github.com/MOLAorg/mola_lidar_odometry)).

**Status: private, work in progress, internal research use only.**

## What makes DLIO different

Unlike Fast-LIO2 and EllipseLIO (both IESKF-based), DLIO has no filter and no
factor graph: pose comes from GICP scan-to-submap registration, and the IMU
is fused by a **nonlinear geometric observer** with constant gains -- a
fundamentally different mechanism for handling gravity/bias, which is why
this method is a useful third data point for the gravity-initialization
research the MOLA benchmark suite tracks. Every point is deskewed against a
continuous IMU-integrated trajectory (not a per-scan linear interpolation),
and the local map is a **submap** selected from keyframes by a convex/concave
hull + kNN search, not a voxel/octree structure.

## What this provides

1. **`mola::DlioOdometry`** -- an online module loadable from a `mola-cli`
   YAML pipeline (`type: mola::DlioOdometry`), publishing live localization
   and map (submap) updates, plus a minimal 3-D visualization (pose,
   trajectory, submap, status sub-window).
2. **`mola-dlio-cli`** -- an offline, loss-free command-line tool: feeds one
   observation at a time and waits for it to finish before advancing, so no
   scan is ever dropped.

Both are built on `dlio_core` (`third_party/dlio_core/`), DLIO's algorithm
with every ROS dependency stripped out (plain C++ structs in/out), including
its vendored `nano_gicp` (GICP + a bundled `nanoflann`).

## Datasets

- **Oxford Spires**: via the generic `mola::Rosbag2Dataset` (package
  `mola_input_rosbag2`), same approach as `mola_lidar_odometry`'s and
  `mola_fast_lio2_wrapper`'s Oxford Spires launch files. MulRan is out of
  scope for this wrapper (see the parent plan).

## Build

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select mola_dlio_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Usage

```bash
# Online, live GUI:
mola-dlio-gui-oxford-spires /mnt/datasets/public/oxford-spires/<sequence>/
mola-dlio-gui-oxford-spires /mnt/datasets/public/oxford-spires/<sequence>/ --headless

# Offline, lossless, with TUM trajectory output:
SEQ=/mnt/datasets/public/oxford-spires/2024-03-13-observatory-quarter-01
BAG=$SEQ/raw/ros2bag/1710338090_2024-03-13-13-54-51
LIDAR_POSE_X=0 LIDAR_POSE_Y=0 LIDAR_POSE_Z=0 LIDAR_POSE_YAW=0 LIDAR_POSE_PITCH=0 LIDAR_POSE_ROLL=0 \
MOLA_USE_FIXED_LIDAR_POSE=true \
IMU_POSE_X=0.018771 IMU_POSE_Y=-0.008218 IMU_POSE_Z=-0.070474 \
IMU_POSE_YAW=-90.6263 IMU_POSE_PITCH=-0.1665 IMU_POSE_ROLL=-0.1287 \
MOLA_USE_FIXED_IMU_POSE=true \
mola-dlio-cli -c pipelines/dlio-oxford-spires.yaml \
  --input-rosbag2 "$BAG" \
  --lidar-topic /hesai/pandar --imu-topic /alphasense_driver_ros/imu \
  --output-tum-path /tmp/dlio_oq01.tum
```

See `pipelines/dlio-oxford-spires.yaml` for the full parameter list (LiDAR-IMU
extrinsics, preprocessing/crop-box/voxel resolution, GICP settings, geometric
observer gains), with the most commonly-tuned ones exposed as env vars
(`DLIO_CROPBOX_SIZE`, `DLIO_VOXEL_RES`, `DLIO_KEYFRAME_THRESH_D`,
`DLIO_KEYFRAME_THRESH_R`, `DLIO_GICP_MAX_CORR_DIST`).

## Gravity/bias instrumentation

Set `MOLA_DLIO_DUMP_ESTIMATED_GRAV=1` to write `grav.txt` (in the current
working directory), format `<time> <gx> <gy> <gz>` -- the same format every
wrapper in this benchmark suite uses.

DLIO does **not** carry gravity as an explicit filter state the way an IESKF
does (there is no `state.grav` to read). Two different things are dumped
under the same env var, both documented in `dlio_core/DlioCore.h` and
`Types.hpp`:

1. A **one-off estimate** from the IMU calibration window (`imu_calib_time_sec`
   seconds of static IMU at startup): the average accelerometer reading,
   normalized to the configured gravity magnitude. Written once, as the
   first line, with a `#`-prefixed comment noting the sample count.
2. A **per-scan reconstruction**: `R^T * [0, 0, -g]`, i.e. the world-frame
   gravity vector expressed in the current body-frame orientation estimate.
   This is not a filter state -- it is recomputed from the orientation every
   scan -- but it is the value that is comparable across methods (e.g.
   Fast-LIO2/EllipseLIO's IESKF `state.grav`) since DLIO's geometric observer
   only ever *implicitly* uses gravity via the world-frame `z` acceleration
   correction in `propagateState()`, never as a directly observable state
   variable.

## Notes

- `dlio_core` (`third_party/dlio_core/`) is DLIO's algorithm with every ROS
  dependency stripped out and is unit-tested standalone (`test_dlio_core`,
  built via `ament_add_gtest`). It builds with `-DPCL_NO_PRECOMPILE` for the
  same reason `fast_lio2_core` does -- see that package's own README note.
- Every deviation from upstream beyond mechanical de-ROS-ing (blocking IMU
  wait replaced with a poll/retry queue; three function-local `static`
  variables converted to instance members to make the library safely
  re-instantiable; the per-sensor point-timestamp format detection dropped
  in favor of the single normalized convention the MRPT adapter already
  provides) is documented in `THIRD_PARTY_NOTICES.md` and in
  `~/plans/lio-wrappers-dlio.md`'s work log.
