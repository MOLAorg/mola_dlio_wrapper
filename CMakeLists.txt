# ------------------------------------------------------------------------------
#                       mola_dlio_wrapper
#
# Copyright (C) 2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file.
# See THIRD_PARTY_NOTICES.md for the licensing terms of the vendored
# DLIO / nano_gicp / nanoflann code under third_party/.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

project(mola_dlio_wrapper LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(Eigen3 REQUIRED)
# Deliberately NOT requesting the "surface" component (needed for
# pcl::ConvexHull/ConcaveHull, used by dlio_core's submap construction):
# PCLConfig.cmake's per-component dependency resolution pulls in a
# find_package(VTK) for "surface" on this machine, and the installed VTK
# 9.5 cmake config has a broken "VTK::mpi" target reference that aborts the
# whole CMake configure. The surface/convex-hull headers and library are
# installed regardless of the COMPONENTS list (they are not gated at the
# apt-package level), so third_party/dlio_core/CMakeLists.txt links
# `pcl_surface` (and its `qhull_r` dependency) directly by name instead of
# going through ${PCL_LIBRARIES}.
find_package(PCL 1.8 REQUIRED COMPONENTS common filters registration kdtree)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)

# ament_cmake / BUILD_TESTING must be set up before add_subdirectory() below,
# since dlio_core's own CMakeLists.txt uses ament_add_gtest():
if(DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# -----------------------------------------------------------------------------
#  dlio_core: the ROS-free, MOLA-free DLIO algorithm library.
# -----------------------------------------------------------------------------
add_subdirectory(third_party/dlio_core)

# -----------------------------------------------------------------------------
#  mola_dlio_wrapper: the online mola-cli module
# -----------------------------------------------------------------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/DlioCoreBridge.cpp
    module/src/DlioOdometry.cpp
    module/src/DlioOdometry_Adapters.cpp
    # Last:
    module/src/register.cpp
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mrpt::obs
    mola::mola_kernel
    mola::mola_yaml
  PRIVATE_LINK_LIBRARIES
    # dlio_core is a plain (non ament/non exported) static lib: link it
    # PRIVATE so `install(EXPORT ...)` doesn't need to export it too (its
    # symbols get fully absorbed into this .so). Only DlioOdometry.cpp's own
    # compilation needs its headers (see the PRIVATE include dirs below) --
    # the public header only needs the light Types.hpp.
    dlio_core
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-maps
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  # module/include/.../DlioOdometry.h (installed, public) only needs the
  # light, PCL/nano_gicp-free dlio_core/Types.hpp, so its include dir is
  # PUBLIC too: needed by any consumer of this library, in particular
  # apps/mola-dlio-cli.cpp (a separate target below).
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/dlio_core/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
    module/src
    third_party/dlio_core/include
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Install pipeline / mola-cli launch YAMLs:
install(
  DIRECTORY
    mola-cli-launchs
    pipelines
  DESTINATION
    share/${PROJECT_NAME}
)

# Install gui-* convenience scripts:
install(
  PROGRAMS
    scripts/mola-dlio-gui-oxford-spires
  DESTINATION
    bin
)

# -----------------------------------------------------------------------------
#  mola-dlio-cli: the offline, lossless CLI app
# -----------------------------------------------------------------------------
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional

mola_add_executable(
  TARGET  mola-dlio-cli
  SOURCES apps/mola-dlio-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    # For dlio_core/Types.hpp, transitively pulled in by the public
    # DlioOdometry.h.
    dlio_core
    mrpt::maps
    mrpt::tclap
)

if(mola_input_mulran_dataset_FOUND)
  message(STATUS "mola-dlio-cli: With Mulran = YES")
  target_link_libraries(mola-dlio-cli mola::mola_input_mulran_dataset)
  target_compile_definitions(mola-dlio-cli PRIVATE HAVE_MOLA_INPUT_MULRAN)
else()
  message(STATUS "mola-dlio-cli: With Mulran = NO")
endif()

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-dlio-cli: With Rawlog = YES")
  target_link_libraries(mola-dlio-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-dlio-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-dlio-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-dlio-cli: With rosbag2 = YES")
  target_link_libraries(mola-dlio-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-dlio-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-dlio-cli: With rosbag2 = NO")
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  ament_package()
endif()
