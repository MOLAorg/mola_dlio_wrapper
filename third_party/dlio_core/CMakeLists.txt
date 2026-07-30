# dlio_core: the DLIO algorithm, stripped of ROS and MOLA dependencies.
# See ../../THIRD_PARTY_NOTICES.md.
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

add_library(dlio_core STATIC
  src/DlioCore.cpp
  src/nano_gicp/nanoflann.cc
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)

target_include_directories(dlio_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dlio_core PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  # Not part of ${PCL_LIBRARIES}: the top-level CMakeLists.txt deliberately
  # does not request PCL's "surface" component (see its comment for why),
  # so pcl::ConvexHull/ConcaveHull's library and its qhull dependency are
  # linked directly by name here.
  pcl_surface
  qhull_r
  Boost::boost
  OpenMP::OpenMP_CXX
)

# nano_gicp/dlio.h build on pcl::PointCloud<dlio::Point>, a custom point
# type: force PCL's filter/registration/surface templates to be instantiated
# from headers instead of linked from the precompiled system libpcl_*.so
# (which only instantiate the stock point types), mirroring
# fast_lio2_core's own use of this flag and its rationale comment.
target_compile_definitions(dlio_core PUBLIC PCL_NO_PRECOMPILE)
target_include_directories(dlio_core PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(dlio_core PUBLIC ${PCL_DEFINITIONS})

set_target_properties(dlio_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_dlio_core test/test_dlio_core.cpp)
    target_link_libraries(test_dlio_core dlio_core)
  endif()
endif()
