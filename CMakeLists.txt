cmake_minimum_required(VERSION 3.10)
project(eventail_simulation VERSION 1.0)
include(cmake/utils.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif ()
set(CMAKE_CXX_STANDARD 17)

find_package_and_export(Eigen3 REQUIRED)
find_package_and_export(Glog REQUIRED)
find_package_and_export(gflags REQUIRED)

set(HEADERS
  include/eventail_simulation/analyzer.hpp
  include/eventail_simulation/geometry.hpp
  include/eventail_simulation/linear_averagor.hpp
  include/eventail_simulation/linear_solver.hpp
  include/eventail_simulation/polyjam_averagor.hpp
  include/eventail_simulation/polyjam_solver.hpp
  include/eventail_simulation/random.hpp
  include/eventail_simulation/simulator.hpp
  include/eventail_simulation/timer.hpp
  include/eventail_simulation/types.hpp
)

set(SOURCES
  src/analyzer.cpp
  src/linear_averagor.cpp
  src/linear_solver.cpp
  src/polyjam_averagor.cpp
  src/polyjam_solver.cpp
  src/random.cpp
  src/simulator.cpp
  src/timer.cpp
)

library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

executable(runtime_analysis
  simulation/runtime_analysis.cpp
  LIBRARIES ${PROJECT_NAME})

executable(numerical_analysis
  simulation/numerical_analysis.cpp
  LIBRARIES ${PROJECT_NAME})

executable(inlier_ratio_analysis
  simulation/inlier_ratio_analysis.cpp
  LIBRARIES ${PROJECT_NAME})

executable(event_number_analysis_template
  simulation/event_number_analysis_template.cpp
  LIBRARIES ${PROJECT_NAME})

executable(single_cluster_analysis_template
  simulation/single_cluster_analysis_template.cpp
  LIBRARIES ${PROJECT_NAME})

executable(multiple_clusters_analysis_template
  simulation/multiple_clusters_analysis_template.cpp
  LIBRARIES ${PROJECT_NAME})

executable(consistency_analysis_template
  simulation/consistency_analysis_template.cpp
  LIBRARIES ${PROJECT_NAME})

package(
  INCLUDE_DIRS
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    Eigen3::Eigen
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES})
