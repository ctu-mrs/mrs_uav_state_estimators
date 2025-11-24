cmake_minimum_required(VERSION 3.28)
project(mrs_uav_state_estimators)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_state_estimators]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  pluginlib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  mrs_uav_managers
  tf2_geometry_msgs
  Eigen3
  mrs_uav_testing
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavStateEstimators_Common
  MrsUavStateEstimators_GpsBaro
  MrsUavStateEstimators_GpsGarmin
  MrsUavStateEstimators_GarminAgl
  MrsUavStateEstimators_Passthrough
  MrsUavStateEstimators_F9P
  MrsUavStateEstimators_Rtk
  MrsUavStateEstimators_RtkGarmin
  MrsUavStateEstimators_GroundTruth
  MrsUavStateEstimators_Dummy
)

# Common estimators

add_library(MrsUavStateEstimators_Common SHARED)
add_library(mrs_uav_state_estimators::common ALIAS MrsUavStateEstimators_Common)

target_sources(MrsUavStateEstimators_Common
  PRIVATE
    src/estimators/state/state_generic.cpp
    src/estimators/altitude/alt_generic.cpp
    src/estimators/lateral/lat_generic.cpp
    src/estimators/heading/hdg_generic.cpp
    src/estimators/heading/hdg_passthrough.cpp
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    # include/mrs_uav_state_estimators
    # include/mrs_uav_state_estimators/estimators
    "include/mrs_uav_state_estimators/estimators/correction.h"
    "include/mrs_uav_state_estimators/estimators/partial_estimator.h"
    # include/mrs_uav_state_estimators/estimators/agl
    "include/mrs_uav_state_estimators/estimators/agl/garmin_agl.h"
    # include/mrs_uav_state_estimators/estimators/altitude
    "include/mrs_uav_state_estimators/estimators/altitude/alt_generic.h"
    "include/mrs_uav_state_estimators/estimators/altitude/altitude_estimator.h"
    # include/mrs_uav_state_estimators/estimators/heading
    "include/mrs_uav_state_estimators/estimators/heading/hdg_generic.h"
    "include/mrs_uav_state_estimators/estimators/heading/hdg_passthrough.h"
    "include/mrs_uav_state_estimators/estimators/heading/heading_estimator.h"
    # include/mrs_uav_state_estimators/estimators/lateral
    "include/mrs_uav_state_estimators/estimators/lateral/lat_generic.h"
    "include/mrs_uav_state_estimators/estimators/lateral/lateral_estimator.h"
    # include/mrs_uav_state_estimators/estimators/state
    "include/mrs_uav_state_estimators/estimators/state/dummy.h"
    "include/mrs_uav_state_estimators/estimators/state/f9p.h"
    "include/mrs_uav_state_estimators/estimators/state/ground_truth.h"
    "include/mrs_uav_state_estimators/estimators/state/passthrough.h"
    "include/mrs_uav_state_estimators/estimators/state/state_generic.h"
    # include/mrs_uav_state_estimators/processors
    "include/mrs_uav_state_estimators/processors/proc_excessive_tilt.h"
    "include/mrs_uav_state_estimators/processors/proc_mag_declination.h"
    "include/mrs_uav_state_estimators/processors/proc_median_filter.h"
    "include/mrs_uav_state_estimators/processors/proc_saturate.h"
    "include/mrs_uav_state_estimators/processors/proc_tf_to_world.h"
    "include/mrs_uav_state_estimators/processors/processor.h"
    # include/mrs_uav_state_estimators/processors/mag_declination
    "include/mrs_uav_state_estimators/processors/mag_declination/geo_mag_declination.h"
    "include/mrs_uav_state_estimators/processors/mag_declination/geo_magnetic_tables.hpp"
)

target_link_libraries(MrsUavStateEstimators_Common
  PUBLIC
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    pluginlib::pluginlib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(MrsUavStateEstimators_Common PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# GPS Baro

add_library(MrsUavStateEstimators_GpsBaro SHARED
  src/estimators/state/gps_baro.cpp
)

target_link_libraries(MrsUavStateEstimators_GpsBaro
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_GpsBaro PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# GPS Garmin

add_library(MrsUavStateEstimators_GpsGarmin SHARED
  src/estimators/state/gps_garmin.cpp
)

target_link_libraries(MrsUavStateEstimators_GpsGarmin
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_GpsGarmin PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Garmin AGL

add_library(MrsUavStateEstimators_GarminAgl SHARED
  src/estimators/agl/garmin_agl.cpp
)

target_link_libraries(MrsUavStateEstimators_GarminAgl
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_GarminAgl PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Passthrough

add_library(MrsUavStateEstimators_Passthrough SHARED
  src/estimators/state/passthrough.cpp
)

target_link_libraries(MrsUavStateEstimators_Passthrough
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_Passthrough PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# F9P

add_library(MrsUavStateEstimators_F9P SHARED
  src/estimators/state/f9p.cpp
)

target_link_libraries(MrsUavStateEstimators_F9P
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_F9P PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Rtk

add_library(MrsUavStateEstimators_Rtk SHARED
  src/estimators/state/rtk.cpp
)

target_link_libraries(MrsUavStateEstimators_Rtk
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_Rtk PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Rtk Garmin

add_library(MrsUavStateEstimators_RtkGarmin SHARED
  src/estimators/state/rtk_garmin.cpp
)

target_link_libraries(MrsUavStateEstimators_RtkGarmin
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_RtkGarmin PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Ground truth

add_library(MrsUavStateEstimators_GroundTruth SHARED
  src/estimators/state/ground_truth.cpp
)

target_link_libraries(MrsUavStateEstimators_GroundTruth
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_GroundTruth PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Dummy

add_library(MrsUavStateEstimators_Dummy SHARED
  src/estimators/state/dummy.cpp
)

target_link_libraries(MrsUavStateEstimators_Dummy
  PUBLIC
    mrs_uav_state_estimators::common
)

target_compile_definitions(MrsUavStateEstimators_Dummy PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Interface

add_library(MrsUavStateEstimators_MrsUavStateEstimators INTERFACE)
add_library(mrs_uav_state_estimators::mrs_uav_state_estimators ALIAS
  MrsUavStateEstimators_MrsUavStateEstimators
)
set_target_properties(MrsUavStateEstimators_MrsUavStateEstimators
  PROPERTIES
    EXPORT_NAME mrs_uav_state_estimators
    OUTPUT_NAME mrs_uav_state_estimators
)

target_link_libraries(MrsUavStateEstimators_MrsUavStateEstimators
  INTERFACE
    ${LIBRARIES}
)

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    MrsUavStateEstimators_MrsUavStateEstimators
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
