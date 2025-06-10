cmake_minimum_required(VERSION 3.5)
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

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O1")
  add_definitions("-g")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")
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
)

set(LIBRARIES
  MrsUavStateEstimators_Common
  MrsUavStateEstimators_GpsBaro
  MrsUavStateEstimators_GpsGarmin
  MrsUavStateEstimators_GarminAgl
  MrsUavStateEstimators_Passthrough
  # MrsUavStateEstimators_Rtk
  # MrsUavStateEstimators_RtkGarmin
  # MrsUavStateEstimators_GroundTruth
  # MrsUavStateEstimators_Dummy
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_uav_managers_INCLUDE_DIRS}
)

# Common estimators

add_library(MrsUavStateEstimators_Common SHARED
  src/estimators/state/state_generic.cpp
  src/estimators/altitude/alt_generic.cpp
  src/estimators/lateral/lat_generic.cpp
  src/estimators/heading/hdg_generic.cpp
  src/estimators/heading/hdg_passthrough.cpp
  )

ament_target_dependencies(MrsUavStateEstimators_Common
  ${DEPENDENCIES}
)

target_compile_definitions(MrsUavStateEstimators_Common PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# GPS Baro

add_library(MrsUavStateEstimators_GpsBaro SHARED
  src/estimators/state/gps_baro.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_GpsBaro
  MrsUavStateEstimators_Common
  )

## Add configure headers for dynamic reconfigure
ament_target_dependencies(MrsUavStateEstimators_GpsBaro
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavStateEstimators_GpsBaro PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# GPS Garmin

add_library(MrsUavStateEstimators_GpsGarmin SHARED
  src/estimators/state/gps_garmin.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_GpsGarmin
  MrsUavStateEstimators_Common
  )

## Add configure headers for dynamic reconfigure
ament_target_dependencies(MrsUavStateEstimators_GpsGarmin
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavStateEstimators_GpsGarmin PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Garmin AGL

add_library(MrsUavStateEstimators_GarminAgl SHARED
  src/estimators/agl/garmin_agl.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_GarminAgl
  MrsUavStateEstimators_Common
  )

## Add configure headers for dynamic reconfigure
ament_target_dependencies(MrsUavStateEstimators_GarminAgl
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavStateEstimators_GarminAgl PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Passthrough

add_library(MrsUavStateEstimators_Passthrough SHARED
  src/estimators/state/passthrough.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_Passthrough
  MrsUavStateEstimators_Common
  )

## Add configure headers for dynamic reconfigure
ament_target_dependencies(MrsUavStateEstimators_Passthrough
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavStateEstimators_Passthrough PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

# BUILD_TESTING is true by default when building with colcon
if(BUILD_TESTING)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
