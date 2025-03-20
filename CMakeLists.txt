cmake_minimum_required(VERSION 3.5)
project(mrs_uav_state_estimators)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
else()
  add_definitions("-O2")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
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

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_uav_managers_INCLUDE_DIRS}
)

set(LIBRARIES
  MrsUavStateEstimators_Common
  # MrsUavStateEstimators_GpsGarmin
  # MrsUavStateEstimators_GpsBaro
  # MrsUavStateEstimators_Rtk
  # MrsUavStateEstimators_RtkGarmin
  # MrsUavStateEstimators_Passthrough
  # MrsUavStateEstimators_GroundTruth
  # MrsUavStateEstimators_Dummy
  # MrsUavStateEstimators_GarminAgl
  )

# HW Api Plugin

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

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api estimator_plugins.xml)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
