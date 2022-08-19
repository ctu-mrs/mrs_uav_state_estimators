cmake_minimum_required(VERSION 2.8.3)
project(mrs_uav_state_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_lib
  mrs_msgs
  nodelet
  tf2_geometry_msgs
  dynamic_reconfigure
  message_generation
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

add_message_files(DIRECTORY msg FILES
  EstimatorDiagnostics.msg
  EstimatorOutput.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

generate_dynamic_reconfigure_options(
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES GpsGarmin
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf2_geometry_msgs geometry_msgs mrs_lib mrs_msgs message_runtime
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

set(src_files 
  src/estimators/estimator.cpp 
  src/estimators/state/state_estimator.cpp 
  src/estimators/state/gps_garmin.cpp 
  src/estimators/altitude/garmin.cpp 
  src/estimators/lateral/gps.cpp 
  # src/estimators_testing.cpp 
  )

## Declare a C++ executable
# add_library(EstimationTesting src/estimators/estimator.cpp src/estimators/altitude/garmin.cpp src/estimators/lateral/gps.cpp src/estimators_testing.cpp )
add_library(GpsGarmin 
  ${src_files}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(GpsGarmin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
# add_dependencies(EstimationTesting ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
# add_library(EstimationTesting src/estimators/estimator.cpp src/estimators/altitude/garmin.cpp src/estimators/lateral/gps.cpp src/estimators_testing.cpp )
add_library(EstimationManager 
  src/estimation_manager.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(EstimationManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
