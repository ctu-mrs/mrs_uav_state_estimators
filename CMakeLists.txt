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
  Diagnostics.msg
  EstimatorDiagnostics.msg
  EstimatorOutput.msg
  EstimatorCorrection.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/Correction.cfg
  cfg/LateralEstimator.cfg
  cfg/AltitudeEstimator.cfg
  cfg/HeadingEstimator.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Aloam GpsGarmin GpsBaro Rtk GroundTruth Dummy EstimationManager TransformManager
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
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

set(src_files 
  src/estimators/estimator.cpp 
  src/estimators/state/state_estimator.cpp 
  src/estimators/state/gps_garmin.cpp 
  src/estimators/state/gps_baro.cpp 
  src/estimators/state/aloam.cpp 
  src/estimators/state/rtk.cpp 
  src/estimators/state/ground_truth.cpp 
  src/estimators/state/dummy.cpp 
  src/estimators/altitude/alt_generic.cpp 
  src/estimators/lateral/lat_generic.cpp 
  src/estimators/heading/hdg_generic.cpp 
  src/estimators/heading/hdg_passthrough.cpp 
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
add_dependencies(GpsGarmin ${PROJECT_NAME}_gencfg)

add_library(GpsBaro 
  ${src_files}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(GpsBaro
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(GpsBaro ${PROJECT_NAME}_gencfg)

add_library(Aloam 
  ${src_files}
  )

target_link_libraries(Aloam
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(Aloam ${PROJECT_NAME}_gencfg)

add_library(Rtk 
  ${src_files}
  )

target_link_libraries(Rtk
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(Rtk ${PROJECT_NAME}_gencfg)

add_library(GroundTruth 
  ${src_files}
  )

target_link_libraries(GroundTruth
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(GroundTruth ${PROJECT_NAME}_gencfg)

add_library(Dummy 
  ${src_files}
  )

target_link_libraries(Dummy
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(Dummy ${PROJECT_NAME}_gencfg)

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

## Declare a C++ executable
add_library(TransformManager 
  src/transform_manager.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(TransformManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# tests
# add_executable(corrections_tests tests/src/test_corrections.cpp)
# target_link_libraries(corrections_tests
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   ) 

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(CorrectionsTest
    tests/corrections/corrections.test
    tests/corrections/test_corrections.cpp
    )

  target_link_libraries(CorrectionsTest
    # EstimationManager
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    )

  add_dependencies(CorrectionsTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

endif()
