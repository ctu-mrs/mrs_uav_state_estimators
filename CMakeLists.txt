cmake_minimum_required(VERSION 3.5)
project(mrs_uav_state_estimators)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(LIBRARIES
  MrsUavStateEstimators_Common
  MrsUavStateEstimators_GpsGarmin
  MrsUavStateEstimators_GpsBaro
  MrsUavStateEstimators_Rtk
  MrsUavStateEstimators_RtkGarmin
  MrsUavStateEstimators_Passthrough
  MrsUavStateEstimators_GroundTruth
  MrsUavStateEstimators_Dummy
  MrsUavStateEstimators_GarminAgl
  )

# clang-tidy linting
# find_program(CLANG_TIDY_COMMAND NAMES clang-tidy)
# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-header-filter='${CMAKE_SOURCE_DIR}/*'")
# set(CLANG_TIDY_DEFINITIONS "--use-color")

generate_dynamic_reconfigure_options(
  cfg/Correction.cfg
  cfg/LateralEstimator.cfg
  cfg/AltitudeEstimator.cfg
  cfg/HeadingEstimator.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

set(ROSLINT_CPP_OPTS "--filter=-whitespace/comments,-whitespace/braces,-legal/copyright,-readability/multiline_comment,-whitespace/blank_line,-whitespace/line_length,-readability/todo,-build/include_what_you_use,-runtime/references,-build/include")
roslint_cpp()

add_library(MrsUavStateEstimators_Common
  src/estimators/state/state_generic.cpp
  src/estimators/altitude/alt_generic.cpp
  src/estimators/lateral/lat_generic.cpp
  src/estimators/heading/hdg_generic.cpp
  src/estimators/heading/hdg_passthrough.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_dependencies(MrsUavStateEstimators_Common
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_GpsGarmin
  src/estimators/state/gps_garmin.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_GpsGarmin
  MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_GpsGarmin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_GpsBaro
  src/estimators/state/gps_baro.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MrsUavStateEstimators_GpsBaro
  MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_GpsBaro
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_Rtk
  src/estimators/state/rtk.cpp
  )

target_link_libraries(MrsUavStateEstimators_Rtk
  MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_Rtk
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_RtkGarmin
  src/estimators/state/rtk_garmin.cpp
  )

target_link_libraries(MrsUavStateEstimators_RtkGarmin
  MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_RtkGarmin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_Passthrough
  src/estimators/state/passthrough.cpp
  )

target_link_libraries(MrsUavStateEstimators_Passthrough
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_Passthrough
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_GroundTruth
  src/estimators/state/ground_truth.cpp
  )

target_link_libraries(MrsUavStateEstimators_GroundTruth
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_GroundTruth
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_Dummy
  src/estimators/state/dummy.cpp
  )

target_link_libraries(MrsUavStateEstimators_Dummy
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_Dummy
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

add_library(MrsUavStateEstimators_GarminAgl
  src/estimators/agl/garmin_agl.cpp
  )

target_link_libraries(MrsUavStateEstimators_GarminAgl
  MrsUavStateEstimators_Common
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Add configure headers for dynamic reconfigure
add_dependencies(MrsUavStateEstimators_GarminAgl
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/mrs_uav_state_estimators/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES estimator_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
