find_package(rostest REQUIRED)

# Corrections test

## DOES NOT COMPILE
# add_rostest_gtest(CorrectionsTest
#   corrections/corrections.test
#   corrections/test_corrections.cpp
#   )

# target_link_libraries(CorrectionsTest
#   mrs_lib
#   ${catkin_LIBRARIES}
#   )

# add_dependencies(CorrectionsTest
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )

# GpsBaro basic

add_rostest_gtest(GpsBaro_basic
  gps_baro/basic/gps_baro_basic.test
  gps_baro/basic/test.cpp
  )

target_link_libraries(GpsBaro_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(GpsBaro_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# GpsGarmin basic

add_rostest_gtest(GpsGarmin_basic
  gps_garmin/basic/gps_garmin_basic.test
  gps_garmin/basic/test.cpp
  )

target_link_libraries(GpsGarmin_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(GpsGarmin_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Rtk basic

add_rostest_gtest(Rtk_basic
  rtk/basic/rtk_basic.test
  rtk/basic/test.cpp
  )

target_link_libraries(Rtk_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(Rtk_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# RtkGarmin basic

add_rostest_gtest(RtkGarmin_basic
  rtk_garmin/basic/rtk_garmin_basic.test
  rtk_garmin/basic/test.cpp
  )

target_link_libraries(RtkGarmin_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(RtkGarmin_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
