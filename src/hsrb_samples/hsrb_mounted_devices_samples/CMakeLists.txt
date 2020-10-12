cmake_minimum_required(VERSION 2.8.3)
project(hsrb_mounted_devices_samples)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tmc_control_msgs
  tmc_manipulation_msgs
  tmc_msgs
  tmc_suction
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tmc_control_msgs
  tmc_manipulation_msgs
  tmc_msgs
  tmc_suction
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable for speak object weight node
add_executable(hsrb_speak_object_weight
  src/${PROJECT_NAME}/force_torque_sensor/speak_object_weight.cpp)

add_executable(hsrb_suction_controller
  src/${PROJECT_NAME}/suction_mechanism/suction_controller.cpp)

add_executable(hsrb_change_led_color
  src/${PROJECT_NAME}/status_led/change_led_color.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(hsrb_speak_object_weight
  ${catkin_LIBRARIES}
)

target_link_libraries(hsrb_suction_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(hsrb_change_led_color
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # For force torque sensor sample
  add_rostest(test/force_torque_sensor/python_speak_object_weight-test.test)

  add_rostest_gtest(speak_object_weight_ja_test
                    test/force_torque_sensor/cpp_speak_object_weight_ja-test.test
                    test/force_torque_sensor/speak_object_weight-test.cpp)
  target_link_libraries(speak_object_weight_ja_test ${catkin_LIBRARIES})

  add_rostest_gtest(speak_object_weight_en_test
                    test/force_torque_sensor/cpp_speak_object_weight_en-test.test
                    test/force_torque_sensor/speak_object_weight-test.cpp)
  target_link_libraries(speak_object_weight_en_test ${catkin_LIBRARIES})

  # For suction mechanism sample
  add_rostest(test/suction_mechanism/python_suction_controller-test.test)

  add_rostest_gtest(suction_controller_success_test
                    test/suction_mechanism/cpp_suction_controller_success-test.test
                    test/suction_mechanism/suction_controller_success-test.cpp)
  target_link_libraries(suction_controller_success_test ${catkin_LIBRARIES})

  add_rostest_gtest(suction_controller_failure_test
                    test/suction_mechanism/cpp_suction_controller_failure-test.test
                    test/suction_mechanism/suction_controller_failure-test.cpp)
  target_link_libraries(suction_controller_failure_test ${catkin_LIBRARIES})

  # For change led color sample
  add_rostest(test/status_led/python_change_led_color-test.test)

  add_rostest_gtest(change_led_color_test
                    test/status_led/cpp_change_led_color-test.test
                    test/status_led/change_led_color-test.cpp)
  target_link_libraries(change_led_color_test ${catkin_LIBRARIES})
endif()
