cmake_minimum_required(VERSION 2.8.3)
project(hsrb_motion_samples)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager_msgs
  roscpp
  rospy
  tmc_control_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  control_msgs
  controller_manager_msgs
  roscpp
  rospy
  tmc_control_msgs
  trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(arm_message src/arm_message.cpp)
add_executable(arm_actionlib src/arm_actionlib.cpp)
add_executable(head_message src/head_message.cpp)
add_executable(head_actionlib src/head_actionlib.cpp)
add_executable(gripper_message src/gripper_message.cpp)
add_executable(gripper_actionlib src/gripper_actionlib.cpp)
add_executable(gripper_actionlib_grasp src/gripper_actionlib_grasp.cpp)
add_executable(omni_message src/omni_message.cpp)
add_executable(omni_message_velocity src/omni_message_velocity.cpp)
add_executable(omni_actionlib src/omni_actionlib.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(arm_message
  ${catkin_LIBRARIES}
)
target_link_libraries(arm_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(head_message
  ${catkin_LIBRARIES}
)
target_link_libraries(head_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_message
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_actionlib
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_actionlib_grasp
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_message
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_message_velocity
  ${catkin_LIBRARIES}
)
target_link_libraries(omni_actionlib
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS arm_message arm_actionlib head_message head_actionlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/arm_message.test)
  add_rostest(tests/arm_message_python.test)
  add_rostest(tests/arm_action.test)
  add_rostest(tests/arm_action_python.test)
  add_rostest(tests/head_message.test)
  add_rostest(tests/head_message_python.test)
  add_rostest(tests/head_action.test)
  add_rostest(tests/head_action_python.test)
  add_rostest(tests/gripper_message.test)
  add_rostest(tests/gripper_message_python.test)
  add_rostest(tests/gripper_action.test)
  add_rostest(tests/gripper_action_python.test)
  add_rostest(tests/gripper_action_grasp.test)
  add_rostest(tests/gripper_action_grasp_python.test)
  add_rostest(tests/omni_message.test)
  add_rostest(tests/omni_message_python.test)
  add_rostest(tests/omni_action.test)
  add_rostest(tests/omni_action_python.test)
endif()
