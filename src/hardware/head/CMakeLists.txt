cmake_minimum_required(VERSION 2.8.3)
project(head)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  controller_manager_msgs
  trajectory_msgs
  tf2_sensor_msgs
  tf2 REQUIRED
  tf2_ros REQUIRED
  pcl_ros
)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )



catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf2_ros tf2_sensor_msgs
#  INCLUDE_DIRS include
#  LIBRARIES head
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/head.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(head_node
  src/head_node.cpp)
add_executable(gaze_node
  src/gaze_node.cpp)
add_executable(led_node
  src/led_node.cpp)
add_executable(mic_node
  src/mic_controller.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(head_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(head_node
  ${catkin_LIBRARIES}
  )
target_link_libraries(led_node
  ${catkin_LIBRARIES}
  )

target_link_libraries(gaze_node 
${catkin_LIBRARIES} 
${tf2_LIBRARIES} 
${tf2_ros_LIBRARIES})

target_link_libraries(mic_node
  ${catkin_LIBRARIES}
  )