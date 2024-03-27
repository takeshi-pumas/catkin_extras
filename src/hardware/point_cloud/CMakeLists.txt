cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud
CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
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


add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough 
${catkin_LIBRARIES}
${tf2_LIBRARIES} 
${tf2_ros_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} )
