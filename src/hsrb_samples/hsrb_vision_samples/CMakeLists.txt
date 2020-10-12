cmake_minimum_required(VERSION 2.8.3)
project(hsrb_vision_samples)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(OpenCV ${OpenCV_VERSION} EXACT REQUIRED opencv_highgui)
find_package(PCL ${PCL_VERSION} REQUIRED )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include
)

add_library(hsrb_color_detection src/color_detection.cpp)

# Declare a C++ executable for color detection
add_executable(hsrb_color_detection_main src/color_detection_main.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(hsrb_color_detection_main
  hsrb_color_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_library(hsrb_plane_detection src/plane_detection.cpp)

# Declare a C++ executable for plane detection
add_executable(hsrb_plane_detection_main src/plane_detection_main.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(hsrb_plane_detection_main
  hsrb_plane_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(test/python_color_detection-test.test)
  add_rostest_gtest(color_detection_test
                    test/cpp_color_detection-test.test
                    test/color_detection-test.cpp
                    src/color_detection.cpp)
  target_link_libraries(color_detection_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_rostest_gtest(plane_detection_test
                    test/cpp_plane_detection-test.test
                    test/plane_detection-test.cpp
                    src/plane_detection.cpp)
  target_link_libraries(plane_detection_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()
