cmake_minimum_required(VERSION 3.0.2)
project(bypass_mic)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bypass_mic
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} pulse pulse-simple)

add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES} pulse pulse-simple)
