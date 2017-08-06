cmake_minimum_required(VERSION 2.8.3)
project(ar_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_tracking
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf2 visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ar_pid src/ar_pid.cpp)
target_link_libraries(ar_pid ${catkin_LIBRARIES})
