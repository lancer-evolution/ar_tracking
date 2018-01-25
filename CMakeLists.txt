cmake_minimum_required(VERSION 2.8.3)
project(ar_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL 1.2 REQUIRED PATHS /usr NO_DEFAULT_PATH)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_eigen
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
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ar_pid src/ar_pid.cpp)
target_link_libraries(ar_pid ${catkin_LIBRARIES})
add_executable(orb_scale_estimator src/orb_scale_estimator.cpp)
target_link_libraries(orb_scale_estimator ${catkin_LIBRARIES})
add_executable(ar_trans_publisher src/ar_trans_publisher.cpp)
target_link_libraries(ar_trans_publisher ${catkin_LIBRARIES})
add_executable(pointcloud_to_pcd src/pointcloud_to_pcd.cpp)
target_link_libraries(pointcloud_to_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})
