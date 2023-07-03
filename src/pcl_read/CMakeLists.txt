cmake_minimum_required(VERSION 3.0.2)
project(pcl_read)
 
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
)
 
find_package(PCL REQUIRED)
 
catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
 
link_directories(
  ${PCL_LIBRARY_DIRS}
)
 
add_executable(pcl_read src/pcl_read.cpp)
 
target_link_libraries(pcl_read
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)