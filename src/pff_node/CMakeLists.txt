cmake_minimum_required(VERSION 2.8.12)
project(pff_node)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(pff_node src/pff_node.cpp)
target_link_libraries(pff_node ${catkin_LIBRARIES})
