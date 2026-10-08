cmake_minimum_required(VERSION 3.0.2)
project(aic26_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)