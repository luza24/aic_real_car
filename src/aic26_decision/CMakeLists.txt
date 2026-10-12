cmake_minimum_required(VERSION 3.0.2)
project(aic26_decision)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  aic26_msgs
  geometry_msgs
  move_base_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/mission_common.py
  scripts/waypoint_follower.py
  scripts/capture_cycle.py
  scripts/demo_cycle.py
  scripts/mission_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
