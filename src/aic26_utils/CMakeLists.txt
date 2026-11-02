cmake_minimum_required(VERSION 3.0.2)
project(aic26_utils)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs cv_bridge geometry_msgs rospy sensor_msgs
                 std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera_view.py
  scripts/cleanup_leftovers.py
  scripts/stop_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
