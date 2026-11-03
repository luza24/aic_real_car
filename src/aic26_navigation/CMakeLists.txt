cmake_minimum_required(VERSION 3.0.2)
project(aic26_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  costmap_2d
  geometry_msgs
  global_planner
  map_server
  move_base
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  teb_local_planner
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/localisation_check.py
  scripts/virtual_walls.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
