cmake_minimum_required(VERSION 3.0.2)
project(aic26_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  costmap_2d
  global_planner
  map_server
  move_base
  teb_local_planner
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
