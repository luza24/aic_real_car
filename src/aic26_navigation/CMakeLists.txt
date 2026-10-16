cmake_minimum_required(VERSION 3.0.2)
project(aic26_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  costmap_2d
  map_server
  move_base
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)