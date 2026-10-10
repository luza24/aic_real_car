cmake_minimum_required(VERSION 3.0.2)
project(aic26_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  aic26_msgs
  gazebo_ros
  gazebo_plugins
  gazebo_dev
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs aic26_msgs gazebo_dev
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(traffic_light_visual_plugin SHARED
  src/traffic_light_visual_plugin.cpp
)
target_link_libraries(traffic_light_visual_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(traffic_light_visual_plugin
  ${catkin_EXPORTED_TARGETS}
)

# Runs in gzclient (attached to the floor visual) and switches the sun and the
# URDF headlights between day and night. A world/model plugin cannot do this
# because gzserver has no render scene.
add_library(light_switch_visual_plugin SHARED
  src/light_switch_visual_plugin.cpp
)
target_link_libraries(light_switch_visual_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(light_switch_visual_plugin
  ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS
  scripts/traffic_controller.py
  scripts/day_night.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)