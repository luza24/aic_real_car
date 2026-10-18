cmake_minimum_required(VERSION 3.0.2)
project(aic26_cobot)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_srvs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_protocols.py)
endif()

catkin_install_python(PROGRAMS
  scripts/hardware_common.py
  scripts/chassis_node.py
  scripts/imu_node.py
  scripts/lidar_node.py
  scripts/camera_node.py
  scripts/hardware_status_node.py
  scripts/emergency_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
