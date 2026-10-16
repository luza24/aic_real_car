cmake_minimum_required(VERSION 3.0.2)
project(aic26_perception)

find_package(catkin REQUIRED COMPONENTS
  aic26_msgs
  cv_bridge
  image_transport
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/perception_common.py
  scripts/traffic_light_detector.py
  scripts/people_detector.py
  scripts/car_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES results/README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/results
)