cmake_minimum_required(VERSION 3.0.2)
project(aic26_utils)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera_view.py
  scripts/speech_broadcaster.py
  scripts/kokoro_worker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/setup_kokoro.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
