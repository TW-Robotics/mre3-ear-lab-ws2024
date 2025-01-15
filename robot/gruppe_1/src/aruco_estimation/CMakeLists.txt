cmake_minimum_required(VERSION 3.0.2)
project(aruco_estimation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
)

catkin_python_setup()


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

############################################################
# Installiere Python-Module
############################################################

# Kopiere das gesamte amr_control Python-Paket in den Python-Installationspfad
install(
  DIRECTORY src/aruco_estimation/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

############################################################
# Installiere Python-Skripte (Executable Nodes)
############################################################

catkin_install_python(PROGRAMS
  src/aruco_estimation/aruco_detect.py
  src/aruco_estimation/main.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)