cmake_minimum_required(VERSION 2.8.3)
project(commander)

## Füge Abhängigkeiten hinzu
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  dynamic_reconfigure
)

catkin_python_setup()

## Füge die generierten dynamischen Rekonfigurationsoptionen hinzu
# ACHTUNG: Diese Zeile muss vor dem catkin_package-Aufruf kommen!
generate_dynamic_reconfigure_options(

)

## catkin spezifische Einstellungen
catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

############################################################
# Installiere Python-Module
############################################################

# Kopiere das gesamte commander Python-Paket in den Python-Installationspfad
install(
  DIRECTORY src/commander/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

############################################################
# Installiere Python-Skripte (Executable Nodes)
############################################################

catkin_install_python(PROGRAMS
  scripts/coordination_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
