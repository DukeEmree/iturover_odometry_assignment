cmake_minimum_required(VERSION 3.0.2)
project(iturover_odometry_assignment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/odometry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
