cmake_minimum_required(VERSION 3.0.2)
project(barn_ddr_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs nav_msgs laser_geometry tf2_ros tf2_sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/laser_to_pointcloud.py
  scripts/carstate_to_twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
