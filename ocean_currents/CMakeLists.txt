cmake_minimum_required(VERSION 2.8.3)
project(ocean_currents)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  uuv_world_ros_plugins_msgs
)

add_service_files(
  FILES
  GetOceanCurrent.srv
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  geographic_msgs
  geometry_msgs
  message_runtime
  rospy
  sensor_msgs
  std_msgs
  uuv_world_ros_plugins_msgs
)

catkin_install_python(PROGRAMS
  nodes/ocean_currents
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
