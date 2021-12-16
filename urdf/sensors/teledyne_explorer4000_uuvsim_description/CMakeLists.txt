cmake_minimum_required(VERSION 3.0.2)
project(teledyne_explorer4000_uuvsim_description)

find_package(catkin REQUIRED
  gazebo_ros
  uuv_sensor_ros_plugins
  xacro)

catkin_package()

install(DIRECTORY urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

