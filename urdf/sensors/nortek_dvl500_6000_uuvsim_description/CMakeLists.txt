cmake_minimum_required(VERSION 3.0.2)
project(nortek_dvl500_6000_uuvsim_description)

find_package(catkin REQUIRED
  gazebo_ros
  uuv_sensor_ros_plugins
  xacro)

catkin_package()

install(DIRECTORY launch urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

