cmake_minimum_required(VERSION 3.0.2)
project(lidar_3dad_sl3_description)

find_package(catkin REQUIRED
  gazebo_ros
  uuv_sensor_ros_plugins
  xacro)

catkin_package()

install(DIRECTORY meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


install(DIRECTORY meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

