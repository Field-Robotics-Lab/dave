cmake_minimum_required(VERSION 3.0.2)
project(teledyne_whn_dsl_description)

find_package(catkin REQUIRED
  gazebo_ros
  ds_sim
  xacro)

catkin_package()

install(DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


install(DIRECTORY launch meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

