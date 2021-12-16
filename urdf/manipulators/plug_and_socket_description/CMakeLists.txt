cmake_minimum_required(VERSION 3.0.2)
project(plug_and_socket_description)

find_package(catkin REQUIRED
  gazebo_ros
  xacro)

catkin_package()

install(DIRECTORY urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


