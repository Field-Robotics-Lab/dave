cmake_minimum_required(VERSION 2.8.3)
project(uuv_dave)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package()

install(DIRECTORY launch urdf worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS nodes/joy_thrusterop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
