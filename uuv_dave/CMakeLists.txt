cmake_minimum_required(VERSION 2.8.3)
project(uuv_dave)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package()

install(DIRECTORY launch urdf models worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
