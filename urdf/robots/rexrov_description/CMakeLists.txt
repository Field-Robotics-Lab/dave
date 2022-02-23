cmake_minimum_required(VERSION 3.1.1)
project(rexrov_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
