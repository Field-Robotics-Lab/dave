cmake_minimum_required(VERSION 3.1.1)
project(dave_snippets)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
