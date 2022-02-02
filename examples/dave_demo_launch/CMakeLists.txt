cmake_minimum_required(VERSION 3.1.1)
project(dave_demo_launch)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

# for launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
