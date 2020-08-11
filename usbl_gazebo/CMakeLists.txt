cmake_minimum_required(VERSION 3.0.2)
project(usbl_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(transponderPlugin SHARED src/transponderPlugin.cpp)
add_library(transceiverPlugin SHARED src/transceiverPlugin.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(transponderPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )
target_link_libraries(transceiverPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )