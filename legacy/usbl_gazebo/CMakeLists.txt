cmake_minimum_required(VERSION 3.0.2)
project(usbl_gazebo)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES

  USBLCommand.msg
  USBLResponse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  gazebo_ros
  message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(transponderPlugin SHARED src/transponderPlugin.cpp)
add_dependencies(transponderPlugin usbl_gazebo_generate_messages_cpp)

add_library(transceiverPlugin SHARED src/transceiverPlugin.cpp)
add_dependencies(transceiverPlugin usbl_gazebo_generate_messages_cpp)
