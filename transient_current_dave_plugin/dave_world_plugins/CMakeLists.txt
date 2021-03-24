cmake_minimum_required(VERSION 2.8.3)
project(dave_world_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	gazebo_dev
	gazebo_msgs)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)
set(hybrid_glider_GAZEBO_PLUGINS_LIST "")

# add_subdirectory(msgs)
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

message(STATUS "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})
file(GLOB msgs msgs/*.proto)

catkin_package(
	INCLUDE_DIRS
    include
    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
    ${GAZEBO_MSG_INCLUDE_DIRS}
	LIBRARIES
    dave_underwater_current_plugin
	CATKIN_DEPENDS
		gazebo_msgs
		gazebo_dev
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

###########
## Build ##
###########

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(stratified_current_velocity_msgs SHARED ${PROTO_SRCS})
target_link_libraries(stratified_current_velocity_msgs ${PROTOBUF_LIBRARY})

add_library(dave_underwater_current_plugin
  src/UnderwaterCurrentPlugin.cc
  src/GaussMarkovProcess.cc
  src/TidalOscillation.cc)

target_link_libraries(dave_underwater_current_plugin
  stratified_current_velocity_msgs
  ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})

add_dependencies(dave_underwater_current_plugin
  stratified_current_velocity_msgs
  ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
		dave_underwater_current_plugin stratified_current_velocity_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dave_world_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
