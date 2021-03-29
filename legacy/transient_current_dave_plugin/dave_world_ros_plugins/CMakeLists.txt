cmake_minimum_required(VERSION 2.8.3)
project(dave_world_ros_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  dave_world_plugins
  dave_world_ros_plugins_msgs
  gazebo_dev)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
  dave_underwater_current_ros_plugin
  dave_sc_ros_interface_plugin
 CATKIN_DEPENDS
   roscpp
   gazebo_msgs
   geometry_msgs
   dave_world_plugins
   dave_world_ros_plugins_msgs
   gazebo_dev
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(dave_underwater_current_ros_plugin src/UnderwaterCurrentROSPlugin.cc)
target_link_libraries(dave_underwater_current_ros_plugin ${catkin_LIBRARIES})
add_dependencies(dave_underwater_current_ros_plugin ${catkin_EXPORTED_TARGETS})

add_library(dave_sc_ros_interface_plugin src/SphericalCoordinatesROSInterfacePlugin.cc)
target_link_libraries(dave_sc_ros_interface_plugin ${catkin_LIBRARIES})
add_dependencies(dave_sc_ros_interface_plugin ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS
    dave_underwater_current_ros_plugin
    dave_sc_ros_interface_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".~" EXCLUDE
)
