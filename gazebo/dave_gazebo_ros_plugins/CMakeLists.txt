cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_ros_plugins)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    roscpp
    gazebo_msgs
    geometry_msgs
    dave_gazebo_world_plugins
    message_generation
    std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

set(DAVE_GAZEBO_ROS_PLUGINS_LIST "")

# ocean current msg and services
add_service_files(
  FILES
  SetCurrentModel.srv
  GetCurrentModel.srv
  SetCurrentVelocity.srv
  SetCurrentDirection.srv
  SetStratifiedCurrentVelocity.srv
  SetStratifiedCurrentDirection.srv
  SetOriginSphericalCoord.srv
  GetOriginSphericalCoord.srv
  TransformToSphericalCoord.srv
  TransformFromSphericalCoord.srv
)

add_message_files(
  FILES
  StratifiedCurrentDatabase.msg
  StratifiedCurrentVelocity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    dave_ocean_current_plugin
    dave_sc_coordinates_interface
  CATKIN_DEPENDS
    roscpp
    gazebo_msgs
    geometry_msgs
    dave_gazebo_world_plugins
    gazebo_dev
    std_msgs
    geometry_msgs
    message_runtime
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
#                   ...
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${GAXEBO_LIBRARY_DIRS}
#                ...
)

# ocean current plugin
add_library(dave_ocean_current_plugin
   SHARED
   src/ocean_current_plugin.cpp
)
add_dependencies(dave_ocean_current_plugin
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dave_ocean_current_plugin
   ${catkin_LIBRARIES}
)
list(APPEND DAVE_GAZEBO_ROS_PLUGINS_LIST dave_ocean_current_plugin)

# pulse lidar plugin
add_library(dave_pulse_lidar_plugin
   SHARED
   src/pulse_lidar_plugin.cpp
)
add_dependencies(dave_pulse_lidar_plugin
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dave_pulse_lidar_plugin
   ${catkin_LIBRARIES}
)
list(APPEND DAVE_GAZEBO_ROS_PLUGINS_LIST dave_pulse_lidar_plugin)

# spherical coordinate interfafce
add_library(dave_sc_coordinates_interface
   SHARED
   src/spherical_coordinates_interface.cpp
)
add_dependencies(dave_sc_coordinates_interface
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dave_sc_coordinates_interface
   ${catkin_LIBRARIES}
)
list(APPEND DAVE_GAZEBO_ROS_PLUGINS_LIST dave_sc_coordinates_interface)

# Plugin compile directions

#add_library(plugin_name
#    SHARED
#    src/plugin_source_file1.cc
#    src/plugin_source_file2.cc
#    ...
#)
#add_dependencies(plugin_name
#  ${catkin_EXPORTED_TARGETS}
#  ...
#)
#target_link_libraries(plugin_name
#    ${catkin_LIBRARIES}
#    ...
#)
#list(APPEND DAVE_GAZEBO_ROS_PLUGINS_LIST plugin_name)

# Plugin install directions

install(TARGETS ${DAVE_GAZEBO_ROS_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".h"
)

