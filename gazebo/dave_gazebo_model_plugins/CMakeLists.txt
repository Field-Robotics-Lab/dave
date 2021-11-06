cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_model_plugins)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
  add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  message_generation
  gazebo_dev
  dave_gazebo_world_plugins
  dave_gazebo_ros_plugins
)
find_package(Boost REQUIRED COMPONENTS system)

set(DAVE_GAZEBO_MODEL_PLUGINS_LIST "")

add_message_files(
  FILES
  UsblCommand.msg
  UsblResponse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    LIBRARIES
      dave_transient_current_plugin
    CATKIN_DEPENDS
      roscpp
      gazebo_msgs
      gazebo_dev
      gazebo_ros
      message_runtime
      dave_gazebo_world_plugins
      dave_gazebo_ros_plugins
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
#                   ...
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${GAZEBO_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_BINARY_DIR}/msgs
#                ...
)

# ocean current model plugin
add_library(dave_ocean_current_model_plugin
  SHARED
  src/ocean_current_model_plugin.cc
)
add_dependencies(dave_ocean_current_model_plugin
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dave_ocean_current_model_plugin
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARY}
)
list(APPEND DAVE_GAZEBO_MODEL_PLUGINS_LIST dave_ocean_current_model_plugin)

# Declare C++ libraries for USBL
add_library(transponderPlugin
  SHARED
  src/usbl_transponder_plugin.cc
)
add_dependencies(transponderPlugin
  dave_gazebo_model_plugins_generate_messages_cpp
)

add_library(transceiverPlugin
  SHARED
  src/usbl_transceiver_plugin.cc
)
add_dependencies(transceiverPlugin
  dave_gazebo_model_plugins_generate_messages_cpp
)

add_library(plugAndSocketPlugin
  SHARED
  src/plug_and_socket_plugin.cc
)
target_link_libraries(plugAndSocketPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Custom surface properties
add_library(custom_surface_properties
  SHARED
  src/custom_surface_properties.cc
)
target_link_libraries(custom_surface_properties
  ${catkin_LIBRARIES}
)

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
#list(APPEND DAVE_GAZEBO_MODEL_PLUGINS_LIST plugin_name)

# Plugin install directions

install(TARGETS ${DAVE_GAZEBO_MODEL_PLUGINS_LIST}
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

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)