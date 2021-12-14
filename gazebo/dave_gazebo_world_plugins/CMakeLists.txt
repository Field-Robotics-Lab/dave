cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_world_plugins)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(GDAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dave_gazebo_world_plugins_msgs
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

set(DAVE_GAZEBO_WORLD_PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    dave_ocean_current_world_plugin
  CATKIN_DEPENDS
    dave_gazebo_world_plugins_msgs
    gazebo_dev
    gazebo_msgs
    gazebo_ros
    roscpp
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
#                   ...
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${GAZEBO_LIBRARY_DIRS}
#                ...
)

# ocean current world plugin
add_library(dave_ocean_current_world_plugin
   SHARED
   src/ocean_current_world_plugin.cc
   src/gauss_markov_process.cc
   src/tidal_oscillation.cc
)
add_dependencies(dave_ocean_current_world_plugin
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dave_ocean_current_world_plugin
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARY}
   ${GAZEBO_LIBRARIES}
)
list(APPEND DAVE_GAZEBO_WORLD_PLUGINS_LIST dave_ocean_current_world_plugin)

# bathymetry world plugin
add_library(dave_bathymetry_world_plugin src/bathymetry_world_plugin.cc)
target_link_libraries(dave_bathymetry_world_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${GDAL_LIBRARY}
)
add_dependencies(dave_bathymetry_world_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND DAVE_GAZEBO_WORLD_PLUGINS_LIST dave_bathymetry_world_plugin)

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
#list(APPEND DAVE_GAZEBO_WORLD_PLUGINS_LIST plugin_name)

# Plugin install directions

install(TARGETS ${DAVE_GAZEBO_WORLD_PLUGINS_LIST}
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
