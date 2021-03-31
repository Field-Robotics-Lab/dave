cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_sensor_plugins)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev)
find_package(Boost REQUIRED COMPONENTS system)

set(DAVE_GAZEBO_SENSOR_PLUGINS_LIST "")

catkin_package(
  LIBRARIES
)

include_directories(${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
#                   ...
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${GAZEBO_LIBRARY_DIRS}
#                ...
)

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
#list(APPEND DAVE_GAZEBO_SENSOR_PLUGINS_LIST plugin_name)

# Plugin install directions

install(TARGETS ${DAVE_GAZEBO_SENSOR_PLUGINS_LIST}
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

