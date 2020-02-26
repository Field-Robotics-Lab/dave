cmake_minimum_required(VERSION 2.8.3)
project(sonar_dave_uuv)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  sensor_msgs
  image_transport
  geometry_msgs
  std_msgs
  roscpp
  tf
  gazebo_plugins
  message_filters
  tf2_ros
  tf2
  laser_geometry)

find_package(Eigen3 REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(SONAR_DAVE_PLUGINS_LIST "")

# Add Gazebo custom protobuf messages
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
  INCLUDE_DIRS include
  LIBRARIES
    image_sonar_dave_plugin
  CATKIN_DEPENDS
    gazebo_dev
    sensor_msgs
    geometry_msgs
    std_msgs
    roscpp
    tf
    gazebo_plugins
    message_filters
    tf2_ros
    tf2
    image_transport
    laser_geometry
)

###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(image_sonar_dave_plugin src/gazebo_ros_image_sonar.cpp)
target_link_libraries(image_sonar_dave_plugin ${OGRE_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(image_sonar_dave_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND SONAR_DAVE_PLUGINS_LIST image_sonar_dave_plugin)


#############
## Install ##
#############

install(
  TARGETS ${SONAR_DAVE_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

# Install urdf files
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

