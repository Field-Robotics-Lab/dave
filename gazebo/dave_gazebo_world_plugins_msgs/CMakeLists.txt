cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_world_plugins_msgs)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
)

find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)
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
    ${CMAKE_CURRENT_BINARY_DIR}
    ${GAZEBO_MSG_INCLUDE_DIRS}
  LIBRARIES
    dave_gazebo_world_plugins_msgs
  CATKIN_DEPENDS
    gazebo_dev
    gazebo_msgs
  DEPENDS
    Protobuf
)

include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(${GAZEBO_MSG_INCLUDE_DIRS}
)

# gazebo messages
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
  msgs/StratifiedCurrentVelocity.proto
)
add_library(dave_gazebo_world_plugins_msgs SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(dave_gazebo_world_plugins_msgs ${PROTOBUF_LIBRARY})

install(TARGETS dave_gazebo_world_plugins_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)
