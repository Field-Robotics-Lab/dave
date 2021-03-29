cmake_minimum_required(VERSION 2.8.3)
project(uuv_mating)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Check for c++11 / c++0x support
if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(LOCAL_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(uuv_mating_plugin SHARED src/uuv_mating.cc)
target_link_libraries(uuv_mating_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
set_target_properties(
  uuv_mating_plugin
  PROPERTIES
  COMPILE_FLAGS
  "${LOCAL_CXX_FLAGS}"
)
