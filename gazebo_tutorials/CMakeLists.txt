cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
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
target_link_libraries(uuv_mating_plugin ${GAZEBO_LIBRARIES})

add_library(uuv_move_joint SHARED src/uuv_move_joint.cc)
target_link_libraries(uuv_move_joint ${GAZEBO_LIBRARIES})
