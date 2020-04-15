cmake_minimum_required(VERSION 2.8.3)
project(dvl_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  uuv_sensor_ros_plugins
  )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

# Nothing required for now, but the block is kept in case the package is expanded
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dvl_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Nothing required for now, but the block is kept in case the packages is expanded
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

