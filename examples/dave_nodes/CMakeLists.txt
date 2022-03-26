cmake_minimum_required(VERSION 3.1.1)
project(dave_nodes)
find_package(catkin REQUIRED)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SENSOR_ROS_PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
 )

# for launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

# for config
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

# for Python scripts
catkin_install_python(PROGRAMS
  src/simple_box_motion.py
  src/joy_thrusterop.py
  src/dvl_gradient_plot.py
  src/dvl_state_and_gradient_dsl.py
  src/dvl_state_and_gradient_uuvsim.py
  src/bimanual_simple_demo.py
  src/teleport_vehicle.py
  src/bimanual_integrated_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
