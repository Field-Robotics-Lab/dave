cmake_minimum_required(VERSION 3.0.2)
project(dave_gazebo_sensors)

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GAXEBO_MSG_INCLUDE_DIRS}
)
# include_directories("include")

set(SENSORS_SRCS
#    src/sensor_name.cc       include/sensor_name.hpp
    src/dave_sensors.cc)

# dave_sensors (passed at STARTUP as a system plugin)
# will need to include local dependencies (if we make any) in last 2 lines
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
add_library(dave_sensors ${SENSORS_SRCS})
target_link_libraries(dave_sensors ${GAZEBO_LIBRARIES})
#add_dependencies(dave_sensors any local dependencies)

# Compile instructions for specific sensors (if required)

catkin_package(
  # INCLUDE_DIRS include
  #LIBRARIES locally defined dependencies
  CATKIN_DEPENDS roscpp gazebo_ros
  DEPENDS
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

