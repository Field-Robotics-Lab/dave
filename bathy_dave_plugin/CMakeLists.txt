cmake_minimum_required(VERSION 2.8.3)
project(bathy_dave_plugin)

add_compile_options(-std=c++11)

find_package(GDAL REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   gazebo_ros
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} /usr/local/lib/)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gearth_dave_plugin src/gearth_plugin.cc)
target_link_libraries(gearth_dave_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${GDAL_LIBRARY}
  )
add_dependencies(gearth_dave_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(bathy_dave_plugin src/bathy_plugin.cc)
target_link_libraries(bathy_dave_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${GDAL_LIBRARY}
  )
add_dependencies(bathy_dave_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


catkin_package(
LIBRARIES gearth_dave_plugin bathy_dave_plugin
)
