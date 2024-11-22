cmake_minimum_required(VERSION 3.0.2)
project(airsim_bridge)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

## include blasfeo and hpipm, assumed to be installed in "/opt"
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  eigen_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES control_demo
CATKIN_DEPENDS 
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_debug
  odom_debug.cpp
  )
target_link_libraries(odom_debug
  ${catkin_LIBRARIES} )
add_dependencies(odom_debug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(waypointer
  waypoint_node.cpp
  include/path_generator.cpp)
target_link_libraries(waypointer
  ${catkin_LIBRARIES} )
add_dependencies(waypointer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
