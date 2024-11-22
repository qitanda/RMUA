cmake_minimum_required(VERSION 3.0.2)
project(px4ctrl)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  mavros
  eigen_conversions
  airsim_ros
)

catkin_package(

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(px4ctrl_node
  src/main.cpp
  src/px4ctrl.cpp
  include/controller.cpp
  include/param.cpp
)
add_dependencies(px4ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
