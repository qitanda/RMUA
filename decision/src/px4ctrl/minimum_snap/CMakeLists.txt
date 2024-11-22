cmake_minimum_required(VERSION 3.0.2)
project(minimum_snap)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  eigen_conversions
)

catkin_package(

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(planning_node
  src/main.cpp
  src/poly_planning.cpp
  # include/minimum_snap.cpp
  include/poly_traj.cpp
)
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
