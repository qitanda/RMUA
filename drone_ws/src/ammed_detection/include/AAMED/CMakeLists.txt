cmake_minimum_required(VERSION 3.16.3)
project(AAMED VERSION 1.0)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV
  cv_bridge
  image_transport
  eigen_conversions
)

include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

include_directories("src")
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp")
list(REMOVE_ITEM PROJECT_SOURCES "src/main.cpp")


add_executable(AAMEDemo ${PROJECT_SOURCES} "src/main.cpp")
add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
target_link_libraries(AAMEDemo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
)

