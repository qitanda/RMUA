# cmake_minimum_required(VERSION 3.0.2)
# project(ammed_detection)

# set(CMAKE_CXX_STANDARD 11)

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   cv_bridge
#   image_transport
#   eigen_conversions
#   OpenCV
#   Eigen3
#   pcl_ros
#   pcl_conversions
# )


# catkin_package(
# )

# ###########
# ## Build ##
# ###########
# file(GLOB_RECURSE AAMED_H
#      ${CMAKE_SOURCE_DIR}
#     "include/AAMED/src/*.h"
# )
# file(GLOB_RECURSE AAMED_SOURCES
#      ${CMAKE_SOURCE_DIR}
#     "include/AAMED/src/*.cpp"
# )

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
# include_directories(include ${OpenCV_INCLUDE_DIRS})
# include_directories(include ${Eigen3_INCLUDE_DIRS})
# ## Declare a C++ library
# add_library(AAMED1
#   ${AAMED_SOURCES}
#   src/first_detection.cpp

# )

# add_executable(circle_detect 
#   circle_detection.cpp
#   # src/detection.cpp
# )
# target_link_libraries(circle_detect
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${EIGEN3_LIBRARIES} 
#   AAMED1
# )
# add_executable(yellow_detect
#   # src/detection.cpp
#   # red_detection_1.cpp
#   src/yellow_detection.cpp
# )



# target_link_libraries(yellow_detect
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${EIGEN3_LIBRARIES} 
#   AAMED1
# )

# add_executable(white_detect
#   # src/detection.cpp
#   # red_detection_1.cpp
#   src/white_detection.cpp
# )



# target_link_libraries(white_detect
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${EIGEN3_LIBRARIES} 
#   AAMED
# )


