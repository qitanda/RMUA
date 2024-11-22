cmake_minimum_required(VERSION 3.10)
project(imgwarp LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(imgwarp imgwarp.cpp)
target_link_libraries(imgwarp PUBLIC ${OpenCV_LIBS})
