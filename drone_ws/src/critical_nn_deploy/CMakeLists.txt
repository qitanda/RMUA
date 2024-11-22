cmake_minimum_required(VERSION 3.10)
project(critical_nn_deploy LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(TENSORRT_ROOT /home/criticalhit/Downloads/TENSORRT/TensorRT-8.4.0.6)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.3)
set(CUDNN_TOOLKIT_ROOT_DIR /home/criticalhit/Downloads/cudnn-linux-x86_64-8.4.0.27_cuda11.6-archive)
# LibYAML
find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package (yaml-cpp)
if (yaml-cpp_FOUND)
  message("YAML Libs: ${YAML_CPP_LIBRARIES}")
  message("YAML Headers: ${YAML_CPP_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "yaml-cpp NOT FOUND\n")
endif (yaml-cpp_FOUND)

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV headers at ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Found OpenCV libs at ${OpenCV_LIBS}")

# Protobuf
find_package(Protobuf REQUIRED)
message(STATUS "Found Protobuf headers at ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Found Protobuf libs at ${PROTOBUF_LIBRARY}")

# CUDA
find_path(CUDA_INCLUDE_DIR cuda_runtime.h
  HINTS ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include
)
find_library(CUDA_LIB cuda
  HINTS ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUDART_LIB cudart
  HINTS ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(CUBLAS_LIB cublas
  HINTS ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
# find_library(CUDNN_LIB cudnn
#   HINTS ${CUDNN_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)

include_directories(${CUDNN_TOOLKIT_ROOT_DIR}/include)
link_directories(${CUDNN_TOOLKIT_ROOT_DIR}/lib)

set(CUDA_LIBRARY ${CUDA_LIB} ${CUDART_LIB} ${CUBLAS_LIB})
message(STATUS "Found CUDA libs at ${CUDA_INCLUDE_DIR}")
message(STATUS "Found CUDA libs at ${CUDA_LIBRARY}")

set(TENSORRT_LIBRARY 
  ${TENSORRT_ROOT}/lib/libnvinfer.so 
  ${TENSORRT_ROOT}/lib/libnvinfer_plugin.so
  ${TENSORRT_ROOT}/lib/libnvonnxparser.so
  # ${TENSORRT_ROOT}/lib/libnvinfer_builder_resource.so.8.4.3
)
set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  eigen_conversions
  message_generation
  airsim_ros
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  pcl_handle
  AAMED
  kalmanfilter
)
# add_message_files(
#    FILES
#   Circle.msg
#  )
# generate_messages(
#    DEPENDENCIES
#    std_msgs
#    sensor_msgs
# )

catkin_package(
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/common
  ${PROJECT_SOURCE_DIR}/tensorRT
  ${PROJECT_SOURCE_DIR}/tensorRT/onnxparser
  ${PROJECT_SOURCE_DIR}/rangenet
  ${OpenCV_INCLUDE_DIRS} 
  ${Protobuf_INCLUDE_DIRS} 
  ${CUDA_INCLUDE_DIR} 
  ${TENSORRT_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  /usr/local/eigen3
  
)

set(PLUGIN_SOURCES
  tensorRT/plugins/DCNv2.cu
  tensorRT/plugins/HSigmoid.cu
  tensorRT/plugins/HSwish.cu
)

set(TRTINFER_SOURCES
  tensorRT/cuda_tools.cpp
  tensorRT/ilogger.cpp
  tensorRT/json.cpp
  tensorRT/onnxplugin.cpp
  tensorRT/plugin_binary_io.cpp
  tensorRT/preprocess_kernel.cu
  tensorRT/trt_builder.cpp
  tensorRT/trt_infer.cpp
  tensorRT/trt_tensor.cpp
)

add_definitions(-w)

add_subdirectory(tensorRT/onnxparser)

add_library(tensorRT SHARED ${TRTINFER_SOURCES} ${PLUGIN_SOURCES})
target_include_directories(tensorRT PUBLIC ${OpenCV_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS} ${CUDA_INCLUDE_DIR} ${TENSORRT_INCLUDE_DIR})
target_link_libraries(tensorRT PUBLIC nvonnxparser ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS})

set_target_properties(tensorRT PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(tensorRT PUBLIC cxx_std_11)
set_target_properties(tensorRT PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET tensorRT PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_library(tensorRT_static STATIC ${TRTINFER_SOURCES})
target_include_directories(tensorRT_static PUBLIC ${OpenCV_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS} ${CUDA_INCLUDE_DIR} ${TENSORRT_INCLUDE_DIR})
target_link_libraries(tensorRT_static PUBLIC nvonnxparser ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS})

set_target_properties(tensorRT_static PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(tensorRT_static PUBLIC cxx_std_11)
set_target_properties(tensorRT_static PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET tensorRT_static PROPERTY CUDA_SEPARABLE_COMPILATION ON)

# add_subdirectory(imgwarp)

add_library(imgwarp imgwarp/imgwarp.cpp)
target_link_libraries(imgwarp PUBLIC ${OpenCV_LIBS})

file(GLOB_RECURSE AAMED_H
     ${CMAKE_SOURCE_DIR}
    "AAMED/src/*.h"
)
file(GLOB_RECURSE AAMED_SOURCES
     ${CMAKE_SOURCE_DIR}
    "AAMED/src/*.cpp"
)
add_library(AAMED
  ${AAMED_SOURCES}
  circle_detection/first_detection.cpp
)

add_executable(stereo_depth_node
  main.cpp
  pcl_handle/pcl_handle.cpp
  circle_detection/first_detection.cpp
  kalmanfilter/kalman_filter.cpp
)

target_link_libraries(stereo_depth_node 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  tensorRT
  imgwarp
  AAMED
  )

set_target_properties(stereo_depth_node PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(stereo_depth_node PUBLIC cxx_std_11)
set_target_properties(stereo_depth_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET stereo_depth_node PROPERTY CUDA_SEPARABLE_COMPILATION ON)
