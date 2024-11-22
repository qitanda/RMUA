cmake_minimum_required(VERSION 3.10)
project(NNDeploy LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(TENSORRT_ROOT /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.6)
set(CUDNN_TOOLKIT_ROOT_DIR /home/rmuc3/Downloads/cudnn-linux-x86_64-8.4.0.27_cuda11.6-archive)

# LibYAML
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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  eigen_conversions
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
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

# TensorRT
# find_path(TENSORRT_INCLUDE_DIR NvInfer.h
#   HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES include)
# message(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")
# find_library(TENSORRT_LIBRARY_INFER nvinfer
#   HINTS ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)
# find_library(TENSORRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
#   HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
#   PATH_SUFFIXES lib lib64 lib/x64)
# set(TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_INFER_PLUGIN})
# message(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")
# find_package_handle_standard_args(
#   TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY)
# if(NOT TENSORRT_FOUND)
#   message(ERROR " Cannot find TensorRT library.")
# endif()

set(TENSORRT_LIBRARY 
  ${TENSORRT_ROOT}/lib/libnvinfer.so 
  ${TENSORRT_ROOT}/lib/libnvinfer_plugin.so
  ${TENSORRT_ROOT}/lib/libnvonnxparser.so
  # ${TENSORRT_ROOT}/lib/libnvinfer_builder_resource.so.8.4.3
)
set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)

# link_libraries(
#   ${TENSORRT_ROOT}/lib/libnvinfer.so 
#   ${TENSORRT_ROOT}/lib/libnvinfer_plugin.so
#   /home/rmuc3/Downloads/TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4/TensorRT-8.4.3.1/lib/libnvinfer_builder_resource.so.8.4.3
# )

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


# add_executable(inference
#   main.cpp  
#   unet/unet.cpp
#   yolo/yolo_decode.cu
#   yolo/yolo.cpp)

# target_link_libraries(inference 
#   ${OpenCV_LIBS}
#   # nvinfer nvinfer_plugin cuda cublas cudart cudnn 
#   tensorRT
#   protobuf)

# set_target_properties(inference PROPERTIES CUDA_ARCHITECTURES "75")
# target_compile_features(inference PUBLIC cxx_std_11)
# set_target_properties(inference PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
# set_property(TARGET inference PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(stereo
  stereo.cpp
  stereo/stereo.cpp)

target_link_libraries(stereo 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS}
  tensorRT
  imgwarp
  )

set_target_properties(stereo PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(stereo PUBLIC cxx_std_11)
set_target_properties(stereo PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET stereo PROPERTY CUDA_SEPARABLE_COMPILATION ON)


add_executable(lanedet
  lanedet.cpp
  lanedet/lanedet.cpp)

target_link_libraries(lanedet 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS}
  tensorRT
  imgwarp
  )

set_target_properties(lanedet PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(lanedet PUBLIC cxx_std_11)
set_target_properties(lanedet PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET lanedet PROPERTY CUDA_SEPARABLE_COMPILATION ON)



add_executable(stereo2
stereo2.cpp)

target_link_libraries(stereo2 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS}
  tensorRT
  imgwarp
  )

set_target_properties(stereo2 PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(stereo2 PUBLIC cxx_std_11)
set_target_properties(stereo2 PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET stereo2 PROPERTY CUDA_SEPARABLE_COMPILATION ON)


add_executable(detector
detector.cpp
yolo/yolo.cpp
yolo/yolo_decode.cu
unet/unet.cpp
stereo/stereo.cpp)

target_link_libraries(detector 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS}
  tensorRT
  )

set_target_properties(detector PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(detector PUBLIC cxx_std_11)
set_target_properties(detector PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET detector PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(bisenet
  bisenet.cpp)

target_link_libraries(bisenet 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS}
  tensorRT
  )

set_target_properties(bisenet PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(bisenet PUBLIC cxx_std_11)
set_target_properties(bisenet PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET bisenet PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(rangenet
  rangenet.cpp
  # rangenet/net.cpp
  # rangenet/netTensorRT.cpp
  # rangenet/selector.cpp
)

target_link_libraries(rangenet 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
  tensorRT
  )

set_target_properties(rangenet PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(rangenet PUBLIC cxx_std_11)
set_target_properties(rangenet PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET rangenet PROPERTY CUDA_SEPARABLE_COMPILATION ON)


add_executable(superpoint
  superpoint.cpp
)

target_link_libraries(superpoint 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARY} ${TENSORRT_LIBRARY} ${CUDA_LIBRARY} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
  tensorRT
  )

set_target_properties(superpoint PROPERTIES CUDA_ARCHITECTURES "75")
target_compile_features(superpoint PUBLIC cxx_std_11)
set_target_properties(superpoint PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET superpoint PROPERTY CUDA_SEPARABLE_COMPILATION ON)