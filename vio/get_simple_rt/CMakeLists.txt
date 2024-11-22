cmake_minimum_required(VERSION 2.8 FATAL_ERROR) # #对于cmake版本的最低版本的要求

project(get_simple_rt) #  建立的工程名，例如源代码目录路径的变量名为CH_DIR

#因为PCL是模块化的，也可以如下操作：
#           一个组件  find_package(PCL 1.6 REQUIRED COMPONENTS  io)
#           多个组件  find_package(PCL 1.6 REQUIRED COMPONENTS commom io)
#           所有组件  find_package(PCL 1.6 REQUIRED )   
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp)
target_link_libraries (main ${PCL_LIBRARIES}) 

