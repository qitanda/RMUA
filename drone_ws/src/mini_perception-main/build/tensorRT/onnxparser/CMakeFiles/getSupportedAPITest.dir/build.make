# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build

# Include any dependencies generated for this target.
include tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/depend.make

# Include the progress variables for this target.
include tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/progress.make

# Include the compile flags for this target's objects.
include tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/flags.make

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/flags.make
tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o: ../tensorRT/onnxparser/getSupportedAPITest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/getSupportedAPITest.cpp

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.i"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/getSupportedAPITest.cpp > CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.i

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.s"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/getSupportedAPITest.cpp -o CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.s

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/flags.make
tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o: ../tensorRT/onnxparser/ModelImporter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/ModelImporter.cpp

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.i"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/ModelImporter.cpp > CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.i

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.s"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/ModelImporter.cpp -o CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.s

# Object files for target getSupportedAPITest
getSupportedAPITest_OBJECTS = \
"CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o" \
"CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o"

# External object files for target getSupportedAPITest
getSupportedAPITest_EXTERNAL_OBJECTS =

tensorRT/onnxparser/getSupportedAPITest: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/getSupportedAPITest.cpp.o
tensorRT/onnxparser/getSupportedAPITest: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/ModelImporter.cpp.o
tensorRT/onnxparser/getSupportedAPITest: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/build.make
tensorRT/onnxparser/getSupportedAPITest: ../lib/libnvonnxparser_static.a
tensorRT/onnxparser/getSupportedAPITest: ../lib/libonnx_proto.so
tensorRT/onnxparser/getSupportedAPITest: /usr/local/lib/libprotobuf.so
tensorRT/onnxparser/getSupportedAPITest: /usr/local/lib/libprotobuf.so
tensorRT/onnxparser/getSupportedAPITest: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer.so
tensorRT/onnxparser/getSupportedAPITest: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer_plugin.so
tensorRT/onnxparser/getSupportedAPITest: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvonnxparser.so
tensorRT/onnxparser/getSupportedAPITest: tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable getSupportedAPITest"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getSupportedAPITest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/build: tensorRT/onnxparser/getSupportedAPITest

.PHONY : tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/build

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/clean:
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser && $(CMAKE_COMMAND) -P CMakeFiles/getSupportedAPITest.dir/cmake_clean.cmake
.PHONY : tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/clean

tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/depend:
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tensorRT/onnxparser/CMakeFiles/getSupportedAPITest.dir/depend

