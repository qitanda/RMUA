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
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend.make

# Include the progress variables for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/progress.make

# Include the compile flags for this target's objects.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/flags.make

tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/local/bin/protoc /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto -I /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx --cpp_out dllexport_decl=ONNX_API:/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx

tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.h: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.h

tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Running C++ protocol buffer compiler on /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/local/bin/protoc /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto -I /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx --cpp_out dllexport_decl=ONNX_API:/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx

tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.h: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.h

tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Running C++ protocol buffer compiler on /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/local/bin/protoc /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto -I /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx --cpp_out dllexport_decl=ONNX_API:/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx

tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.h: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.h

tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto: ../tensorRT/onnxparser/third_party/onnx/onnx/onnx.in.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Running gen_proto.py on onnx/onnx.in.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/python3 /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/third_party/onnx/onnx/gen_proto.py -p onnx2trt_onnx -o /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx onnx -m

tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto: ../tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators.in.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Running gen_proto.py on onnx/onnx-operators.in.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/python3 /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/third_party/onnx/onnx/gen_proto.py -p onnx2trt_onnx -o /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx onnx-operators -m

tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto: ../tensorRT/onnxparser/third_party/onnx/onnx/onnx-data.in.proto
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Running gen_proto.py on onnx/onnx-data.in.proto"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/python3 /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/third_party/onnx/onnx/gen_proto.py -p onnx2trt_onnx -o /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx onnx-data -m

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/flags.make
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.i"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc > CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.i

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.s"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc -o CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.s

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/flags.make
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.i"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc > CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.i

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.s"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc -o CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.s

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/flags.make
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.i"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc > CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.i

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.s"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc -o CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.s

# Object files for target onnx_proto
onnx_proto_OBJECTS = \
"CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o" \
"CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o" \
"CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o"

# External object files for target onnx_proto
onnx_proto_EXTERNAL_OBJECTS =

../lib/libonnx_proto.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx_onnx2trt_onnx-ml.pb.cc.o
../lib/libonnx_proto.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc.o
../lib/libonnx_proto.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/onnx/onnx-data_onnx2trt_onnx.pb.cc.o
../lib/libonnx_proto.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/build.make
../lib/libonnx_proto.so: /usr/local/lib/libprotobuf.so
../lib/libonnx_proto.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX shared library ../../../../../lib/libonnx_proto.so"
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onnx_proto.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/build: ../lib/libonnx_proto.so

.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/build

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/clean:
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -P CMakeFiles/onnx_proto.dir/cmake_clean.cmake
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/clean

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.cc
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.pb.h
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.cc
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.pb.h
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.cc
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.pb.h
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx_onnx2trt_onnx-ml.proto
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-operators_onnx2trt_onnx-ml.proto
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend: tensorRT/onnxparser/third_party/onnx/onnx/onnx-data_onnx2trt_onnx.proto
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/tensorRT/onnxparser/third_party/onnx /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnx_proto.dir/depend

