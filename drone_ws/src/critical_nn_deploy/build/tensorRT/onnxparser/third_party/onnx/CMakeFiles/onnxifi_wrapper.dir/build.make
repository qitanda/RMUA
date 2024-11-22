# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/aiden/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/aiden/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aiden/mini_perception

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aiden/mini_perception/build

# Include any dependencies generated for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/compiler_depend.make

# Include the progress variables for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/progress.make

# Include the compile flags for this target's objects.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/flags.make

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/flags.make
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o: /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_wrapper.c
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o -MF CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o.d -o CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o -c /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_wrapper.c

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.i"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_wrapper.c > CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.i

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.s"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_wrapper.c -o CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.s

# Object files for target onnxifi_wrapper
onnxifi_wrapper_OBJECTS = \
"CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o"

# External object files for target onnxifi_wrapper
onnxifi_wrapper_EXTERNAL_OBJECTS =

/home/aiden/mini_perception/lib/libonnxifi.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/onnx/onnxifi_wrapper.c.o
/home/aiden/mini_perception/lib/libonnxifi.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/build.make
/home/aiden/mini_perception/lib/libonnxifi.so: /home/aiden/mini_perception/lib/libonnxifi_loader.a
/home/aiden/mini_perception/lib/libonnxifi.so: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared module /home/aiden/mini_perception/lib/libonnxifi.so"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onnxifi_wrapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/build: /home/aiden/mini_perception/lib/libonnxifi.so
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/build

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/clean:
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -P CMakeFiles/onnxifi_wrapper.dir/cmake_clean.cmake
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/clean

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/depend:
	cd /home/aiden/mini_perception/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aiden/mini_perception /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx /home/aiden/mini_perception/build /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_wrapper.dir/depend
