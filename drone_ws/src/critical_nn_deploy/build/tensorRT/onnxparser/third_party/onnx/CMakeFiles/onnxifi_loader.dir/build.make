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
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/compiler_depend.make

# Include the progress variables for this target.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/progress.make

# Include the compile flags for this target's objects.
include tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/flags.make

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/flags.make
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o: /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_loader.c
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o -MF CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o.d -o CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o -c /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_loader.c

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.i"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_loader.c > CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.i

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.s"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx/onnx/onnxifi_loader.c -o CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.s

# Object files for target onnxifi_loader
onnxifi_loader_OBJECTS = \
"CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o"

# External object files for target onnxifi_loader
onnxifi_loader_EXTERNAL_OBJECTS =

/home/aiden/mini_perception/lib/libonnxifi_loader.a: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/onnx/onnxifi_loader.c.o
/home/aiden/mini_perception/lib/libonnxifi_loader.a: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/build.make
/home/aiden/mini_perception/lib/libonnxifi_loader.a: tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library /home/aiden/mini_perception/lib/libonnxifi_loader.a"
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -P CMakeFiles/onnxifi_loader.dir/cmake_clean_target.cmake
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onnxifi_loader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/build: /home/aiden/mini_perception/lib/libonnxifi_loader.a
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/build

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/clean:
	cd /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx && $(CMAKE_COMMAND) -P CMakeFiles/onnxifi_loader.dir/cmake_clean.cmake
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/clean

tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/depend:
	cd /home/aiden/mini_perception/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aiden/mini_perception /home/aiden/mini_perception/tensorRT/onnxparser/third_party/onnx /home/aiden/mini_perception/build /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx /home/aiden/mini_perception/build/tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tensorRT/onnxparser/third_party/onnx/CMakeFiles/onnxifi_loader.dir/depend

