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
include CMakeFiles/imgwarp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/imgwarp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/imgwarp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imgwarp.dir/flags.make

CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o: CMakeFiles/imgwarp.dir/flags.make
CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o: /home/aiden/mini_perception/imgwarp/imgwarp.cpp
CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o: CMakeFiles/imgwarp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o -MF CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o.d -o CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o -c /home/aiden/mini_perception/imgwarp/imgwarp.cpp

CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/imgwarp/imgwarp.cpp > CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.i

CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/imgwarp/imgwarp.cpp -o CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.s

# Object files for target imgwarp
imgwarp_OBJECTS = \
"CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o"

# External object files for target imgwarp
imgwarp_EXTERNAL_OBJECTS =

/home/aiden/mini_perception/lib/libimgwarp.a: CMakeFiles/imgwarp.dir/imgwarp/imgwarp.cpp.o
/home/aiden/mini_perception/lib/libimgwarp.a: CMakeFiles/imgwarp.dir/build.make
/home/aiden/mini_perception/lib/libimgwarp.a: CMakeFiles/imgwarp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/aiden/mini_perception/lib/libimgwarp.a"
	$(CMAKE_COMMAND) -P CMakeFiles/imgwarp.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imgwarp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imgwarp.dir/build: /home/aiden/mini_perception/lib/libimgwarp.a
.PHONY : CMakeFiles/imgwarp.dir/build

CMakeFiles/imgwarp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imgwarp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imgwarp.dir/clean

CMakeFiles/imgwarp.dir/depend:
	cd /home/aiden/mini_perception/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aiden/mini_perception /home/aiden/mini_perception /home/aiden/mini_perception/build /home/aiden/mini_perception/build /home/aiden/mini_perception/build/CMakeFiles/imgwarp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imgwarp.dir/depend

