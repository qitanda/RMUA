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
CMAKE_SOURCE_DIR = /home/zhuji/drone_ws/src/camera/include/AAMED

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build

# Include any dependencies generated for this target.
include CMakeFiles/AAMED.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/AAMED.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/AAMED.dir/flags.make

CMakeFiles/AAMED.dir/src/Contours.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/Contours.cpp.o: ../src/Contours.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/AAMED.dir/src/Contours.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/Contours.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/Contours.cpp

CMakeFiles/AAMED.dir/src/Contours.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/Contours.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/Contours.cpp > CMakeFiles/AAMED.dir/src/Contours.cpp.i

CMakeFiles/AAMED.dir/src/Contours.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/Contours.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/Contours.cpp -o CMakeFiles/AAMED.dir/src/Contours.cpp.s

CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o: ../src/EllipseNonMaximumSuppression.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/EllipseNonMaximumSuppression.cpp

CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/EllipseNonMaximumSuppression.cpp > CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.i

CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/EllipseNonMaximumSuppression.cpp -o CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.s

CMakeFiles/AAMED.dir/src/FLED.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/FLED.cpp.o: ../src/FLED.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/AAMED.dir/src/FLED.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/FLED.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED.cpp

CMakeFiles/AAMED.dir/src/FLED.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/FLED.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED.cpp > CMakeFiles/AAMED.dir/src/FLED.cpp.i

CMakeFiles/AAMED.dir/src/FLED.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/FLED.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED.cpp -o CMakeFiles/AAMED.dir/src/FLED.cpp.s

CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o: ../src/FLED_Initialization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_Initialization.cpp

CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_Initialization.cpp > CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.i

CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_Initialization.cpp -o CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.s

CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o: ../src/FLED_PrivateFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_PrivateFunctions.cpp

CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_PrivateFunctions.cpp > CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.i

CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_PrivateFunctions.cpp -o CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.s

CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o: ../src/FLED_drawAndWriteFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_drawAndWriteFunctions.cpp

CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_drawAndWriteFunctions.cpp > CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.i

CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/FLED_drawAndWriteFunctions.cpp -o CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.s

CMakeFiles/AAMED.dir/src/Group.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/Group.cpp.o: ../src/Group.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/AAMED.dir/src/Group.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/Group.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/Group.cpp

CMakeFiles/AAMED.dir/src/Group.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/Group.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/Group.cpp > CMakeFiles/AAMED.dir/src/Group.cpp.i

CMakeFiles/AAMED.dir/src/Group.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/Group.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/Group.cpp -o CMakeFiles/AAMED.dir/src/Group.cpp.s

CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o: ../src/LinkMatrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/LinkMatrix.cpp

CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/LinkMatrix.cpp > CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.i

CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/LinkMatrix.cpp -o CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.s

CMakeFiles/AAMED.dir/src/Node_FC.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/Node_FC.cpp.o: ../src/Node_FC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/AAMED.dir/src/Node_FC.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/Node_FC.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/Node_FC.cpp

CMakeFiles/AAMED.dir/src/Node_FC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/Node_FC.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/Node_FC.cpp > CMakeFiles/AAMED.dir/src/Node_FC.cpp.i

CMakeFiles/AAMED.dir/src/Node_FC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/Node_FC.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/Node_FC.cpp -o CMakeFiles/AAMED.dir/src/Node_FC.cpp.s

CMakeFiles/AAMED.dir/src/Segmentation.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/Segmentation.cpp.o: ../src/Segmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/AAMED.dir/src/Segmentation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/Segmentation.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/Segmentation.cpp

CMakeFiles/AAMED.dir/src/Segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/Segmentation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/Segmentation.cpp > CMakeFiles/AAMED.dir/src/Segmentation.cpp.i

CMakeFiles/AAMED.dir/src/Segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/Segmentation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/Segmentation.cpp -o CMakeFiles/AAMED.dir/src/Segmentation.cpp.s

CMakeFiles/AAMED.dir/src/Validation.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/Validation.cpp.o: ../src/Validation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/AAMED.dir/src/Validation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/Validation.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/Validation.cpp

CMakeFiles/AAMED.dir/src/Validation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/Validation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/Validation.cpp > CMakeFiles/AAMED.dir/src/Validation.cpp.i

CMakeFiles/AAMED.dir/src/Validation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/Validation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/Validation.cpp -o CMakeFiles/AAMED.dir/src/Validation.cpp.s

CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o: ../src/adaptApproxPolyDP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproxPolyDP.cpp

CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproxPolyDP.cpp > CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.i

CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproxPolyDP.cpp -o CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.s

CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o: ../src/adaptApproximateContours.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproximateContours.cpp

CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproximateContours.cpp > CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.i

CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/adaptApproximateContours.cpp -o CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.s

CMakeFiles/AAMED.dir/src/main.cpp.o: CMakeFiles/AAMED.dir/flags.make
CMakeFiles/AAMED.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/AAMED.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AAMED.dir/src/main.cpp.o -c /home/zhuji/drone_ws/src/camera/include/AAMED/src/main.cpp

CMakeFiles/AAMED.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AAMED.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuji/drone_ws/src/camera/include/AAMED/src/main.cpp > CMakeFiles/AAMED.dir/src/main.cpp.i

CMakeFiles/AAMED.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AAMED.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuji/drone_ws/src/camera/include/AAMED/src/main.cpp -o CMakeFiles/AAMED.dir/src/main.cpp.s

# Object files for target AAMED
AAMED_OBJECTS = \
"CMakeFiles/AAMED.dir/src/Contours.cpp.o" \
"CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o" \
"CMakeFiles/AAMED.dir/src/FLED.cpp.o" \
"CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o" \
"CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o" \
"CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o" \
"CMakeFiles/AAMED.dir/src/Group.cpp.o" \
"CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o" \
"CMakeFiles/AAMED.dir/src/Node_FC.cpp.o" \
"CMakeFiles/AAMED.dir/src/Segmentation.cpp.o" \
"CMakeFiles/AAMED.dir/src/Validation.cpp.o" \
"CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o" \
"CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o" \
"CMakeFiles/AAMED.dir/src/main.cpp.o"

# External object files for target AAMED
AAMED_EXTERNAL_OBJECTS =

libAAMED.so: CMakeFiles/AAMED.dir/src/Contours.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/EllipseNonMaximumSuppression.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/FLED.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/FLED_Initialization.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/FLED_PrivateFunctions.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/FLED_drawAndWriteFunctions.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/Group.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/LinkMatrix.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/Node_FC.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/Segmentation.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/Validation.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/adaptApproxPolyDP.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/adaptApproximateContours.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/src/main.cpp.o
libAAMED.so: CMakeFiles/AAMED.dir/build.make
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
libAAMED.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
libAAMED.so: CMakeFiles/AAMED.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX shared library libAAMED.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AAMED.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/AAMED.dir/build: libAAMED.so

.PHONY : CMakeFiles/AAMED.dir/build

CMakeFiles/AAMED.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/AAMED.dir/cmake_clean.cmake
.PHONY : CMakeFiles/AAMED.dir/clean

CMakeFiles/AAMED.dir/depend:
	cd /home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhuji/drone_ws/src/camera/include/AAMED /home/zhuji/drone_ws/src/camera/include/AAMED /home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build /home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build /home/zhuji/drone_ws/src/camera/include/AAMED/cmake-build/CMakeFiles/AAMED.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AAMED.dir/depend

