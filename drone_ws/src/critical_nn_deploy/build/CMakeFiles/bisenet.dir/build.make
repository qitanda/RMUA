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
include CMakeFiles/bisenet.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/bisenet.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/bisenet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bisenet.dir/flags.make

CMakeFiles/bisenet.dir/bisenet.cpp.o: CMakeFiles/bisenet.dir/flags.make
CMakeFiles/bisenet.dir/bisenet.cpp.o: /home/aiden/mini_perception/bisenet.cpp
CMakeFiles/bisenet.dir/bisenet.cpp.o: CMakeFiles/bisenet.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bisenet.dir/bisenet.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/bisenet.dir/bisenet.cpp.o -MF CMakeFiles/bisenet.dir/bisenet.cpp.o.d -o CMakeFiles/bisenet.dir/bisenet.cpp.o -c /home/aiden/mini_perception/bisenet.cpp

CMakeFiles/bisenet.dir/bisenet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bisenet.dir/bisenet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/bisenet.cpp > CMakeFiles/bisenet.dir/bisenet.cpp.i

CMakeFiles/bisenet.dir/bisenet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bisenet.dir/bisenet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/bisenet.cpp -o CMakeFiles/bisenet.dir/bisenet.cpp.s

# Object files for target bisenet
bisenet_OBJECTS = \
"CMakeFiles/bisenet.dir/bisenet.cpp.o"

# External object files for target bisenet
bisenet_EXTERNAL_OBJECTS =

bisenet: CMakeFiles/bisenet.dir/bisenet.cpp.o
bisenet: CMakeFiles/bisenet.dir/build.make
bisenet: /usr/lib/x86_64-linux-gnu/libprotobuf.so
bisenet: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer.so
bisenet: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer_plugin.so
bisenet: /usr/lib/x86_64-linux-gnu/libcuda.so
bisenet: /usr/local/cuda-11.6/lib64/libcudart.so
bisenet: /usr/local/cuda-11.6/lib64/libcublas.so
bisenet: /home/aiden/mini_perception/lib/libtensorRT.so
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
bisenet: /usr/lib/x86_64-linux-gnu/libcuda.so
bisenet: /usr/local/cuda-11.6/lib64/libcudart.so
bisenet: /usr/local/cuda-11.6/lib64/libcublas.so
bisenet: /home/aiden/mini_perception/lib/libnvonnxparser.so.8.2.1
bisenet: /usr/lib/x86_64-linux-gnu/libprotobuf.so
bisenet: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer.so
bisenet: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer_plugin.so
bisenet: /home/aiden/mini_perception/lib/libonnx_proto.a
bisenet: /usr/lib/x86_64-linux-gnu/libprotobuf.so
bisenet: CMakeFiles/bisenet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bisenet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bisenet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bisenet.dir/build: bisenet
.PHONY : CMakeFiles/bisenet.dir/build

CMakeFiles/bisenet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bisenet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bisenet.dir/clean

CMakeFiles/bisenet.dir/depend:
	cd /home/aiden/mini_perception/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aiden/mini_perception /home/aiden/mini_perception /home/aiden/mini_perception/build /home/aiden/mini_perception/build /home/aiden/mini_perception/build/CMakeFiles/bisenet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bisenet.dir/depend

