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
include CMakeFiles/bisenet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bisenet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bisenet.dir/flags.make

CMakeFiles/bisenet.dir/bisenet.cpp.o: CMakeFiles/bisenet.dir/flags.make
CMakeFiles/bisenet.dir/bisenet.cpp.o: ../bisenet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bisenet.dir/bisenet.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bisenet.dir/bisenet.cpp.o -c /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/bisenet.cpp

CMakeFiles/bisenet.dir/bisenet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bisenet.dir/bisenet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/bisenet.cpp > CMakeFiles/bisenet.dir/bisenet.cpp.i

CMakeFiles/bisenet.dir/bisenet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bisenet.dir/bisenet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/bisenet.cpp -o CMakeFiles/bisenet.dir/bisenet.cpp.s

# Object files for target bisenet
bisenet_OBJECTS = \
"CMakeFiles/bisenet.dir/bisenet.cpp.o"

# External object files for target bisenet
bisenet_EXTERNAL_OBJECTS =

bisenet: CMakeFiles/bisenet.dir/bisenet.cpp.o
bisenet: CMakeFiles/bisenet.dir/build.make
bisenet: /usr/local/lib/libprotobuf.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer_plugin.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvonnxparser.so
bisenet: /usr/lib/x86_64-linux-gnu/libcuda.so
bisenet: /usr/local/cuda-11.6/lib64/libcudart.so
bisenet: /usr/local/cuda-11.6/lib64/libcublas.so
bisenet: ../lib/libtensorRT.so
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
bisenet: ../lib/libnvonnxparser.so.8.2.1
bisenet: /usr/local/lib/libprotobuf.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvinfer_plugin.so
bisenet: /home/rmuc3/Downloads/TensorRT-8.4.0.6.Linux.x86_64-gnu.cuda-11.6.cudnn8.3/TensorRT-8.4.0.6/lib/libnvonnxparser.so
bisenet: ../lib/libonnx_proto.so
bisenet: /usr/local/lib/libprotobuf.so
bisenet: CMakeFiles/bisenet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bisenet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bisenet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bisenet.dir/build: bisenet

.PHONY : CMakeFiles/bisenet.dir/build

CMakeFiles/bisenet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bisenet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bisenet.dir/clean

CMakeFiles/bisenet.dir/depend:
	cd /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build /home/rmuc3/zj_ws/drone_ws/src/mini_perception-main/build/CMakeFiles/bisenet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bisenet.dir/depend

