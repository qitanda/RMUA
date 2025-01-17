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
include CMakeFiles/tensorRT.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/tensorRT.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tensorRT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tensorRT.dir/flags.make

CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o: /home/aiden/mini_perception/tensorRT/cuda_tools.cpp
CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o -c /home/aiden/mini_perception/tensorRT/cuda_tools.cpp

CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/cuda_tools.cpp > CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/cuda_tools.cpp -o CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o: /home/aiden/mini_perception/tensorRT/ilogger.cpp
CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o -c /home/aiden/mini_perception/tensorRT/ilogger.cpp

CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/ilogger.cpp > CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/ilogger.cpp -o CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o: /home/aiden/mini_perception/tensorRT/json.cpp
CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o -c /home/aiden/mini_perception/tensorRT/json.cpp

CMakeFiles/tensorRT.dir/tensorRT/json.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/json.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/json.cpp > CMakeFiles/tensorRT.dir/tensorRT/json.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/json.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/json.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/json.cpp -o CMakeFiles/tensorRT.dir/tensorRT/json.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o: /home/aiden/mini_perception/tensorRT/onnxplugin.cpp
CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o -c /home/aiden/mini_perception/tensorRT/onnxplugin.cpp

CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/onnxplugin.cpp > CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/onnxplugin.cpp -o CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o: /home/aiden/mini_perception/tensorRT/plugin_binary_io.cpp
CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o -c /home/aiden/mini_perception/tensorRT/plugin_binary_io.cpp

CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/plugin_binary_io.cpp > CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/plugin_binary_io.cpp -o CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o: CMakeFiles/tensorRT.dir/includes_CUDA.rsp
CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o: /home/aiden/mini_perception/tensorRT/preprocess_kernel.cu
CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CUDA object CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o"
	/usr/local/cuda-11.6/bin/nvcc -forward-unknown-to-host-compiler $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o -MF CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o.d -x cu -rdc=true -c /home/aiden/mini_perception/tensorRT/preprocess_kernel.cu -o CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o

CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o: /home/aiden/mini_perception/tensorRT/trt_builder.cpp
CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o -c /home/aiden/mini_perception/tensorRT/trt_builder.cpp

CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/trt_builder.cpp > CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/trt_builder.cpp -o CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o: /home/aiden/mini_perception/tensorRT/trt_infer.cpp
CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o -c /home/aiden/mini_perception/tensorRT/trt_infer.cpp

CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/trt_infer.cpp > CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/trt_infer.cpp -o CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o: /home/aiden/mini_perception/tensorRT/trt_tensor.cpp
CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o -MF CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o.d -o CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o -c /home/aiden/mini_perception/tensorRT/trt_tensor.cpp

CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aiden/mini_perception/tensorRT/trt_tensor.cpp > CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.i

CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aiden/mini_perception/tensorRT/trt_tensor.cpp -o CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.s

CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o: CMakeFiles/tensorRT.dir/includes_CUDA.rsp
CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o: /home/aiden/mini_perception/tensorRT/plugins/DCNv2.cu
CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CUDA object CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o"
	/usr/local/cuda-11.6/bin/nvcc -forward-unknown-to-host-compiler $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o -MF CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o.d -x cu -rdc=true -c /home/aiden/mini_perception/tensorRT/plugins/DCNv2.cu -o CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o

CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o: CMakeFiles/tensorRT.dir/includes_CUDA.rsp
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o: /home/aiden/mini_perception/tensorRT/plugins/HSigmoid.cu
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CUDA object CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o"
	/usr/local/cuda-11.6/bin/nvcc -forward-unknown-to-host-compiler $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o -MF CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o.d -x cu -rdc=true -c /home/aiden/mini_perception/tensorRT/plugins/HSigmoid.cu -o CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o: CMakeFiles/tensorRT.dir/flags.make
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o: CMakeFiles/tensorRT.dir/includes_CUDA.rsp
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o: /home/aiden/mini_perception/tensorRT/plugins/HSwish.cu
CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o: CMakeFiles/tensorRT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CUDA object CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o"
	/usr/local/cuda-11.6/bin/nvcc -forward-unknown-to-host-compiler $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -MD -MT CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o -MF CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o.d -x cu -rdc=true -c /home/aiden/mini_perception/tensorRT/plugins/HSwish.cu -o CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

# Object files for target tensorRT
tensorRT_OBJECTS = \
"CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o"

# External object files for target tensorRT
tensorRT_EXTERNAL_OBJECTS =

CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/build.make
CMakeFiles/tensorRT.dir/cmake_device_link.o: /home/aiden/mini_perception/lib/libnvonnxparser.so.8.2.1
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libprotobuf.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer_plugin.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libcuda.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/local/cuda-11.6/lib64/libcudart.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/local/cuda-11.6/lib64/libcublas.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /home/aiden/mini_perception/lib/libonnx_proto.a
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libprotobuf.so
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/deviceLinkLibs.rsp
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/deviceObjects1.rsp
CMakeFiles/tensorRT.dir/cmake_device_link.o: CMakeFiles/tensorRT.dir/dlink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking CUDA device code CMakeFiles/tensorRT.dir/cmake_device_link.o"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tensorRT.dir/dlink.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tensorRT.dir/build: CMakeFiles/tensorRT.dir/cmake_device_link.o
.PHONY : CMakeFiles/tensorRT.dir/build

# Object files for target tensorRT
tensorRT_OBJECTS = \
"CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o" \
"CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o"

# External object files for target tensorRT
tensorRT_EXTERNAL_OBJECTS =

/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/cuda_tools.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/ilogger.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/json.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/onnxplugin.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/plugin_binary_io.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/preprocess_kernel.cu.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/trt_builder.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/trt_infer.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/trt_tensor.cpp.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/plugins/DCNv2.cu.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/plugins/HSigmoid.cu.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/tensorRT/plugins/HSwish.cu.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/build.make
/home/aiden/mini_perception/lib/libtensorRT.so: /home/aiden/mini_perception/lib/libnvonnxparser.so.8.2.1
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/aiden/mini_perception/lib/libtensorRT.so: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer.so
/home/aiden/mini_perception/lib/libtensorRT.so: /home/aiden/TensorRT-8.4.3.1/lib/libnvinfer_plugin.so
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libcuda.so
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/local/cuda-11.6/lib64/libcudart.so
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/local/cuda-11.6/lib64/libcublas.so
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /home/aiden/mini_perception/lib/libonnx_proto.a
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/cmake_device_link.o
/home/aiden/mini_perception/lib/libtensorRT.so: CMakeFiles/tensorRT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aiden/mini_perception/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking CXX shared library /home/aiden/mini_perception/lib/libtensorRT.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tensorRT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tensorRT.dir/build: /home/aiden/mini_perception/lib/libtensorRT.so
.PHONY : CMakeFiles/tensorRT.dir/build

CMakeFiles/tensorRT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tensorRT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tensorRT.dir/clean

CMakeFiles/tensorRT.dir/depend:
	cd /home/aiden/mini_perception/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aiden/mini_perception /home/aiden/mini_perception /home/aiden/mini_perception/build /home/aiden/mini_perception/build /home/aiden/mini_perception/build/CMakeFiles/tensorRT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tensorRT.dir/depend

