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
CMAKE_SOURCE_DIR = /home/acp/catkin_ws/L3CamSimulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/acp/catkin_ws/L3CamSimulator/build

# Include any dependencies generated for this target.
include CMakeFiles/l3cam_simulator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/l3cam_simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/l3cam_simulator.dir/flags.make

CMakeFiles/l3cam_simulator.dir/main.cpp.o: CMakeFiles/l3cam_simulator.dir/flags.make
CMakeFiles/l3cam_simulator.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/acp/catkin_ws/L3CamSimulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/l3cam_simulator.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/l3cam_simulator.dir/main.cpp.o -c /home/acp/catkin_ws/L3CamSimulator/main.cpp

CMakeFiles/l3cam_simulator.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/l3cam_simulator.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/acp/catkin_ws/L3CamSimulator/main.cpp > CMakeFiles/l3cam_simulator.dir/main.cpp.i

CMakeFiles/l3cam_simulator.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/l3cam_simulator.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/acp/catkin_ws/L3CamSimulator/main.cpp -o CMakeFiles/l3cam_simulator.dir/main.cpp.s

CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o: CMakeFiles/l3cam_simulator.dir/flags.make
CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o: ../udpSender.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/acp/catkin_ws/L3CamSimulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o -c /home/acp/catkin_ws/L3CamSimulator/udpSender.cpp

CMakeFiles/l3cam_simulator.dir/udpSender.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/l3cam_simulator.dir/udpSender.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/acp/catkin_ws/L3CamSimulator/udpSender.cpp > CMakeFiles/l3cam_simulator.dir/udpSender.cpp.i

CMakeFiles/l3cam_simulator.dir/udpSender.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/l3cam_simulator.dir/udpSender.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/acp/catkin_ws/L3CamSimulator/udpSender.cpp -o CMakeFiles/l3cam_simulator.dir/udpSender.cpp.s

# Object files for target l3cam_simulator
l3cam_simulator_OBJECTS = \
"CMakeFiles/l3cam_simulator.dir/main.cpp.o" \
"CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o"

# External object files for target l3cam_simulator
l3cam_simulator_EXTERNAL_OBJECTS =

l3cam_simulator: CMakeFiles/l3cam_simulator.dir/main.cpp.o
l3cam_simulator: CMakeFiles/l3cam_simulator.dir/udpSender.cpp.o
l3cam_simulator: CMakeFiles/l3cam_simulator.dir/build.make
l3cam_simulator: /usr/local/lib/libopencv_gapi.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_stitching.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_alphamat.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_aruco.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_barcode.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_bgsegm.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_bioinspired.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_ccalib.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_dnn_superres.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_dpm.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_face.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_freetype.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_fuzzy.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_hdf.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_hfs.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_img_hash.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_intensity_transform.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_line_descriptor.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_mcc.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_quality.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_rapid.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_reg.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_rgbd.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_saliency.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_stereo.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_structured_light.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_superres.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_surface_matching.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_tracking.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_videostab.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_viz.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_xfeatures2d.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_xobjdetect.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_xphoto.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_shape.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_highgui.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_datasets.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_plot.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_text.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_ml.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_optflow.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_ximgproc.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_video.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_videoio.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_dnn.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_imgcodecs.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_objdetect.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_calib3d.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_features2d.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_flann.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_photo.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_imgproc.so.4.5.2
l3cam_simulator: /usr/local/lib/libopencv_core.so.4.5.2
l3cam_simulator: CMakeFiles/l3cam_simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/acp/catkin_ws/L3CamSimulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable l3cam_simulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/l3cam_simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/l3cam_simulator.dir/build: l3cam_simulator

.PHONY : CMakeFiles/l3cam_simulator.dir/build

CMakeFiles/l3cam_simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/l3cam_simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/l3cam_simulator.dir/clean

CMakeFiles/l3cam_simulator.dir/depend:
	cd /home/acp/catkin_ws/L3CamSimulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/acp/catkin_ws/L3CamSimulator /home/acp/catkin_ws/L3CamSimulator /home/acp/catkin_ws/L3CamSimulator/build /home/acp/catkin_ws/L3CamSimulator/build /home/acp/catkin_ws/L3CamSimulator/build/CMakeFiles/l3cam_simulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/l3cam_simulator.dir/depend

