# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/peter/babyJaws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/babyJaws/build

# Include any dependencies generated for this target.
include rov/CMakeFiles/lights_node.dir/depend.make

# Include the progress variables for this target.
include rov/CMakeFiles/lights_node.dir/progress.make

# Include the compile flags for this target's objects.
include rov/CMakeFiles/lights_node.dir/flags.make

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o: rov/CMakeFiles/lights_node.dir/flags.make
rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o: /home/peter/babyJaws/src/rov/src/robotLights.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/peter/babyJaws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o"
	cd /home/peter/babyJaws/build/rov && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lights_node.dir/src/robotLights.cpp.o -c /home/peter/babyJaws/src/rov/src/robotLights.cpp

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lights_node.dir/src/robotLights.cpp.i"
	cd /home/peter/babyJaws/build/rov && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/peter/babyJaws/src/rov/src/robotLights.cpp > CMakeFiles/lights_node.dir/src/robotLights.cpp.i

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lights_node.dir/src/robotLights.cpp.s"
	cd /home/peter/babyJaws/build/rov && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/peter/babyJaws/src/rov/src/robotLights.cpp -o CMakeFiles/lights_node.dir/src/robotLights.cpp.s

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.requires:
.PHONY : rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.requires

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.provides: rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.requires
	$(MAKE) -f rov/CMakeFiles/lights_node.dir/build.make rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.provides.build
.PHONY : rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.provides

rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.provides.build: rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o

# Object files for target lights_node
lights_node_OBJECTS = \
"CMakeFiles/lights_node.dir/src/robotLights.cpp.o"

# External object files for target lights_node
lights_node_EXTERNAL_OBJECTS =

/home/peter/babyJaws/devel/lib/rov/lights_node: rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o
/home/peter/babyJaws/devel/lib/rov/lights_node: rov/CMakeFiles/lights_node.dir/build.make
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libimage_transport.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libmessage_filters.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libclass_loader.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/libPocoFoundation.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libroscpp.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libxmlrpcpp.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libroslib.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libcv_bridge.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/librosconsole.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/librosconsole_log4cxx.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/librosconsole_backend_interface.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/liblog4cxx.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libroscpp_serialization.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/librostime.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /opt/ros/jade/lib/libcpp_common.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/peter/babyJaws/devel/lib/rov/lights_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/peter/babyJaws/devel/lib/rov/lights_node: rov/CMakeFiles/lights_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/peter/babyJaws/devel/lib/rov/lights_node"
	cd /home/peter/babyJaws/build/rov && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lights_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rov/CMakeFiles/lights_node.dir/build: /home/peter/babyJaws/devel/lib/rov/lights_node
.PHONY : rov/CMakeFiles/lights_node.dir/build

rov/CMakeFiles/lights_node.dir/requires: rov/CMakeFiles/lights_node.dir/src/robotLights.cpp.o.requires
.PHONY : rov/CMakeFiles/lights_node.dir/requires

rov/CMakeFiles/lights_node.dir/clean:
	cd /home/peter/babyJaws/build/rov && $(CMAKE_COMMAND) -P CMakeFiles/lights_node.dir/cmake_clean.cmake
.PHONY : rov/CMakeFiles/lights_node.dir/clean

rov/CMakeFiles/lights_node.dir/depend:
	cd /home/peter/babyJaws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/babyJaws/src /home/peter/babyJaws/src/rov /home/peter/babyJaws/build /home/peter/babyJaws/build/rov /home/peter/babyJaws/build/rov/CMakeFiles/lights_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rov/CMakeFiles/lights_node.dir/depend

