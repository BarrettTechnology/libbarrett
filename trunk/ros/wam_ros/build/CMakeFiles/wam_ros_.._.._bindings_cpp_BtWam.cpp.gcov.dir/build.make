# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/libbarrett/ros/wam_ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/libbarrett/ros/wam_ros/build

# Utility rule file for wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.

CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov:
	cd /home/robot/libbarrett/ros/wam_ros && rosgcov ../../bindings/cpp/BtWam.cpp /home/robot/libbarrett/ros/wam_ros/build

wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov: CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov
wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov: CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/build.make
.PHONY : wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov

# Rule to build all files generated by this target.
CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/build: wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov
.PHONY : CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/build

CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/cmake_clean.cmake
.PHONY : CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/clean

CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/depend:
	cd /home/robot/libbarrett/ros/wam_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/libbarrett/ros/wam_ros /home/robot/libbarrett/ros/wam_ros /home/robot/libbarrett/ros/wam_ros/build /home/robot/libbarrett/ros/wam_ros/build /home/robot/libbarrett/ros/wam_ros/build/CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/wam_ros_.._.._bindings_cpp_BtWam.cpp.gcov.dir/depend

