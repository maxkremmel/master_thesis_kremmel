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
CMAKE_SOURCE_DIR = /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp

# Include any dependencies generated for this target.
include kiss_icp/pipeline/CMakeFiles/pipeline.dir/depend.make

# Include the progress variables for this target.
include kiss_icp/pipeline/CMakeFiles/pipeline.dir/progress.make

# Include the compile flags for this target's objects.
include kiss_icp/pipeline/CMakeFiles/pipeline.dir/flags.make

kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.o: kiss_icp/pipeline/CMakeFiles/pipeline.dir/flags.make
kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.o: /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.o"
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pipeline.dir/KissICP.cpp.o -c /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp

kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pipeline.dir/KissICP.cpp.i"
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp > CMakeFiles/pipeline.dir/KissICP.cpp.i

kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pipeline.dir/KissICP.cpp.s"
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/cpp/kiss_icp/pipeline/KissICP.cpp -o CMakeFiles/pipeline.dir/KissICP.cpp.s

# Object files for target pipeline
pipeline_OBJECTS = \
"CMakeFiles/pipeline.dir/KissICP.cpp.o"

# External object files for target pipeline
pipeline_EXTERNAL_OBJECTS =

kiss_icp/pipeline/libpipeline.a: kiss_icp/pipeline/CMakeFiles/pipeline.dir/KissICP.cpp.o
kiss_icp/pipeline/libpipeline.a: kiss_icp/pipeline/CMakeFiles/pipeline.dir/build.make
kiss_icp/pipeline/libpipeline.a: kiss_icp/pipeline/CMakeFiles/pipeline.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libpipeline.a"
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && $(CMAKE_COMMAND) -P CMakeFiles/pipeline.dir/cmake_clean_target.cmake
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pipeline.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kiss_icp/pipeline/CMakeFiles/pipeline.dir/build: kiss_icp/pipeline/libpipeline.a

.PHONY : kiss_icp/pipeline/CMakeFiles/pipeline.dir/build

kiss_icp/pipeline/CMakeFiles/pipeline.dir/clean:
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline && $(CMAKE_COMMAND) -P CMakeFiles/pipeline.dir/cmake_clean.cmake
.PHONY : kiss_icp/pipeline/CMakeFiles/pipeline.dir/clean

kiss_icp/pipeline/CMakeFiles/pipeline.dir/depend:
	cd /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/ros /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/kiss-icp/cpp/kiss_icp/pipeline /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline /home/max/Dokumente/master_thesis_kremmel/master_thesis_ros/catkin_ws/build/kiss_icp/kiss_icp/pipeline/CMakeFiles/pipeline.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kiss_icp/pipeline/CMakeFiles/pipeline.dir/depend

