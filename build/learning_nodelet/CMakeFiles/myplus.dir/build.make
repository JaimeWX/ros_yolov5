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
CMAKE_SOURCE_DIR = /home/westwell/wwws/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/westwell/wwws/catkin_ws/build

# Include any dependencies generated for this target.
include learning_nodelet/CMakeFiles/myplus.dir/depend.make

# Include the progress variables for this target.
include learning_nodelet/CMakeFiles/myplus.dir/progress.make

# Include the compile flags for this target's objects.
include learning_nodelet/CMakeFiles/myplus.dir/flags.make

learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.o: learning_nodelet/CMakeFiles/myplus.dir/flags.make
learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.o: /home/westwell/wwws/catkin_ws/src/learning_nodelet/src/myplus.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/westwell/wwws/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.o"
	cd /home/westwell/wwws/catkin_ws/build/learning_nodelet && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myplus.dir/src/myplus.cpp.o -c /home/westwell/wwws/catkin_ws/src/learning_nodelet/src/myplus.cpp

learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myplus.dir/src/myplus.cpp.i"
	cd /home/westwell/wwws/catkin_ws/build/learning_nodelet && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/westwell/wwws/catkin_ws/src/learning_nodelet/src/myplus.cpp > CMakeFiles/myplus.dir/src/myplus.cpp.i

learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myplus.dir/src/myplus.cpp.s"
	cd /home/westwell/wwws/catkin_ws/build/learning_nodelet && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/westwell/wwws/catkin_ws/src/learning_nodelet/src/myplus.cpp -o CMakeFiles/myplus.dir/src/myplus.cpp.s

# Object files for target myplus
myplus_OBJECTS = \
"CMakeFiles/myplus.dir/src/myplus.cpp.o"

# External object files for target myplus
myplus_EXTERNAL_OBJECTS =

/home/westwell/wwws/catkin_ws/devel/lib/libmyplus.so: learning_nodelet/CMakeFiles/myplus.dir/src/myplus.cpp.o
/home/westwell/wwws/catkin_ws/devel/lib/libmyplus.so: learning_nodelet/CMakeFiles/myplus.dir/build.make
/home/westwell/wwws/catkin_ws/devel/lib/libmyplus.so: learning_nodelet/CMakeFiles/myplus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/westwell/wwws/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/westwell/wwws/catkin_ws/devel/lib/libmyplus.so"
	cd /home/westwell/wwws/catkin_ws/build/learning_nodelet && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myplus.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
learning_nodelet/CMakeFiles/myplus.dir/build: /home/westwell/wwws/catkin_ws/devel/lib/libmyplus.so

.PHONY : learning_nodelet/CMakeFiles/myplus.dir/build

learning_nodelet/CMakeFiles/myplus.dir/clean:
	cd /home/westwell/wwws/catkin_ws/build/learning_nodelet && $(CMAKE_COMMAND) -P CMakeFiles/myplus.dir/cmake_clean.cmake
.PHONY : learning_nodelet/CMakeFiles/myplus.dir/clean

learning_nodelet/CMakeFiles/myplus.dir/depend:
	cd /home/westwell/wwws/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/westwell/wwws/catkin_ws/src /home/westwell/wwws/catkin_ws/src/learning_nodelet /home/westwell/wwws/catkin_ws/build /home/westwell/wwws/catkin_ws/build/learning_nodelet /home/westwell/wwws/catkin_ws/build/learning_nodelet/CMakeFiles/myplus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : learning_nodelet/CMakeFiles/myplus.dir/depend

