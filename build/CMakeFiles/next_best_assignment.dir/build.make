# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ssfc/libMultiRobotPlanning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ssfc/libMultiRobotPlanning/build

# Include any dependencies generated for this target.
include CMakeFiles/next_best_assignment.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/next_best_assignment.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/next_best_assignment.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/next_best_assignment.dir/flags.make

CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o: CMakeFiles/next_best_assignment.dir/flags.make
CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o: ../example/next_best_assignment.cpp
CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o: CMakeFiles/next_best_assignment.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ssfc/libMultiRobotPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o -MF CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o.d -o CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o -c /home/ssfc/libMultiRobotPlanning/example/next_best_assignment.cpp

CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ssfc/libMultiRobotPlanning/example/next_best_assignment.cpp > CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.i

CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ssfc/libMultiRobotPlanning/example/next_best_assignment.cpp -o CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.s

# Object files for target next_best_assignment
next_best_assignment_OBJECTS = \
"CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o"

# External object files for target next_best_assignment
next_best_assignment_EXTERNAL_OBJECTS =

next_best_assignment: CMakeFiles/next_best_assignment.dir/example/next_best_assignment.cpp.o
next_best_assignment: CMakeFiles/next_best_assignment.dir/build.make
next_best_assignment: /home/ssfc/anaconda3/lib/libboost_program_options.so.1.82.0
next_best_assignment: CMakeFiles/next_best_assignment.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ssfc/libMultiRobotPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable next_best_assignment"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/next_best_assignment.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/next_best_assignment.dir/build: next_best_assignment
.PHONY : CMakeFiles/next_best_assignment.dir/build

CMakeFiles/next_best_assignment.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/next_best_assignment.dir/cmake_clean.cmake
.PHONY : CMakeFiles/next_best_assignment.dir/clean

CMakeFiles/next_best_assignment.dir/depend:
	cd /home/ssfc/libMultiRobotPlanning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ssfc/libMultiRobotPlanning /home/ssfc/libMultiRobotPlanning /home/ssfc/libMultiRobotPlanning/build /home/ssfc/libMultiRobotPlanning/build /home/ssfc/libMultiRobotPlanning/build/CMakeFiles/next_best_assignment.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/next_best_assignment.dir/depend

