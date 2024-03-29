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

# Utility rule file for clang-format.

# Include any custom commands dependencies for this target.
include CMakeFiles/clang-format.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/clang-format.dir/progress.make

CMakeFiles/clang-format:
	clang-format -i /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/a_star.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/a_star_isolated.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/low_level.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/a_star_epsilon.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/a_star_epsilon_isolated.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/assignment.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/cbs.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/cbs_isolated.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/cbs_ta.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/ecbs.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/ecbs_isolated.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/ecbs_ta.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/neighbor.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/next_best_assignment.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/planresult.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/sipp.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/sipp_isolated.hpp /home/ssfc/libMultiRobotPlanning/include/libMultiRobotPlanning/util.hpp /home/ssfc/libMultiRobotPlanning/example/a_star.cpp /home/ssfc/libMultiRobotPlanning/example/a_star_isolated.cpp /home/ssfc/libMultiRobotPlanning/example/a_star_epsilon.cpp /home/ssfc/libMultiRobotPlanning/example/a_star_epsilon_isolated.cpp /home/ssfc/libMultiRobotPlanning/example/assignment.cpp /home/ssfc/libMultiRobotPlanning/example/cbs.cpp /home/ssfc/libMultiRobotPlanning/example/cbs_isolated.cpp /home/ssfc/libMultiRobotPlanning/example/cbs_ta.cpp /home/ssfc/libMultiRobotPlanning/example/ecbs.cpp /home/ssfc/libMultiRobotPlanning/example/ecbs_isolated.cpp /home/ssfc/libMultiRobotPlanning/example/ecbs_ta.cpp /home/ssfc/libMultiRobotPlanning/example/mapf_prioritized_sipp.cpp /home/ssfc/libMultiRobotPlanning/example/next_best_assignment.cpp /home/ssfc/libMultiRobotPlanning/example/shortest_path_heuristic.cpp /home/ssfc/libMultiRobotPlanning/example/shortest_path_heuristic.hpp /home/ssfc/libMultiRobotPlanning/example/sipp.cpp /home/ssfc/libMultiRobotPlanning/example/sipp_isolated.cpp /home/ssfc/libMultiRobotPlanning/example/timer.hpp

clang-format: CMakeFiles/clang-format
clang-format: CMakeFiles/clang-format.dir/build.make
.PHONY : clang-format

# Rule to build all files generated by this target.
CMakeFiles/clang-format.dir/build: clang-format
.PHONY : CMakeFiles/clang-format.dir/build

CMakeFiles/clang-format.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clang-format.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clang-format.dir/clean

CMakeFiles/clang-format.dir/depend:
	cd /home/ssfc/libMultiRobotPlanning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ssfc/libMultiRobotPlanning /home/ssfc/libMultiRobotPlanning /home/ssfc/libMultiRobotPlanning/build /home/ssfc/libMultiRobotPlanning/build /home/ssfc/libMultiRobotPlanning/build/CMakeFiles/clang-format.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clang-format.dir/depend

