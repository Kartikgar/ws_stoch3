# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/kartik/ws_stoch3/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kartik/ws_stoch3/build

# Utility rule file for qpOASES_Python.

# Include the progress variables for this target.
include qpOASES/CMakeFiles/qpOASES_Python.dir/progress.make

qpOASES/CMakeFiles/qpOASES_Python: /home/kartik/ws_stoch3/devel/lib/libqpoases.so
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building Python wrappers."
	cd /home/kartik/ws_stoch3/src/qpOASES && python setup.py build_ext --include-dirs include --build-lib /home/kartik/ws_stoch3/devel/lib/python2.7/dist-packages --build-temp /home/kartik/ws_stoch3/build/qpOASES --sources interfaces/python/qpoases.pxd interfaces/python/qpoases.pyx src/BLASReplacement.cpp src/Constraints.cpp src/Indexlist.cpp src/Matrices.cpp src/Options.cpp src/QProblemB.cpp src/SolutionAnalysis.cpp src/SubjectTo.cpp src/Bounds.cpp src/Flipper.cpp src/LAPACKReplacement.cpp src/MessageHandling.cpp src/OQPinterface.cpp src/QProblem.cpp src/SQProblem.cpp src/Utils.cpp

qpOASES_Python: qpOASES/CMakeFiles/qpOASES_Python
qpOASES_Python: qpOASES/CMakeFiles/qpOASES_Python.dir/build.make

.PHONY : qpOASES_Python

# Rule to build all files generated by this target.
qpOASES/CMakeFiles/qpOASES_Python.dir/build: qpOASES_Python

.PHONY : qpOASES/CMakeFiles/qpOASES_Python.dir/build

qpOASES/CMakeFiles/qpOASES_Python.dir/clean:
	cd /home/kartik/ws_stoch3/build/qpOASES && $(CMAKE_COMMAND) -P CMakeFiles/qpOASES_Python.dir/cmake_clean.cmake
.PHONY : qpOASES/CMakeFiles/qpOASES_Python.dir/clean

qpOASES/CMakeFiles/qpOASES_Python.dir/depend:
	cd /home/kartik/ws_stoch3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kartik/ws_stoch3/src /home/kartik/ws_stoch3/src/qpOASES /home/kartik/ws_stoch3/build /home/kartik/ws_stoch3/build/qpOASES /home/kartik/ws_stoch3/build/qpOASES/CMakeFiles/qpOASES_Python.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : qpOASES/CMakeFiles/qpOASES_Python.dir/depend

