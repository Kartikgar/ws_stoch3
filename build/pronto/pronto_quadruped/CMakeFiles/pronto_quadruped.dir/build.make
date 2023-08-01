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

# Include any dependencies generated for this target.
include pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/depend.make

# Include the progress variables for this target.
include pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/progress.make

# Include the compile flags for this target's objects.
include pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/LegOdometer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/LegOdometer.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/LegOdometer.cpp > CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/LegOdometer.cpp -o CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DynamicStanceEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DynamicStanceEstimator.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DynamicStanceEstimator.cpp > CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DynamicStanceEstimator.cpp -o CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ForceSensorStanceEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ForceSensorStanceEstimator.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ForceSensorStanceEstimator.cpp > CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ForceSensorStanceEstimator.cpp -o CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FootSensorStanceDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FootSensorStanceDetector.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FootSensorStanceDetector.cpp > CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FootSensorStanceDetector.cpp -o CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/StanceEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/StanceEstimator.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/StanceEstimator.cpp > CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/StanceEstimator.cpp -o CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FlexEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FlexEstimator.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FlexEstimator.cpp > CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/FlexEstimator.cpp -o CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DataLogger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DataLogger.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DataLogger.cpp > CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/DataLogger.cpp -o CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o


pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/flags.make
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o: /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ImuBiasLock.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o -c /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ImuBiasLock.cpp

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.i"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ImuBiasLock.cpp > CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.i

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.s"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kartik/ws_stoch3/src/pronto/pronto_quadruped/src/ImuBiasLock.cpp -o CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.s

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.requires:

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.provides: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.requires
	$(MAKE) -f pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.provides.build
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.provides

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.provides.build: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o


# Object files for target pronto_quadruped
pronto_quadruped_OBJECTS = \
"CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o" \
"CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o"

# External object files for target pronto_quadruped
pronto_quadruped_EXTERNAL_OBJECTS =

/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build.make
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /home/kartik/ws_stoch3/devel/lib/libpronto_core.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /home/kartik/ws_stoch3/devel/lib/libpronto_utils.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /home/kartik/ws_stoch3/devel/lib/libbacklash_filter.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: /home/kartik/ws_stoch3/devel/lib/libkalman_filter.so
/home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kartik/ws_stoch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX shared library /home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so"
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pronto_quadruped.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build: /home/kartik/ws_stoch3/devel/lib/libpronto_quadruped.so

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/build

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/LegOdometer.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DynamicStanceEstimator.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ForceSensorStanceEstimator.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FootSensorStanceDetector.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/StanceEstimator.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/FlexEstimator.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/DataLogger.cpp.o.requires
pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires: pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/src/ImuBiasLock.cpp.o.requires

.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/requires

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/clean:
	cd /home/kartik/ws_stoch3/build/pronto/pronto_quadruped && $(CMAKE_COMMAND) -P CMakeFiles/pronto_quadruped.dir/cmake_clean.cmake
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/clean

pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/depend:
	cd /home/kartik/ws_stoch3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kartik/ws_stoch3/src /home/kartik/ws_stoch3/src/pronto/pronto_quadruped /home/kartik/ws_stoch3/build /home/kartik/ws_stoch3/build/pronto/pronto_quadruped /home/kartik/ws_stoch3/build/pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pronto/pronto_quadruped/CMakeFiles/pronto_quadruped.dir/depend
