cmake_minimum_required(VERSION 3.10)
project(libTrajectoryGenerator)

add_subdirectory(pybind11)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(../)

pybind11_add_module(TrajectoryCore
	pybind11_trajectory_core.cpp
	)
pybind11_add_module(Serial3RKinematics
	pybind11_serial3r.cpp
	)
