cmake_minimum_required(VERSION 3.0.2)
project(test)

add_compile_options(-std=c++14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

include_directories(../)

add_subdirectory(statics)
add_subdirectory(kinematics)
add_subdirectory(utils)
