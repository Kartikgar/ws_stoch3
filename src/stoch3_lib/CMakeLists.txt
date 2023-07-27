cmake_minimum_required(VERSION 3.0.2)
project(stoch3_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  std_msgs
  sensor_msgs
  geometry_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
    include
    include/${PROJECT_NAME}/libTrajectoryGenerator
    ${EIGEN3_INCLUDE_DIR}
  #  LIBRARIES stoch3_lib
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib 
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}/libTrajectoryGenerator
  ${EIGEN3_INCLUDE_DIR}
)


#############
## Install ##
#############

#############
## Testing ##
#############

add_executable(test_velocity_kinematics_node
  test/test_velocity_kinematics.cpp
  )
target_link_libraries(test_velocity_kinematics_node
  ${catkin_LIBRARIES}
)
