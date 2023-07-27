cmake_minimum_required(VERSION 3.0.2)
project(stoch3_rviz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  stoch3_msgs
  visualization_msgs
  urdf
  stoch3_lib
)

find_package(urdfdom_headers REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stoch3_rviz
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs stoch3_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if (SIM_BUILD)
  add_executable(stoch3_visualizer
    src/stoch3_visualizer.cpp
  )
  target_link_libraries(stoch3_visualizer ${catkin_LIBRARIES})
  add_dependencies(stoch3_visualizer ${stoch3_msgs_EXPORTED_TARGETS})
endif()


#############
## Install ##
#############
catkin_install_python(PROGRAMS
   scripts/stoch3_tf_broadcaster.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
