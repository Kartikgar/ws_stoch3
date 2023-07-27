cmake_minimum_required(VERSION 3.0.2)
project(stoch3_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  control_toolbox
  controller_manager
  hardware_interface
  stoch3_hardware_interface
  transmission_interface
  joint_limits_interface
  pluginlib
  urdf
  angles
  std_msgs
  stoch3_msgs
  stoch3_control
  stoch3_description
  xacro
  gazebo_msgs
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
  LIBRARIES stoch3_robot_hw_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(set_model_configuration_node
  src/set_model_configuration_node.cpp
)
target_link_libraries(set_model_configuration_node ${catkin_LIBRARIES})


add_library(stoch3_robot_hw_sim src/stoch3_robot_hw_sim.cpp)
target_link_libraries(stoch3_robot_hw_sim ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS stoch3_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
