cmake_minimum_required(VERSION 3.0.2)
project(stoch3_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")

find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  sensor_msgs
  std_msgs
  roslaunch
  tf2
)

catkin_package(
  INCLUDE_DIRS 
  include
  ${EIGEN3_INCLUDE_DIR}
  #  LIBRARIES stoch3_hardware_interface
  CATKIN_DEPENDS controller_manager roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}

  include/${PROJECT_NAME}/
  ${stoch3_INCLUDE_DIRS}/stoch3/libTrajectoryGernerator/
  ${stoch3_INCLUDE_DIRS}/stoch3/stochrl_linear_policies/

  ${EIGEN3_INCLUDE_DIR} 
)


if (NOT SIM_BUILD)
  message (WARNING "BUILDING CODE FOR RUNNING ON RPi.")

  # Build the xsens MTSDK libraries
  add_subdirectory(xsens/xspublic)


  include_directories(xsens/include)

  ### Declaring C++ executable ###
  include_directories(/opt/vc/include)
  link_directories(/opt/vc/lib)

  add_executable(${PROJECT_NAME}_node 
    src/stoch3_hardware_interface_node.cpp
    src/stoch3_hardware_interface.cpp
    src/imu_interface.cpp
    src/pi3hat.cc
    src/pi3hat_interface.cpp
    )
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    xscontroller xscommon xstypes 
    pthread rt dl
    bcm_host
    )
  add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
else ()
  message(WARNING "BUILDING CODE FOR SIMULATION.")
endif()


# Stoch3 Joint Offset Calibration 
add_executable(stoch3_joint_offset_calibration_node
  src/stoch3_joint_offset_calibration.cpp
  )
target_link_libraries(stoch3_joint_offset_calibration_node
  ${catkin_LIBRARIES}
  )
add_dependencies(stoch3_joint_offset_calibration_node ${catkin_EXPORTED_TARGETS})
##########
## INSTALL
##########

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".git" EXCLUDE
)

