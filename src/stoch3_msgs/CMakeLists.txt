cmake_minimum_required(VERSION 3.0.2)
project(stoch3_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)

#############
## Service ##
#############

add_message_files(
  DIRECTORY msg
  FILES
    ControllerState.msg
    MotorState.msg
    LegCommand.msg
    LegCommandStamped.msg
    QuadrupedLegCommand.msg
    LegState.msg
    LegStateStamped.msg
    QuadrupedLegState.msg
    QuadrupedLegFeedback.msg
    QuadrupedRobotState.msg
    RobotCommand.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    Command.srv
    Gait.srv
    SwitchController.srv
    ControllerSupervisorState.srv
)

############
## Action ##
############

add_action_files(
  DIRECTORY action
  FILES SitStand.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stoch3_msgs
   CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)

