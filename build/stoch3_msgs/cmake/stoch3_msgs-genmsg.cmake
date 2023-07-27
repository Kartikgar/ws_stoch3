# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "stoch3_msgs: 18 messages, 4 services")

set(MSG_I_FLAGS "-Istoch3_msgs:/home/kartik/ws_stoch3/src/stoch3_msgs/msg;-Istoch3_msgs:/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(stoch3_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" "stoch3_msgs/LegCommand:geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" "geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:stoch3_msgs/LegState:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" "geometry_msgs/Vector3:stoch3_msgs/LegState:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" "geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" "stoch3_msgs/LegCommand:geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" "stoch3_msgs/SitStandResult:actionlib_msgs/GoalID:stoch3_msgs/SitStandActionFeedback:actionlib_msgs/GoalStatus:stoch3_msgs/SitStandFeedback:stoch3_msgs/SitStandGoal:stoch3_msgs/SitStandActionResult:std_msgs/Header:stoch3_msgs/SitStandActionGoal"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" "stoch3_msgs/SitStandFeedback:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" "stoch3_msgs/SitStandGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" "stoch3_msgs/SitStandResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" ""
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" "geometry_msgs/Twist:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_custom_target(_stoch3_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stoch3_msgs" "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" "geometry_msgs/Vector3:stoch3_msgs/LegState:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)

### Generating Services
_generate_srv_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_cpp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
)

### Generating Module File
_generate_module_cpp(stoch3_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(stoch3_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(stoch3_msgs_generate_messages stoch3_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_cpp _stoch3_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stoch3_msgs_gencpp)
add_dependencies(stoch3_msgs_gencpp stoch3_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stoch3_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)

### Generating Services
_generate_srv_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_eus(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
)

### Generating Module File
_generate_module_eus(stoch3_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(stoch3_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(stoch3_msgs_generate_messages stoch3_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_eus _stoch3_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stoch3_msgs_geneus)
add_dependencies(stoch3_msgs_geneus stoch3_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stoch3_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)

### Generating Services
_generate_srv_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_lisp(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
)

### Generating Module File
_generate_module_lisp(stoch3_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(stoch3_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(stoch3_msgs_generate_messages stoch3_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_lisp _stoch3_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stoch3_msgs_genlisp)
add_dependencies(stoch3_msgs_genlisp stoch3_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stoch3_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)

### Generating Services
_generate_srv_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_nodejs(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
)

### Generating Module File
_generate_module_nodejs(stoch3_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(stoch3_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(stoch3_msgs_generate_messages stoch3_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_nodejs _stoch3_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stoch3_msgs_gennodejs)
add_dependencies(stoch3_msgs_gennodejs stoch3_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stoch3_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_msg_py(stoch3_msgs
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)

### Generating Services
_generate_srv_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)
_generate_srv_py(stoch3_msgs
  "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
)

### Generating Module File
_generate_module_py(stoch3_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(stoch3_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(stoch3_msgs_generate_messages stoch3_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg" NAME_WE)
add_dependencies(stoch3_msgs_generate_messages_py _stoch3_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stoch3_msgs_genpy)
add_dependencies(stoch3_msgs_genpy stoch3_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stoch3_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stoch3_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(stoch3_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(stoch3_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(stoch3_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stoch3_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(stoch3_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(stoch3_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(stoch3_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stoch3_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(stoch3_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(stoch3_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(stoch3_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stoch3_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(stoch3_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(stoch3_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(stoch3_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stoch3_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(stoch3_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(stoch3_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(stoch3_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
