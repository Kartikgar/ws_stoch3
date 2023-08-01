# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pronto_msgs: 11 messages, 0 services")

set(MSG_I_FLAGS "-Ipronto_msgs:/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pronto_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" "geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" "geometry_msgs/Vector3:geometry_msgs/Transform:geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" "geometry_msgs/Vector3:geometry_msgs/Wrench:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" "geometry_msgs/Vector3:geometry_msgs/Wrench:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" "geometry_msgs/Vector3:geometry_msgs/Transform:geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_custom_target(_pronto_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pronto_msgs" "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_cpp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(pronto_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pronto_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pronto_msgs_generate_messages pronto_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_cpp _pronto_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pronto_msgs_gencpp)
add_dependencies(pronto_msgs_gencpp pronto_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pronto_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)
_generate_msg_eus(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(pronto_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pronto_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pronto_msgs_generate_messages pronto_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_eus _pronto_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pronto_msgs_geneus)
add_dependencies(pronto_msgs_geneus pronto_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pronto_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)
_generate_msg_lisp(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(pronto_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pronto_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pronto_msgs_generate_messages pronto_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_lisp _pronto_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pronto_msgs_genlisp)
add_dependencies(pronto_msgs_genlisp pronto_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pronto_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)
_generate_msg_nodejs(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pronto_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pronto_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pronto_msgs_generate_messages pronto_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_nodejs _pronto_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pronto_msgs_gennodejs)
add_dependencies(pronto_msgs_gennodejs pronto_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pronto_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)
_generate_msg_py(pronto_msgs
  "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(pronto_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pronto_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pronto_msgs_generate_messages pronto_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/GPSData.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/FilterState.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedStance.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/ControllerFootContact.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VisualOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/BipedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/QuadrupedForceTorqueSensors.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/IndexedMeasurement.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/LidarOdometryUpdate.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/ws_stoch3/src/pronto/pronto_msgs/msg/VelocityWithSigmaBounds.msg" NAME_WE)
add_dependencies(pronto_msgs_generate_messages_py _pronto_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pronto_msgs_genpy)
add_dependencies(pronto_msgs_genpy pronto_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pronto_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pronto_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pronto_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(pronto_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pronto_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pronto_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(pronto_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pronto_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pronto_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(pronto_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pronto_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pronto_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(pronto_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pronto_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pronto_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(pronto_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
