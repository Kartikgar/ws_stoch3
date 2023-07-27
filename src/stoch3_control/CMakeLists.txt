cmake_minimum_required(VERSION 3.0.2)
project(stoch3_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  actionlib
  realtime_tools
  pluginlib
  control_toolbox
  control_msgs
  controller_interface
  hardware_interface
  stoch3_lib
  stoch3_msgs
  stoch3_hardware_interface
  roscpp
)

find_package(qpoases REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
    include
  #LIBRARIES
  CATKIN_DEPENDS 
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${qpoases_INCLUDE_DIRS}
)


## LOW LEVEL CONTROLLERS

# Idle controller
add_library(idle_controller  src/low_level_controllers/idle_controller.cpp)
target_link_libraries(idle_controller ${catkin_LIBRARIES})
add_dependencies(idle_controller ${catkin_EXPORTED_TARGETS})

# Motor state controller
add_library(motor_state_controller  src/low_level_controllers/motor_state_controller.cpp)
target_link_libraries(motor_state_controller ${catkin_LIBRARIES})
add_dependencies(motor_state_controller ${catkin_EXPORTED_TARGETS})

# Foot state estimator
add_library(foot_state_estimator  src/low_level_controllers/foot_state_estimator.cpp)
target_link_libraries(foot_state_estimator ${catkin_LIBRARIES})
add_dependencies(foot_state_estimator ${catkin_EXPORTED_TARGETS})

# Leg controller
add_library(leg_controller  src/low_level_controllers/leg_controller.cpp)
target_link_libraries(leg_controller ${catkin_LIBRARIES})
add_dependencies(leg_controller ${catkin_EXPORTED_TARGETS})


## HIGH LEVEL CONTROLLERS

# State estimator
add_executable(state_estimator src/high_level_controllers/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES})
add_dependencies(state_estimator ${stoch3_msgs_EXPORTED_TARGETS})

# Controller supervisor
add_executable(controller_supervisor 
  src/high_level_controllers/controller_supervisor.cpp
  src/high_level_controllers/walking_controller.cpp
  src/high_level_controllers/pose_controller.cpp
  src/high_level_controllers/sit_controller.cpp
  src/high_level_controllers/stand_controller.cpp
  src/high_level_controllers/utils.cpp)
target_link_libraries(controller_supervisor ${catkin_LIBRARIES} ${qpoases_LIBRARIES})
add_dependencies(controller_supervisor ${catkin_EXPORTED_TARGETS})

#######################
## Test Applications ##
#######################
# Test application to send commands
add_executable(test_motor_command_node 
  test/test_motor_command.cpp
  )
target_link_libraries(test_motor_command_node
  ${catkin_LIBRARIES}
)

add_executable(test_nan 
  test/test_nan.cpp
  )
target_link_libraries(test_nan
  ${catkin_LIBRARIES}
)

add_executable(test_fk
  test/test_fk.cpp
)
target_link_libraries(test_fk ${catkin_LIBRARIES})
add_dependencies(test_fk ${catkin_EXPORTED_TARGETS})


add_executable(test_ik
  test/test_ik.cpp
)
target_link_libraries(test_ik ${catkin_LIBRARIES})

add_executable(test_ik_ellipse
  test/test_ik_ellipse.cpp
)
target_link_libraries(test_ik_ellipse ${catkin_LIBRARIES})

add_executable(test_updown
  test/test_updown.cpp
)
target_link_libraries(test_updown ${catkin_LIBRARIES})

add_executable(test_sit_stand
  test/test_sit_stand.cpp
)
target_link_libraries(test_sit_stand ${catkin_LIBRARIES})

add_executable(test_sit_stand_action
  test/test_sit_stand_action.cpp
)
target_link_libraries(test_sit_stand_action ${catkin_LIBRARIES})
add_dependencies(test_sit_stand_action ${stoch3_msgs_EXPORTED_TARGETS})

add_executable(test_calibrate
  test/test_calibrate.cpp
)
target_link_libraries(test_calibrate ${catkin_LIBRARIES})
add_dependencies(test_calibrate ${stoch3_msgs_EXPORTED_TARGETS})

add_executable(compute_foot_forces
  test/compute_foot_forces.cpp
)
target_link_libraries(compute_foot_forces ${catkin_LIBRARIES})
add_dependencies(compute_foot_forces ${stoch3_msgs_EXPORTED_TARGETS})

add_executable(test_optimal_foot_forces
  test/test_optimal_foot_forces.cpp
)
target_link_libraries(test_optimal_foot_forces ${catkin_LIBRARIES} ${qpoases_LIBRARIES})
add_dependencies(test_optimal_foot_forces ${stoch3_msgs_EXPORTED_TARGETS})

add_executable(measure_joint_tracking_performance
  test/measure_joint_tracking_performance.cpp
)
target_link_libraries(measure_joint_tracking_performance ${catkin_LIBRARIES})
add_dependencies(measure_joint_tracking_performance ${stoch3_msgs_EXPORTED_TARGETS})

add_executable(test_nan_input
  test/test_nan_input.cpp
)
target_link_libraries(test_nan_input ${catkin_LIBRARIES} ${qpoases_LIBRARIES})
add_dependencies(test_nan_input ${stoch3_msgs_EXPORTED_TARGETS})

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install (TARGETS idle_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install targets
install (TARGETS leg_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install targets
install (TARGETS motor_state_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install targets
install (TARGETS foot_state_estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES stoch3_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
