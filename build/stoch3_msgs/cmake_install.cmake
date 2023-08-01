# Install script for directory: /home/kartik/ws_stoch3/src/stoch3_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kartik/ws_stoch3/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/msg" TYPE FILE FILES
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/ControllerState.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/MotorState.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommand.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegCommandStamped.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegCommand.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegState.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/LegStateStamped.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegState.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedLegFeedback.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/QuadrupedRobotState.msg"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/msg/RobotCommand.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/srv" TYPE FILE FILES
    "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Command.srv"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/Gait.srv"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/SwitchController.srv"
    "/home/kartik/ws_stoch3/src/stoch3_msgs/srv/ControllerSupervisorState.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/action" TYPE FILE FILES "/home/kartik/ws_stoch3/src/stoch3_msgs/action/SitStand.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/msg" TYPE FILE FILES
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandAction.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionGoal.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionResult.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandActionFeedback.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandGoal.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandResult.msg"
    "/home/kartik/ws_stoch3/devel/share/stoch3_msgs/msg/SitStandFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/cmake" TYPE FILE FILES "/home/kartik/ws_stoch3/build/stoch3_msgs/catkin_generated/installspace/stoch3_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kartik/ws_stoch3/devel/include/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kartik/ws_stoch3/devel/share/roseus/ros/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kartik/ws_stoch3/devel/share/common-lisp/ros/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kartik/ws_stoch3/devel/share/gennodejs/ros/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/kartik/ws_stoch3/devel/lib/python2.7/dist-packages/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kartik/ws_stoch3/devel/lib/python2.7/dist-packages/stoch3_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kartik/ws_stoch3/build/stoch3_msgs/catkin_generated/installspace/stoch3_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/cmake" TYPE FILE FILES "/home/kartik/ws_stoch3/build/stoch3_msgs/catkin_generated/installspace/stoch3_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs/cmake" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/stoch3_msgs/catkin_generated/installspace/stoch3_msgsConfig.cmake"
    "/home/kartik/ws_stoch3/build/stoch3_msgs/catkin_generated/installspace/stoch3_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_msgs" TYPE FILE FILES "/home/kartik/ws_stoch3/src/stoch3_msgs/package.xml")
endif()

