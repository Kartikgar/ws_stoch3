# Install script for directory: /home/kartik/ws_stoch3/src/stoch3_cvx_mpc_libtraj

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/stoch3_cvx_mpc_libtraj.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_cvx_mpc_libtraj/cmake" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/stoch3_cvx_mpc_libtrajConfig.cmake"
    "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/stoch3_cvx_mpc_libtrajConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stoch3_cvx_mpc_libtraj" TYPE FILE FILES "/home/kartik/ws_stoch3/src/stoch3_cvx_mpc_libtraj/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/torque_stance_leg_controller_joy.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/libtraj_swing_leg_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/traj_gererator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/locomotion_controller_joy.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/controller_joy.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/cvx_MPC_joy.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/stoch3_params.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/gait_generator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/leg_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/controller_walk.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stoch3_cvx_mpc_libtraj" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/catkin_generated/installspace/cvx_MPC_walk.py")
endif()

