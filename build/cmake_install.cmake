# Install script for directory: /home/kartik/ws_stoch3/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE PROGRAM FILES "/home/kartik/ws_stoch3/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/setup.bash;/home/kartik/ws_stoch3/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/setup.bash"
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/setup.sh;/home/kartik/ws_stoch3/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/setup.sh"
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/setup.zsh;/home/kartik/ws_stoch3/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/setup.zsh"
    "/home/kartik/ws_stoch3/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kartik/ws_stoch3/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kartik/ws_stoch3/install" TYPE FILE FILES "/home/kartik/ws_stoch3/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kartik/ws_stoch3/build/gtest/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/fovis/catkin/fovis/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_geotiff_launch/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_slam/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_slam_launch/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_utils/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/qpOASES/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/ros_control/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/common_utils_drs/eigen_utils/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/common_utils_drs/eigen_utils_fftw/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_core/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_biped_core/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/rqt_controller_manager/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/controller_manager_msgs/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/fovis_ros/fovis_msgs/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_map_tools/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_nav_msgs/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_msgs/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/common_utils_drs/stl_utils/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/hardware_interface/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/combined_robot_hw/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/controller_interface/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_geotiff/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_geotiff_plugins/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_marker_drawing/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_quadruped_commons/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_quadruped/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_msgs/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/controller_manager/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/controller_manager_tests/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/combined_robot_hw_tests/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_compressed_map_transport/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_cvx_mpc_libtraj/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_hardware_interface/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_lib/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_analysis/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_control/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_rviz/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_teleop/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_state_machine/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_imu_attitude_to_tf/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_imu_tools/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_map_server/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_trajectory_server/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/hector_slam/hector_mapping/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/fovis_ros/fovis_ros/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_ros/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_quadruped_ros/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/transmission_interface/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/ros_control/joint_limits_interface/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto/pronto_biped_ros/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_description/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto_anymal_example/anymal_b_simple_description/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto_anymal_example/anymal_b_robcogen/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto_anymal_example/pronto_anymal_b_commons/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/pronto_anymal_example/pronto_anymal_b/cmake_install.cmake")
  include("/home/kartik/ws_stoch3/build/stoch3_gazebo/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/kartik/ws_stoch3/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
