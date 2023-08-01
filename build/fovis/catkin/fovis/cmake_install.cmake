# Install script for directory: /home/kartik/ws_stoch3/src/fovis/catkin/fovis

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kartik/ws_stoch3/build/fovis/catkin/fovis/catkin_generated/installspace/fovis.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fovis/cmake" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/fovis/catkin/fovis/catkin_generated/installspace/fovisConfig.cmake"
    "/home/kartik/ws_stoch3/build/fovis/catkin/fovis/catkin_generated/installspace/fovisConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fovis" TYPE FILE FILES "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kartik/ws_stoch3/devel/lib/libfovis.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/fovis" TYPE FILE FILES
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/absolute_orientation_horn.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/camera_intrinsics.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/depth_image.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/depth_source.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/fast.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/feature_match.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/feature_matcher.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/fovis.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/frame.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/gauss_pyramid.h"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/grid_filter.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/initial_homography_estimation.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/intensity_descriptor.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/internal_utils.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/keypoint.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/motion_estimation.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/normalize_image.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/options.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/primesense_depth.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/pyramid_level.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/rectification.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/refine_feature_match.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/refine_motion_estimate.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/sad.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/stereo_calibration.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/stereo_depth.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/stereo_disparity.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/stereo_frame.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/stereo_rectify.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/tictoc.hpp"
    "/home/kartik/ws_stoch3/src/fovis/catkin/fovis/../../libfovis/libfovis/visual_odometry.hpp"
    )
endif()

