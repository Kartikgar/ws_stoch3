# Install script for directory: /home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kartik/ws_stoch3/build/pronto_anymal_example/anymal_b_simple_description/catkin_generated/installspace/anymal_b_simple_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/anymal_b_simple_description/cmake" TYPE FILE FILES
    "/home/kartik/ws_stoch3/build/pronto_anymal_example/anymal_b_simple_description/catkin_generated/installspace/anymal_b_simple_descriptionConfig.cmake"
    "/home/kartik/ws_stoch3/build/pronto_anymal_example/anymal_b_simple_description/catkin_generated/installspace/anymal_b_simple_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/anymal_b_simple_description" TYPE FILE FILES "/home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/anymal_b_simple_description" TYPE DIRECTORY FILES
    "/home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description/config"
    "/home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description/launch"
    "/home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description/meshes"
    "/home/kartik/ws_stoch3/src/pronto_anymal_example/anymal_b_simple_description/urdf"
    )
endif()

