# Install script for directory: /home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/egronda/sunfest_2019/src/catkin_ws/devel")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libcaer" TYPE FILE FILES
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/libcaer.h"
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/log.h"
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/network.h"
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/portable_endian.h"
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/frame_utils.h"
    "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/ringbuffer.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libcaer" TYPE DIRECTORY FILES "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/events" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libcaer" TYPE DIRECTORY FILES "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/devices" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libcaer" TYPE DIRECTORY FILES "/home/egronda/sunfest_2019/src/catkin_ws/build/libcaer_catkin/libcaer_src-prefix/src/libcaer_src/include/libcaer/filters" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

