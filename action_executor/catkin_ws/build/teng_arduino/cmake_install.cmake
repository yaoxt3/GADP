# Install script for directory: /home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/teng_arduino/msg" TYPE FILE FILES
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/teng_arduino/cmake" TYPE FILE FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/teng_arduino-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/include/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/share/roseus/ros/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/share/common-lisp/ros/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/share/gennodejs/ros/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/lib/python3/dist-packages/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/devel/lib/python3/dist-packages/teng_arduino")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/teng_arduino.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/teng_arduino/cmake" TYPE FILE FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/teng_arduino-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/teng_arduino/cmake" TYPE FILE FILES
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/teng_arduinoConfig.cmake"
    "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/teng_arduinoConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/teng_arduino" TYPE FILE FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/keyboard.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/dataSaver.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/arduino_mutli_pub.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/multiDataSaver.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/handle.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/dropCatch.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/joy_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/robot_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/teng_arduino" TYPE PROGRAM FILES "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/build/teng_arduino/catkin_generated/installspace/action_execution_no_pc.py")
endif()

