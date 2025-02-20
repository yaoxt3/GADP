# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "teng_arduino: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iteng_arduino:/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(teng_arduino_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_custom_target(_teng_arduino_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teng_arduino" "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" ""
)

get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_custom_target(_teng_arduino_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teng_arduino" "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" ""
)

get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_custom_target(_teng_arduino_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teng_arduino" "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" "geometry_msgs/Twist:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_custom_target(_teng_arduino_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teng_arduino" "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" ""
)

get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_custom_target(_teng_arduino_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teng_arduino" "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" "std_msgs/Float32MultiArray:std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
)
_generate_msg_cpp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
)
_generate_msg_cpp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
)
_generate_msg_cpp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
)
_generate_msg_cpp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
)

### Generating Services

### Generating Module File
_generate_module_cpp(teng_arduino
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(teng_arduino_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(teng_arduino_generate_messages teng_arduino_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_cpp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_cpp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_cpp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_cpp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_cpp _teng_arduino_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teng_arduino_gencpp)
add_dependencies(teng_arduino_gencpp teng_arduino_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teng_arduino_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
)
_generate_msg_eus(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
)
_generate_msg_eus(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
)
_generate_msg_eus(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
)
_generate_msg_eus(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
)

### Generating Services

### Generating Module File
_generate_module_eus(teng_arduino
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(teng_arduino_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(teng_arduino_generate_messages teng_arduino_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_eus _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_eus _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_eus _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_eus _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_eus _teng_arduino_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teng_arduino_geneus)
add_dependencies(teng_arduino_geneus teng_arduino_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teng_arduino_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
)
_generate_msg_lisp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
)
_generate_msg_lisp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
)
_generate_msg_lisp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
)
_generate_msg_lisp(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
)

### Generating Services

### Generating Module File
_generate_module_lisp(teng_arduino
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(teng_arduino_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(teng_arduino_generate_messages teng_arduino_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_lisp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_lisp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_lisp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_lisp _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_lisp _teng_arduino_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teng_arduino_genlisp)
add_dependencies(teng_arduino_genlisp teng_arduino_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teng_arduino_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
)
_generate_msg_nodejs(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
)
_generate_msg_nodejs(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
)
_generate_msg_nodejs(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
)
_generate_msg_nodejs(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
)

### Generating Services

### Generating Module File
_generate_module_nodejs(teng_arduino
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(teng_arduino_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(teng_arduino_generate_messages teng_arduino_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_nodejs _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_nodejs _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_nodejs _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_nodejs _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_nodejs _teng_arduino_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teng_arduino_gennodejs)
add_dependencies(teng_arduino_gennodejs teng_arduino_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teng_arduino_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
)
_generate_msg_py(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
)
_generate_msg_py(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
)
_generate_msg_py(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
)
_generate_msg_py(teng_arduino
  "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
)

### Generating Services

### Generating Module File
_generate_module_py(teng_arduino
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(teng_arduino_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(teng_arduino_generate_messages teng_arduino_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channels.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_py _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/channel.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_py _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/TwistGripper.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_py _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/ObsCache.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_py _teng_arduino_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/for_upload/action_executor/catkin_ws/src/teng_arduino/msg/PoseAction.msg" NAME_WE)
add_dependencies(teng_arduino_generate_messages_py _teng_arduino_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teng_arduino_genpy)
add_dependencies(teng_arduino_genpy teng_arduino_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teng_arduino_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teng_arduino
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(teng_arduino_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(teng_arduino_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teng_arduino
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(teng_arduino_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(teng_arduino_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teng_arduino
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(teng_arduino_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(teng_arduino_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teng_arduino
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(teng_arduino_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(teng_arduino_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teng_arduino
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(teng_arduino_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(teng_arduino_generate_messages_py geometry_msgs_generate_messages_py)
endif()
