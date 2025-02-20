# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "action_infer: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iaction_infer:/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(action_infer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_custom_target(_action_infer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_infer" "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" "geometry_msgs/Vector3:geometry_msgs/Twist"
)

get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_custom_target(_action_infer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_infer" "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" ""
)

get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_custom_target(_action_infer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_infer" "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float32MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer
)
_generate_msg_cpp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer
)
_generate_msg_cpp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer
)

### Generating Services

### Generating Module File
_generate_module_cpp(action_infer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(action_infer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(action_infer_generate_messages action_infer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_cpp _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_cpp _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_cpp _action_infer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_infer_gencpp)
add_dependencies(action_infer_gencpp action_infer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_infer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer
)
_generate_msg_eus(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer
)
_generate_msg_eus(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer
)

### Generating Services

### Generating Module File
_generate_module_eus(action_infer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(action_infer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(action_infer_generate_messages action_infer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_eus _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_eus _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_eus _action_infer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_infer_geneus)
add_dependencies(action_infer_geneus action_infer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_infer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer
)
_generate_msg_lisp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer
)
_generate_msg_lisp(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer
)

### Generating Services

### Generating Module File
_generate_module_lisp(action_infer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(action_infer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(action_infer_generate_messages action_infer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_lisp _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_lisp _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_lisp _action_infer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_infer_genlisp)
add_dependencies(action_infer_genlisp action_infer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_infer_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer
)
_generate_msg_nodejs(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer
)
_generate_msg_nodejs(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer
)

### Generating Services

### Generating Module File
_generate_module_nodejs(action_infer
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(action_infer_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(action_infer_generate_messages action_infer_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_nodejs _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_nodejs _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_nodejs _action_infer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_infer_gennodejs)
add_dependencies(action_infer_gennodejs action_infer_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_infer_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer
)
_generate_msg_py(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer
)
_generate_msg_py(action_infer
  "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer
)

### Generating Services

### Generating Module File
_generate_module_py(action_infer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(action_infer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(action_infer_generate_messages action_infer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/TwistGripper.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_py _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/ObsCache.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_py _action_infer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxt/thesis/yirui/catkin_workspace/src/action_infer/msg/PoseAction.msg" NAME_WE)
add_dependencies(action_infer_generate_messages_py _action_infer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_infer_genpy)
add_dependencies(action_infer_genpy action_infer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_infer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_infer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(action_infer_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(action_infer_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_infer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(action_infer_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(action_infer_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_infer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(action_infer_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(action_infer_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_infer
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(action_infer_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(action_infer_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_infer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(action_infer_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(action_infer_generate_messages_py geometry_msgs_generate_messages_py)
endif()
