# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "segmentation: 7 messages, 0 services")

set(MSG_I_FLAGS "-Isegmentation:/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genjava REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(segmentation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" "sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" "segmentation/imageRoiFeedback:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" ""
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" "segmentation/imageRoiGoal:actionlib_msgs/GoalID:std_msgs/Header:std_msgs/String"
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" "sensor_msgs/Image:segmentation/imageRoiResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_custom_target(_segmentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "segmentation" "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" "sensor_msgs/Image:segmentation/imageRoiActionResult:actionlib_msgs/GoalID:std_msgs/Header:segmentation/imageRoiActionFeedback:segmentation/imageRoiFeedback:segmentation/imageRoiActionGoal:segmentation/imageRoiResult:segmentation/imageRoiGoal:std_msgs/String:actionlib_msgs/GoalStatus"
)

#
#  langs = gencpp;geneus;genjava;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)
_generate_msg_cpp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_cpp(segmentation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(segmentation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_cpp _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_gencpp)
add_dependencies(segmentation_gencpp segmentation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)
_generate_msg_eus(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_eus(segmentation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(segmentation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_eus _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_geneus)
add_dependencies(segmentation_geneus segmentation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_eus)

### Section generating for lang: genjava
### Generating Messages
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)
_generate_msg_java(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_java(segmentation
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_java}"
)

add_custom_target(segmentation_generate_messages_java
  DEPENDS ${ALL_GEN_OUTPUT_FILES_java}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_java)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_java _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_genjava)
add_dependencies(segmentation_genjava segmentation_generate_messages_java)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_java)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)
_generate_msg_lisp(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_lisp(segmentation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(segmentation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_lisp _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_genlisp)
add_dependencies(segmentation_genlisp segmentation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)
_generate_msg_nodejs(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(segmentation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(segmentation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_nodejs _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_gennodejs)
add_dependencies(segmentation_gennodejs segmentation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)
_generate_msg_py(segmentation
  "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg;/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
)

### Generating Services

### Generating Module File
_generate_module_py(segmentation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(segmentation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(segmentation_generate_messages segmentation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiFeedback.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionGoal.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiActionResult.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lukas/robocup/catkinWS/devel/.private/segmentation/share/segmentation/msg/imageRoiAction.msg" NAME_WE)
add_dependencies(segmentation_generate_messages_py _segmentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(segmentation_genpy)
add_dependencies(segmentation_genpy segmentation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS segmentation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/segmentation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(segmentation_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/segmentation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(segmentation_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(segmentation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(segmentation_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genjava_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/segmentation
    DESTINATION ${genjava_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_java)
  add_dependencies(segmentation_generate_messages_java actionlib_msgs_generate_messages_java)
endif()
if(TARGET std_msgs_generate_messages_java)
  add_dependencies(segmentation_generate_messages_java std_msgs_generate_messages_java)
endif()
if(TARGET sensor_msgs_generate_messages_java)
  add_dependencies(segmentation_generate_messages_java sensor_msgs_generate_messages_java)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/segmentation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(segmentation_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/segmentation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(segmentation_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(segmentation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(segmentation_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/segmentation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(segmentation_generate_messages_py sensor_msgs_generate_messages_py)
endif()
