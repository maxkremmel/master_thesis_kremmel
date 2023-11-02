# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "master_thesis_kremmel: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(master_thesis_kremmel_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_custom_target(_master_thesis_kremmel_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "master_thesis_kremmel" "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_custom_target(_master_thesis_kremmel_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "master_thesis_kremmel" "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/master_thesis_kremmel
)
_generate_srv_cpp(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/master_thesis_kremmel
)

### Generating Module File
_generate_module_cpp(master_thesis_kremmel
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/master_thesis_kremmel
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(master_thesis_kremmel_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(master_thesis_kremmel_generate_messages master_thesis_kremmel_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_cpp _master_thesis_kremmel_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_cpp _master_thesis_kremmel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(master_thesis_kremmel_gencpp)
add_dependencies(master_thesis_kremmel_gencpp master_thesis_kremmel_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS master_thesis_kremmel_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/master_thesis_kremmel
)
_generate_srv_eus(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/master_thesis_kremmel
)

### Generating Module File
_generate_module_eus(master_thesis_kremmel
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/master_thesis_kremmel
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(master_thesis_kremmel_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(master_thesis_kremmel_generate_messages master_thesis_kremmel_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_eus _master_thesis_kremmel_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_eus _master_thesis_kremmel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(master_thesis_kremmel_geneus)
add_dependencies(master_thesis_kremmel_geneus master_thesis_kremmel_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS master_thesis_kremmel_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/master_thesis_kremmel
)
_generate_srv_lisp(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/master_thesis_kremmel
)

### Generating Module File
_generate_module_lisp(master_thesis_kremmel
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/master_thesis_kremmel
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(master_thesis_kremmel_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(master_thesis_kremmel_generate_messages master_thesis_kremmel_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_lisp _master_thesis_kremmel_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_lisp _master_thesis_kremmel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(master_thesis_kremmel_genlisp)
add_dependencies(master_thesis_kremmel_genlisp master_thesis_kremmel_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS master_thesis_kremmel_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/master_thesis_kremmel
)
_generate_srv_nodejs(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/master_thesis_kremmel
)

### Generating Module File
_generate_module_nodejs(master_thesis_kremmel
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/master_thesis_kremmel
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(master_thesis_kremmel_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(master_thesis_kremmel_generate_messages master_thesis_kremmel_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_nodejs _master_thesis_kremmel_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_nodejs _master_thesis_kremmel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(master_thesis_kremmel_gennodejs)
add_dependencies(master_thesis_kremmel_gennodejs master_thesis_kremmel_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS master_thesis_kremmel_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel
)
_generate_srv_py(master_thesis_kremmel
  "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel
)

### Generating Module File
_generate_module_py(master_thesis_kremmel
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(master_thesis_kremmel_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(master_thesis_kremmel_generate_messages master_thesis_kremmel_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/Landmark.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_py _master_thesis_kremmel_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/max/master_thesis_kremmel/master_thesis_ros/catkin_ws/src/master_thesis_kremmel/srv/MoveRobot.srv" NAME_WE)
add_dependencies(master_thesis_kremmel_generate_messages_py _master_thesis_kremmel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(master_thesis_kremmel_genpy)
add_dependencies(master_thesis_kremmel_genpy master_thesis_kremmel_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS master_thesis_kremmel_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/master_thesis_kremmel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/master_thesis_kremmel
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(master_thesis_kremmel_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(master_thesis_kremmel_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/master_thesis_kremmel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/master_thesis_kremmel
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(master_thesis_kremmel_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(master_thesis_kremmel_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/master_thesis_kremmel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/master_thesis_kremmel
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(master_thesis_kremmel_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(master_thesis_kremmel_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/master_thesis_kremmel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/master_thesis_kremmel
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(master_thesis_kremmel_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(master_thesis_kremmel_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/master_thesis_kremmel
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(master_thesis_kremmel_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(master_thesis_kremmel_generate_messages_py sensor_msgs_generate_messages_py)
endif()
