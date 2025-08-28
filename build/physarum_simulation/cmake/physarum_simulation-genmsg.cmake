# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "physarum_simulation: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iphysarum_simulation:/home/dieisson/physarum_ws/src/physarum_simulation/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(physarum_simulation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_custom_target(_physarum_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "physarum_simulation" "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" ""
)

get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_custom_target(_physarum_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "physarum_simulation" "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" ""
)

get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_custom_target(_physarum_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "physarum_simulation" "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation
)
_generate_msg_cpp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation
)

### Generating Services
_generate_srv_cpp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation
)

### Generating Module File
_generate_module_cpp(physarum_simulation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(physarum_simulation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(physarum_simulation_generate_messages physarum_simulation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_cpp _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_cpp _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_cpp _physarum_simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(physarum_simulation_gencpp)
add_dependencies(physarum_simulation_gencpp physarum_simulation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS physarum_simulation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation
)
_generate_msg_eus(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation
)

### Generating Services
_generate_srv_eus(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation
)

### Generating Module File
_generate_module_eus(physarum_simulation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(physarum_simulation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(physarum_simulation_generate_messages physarum_simulation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_eus _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_eus _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_eus _physarum_simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(physarum_simulation_geneus)
add_dependencies(physarum_simulation_geneus physarum_simulation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS physarum_simulation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation
)
_generate_msg_lisp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation
)

### Generating Services
_generate_srv_lisp(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation
)

### Generating Module File
_generate_module_lisp(physarum_simulation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(physarum_simulation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(physarum_simulation_generate_messages physarum_simulation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_lisp _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_lisp _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_lisp _physarum_simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(physarum_simulation_genlisp)
add_dependencies(physarum_simulation_genlisp physarum_simulation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS physarum_simulation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation
)
_generate_msg_nodejs(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation
)

### Generating Services
_generate_srv_nodejs(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation
)

### Generating Module File
_generate_module_nodejs(physarum_simulation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(physarum_simulation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(physarum_simulation_generate_messages physarum_simulation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_nodejs _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_nodejs _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_nodejs _physarum_simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(physarum_simulation_gennodejs)
add_dependencies(physarum_simulation_gennodejs physarum_simulation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS physarum_simulation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation
)
_generate_msg_py(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation
)

### Generating Services
_generate_srv_py(physarum_simulation
  "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation
)

### Generating Module File
_generate_module_py(physarum_simulation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(physarum_simulation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(physarum_simulation_generate_messages physarum_simulation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ContainerTask.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_py _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/msg/ColetaEvent.msg" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_py _physarum_simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dieisson/physarum_ws/src/physarum_simulation/srv/LockTask.srv" NAME_WE)
add_dependencies(physarum_simulation_generate_messages_py _physarum_simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(physarum_simulation_genpy)
add_dependencies(physarum_simulation_genpy physarum_simulation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS physarum_simulation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/physarum_simulation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(physarum_simulation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/physarum_simulation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(physarum_simulation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/physarum_simulation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(physarum_simulation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/physarum_simulation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(physarum_simulation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/physarum_simulation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(physarum_simulation_generate_messages_py std_msgs_generate_messages_py)
endif()
