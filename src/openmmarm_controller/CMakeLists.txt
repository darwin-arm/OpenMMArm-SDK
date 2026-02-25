cmake_minimum_required(VERSION 3.8)
project(openmmarm_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(openmmarm_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW QUIET glfw3)

# MuJoCo 仿真库
file(GLOB MUJOCO_HOME_DIRS "$ENV{HOME}/.mujoco/mujoco-*")
file(GLOB MUJOCO_CONDA_DIRS "$ENV{HOME}/anaconda3/lib/python*/site-packages/mujoco")
set(MUJOCO_ROOT_HINTS
  "$ENV{MUJOCO_DIR}"
  "$ENV{MUJOCO_HOME}"
  ${MUJOCO_HOME_DIRS}
  ${MUJOCO_CONDA_DIRS}
)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${MUJOCO_ROOT_HINTS}
  PATH_SUFFIXES include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  HINTS ${MUJOCO_ROOT_HINTS}
  PATH_SUFFIXES lib
)
if(NOT MUJOCO_LIBRARY)
  find_library(MUJOCO_LIBRARY
    NAMES mujoco libmujoco
    HINTS ${MUJOCO_ROOT_HINTS}
  )
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo not found. Please set MUJOCO_DIR or MUJOCO_HOME to MuJoCo root "
    "(must contain include/mujoco/mujoco.h and lib/libmujoco.so, or libmujoco.so in root)."
  )
endif()

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIR}
)

# 可执行文件
add_executable(openmmarm_ctrl
  src/main.cpp
  src/cmd/CmdSdk.cpp
  src/cmd/UdpSdk.cpp
  src/io/IOMujoco.cpp
  src/io/IOUDP.cpp
  src/fsm/FiniteStateMachine.cpp
  src/fsm/FSMState.cpp
  src/fsm/State_Passive.cpp
  src/fsm/State_JointCtrl.cpp
)

if(GLFW_FOUND)
  target_include_directories(openmmarm_ctrl PRIVATE ${GLFW_INCLUDE_DIRS})
  target_link_libraries(openmmarm_ctrl ${GLFW_LIBRARIES})
  target_compile_definitions(openmmarm_ctrl PRIVATE OPENMMARM_HAS_GLFW=1)
  message(STATUS "GLFW found, MuJoCo viewer support enabled")
else()
  message(WARNING "GLFW not found, MuJoCo viewer support disabled")
endif()

# 链接依赖
ament_target_dependencies(openmmarm_ctrl
  rclcpp
  ament_index_cpp
  openmmarm_sdk
)

# 链接 MuJoCo 库和 yaml-cpp
target_link_libraries(openmmarm_ctrl
  ${MUJOCO_LIBRARY}
  yaml-cpp
)

# 安装
install(TARGETS openmmarm_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 自动将 compile_commands.json 软链接到工程源码根目录（供 clangd 使用）
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  add_custom_target(
    link_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
  )
endif()

ament_package()
