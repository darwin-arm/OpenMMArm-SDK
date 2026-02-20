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

# MuJoCo 仿真库
set(MUJOCO_DIR "$ENV{HOME}/anaconda3/lib/python3.12/site-packages/mujoco")

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${MUJOCO_DIR}/include
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

# 链接依赖
ament_target_dependencies(openmmarm_ctrl
  rclcpp
  ament_index_cpp
  openmmarm_sdk
)

# 链接 MuJoCo 库
target_link_libraries(openmmarm_ctrl
  ${MUJOCO_DIR}/libmujoco.so.3.3.7
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
