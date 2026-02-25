# openmmarm_controller

OpenMMArm 控制核心，负责状态机调度、底层 IO 收发和 SDK 指令接入。

## 力矩控制原理

- [TORQUE_CONTROL_MATH.md](TORQUE_CONTROL_MATH.md)：当前代码实现下的力矩控制数学说明（真机/仿真两条链路）

## 架构简述

`openmmarm_controller` 在系统中位于“控制层”：

- 上行：通过 `CmdSdk`（UDP）接收 `ArmCmd`，回传 `ArmState`
- 内部：`FiniteStateMachine` 管理控制状态（当前实现 `PASSIVE`、`JOINT_CTRL`）
- 下行：通过 `IOInterface` 对接
  - `IOROS`（仿真 Topic）
  - `IOUDP`（真机 MCU UDP）

## 使用方法

### 1. 直接启动（仿真模式）

```bash
ros2 run openmmarm_controller openmmarm_ctrl --ros-args -p communication:=ROS
```

### 2. 直接启动（真机模式）

```bash
ros2 run openmmarm_controller openmmarm_ctrl --ros-args \
  -p communication:=UDP \
  -p udp.mcu_ip:=192.168.123.110 \
  -p udp.mcu_port:=8881 \
  -p udp.local_port:=8882 \
  -p udp.sdk_port:=8871
```

说明：`udp.local_port`（controller 与 MCU 通道）和 `udp.sdk_port`（SDK 客户端通道）建议使用不同端口。

### 3. 使用参数文件启动

```bash
ros2 run openmmarm_controller openmmarm_ctrl --ros-args \
  --params-file $(ros2 pkg prefix openmmarm_controller)/share/openmmarm_controller/config/openmmarm_controller.yaml
```

### 4. 启动后检查

```bash
ros2 node list | rg openmmarm_controller
ros2 param get /openmmarm_controller communication
```

若有 SDK 客户端（例如 `openmmarm_hw` 或 `openmmarm_sdk`）连接，可再检查端口监听：

```bash
ss -lun | rg 8871
```

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `communication` | `ROS` | 下行通信类型：`ROS` 或 `UDP` |
| `udp.mcu_ip` | `192.168.123.110` | 真机 MCU IP |
| `udp.mcu_port` | `8881` | 真机 MCU 端口 |
| `udp.local_port` | `8871` | 本地 UDP 绑定端口（MCU 通道） |
| `udp.sdk_port` | `8871` | SDK 指令端口（`ArmCmd`/`ArmState`） |
| `collision.open` | `true` | 是否开启碰撞检测 |
| `collision.limit_torque` | `10.0` | 碰撞力矩阈值 |

## 与其他包的配合

- 与 `openmmarm_hw` 配合：由 `ros2_control` 写入关节目标
- 与 `openmmarm_sdk` 配合：外部程序直接通过 UDP 控制
- 与 `openmmarm_bringup` 配合：统一启动完整流程
