# OpenMMArm Controller 力矩控制数学原理

本文基于 `openmmarm_controller` 当前实现，说明控制指令如何转化为关节力矩。

## 1. 控制闭环与信号流

控制循环由 `FiniteStateMachine::run()` 驱动，默认 `dt = 0.004s`（250Hz）：

1. `State_JointCtrl::setJointCmd()` 从 SDK 的 `ArmCmd` 读取：
   - `q_d, dq_d, tau_d, Kp, Kd`
2. 写入 `LowLevelCmd`：
   - `q, dq, tau, kp, kd`
3. `IOInterface::sendRecv()` 执行下行控制并回读 `LowLevelState`
4. `UdpSdk::updateArmState()` 把反馈回传给上层

所以“力矩真正如何生成”取决于 IO 实现：真机 `IOUDP` 或仿真 `IOMujoco`。

## 2. 符号约定

- `q, dq`：当前关节位置/速度
- `q_d, dq_d`：目标关节位置/速度
- `tau_d`：前馈力矩
- `Kp, Kd`：比例/微分增益（逐关节）
- `M(q)`：惯性矩阵
- `h(q,dq) = C(q,dq)*dq + g(q)`：偏置项（科氏/离心+重力）
- `tau_ext`：外部作用到关节的力矩（接触、人推拽等）

## 3. 协议层控制律（SDK 语义）

SDK 协议定义的统一控制律：

```text
tau = Kp * (q_d - q) + Kd * (dq_d - dq) + tau_d
```

对应：`include/sdk/openmmarm_arm_common.h`

## 4. 当前实现中的执行语义

### 4.1 真机 UDP 模式

`IOUDP::sendCmd()` 只负责把 `q,dq,tau,kp,kd` 发给 MCU。  
控制器进程内不做动力学计算，电机实际力矩由 MCU 固件决定。

### 4.2 MuJoCo impedance 模式（已改为标准关节阻抗力矩控制）

`IOMujoco::sendRecv()` 在 `impedance` 下执行：

```text
tau = h(q,dq) + Kp * (q_d - q) + Kd * (dq_d - dq) + tau_d
```

其中 `h(q,dq)` 在代码中是 `qfrc_bias`。

若定义阻抗误差 `e_tilde = q - q_d`（注意这是阻抗分析常用符号），
在参考加速度近似为 0（`ddq_d ~= 0`）时，闭环可写成：

```text
M(q) * e_tilde_ddot + Kd * e_tilde_dot + Kp * e_tilde = tau_ext + tau_d
```

这就是标准关节阻抗的形式：外力 `tau_ext` 会自然进入误差动力学右端。

### 4.3 MuJoCo position 模式

控制器只写 `ctrl = q_d`，由 MuJoCo `<position>` actuator 内部完成隐式 PD。

### 4.4 无 actuator 的后备路径

如果模型没有可用 actuator，则直接施加：

```text
tau = Kp * (q_d - q) + Kd * (dq_d - dq) + tau_d
```

这条路径与 SDK 协议一致，但不包含 `h(q,dq)` 补偿。

## 5. 与旧实现的差异

旧的 MuJoCo `impedance` 分支是：

```text
tau = M(q) * (Kp * (q_d - q) + Kd * (dq_d - dq)) + h(q,dq)
```

存在两个问题：

1. `tau_d` 未参与控制，和 SDK 协议不一致
2. `Kp/Kd` 被当作“加速度层增益”，不符合常见关节阻抗参数语义

现在已改为第 4.2 节的标准关节阻抗力矩形式，并保留 `tau_d` 前馈项。

## 6. 力矩限幅与仿真安全

- `impedance` 模式注入 `<motor>` actuator，`ctrlrange = [-10, 10]`
- `position` 模式 `<position>` actuator，`forcerange = [-10, 10]`
- 仿真在收到首个有效控制指令（任一关节 `kp > 0`）前不步进

## 7. 关键代码位置

- `src/io/IOMujoco.cpp`：仿真端力矩生成（核心）
- `src/io/IOUDP.cpp`：真机 UDP 下发
- `src/fsm/State_JointCtrl.cpp`：SDK 指令到 `LowLevelCmd` 映射
- `include/sdk/openmmarm_arm_common.h`：协议层控制律定义
