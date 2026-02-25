#include "fsm/State_JointCtrl.h"
#include "cmd/CmdSdk.h"

State_JointCtrl::State_JointCtrl(std::shared_ptr<CtrlComponents> ctrlComp)
    : FSMState(ctrlComp, ArmFSMState::JOINT_CTRL, "JointCtrl") {}

void State_JointCtrl::enter() {
  // 进入关节控制模式
  // 以当前位置为初始目标，防止切换瞬间的力矩突跳

  // 根据控制模式设定不同的 PD 增益
  float default_kp, default_kd;
  if (ctrlComp_->controlMode == "impedance") {
    // 阻抗模式：默认关节刚度/阻尼（可被 SDK 动态覆盖）
    default_kp = 25.0f;
    default_kd = 10.0f;
  } else {
    // Position 模式：增益由 MuJoCo 内建隐式积分器使用
    // 此处 kp/kd 不参与物理计算，仅作占位
    default_kp = 20.0f;
    default_kd = 5.0f;
  }

  for (int i = 0; i < 6; ++i) {
    ctrlComp_->lowCmd->mode[i] = 10; // 位置控制模式
    ctrlComp_->lowCmd->kp[i] = default_kp;
    ctrlComp_->lowCmd->kd[i] = default_kd;
    ctrlComp_->lowCmd->q[i] = ctrlComp_->lowState->q[i];
    ctrlComp_->lowCmd->dq[i] = 0.0f;
    ctrlComp_->lowCmd->tau[i] = 0.0f;
  }
  // 同步更新 fsmArmCmd，使 setJointCmd() 在收到 SDK 指令前保持当前位置
  for (int i = 0; i < 6; ++i) {
    ctrlComp_->fsmArmCmd->q(i) = ctrlComp_->lowState->q[i];
    ctrlComp_->fsmArmCmd->dq(i) = 0.0;
    ctrlComp_->fsmArmCmd->tau(i) = 0.0;
  }
}

void State_JointCtrl::run() { setJointCmd(); }

void State_JointCtrl::setJointCmd() {
  // 如果 SDK 已连接，从 armCmd 读取目标指令
  if (ctrlComp_->cmdSdk && ctrlComp_->cmdSdk->isConnected()) {
    const auto &cmd = ctrlComp_->cmdSdk->armCmd;
    for (int i = 0; i < 6; ++i) {
      ctrlComp_->fsmArmCmd->q(i) = cmd.q_d[i];
      ctrlComp_->fsmArmCmd->dq(i) = cmd.dq_d[i];
      ctrlComp_->fsmArmCmd->tau(i) = cmd.tau_d[i];
    }

    // 允许 SDK 动态设置 PD 增益
    bool hasKp = false;
    for (int i = 0; i < 6; ++i) {
      if (cmd.Kp[i] > 0.0) {
        hasKp = true;
        break;
      }
    }
    if (hasKp) {
      for (int i = 0; i < 6; ++i) {
        ctrlComp_->lowCmd->kp[i] = static_cast<float>(cmd.Kp[i]);
        ctrlComp_->lowCmd->kd[i] = static_cast<float>(cmd.Kd[i]);
      }
    }
  }

  // 从 fsmArmCmd 写入 lowCmd
  for (int i = 0; i < 6; ++i) {
    ctrlComp_->lowCmd->q[i] = static_cast<float>(ctrlComp_->fsmArmCmd->q(i));
    ctrlComp_->lowCmd->dq[i] = static_cast<float>(ctrlComp_->fsmArmCmd->dq(i));
    ctrlComp_->lowCmd->tau[i] =
        static_cast<float>(ctrlComp_->fsmArmCmd->tau(i));
  }
}

void State_JointCtrl::exit() {
  // 退出关节控制模式
}

ArmFSMState State_JointCtrl::checkChange() {
  // 从 SDK 接口读取外部指令
  if (ctrlComp_->cmdSdk && ctrlComp_->cmdSdk->isConnected()) {
    ArmFSMState target = ctrlComp_->cmdSdk->getTargetFSMState();
    // JointCtrl 状态允许切换回 PASSIVE
    if (target == ArmFSMState::PASSIVE) {
      return ArmFSMState::PASSIVE;
    }
  }
  return ArmFSMState::JOINT_CTRL;
}
