#include "fsm/State_Passive.h"
#include "cmd/CmdSdk.h"

State_Passive::State_Passive(std::shared_ptr<CtrlComponents> ctrlComp)
    : FSMState(ctrlComp, ArmFSMState::PASSIVE, "Passive") {}

void State_Passive::enter() {
  // Passive 模式：将机械臂控制在零位（所有关节角度为 0）
  for (int i = 0; i < 6; ++i) {
    ctrlComp_->lowCmd->mode[i] = 10; // 位置控制模式
    ctrlComp_->lowCmd->q[i] = 0.0f; // 目标位置：零位
    ctrlComp_->lowCmd->dq[i] = 0.0f;
    ctrlComp_->lowCmd->tau[i] = 0.0f;
    ctrlComp_->lowCmd->kp[i] = 20.0f; // 刚度
    ctrlComp_->lowCmd->kd[i] = 5.0f;  // 阻尼
  }
}

void State_Passive::run() {
  // 目标位置在 enter() 中已固定为零位，无需每帧更新
}

void State_Passive::exit() {
  // 退出被动模式时的清理工作
}

ArmFSMState State_Passive::checkChange() {
  // 从 SDK 接口读取外部指令
  if (ctrlComp_->cmdSdk && ctrlComp_->cmdSdk->isConnected()) {
    ArmFSMState target = ctrlComp_->cmdSdk->getTargetFSMState();
    // Passive 状态允许切换到 JOINT_CTRL
    if (target == ArmFSMState::JOINT_CTRL) {
      return ArmFSMState::JOINT_CTRL;
    }
  }
  return ArmFSMState::PASSIVE;
}
