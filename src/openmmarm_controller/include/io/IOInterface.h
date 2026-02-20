#pragma once

#include "ctrl/CtrlComponents.h"

/**
 * @brief 硬件抽象接口基类
 *
 * 用于统一真机 (UDP) 和仿真 (MuJoCo) 的通信接口。
 */
class IOInterface {
public:
  IOInterface() = default;
  virtual ~IOInterface() = default;

  /**
   * @brief 初始化接口
   * @return 初始化是否成功
   */
  virtual bool init() = 0;

  /**
   * @brief 发送指令并接收状态
   * @param cmd 底层控制指令
   * @param state 底层状态反馈
   * @return 通信是否成功
   */
  virtual bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) = 0;

  /**
   * @brief 检查连接状态
   * @return 是否已连接
   */
  virtual bool isConnected() = 0;

  /**
   * @brief 检查是否已初始化
   */
  bool initialized() const { return initialized_; }

  // 是否有夹爪
  bool hasGripper = false;

protected:
  bool initialized_ = false;
};
