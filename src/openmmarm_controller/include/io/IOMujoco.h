#pragma once

#include "io/IOInterface.h"
#include <filesystem>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

/**
 * @brief MuJoCo 仿真接口实现
 *
 * 在进程内嵌入 MuJoCo 物理引擎，直接步进仿真。
 * 每次 sendRecv 调用会将控制指令写入 MuJoCo，
 * 执行一步仿真，然后读回关节状态。
 *
 * 加载流程：
 *   1. 解析 URDF 中的 package:// URI，定位所有 mesh 文件
 *   2. 将 mesh 文件符号链接到 URDF 所在目录（MuJoCo 只在此目录搜索）
 *   3. 将 mesh filename 简化为 basename，写入临时 URDF
 *   4. 使用 mj_loadXML 加载临时 URDF
 *   5. 退出时自动清理符号链接和临时文件
 */
class IOMujoco : public IOInterface {
public:
  /**
   * @brief 构造函数
   * @param model_path URDF 或 MJCF 模型文件的绝对路径
   * @param timestep 仿真时间步长 (秒)，默认 0.004 对应 250Hz
   */
  explicit IOMujoco(const std::string &model_path, double timestep = 0.004);
  ~IOMujoco() override;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  std::string model_path_;          // 原始模型文件路径
  std::string resolved_model_path_; // 预处理后的临时模型文件路径
  double timestep_;

  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  bool is_connected_ = false;

  // init() 期间创建的符号链接，析构时清理
  std::vector<std::filesystem::path> created_symlinks_;

  // 关节数量
  static constexpr int NUM_JOINTS = 6;
};
