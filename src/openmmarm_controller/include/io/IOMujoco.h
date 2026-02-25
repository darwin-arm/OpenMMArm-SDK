#pragma once

#include "io/IOInterface.h"
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;

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
   * @param enable_viewer 是否启用 MuJoCo 可视化窗口
   * @param control_mode 控制模式: "position" (隐式 PD) 或 "impedance"
   * (关节阻抗力矩控制)
   */
  explicit IOMujoco(const std::string &model_path, double timestep = 0.004,
                    bool enable_viewer = false,
                    const std::string &control_mode = "impedance");
  ~IOMujoco() override;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  bool initViewer();
  void viewerLoop();
  void closeViewer();

  // GLFW 鼠标交互回调（静态函数，通过 userPointer 访问实例）
  static void mouseButtonCallback(GLFWwindow *w, int button, int action,
                                  int mods);
  static void cursorPosCallback(GLFWwindow *w, double xpos, double ypos);
  static void scrollCallback(GLFWwindow *w, double xoffset, double yoffset);

  std::string model_path_;          // 原始模型文件路径
  std::string resolved_model_path_; // 预处理后的临时模型文件路径
  double timestep_;
  bool enable_viewer_ = false;
  std::string control_mode_ =
      "impedance"; // "position" 或 "impedance"(关节阻抗)
  std::atomic<bool> viewer_initialized_{false};
  std::atomic<bool> viewer_stop_requested_{false};
  std::thread viewer_thread_;
  std::mutex sim_mutex_;
  std::mutex viewer_state_mutex_;
  std::condition_variable viewer_state_cv_;
  bool viewer_init_done_ = false;
  bool viewer_init_ok_ = false;

  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  GLFWwindow *window_ = nullptr;
  mjvCamera camera_;
  mjvOption option_;
  mjvScene scene_;
  mjrContext context_;

  bool is_connected_ = false;

  // 仿真是否已开始推进（收到首个有效控制指令后置 true）
  bool sim_started_ = false;

  // init() 期间创建的符号链接，析构时清理
  std::vector<std::filesystem::path> created_symlinks_;

  // 鼠标交互状态
  bool mouse_button_left_ = false;
  bool mouse_button_middle_ = false;
  bool mouse_button_right_ = false;
  double mouse_last_x_ = 0.0;
  double mouse_last_y_ = 0.0;

  // 关节数量
  static constexpr int NUM_JOINTS = 6;
};
