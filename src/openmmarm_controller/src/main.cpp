/**
 * @file main.cpp
 * @brief OpenMMARM 机械臂控制器 ROS 2 入口
 *
 * 该节点实现了 OpenMMARM 机械臂的控制逻辑，
 * 支持多种控制模式（被动、关节控制、笛卡尔控制等）。
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "cmd/UdpSdk.h"
#include "ctrl/CtrlComponents.h"
#include "fsm/FiniteStateMachine.h"
#include "fsm/State_JointCtrl.h"
#include "fsm/State_Passive.h"
#include "io/IOMujoco.h"
#include "io/IOUDP.h"

// 全局运行标志
static std::atomic<bool> g_running{true};

void signalHandler(int signum) {
  (void)signum;
  g_running = false;
  RCLCPP_INFO(rclcpp::get_logger("openmmarm_controller"),
              "收到退出信号，正在关闭...");
}

class OpenMMArmControllerNode : public rclcpp::Node {
public:
  OpenMMArmControllerNode() : Node("openmmarm_controller") {
    // 声明参数
    this->declare_parameter<std::string>("communication", "SIM");
    this->declare_parameter<std::string>("udp.mcu_ip", "192.168.123.110");
    this->declare_parameter<int>("udp.mcu_port", 8881);
    this->declare_parameter<int>("udp.local_port", 8871);
    this->declare_parameter<int>("udp.sdk_port", 8871);
    this->declare_parameter<std::string>("sim.model_path", "");
    this->declare_parameter<bool>("collision.open", true);
    this->declare_parameter<double>("collision.limit_torque", 10.0);

    // 初始化控制组件
    ctrlComp_ = std::make_shared<CtrlComponents>();

    // 获取包路径
    try {
      ctrlComp_->projectPath =
          ament_index_cpp::get_package_share_directory("openmmarm_controller");
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "无法获取包路径: %s", e.what());
      ctrlComp_->projectPath = ".";
    }

    // 读取参数
    ctrlComp_->collisionOpen = this->get_parameter("collision.open").as_bool();
    ctrlComp_->collisionLimitT =
        this->get_parameter("collision.limit_torque").as_double();

    RCLCPP_INFO(this->get_logger(), "OpenMMARM Controller 节点参数已声明");
  }

  bool initialize() {
    // 注意：此方法必须在 shared_ptr 构造完成后调用
    // 因为 IOROS 需要 shared_from_this()
    RCLCPP_INFO(this->get_logger(), "正在初始化 IO 和 SDK 接口...");

    // 读取通信模式参数
    std::string communication =
        this->get_parameter("communication").as_string();

    // 初始化上层 SDK 指令接口（无论 ROS/UDP 模式都创建，与 z1 一致）
    int sdk_port = this->get_parameter("udp.sdk_port").as_int();
    ctrlComp_->cmdSdk = std::make_shared<UdpSdk>(sdk_port, ctrlComp_->lowState);
    RCLCPP_INFO(this->get_logger(), "SDK 指令接口端口: %d", sdk_port);

    // 初始化 IO 接口（下行通信）
    // shared_from_this() 在此处安全调用，因为 shared_ptr 已构造完成
    if (communication == "UDP") {
      std::string mcu_ip = this->get_parameter("udp.mcu_ip").as_string();
      int mcu_port = this->get_parameter("udp.mcu_port").as_int();
      int local_port = this->get_parameter("udp.local_port").as_int();
      ctrlComp_->ioInter =
          std::make_shared<IOUDP>(mcu_ip, mcu_port, local_port);
      RCLCPP_INFO(this->get_logger(), "使用 UDP 通信模式 [%s:%d]",
                  mcu_ip.c_str(), mcu_port);
    } else if (communication == "SIM") {
      std::string model_path =
          this->get_parameter("sim.model_path").as_string();
      if (model_path.empty()) {
        // 默认使用 openmmarm_description 包中的 URDF
        try {
          std::string desc_path = ament_index_cpp::get_package_share_directory(
              "openmmarm_description");
          model_path = desc_path + "/urdf/openmmarm.urdf";
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "无法自动定位模型文件: %s",
                       e.what());
          return false;
        }
      }
      ctrlComp_->ioInter =
          std::make_shared<IOMujoco>(model_path, ctrlComp_->dt);
      RCLCPP_INFO(this->get_logger(), "使用 MuJoCo 仿真模式，模型: %s",
                  model_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "未知通信模式: %s，使用默认 SIM 模式",
                  communication.c_str());
      std::string model_path =
          this->get_parameter("sim.model_path").as_string();
      if (model_path.empty()) {
        try {
          std::string desc_path = ament_index_cpp::get_package_share_directory(
              "openmmarm_description");
          model_path = desc_path + "/urdf/openmmarm.urdf";
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "无法自动定位模型文件: %s",
                       e.what());
          return false;
        }
      }
      ctrlComp_->ioInter =
          std::make_shared<IOMujoco>(model_path, ctrlComp_->dt);
    }

    RCLCPP_INFO(this->get_logger(), "等待与机械臂建立连接...");

    if (!ctrlComp_->ioInter->init()) {
      RCLCPP_ERROR(this->get_logger(), "IO 接口初始化失败");
      return false;
    }

    // 初始化 SDK 指令接口
    if (ctrlComp_->cmdSdk && !ctrlComp_->cmdSdk->init()) {
      RCLCPP_WARN(this->get_logger(),
                  "SDK 指令接口初始化失败，将无法接收外部指令");
    }

    RCLCPP_INFO(this->get_logger(), "已连接到机械臂 [%s夹爪]",
                ctrlComp_->ioInter->hasGripper ? "有" : "无");

    // 创建状态
    std::vector<std::shared_ptr<FSMState>> states;
    states.push_back(std::make_shared<State_Passive>(ctrlComp_));
    states.push_back(std::make_shared<State_JointCtrl>(ctrlComp_));

    // 创建并启动状态机
    fsm_ = std::make_shared<FiniteStateMachine>(states, ctrlComp_);
    fsm_->start();

    RCLCPP_INFO(this->get_logger(), "状态机已启动");
    return true;
  }

  void shutdown() {
    if (fsm_) {
      fsm_->stop();
    }
    RCLCPP_INFO(this->get_logger(), "控制器已关闭");
  }

private:
  std::shared_ptr<CtrlComponents> ctrlComp_;
  std::shared_ptr<FiniteStateMachine> fsm_;
};

int main(int argc, char **argv) {
  // 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 注册信号处理
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // 创建节点
  auto node = std::make_shared<OpenMMArmControllerNode>();

  // 初始化
  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "控制器初始化失败");
    rclcpp::shutdown();
    return -1;
  }

  // ========== 双线程架构 ==========
  // 线程 1: Main Thread (当前线程) - 保活和生命周期管理
  // 线程 2: Control Loop Thread - 在 FSM::start() 中已启动
  // (MuJoCo 模式下不需要 ROS Executor 线程处理回调)

  RCLCPP_INFO(node->get_logger(), "双线程架构已启动: Main(1Hz) + Control(%dHz)",
              static_cast<int>(1.0 / 0.004));

  // 主循环 (1Hz 保活)
  rclcpp::Rate rate(1);
  while (rclcpp::ok() && g_running) {
    // 主线程只负责保活
    rate.sleep();
  }

  // 清理

  node->shutdown();
  rclcpp::shutdown();

  return 0;
}
