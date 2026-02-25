#include "io/IOMujoco.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>

#ifdef OPENMMARM_HAS_GLFW
#include <GLFW/glfw3.h>
#endif

namespace fs = std::filesystem;

/**
 * @brief 收集 URDF 中所有 mesh 文件的绝对路径
 *
 * 解析 URDF 内容中的 package://包名/子路径 格式的 mesh 引用，
 * 通过 ament_index 将 package:// URI 转换为磁盘上的绝对路径。
 *
 * @param xml_content URDF 文件的原始内容
 * @return 所有 mesh 文件的绝对路径列表
 */
static std::vector<fs::path> collectMeshPaths(const std::string &xml_content) {
  std::vector<fs::path> mesh_paths;

  // 匹配 filename="package://包名/子路径/文件名.ext"
  std::regex mesh_re(R"RE(filename="package://([a-zA-Z0-9_]+)/([^"]+)")RE");
  auto begin =
      std::sregex_iterator(xml_content.begin(), xml_content.end(), mesh_re);
  auto end = std::sregex_iterator();

  // 缓存已解析的包路径
  std::map<std::string, fs::path> pkg_cache;

  for (auto it = begin; it != end; ++it) {
    std::string pkg_name = (*it)[1].str();
    std::string sub_path = (*it)[2].str();

    // 查找或缓存包的 share 目录
    if (pkg_cache.find(pkg_name) == pkg_cache.end()) {
      try {
        std::string share_dir =
            ament_index_cpp::get_package_share_directory(pkg_name);
        pkg_cache[pkg_name] = fs::path(share_dir);
      } catch (const std::exception &e) {
        std::cerr << "[IOMujoco] 无法解析包 " << pkg_name << ": " << e.what()
                  << std::endl;
        continue;
      }
    }

    fs::path full_path = pkg_cache[pkg_name] / sub_path;
    mesh_paths.push_back(full_path);
  }

  // 去重
  std::sort(mesh_paths.begin(), mesh_paths.end());
  mesh_paths.erase(std::unique(mesh_paths.begin(), mesh_paths.end()),
                   mesh_paths.end());

  return mesh_paths;
}

/**
 * @brief 将 mesh 文件通过符号链接放置到目标目录
 *
 * MuJoCo 的 URDF parser 在加载 mesh 时只取文件的 basename，
 * 然后在 XML 文件所在目录搜索。因此需要确保 mesh 文件
 * （或指向它们的符号链接）存在于 XML 所在目录中。
 *
 * @param mesh_paths mesh 文件的绝对路径列表
 * @param target_dir 目标目录 (XML 所在目录)
 * @return 创建的符号链接路径列表 (用于后续清理)
 */
static std::vector<fs::path>
linkMeshesToDirectory(const std::vector<fs::path> &mesh_paths,
                      const fs::path &target_dir) {
  std::vector<fs::path> created_links;

  for (const auto &mesh_path : mesh_paths) {
    if (!fs::exists(mesh_path)) {
      std::cerr << "[IOMujoco] 警告: mesh 文件不存在: " << mesh_path
                << std::endl;
      continue;
    }

    fs::path link_path = target_dir / mesh_path.filename();

    // 如果目标已经存在 (可能是同名文件或之前的链接)
    if (fs::exists(link_path) || fs::is_symlink(link_path)) {
      // 如果已经指向了正确的目标，跳过
      if (fs::is_symlink(link_path) &&
          fs::read_symlink(link_path) == mesh_path) {
        continue;
      }
      // 否则删除旧的链接
      fs::remove(link_path);
    }

    try {
      fs::create_symlink(mesh_path, link_path);
      created_links.push_back(link_path);
      std::cout << "[IOMujoco]   链接: " << mesh_path.filename() << " -> "
                << mesh_path << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[IOMujoco] 无法创建符号链接 " << link_path << ": "
                << e.what() << std::endl;
    }
  }

  return created_links;
}

/**
 * @brief 预处理 URDF 内容，将 mesh filename 简化为 basename
 *
 * MuJoCo 的 URDF parser 只使用 mesh filename 的 basename，
 * 因此将 package://包名/路径/文件名 简化为仅文件名即可。
 *
 * @param xml_content 原始 URDF 内容
 * @return 处理后的 URDF 内容
 */
static std::string simplifyMeshFilenames(const std::string &xml_content) {
  // 将 filename="package://包名/子路径/文件名.ext"
  // 替换为 filename="文件名.ext"
  std::regex mesh_re(
      R"RE(filename="package://[a-zA-Z0-9_]+/(?:[^"/]+/)*([^"]+)")RE");
  return std::regex_replace(xml_content, mesh_re, R"RE(filename="$1")RE");
}

IOMujoco::IOMujoco(const std::string &model_path, double timestep,
                   bool enable_viewer, const std::string &control_mode)
    : model_path_(model_path), timestep_(timestep),
      enable_viewer_(enable_viewer), control_mode_(control_mode) {}

IOMujoco::~IOMujoco() {
  closeViewer();

  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  // 清理创建的符号链接
  for (const auto &link : created_symlinks_) {
    if (fs::is_symlink(link)) {
      fs::remove(link);
    }
  }

  // 清理临时模型文件
  if (!resolved_model_path_.empty()) {
    std::remove(resolved_model_path_.c_str());
  }
}

bool IOMujoco::init() {
  // 确认原始模型文件存在
  fs::path model_fs_path(model_path_);
  if (!fs::exists(model_fs_path)) {
    std::cerr << "[IOMujoco] 模型文件不存在: " << model_path_ << std::endl;
    return false;
  }

  // 获取模型文件所在目录
  fs::path model_dir = fs::canonical(model_fs_path.parent_path());

  // 读取原始模型文件
  std::ifstream ifs(model_path_);
  if (!ifs.is_open()) {
    std::cerr << "[IOMujoco] 无法打开模型文件: " << model_path_ << std::endl;
    return false;
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();
  std::string original_content = ss.str();

  // 第一步：收集所有 mesh 文件的绝对路径
  std::vector<fs::path> mesh_paths = collectMeshPaths(original_content);
  std::cout << "[IOMujoco] 发现 " << mesh_paths.size() << " 个 mesh 文件"
            << std::endl;

  // 第二步：将 mesh 文件符号链接到模型文件所在目录
  created_symlinks_ = linkMeshesToDirectory(mesh_paths, model_dir);
  std::cout << "[IOMujoco] 创建了 " << created_symlinks_.size() << " 个符号链接"
            << std::endl;

  // 第三步：将 URDF 中的 mesh filename 简化为 basename
  std::string xml_content = simplifyMeshFilenames(original_content);

  // 将预处理后的 URDF 写入临时文件 (保持在同目录，MuJoCo 会在此目录搜索 mesh)
  resolved_model_path_ =
      (model_dir / (model_fs_path.stem().string() + "_mujoco.xml")).string();

  std::ofstream ofs(resolved_model_path_);
  if (!ofs.is_open()) {
    std::cerr << "[IOMujoco] 无法写入临时文件: " << resolved_model_path_
              << std::endl;
    return false;
  }
  ofs << xml_content;
  ofs.close();

  std::cout << "[IOMujoco] 已生成预处理模型: " << resolved_model_path_
            << std::endl;

  // === 两步加载策略 ===
  // MuJoCo 的 URDF parser 不识别 <mujoco> 扩展标签，因此无法在 URDF 中定义
  // actuator。 策略：先以 URDF 加载 → 导出为原生 MJCF → 在 MJCF 中注入
  // <actuator> → 重新加载。

  // 第一步：以 URDF 格式加载模型（此时 nu=0）
  char error[1000] = "";
  mjModel *urdf_model =
      mj_loadXML(resolved_model_path_.c_str(), nullptr, error, sizeof(error));

  if (!urdf_model) {
    std::cerr << "[IOMujoco] URDF 模型加载失败: " << error << std::endl;
    return false;
  }

  // 第二步：检查是否需要注入 actuator
  if (urdf_model->nu == 0) {
    std::cout
        << "[IOMujoco] URDF 模型中 nu=0，正在通过 MJCF 中间格式注入 actuator..."
        << std::endl;

    // 导出为原生 MJCF 格式
    std::string mjcf_path =
        (model_dir / (model_fs_path.stem().string() + "_mjcf.xml")).string();

    if (!mj_saveLastXML(mjcf_path.c_str(), urdf_model, error, sizeof(error))) {
      std::cerr << "[IOMujoco] 导出 MJCF 失败: " << error << std::endl;
      mj_deleteModel(urdf_model);
      return false;
    }

    // 读取导出的 MJCF
    std::ifstream mjcf_ifs(mjcf_path);
    if (!mjcf_ifs.is_open()) {
      std::cerr << "[IOMujoco] 无法打开导出的 MJCF: " << mjcf_path << std::endl;
      mj_deleteModel(urdf_model);
      return false;
    }
    std::stringstream mjcf_ss;
    mjcf_ss << mjcf_ifs.rdbuf();
    mjcf_ifs.close();
    std::string mjcf_content = mjcf_ss.str();

    // 在 MJCF 中注入 <actuator> 段（在 </mujoco> 前插入）
    // 收集所有 joint 名称
    std::vector<std::string> joint_names;
    for (int i = 0; i < urdf_model->njnt; ++i) {
      std::string jname(mj_id2name(urdf_model, mjOBJ_JOINT, i));
      // 跳过 fixed joint（MuJoCo 内部可能不会导出 fixed joint，但安全起见检查）
      if (urdf_model->jnt_type[i] != mjJNT_FREE) {
        joint_names.push_back(jname);
      }
    }

    if (!joint_names.empty()) {
      std::stringstream actuator_block;
      actuator_block << "\n  <actuator>\n";
      for (const auto &jname : joint_names) {
        if (control_mode_ == "position") {
          // PID 模式：使用 MuJoCo 内建 position actuator
          // PD 控制在隐式积分器内完成，适合简单场景
          actuator_block << "    <position name=\"pos_" << jname
                         << "\" joint=\"" << jname << "\" kp=\"100\" kv=\"10\""
                         << " ctrllimited=\"true\" ctrlrange=\"-6.28 6.28\""
                         << " forcelimited=\"true\" forcerange=\"-50 50\"/>\n";
        } else {
          // Impedance 模式：使用 motor actuator，显式写入力矩控制量
          // 限幅保护防止极端情况
          actuator_block << "    <motor name=\"motor_" << jname << "\" joint=\""
                         << jname << "\" gear=\"1\""
                         << " ctrllimited=\"true\" ctrlrange=\"-10 10\"/>\n";
        }
      }
      actuator_block << "  </actuator>\n";

      // 在 </mujoco> 前插入
      size_t pos = mjcf_content.rfind("</mujoco>");
      if (pos != std::string::npos) {
        mjcf_content.insert(pos, actuator_block.str());
      }
    }

    // 写回修改后的 MJCF
    std::ofstream mjcf_ofs(mjcf_path);
    mjcf_ofs << mjcf_content;
    mjcf_ofs.close();

    // 释放旧的 URDF 模型
    mj_deleteModel(urdf_model);
    urdf_model = nullptr;

    // 第三步：以修改后的 MJCF 重新加载
    model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      std::cerr << "[IOMujoco] MJCF 模型加载失败: " << error << std::endl;
      return false;
    }

    std::cout << "[IOMujoco] 已通过 MJCF 中间格式成功注入 "
              << joint_names.size() << " 个 actuator" << std::endl;
  } else {
    // 模型自带 actuator，直接使用
    model_ = urdf_model;
  }

  // 设置仿真时间步长
  model_->opt.timestep = timestep_;

  // 使用 implicitfast 积分器
  // MuJoCo 官方要求：position/velocity actuator 必须搭配 implicit 或
  // implicitfast 积分器使用，否则 Euler 积分器会将 PD 反馈力矩当作显式力处理，
  // 导致数值振荡（抖动）。implicitfast 对 actuator 的刚度/阻尼项做隐式积分，
  // 在保持计算效率的同时确保数值稳定性。
  model_->opt.integrator = mjINT_IMPLICITFAST;

  // 创建仿真数据
  data_ = mj_makeData(model_);
  if (!data_) {
    std::cerr << "[IOMujoco] 仿真数据创建失败" << std::endl;
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  // 初始化完成
  is_connected_ = true;
  initialized_ = true;

  if (enable_viewer_ && !initViewer()) {
    std::cerr
        << "[IOMujoco] 警告: MuJoCo 可视化窗口初始化失败，将继续无窗口运行"
        << std::endl;
  }

  std::cout << "[IOMujoco] 初始化完成 - 模型: " << model_path_ << std::endl;
  std::cout << "[IOMujoco]   关节数 (nq): " << model_->nq
            << ", 自由度 (nv): " << model_->nv
            << ", 执行器数 (nu): " << model_->nu << std::endl;
  std::cout << "[IOMujoco]   仿真步长: " << model_->opt.timestep << "s"
            << std::endl;

  return true;
}

bool IOMujoco::sendRecv(const LowLevelCmd *cmd, LowLevelState *state) {
  if (!model_ || !data_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(sim_mutex_);

  // 确定实际可控制的关节数
  int n = std::min(NUM_JOINTS, model_->nv);

  // 检查是否收到有效控制指令（任一关节 kp > 0 表示有主动控制）
  if (!sim_started_) {
    for (int i = 0; i < n; ++i) {
      if (cmd->kp[i] > 0.0f) {
        sim_started_ = true;
        std::cout << "[IOMujoco] 收到有效控制指令，仿真开始推进" << std::endl;
        break;
      }
    }
  }

  // 组件尚未就绪时只返回当前状态，不推进仿真
  if (!sim_started_) {
    for (int i = 0; i < n; ++i) {
      state->mode[i] = cmd->mode[i];
      state->q[i] = static_cast<float>(data_->qpos[i]);
      state->dq[i] = static_cast<float>(data_->qvel[i]);
      state->ddq[i] = 0.0f;
      state->tau_est[i] = 0.0f;
      state->temperature[i] = 25;
    }
    return true;
  }

  // 将控制指令写入 MuJoCo
  if (model_->nu >= n) {
    if (control_mode_ == "position") {
      // ===== Position 模式 =====
      // 使用 position actuator：ctrl 数组存放目标角度
      // PD 控制由 MuJoCo 隐式积分器在内部完成，数值稳定
      for (int i = 0; i < n; ++i) {
        data_->ctrl[i] = static_cast<double>(cmd->q[i]);
      }
    } else {
      // ===== Impedance 模式 (关节阻抗力矩控制) =====
      // 显式控制律：
      //   tau = h(q,dq) + Kp*(q_d-q) + Kd*(dq_d-dq) + tau_d
      // 其中 h(q,dq)=qfrc_bias。
      //
      // 对应标准关节阻抗形式（定义 e_tilde = q-q_d）：
      //   M(q)*e_tilde_ddot + Kd*e_tilde_dot + Kp*e_tilde = tau_ext + tau_d
      // （在参考加速度近似为 0 时成立）。
      for (int i = 0; i < n; ++i) {
        double q_err = static_cast<double>(cmd->q[i]) - data_->qpos[i];
        double dq_err = static_cast<double>(cmd->dq[i]) - data_->qvel[i];
        double tau_pd = cmd->kp[i] * q_err + cmd->kd[i] * dq_err;
        double tau_ff = static_cast<double>(cmd->tau[i]);
        data_->ctrl[i] = data_->qfrc_bias[i] + tau_pd + tau_ff;
      }
    }
  } else {
    // 没有 actuator，直接施加关节力矩（后备方案）
    for (int i = 0; i < n; ++i) {
      double q_err = static_cast<double>(cmd->q[i]) - data_->qpos[i];
      double dq_err = static_cast<double>(cmd->dq[i]) - data_->qvel[i];
      data_->qfrc_applied[i] = cmd->kp[i] * q_err + cmd->kd[i] * dq_err +
                               static_cast<double>(cmd->tau[i]);
    }
  }

  // 步进仿真
  mj_step(model_, data_);

  // 读取仿真状态写入 LowLevelState
  for (int i = 0; i < n; ++i) {
    state->mode[i] = cmd->mode[i];
    state->q[i] = static_cast<float>(data_->qpos[i]);
    state->dq[i] = static_cast<float>(data_->qvel[i]);
    state->ddq[i] = static_cast<float>(data_->qacc[i]);

    // 估计力矩：使用 MuJoCo 计算的执行器力或施加力
    if (model_->nu >= n) {
      state->tau_est[i] = static_cast<float>(data_->actuator_force[i]);
    } else {
      state->tau_est[i] = static_cast<float>(data_->qfrc_applied[i]);
    }

    // 仿真中温度固定为 25°C
    state->temperature[i] = 25;
  }

  return true;
}

bool IOMujoco::isConnected() { return is_connected_; }

bool IOMujoco::initViewer() {
  if (!enable_viewer_) {
    return true;
  }

#ifdef OPENMMARM_HAS_GLFW
  viewer_stop_requested_ = false;
  {
    std::lock_guard<std::mutex> lk(viewer_state_mutex_);
    viewer_init_done_ = false;
    viewer_init_ok_ = false;
  }

  viewer_thread_ = std::thread(&IOMujoco::viewerLoop, this);

  std::unique_lock<std::mutex> lk(viewer_state_mutex_);
  bool signaled = viewer_state_cv_.wait_for(
      lk, std::chrono::seconds(2), [this]() { return viewer_init_done_; });
  if (!signaled) {
    std::cerr << "[IOMujoco] 等待 MuJoCo viewer 线程启动超时" << std::endl;
    viewer_stop_requested_ = true;
    lk.unlock();
    if (viewer_thread_.joinable()) {
      viewer_thread_.join();
    }
    return false;
  }
  return viewer_init_ok_;
#else
  std::cerr << "[IOMujoco] 当前构建未启用 GLFW，无法打开 MuJoCo 可视化窗口"
            << std::endl;
  return false;
#endif
}

void IOMujoco::viewerLoop() {
#ifdef OPENMMARM_HAS_GLFW
  auto signalInit = [this](bool ok) {
    {
      std::lock_guard<std::mutex> lk(viewer_state_mutex_);
      viewer_init_ok_ = ok;
      viewer_init_done_ = true;
    }
    viewer_state_cv_.notify_one();
  };

  if (!glfwInit()) {
    std::cerr << "[IOMujoco] GLFW 初始化失败" << std::endl;
    signalInit(false);
    return;
  }

  window_ =
      glfwCreateWindow(1280, 800, "OpenMMArm MuJoCo Viewer", nullptr, nullptr);
  if (!window_) {
    std::cerr << "[IOMujoco] 无法创建 MuJoCo 可视化窗口" << std::endl;
    glfwTerminate();
    signalInit(false);
    return;
  }

  glfwMakeContextCurrent(window_);
  // 开启垂直同步，渲染线程自行节流，不阻塞控制线程。
  glfwSwapInterval(1);

  // 注册鼠标交互回调（左键旋转、右键平移、滚轮缩放）
  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &IOMujoco::mouseButtonCallback);
  glfwSetCursorPosCallback(window_, &IOMujoco::cursorPosCallback);
  glfwSetScrollCallback(window_, &IOMujoco::scrollCallback);

  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);

  {
    std::lock_guard<std::mutex> lk(sim_mutex_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    camera_.type = mjCAMERA_FREE;
    camera_.lookat[0] = model_->stat.center[0];
    camera_.lookat[1] = model_->stat.center[1];
    camera_.lookat[2] = model_->stat.center[2];
    camera_.distance = model_->stat.extent * 2.0;
  }

  viewer_initialized_ = true;
  std::cout << "[IOMujoco] MuJoCo 可视化窗口已开启" << std::endl;
  signalInit(true);

  while (!viewer_stop_requested_) {
    if (glfwWindowShouldClose(window_)) {
      break;
    }

    {
      std::lock_guard<std::mutex> lk(sim_mutex_);
      mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL,
                      &scene_);
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjr_render(viewport, &scene_, &context_);

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  viewer_initialized_ = false;
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
#else
  (void)this;
#endif
}

// ===== GLFW 鼠标交互回调实现 =====

void IOMujoco::mouseButtonCallback(GLFWwindow *w, int button, int action,
                                   int /*mods*/) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  bool pressed = (action == GLFW_PRESS);
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    self->mouse_button_left_ = pressed;
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    self->mouse_button_middle_ = pressed;
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
    self->mouse_button_right_ = pressed;

  // 记录按下时的光标位置
  glfwGetCursorPos(w, &self->mouse_last_x_, &self->mouse_last_y_);
}

void IOMujoco::cursorPosCallback(GLFWwindow *w, double xpos, double ypos) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  // 没有按键按下时忽略
  if (!self->mouse_button_left_ && !self->mouse_button_middle_ &&
      !self->mouse_button_right_) {
    self->mouse_last_x_ = xpos;
    self->mouse_last_y_ = ypos;
    return;
  }

  double dx = xpos - self->mouse_last_x_;
  double dy = ypos - self->mouse_last_y_;
  self->mouse_last_x_ = xpos;
  self->mouse_last_y_ = ypos;

  int width, height;
  glfwGetWindowSize(w, &width, &height);

  // 确定 MuJoCo 动作类型
  mjtMouse action = mjMOUSE_NONE;
  if (self->mouse_button_left_)
    action = mjMOUSE_ROTATE_V;
  else if (self->mouse_button_middle_)
    action = mjMOUSE_MOVE_V;
  else if (self->mouse_button_right_)
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(self->model_, action, dx / width, dy / height, &self->scene_,
                 &self->camera_);
}

void IOMujoco::scrollCallback(GLFWwindow *w, double /*xoffset*/,
                              double yoffset) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  // 滚轮缩放
  mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset,
                 &self->scene_, &self->camera_);
}

void IOMujoco::closeViewer() {
#ifdef OPENMMARM_HAS_GLFW
  viewer_stop_requested_ = true;
  if (viewer_initialized_) {
    glfwPostEmptyEvent();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
#endif
}
