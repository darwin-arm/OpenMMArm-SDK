#include "io/IOMujoco.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>

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

IOMujoco::IOMujoco(const std::string &model_path, double timestep)
    : model_path_(model_path), timestep_(timestep) {}

IOMujoco::~IOMujoco() {
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

  // 使用预处理后的文件加载 MuJoCo 模型
  char error[1000] = "";
  model_ =
      mj_loadXML(resolved_model_path_.c_str(), nullptr, error, sizeof(error));

  if (!model_) {
    std::cerr << "[IOMujoco] 模型加载失败: " << error << std::endl;
    return false;
  }

  // 设置仿真时间步长
  model_->opt.timestep = timestep_;

  // 检查 actuator 定义
  if (model_->nu == 0) {
    std::cerr << "[IOMujoco] 警告: 模型中没有定义 actuator，"
              << "将使用 qfrc_applied 直接施加力矩" << std::endl;
  }

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

  // 确定实际可控制的关节数
  int n = std::min(NUM_JOINTS, model_->nv);

  // 将控制指令写入 MuJoCo
  if (model_->nu >= n) {
    // 模型定义了足够的 actuator，使用 ctrl 数组
    for (int i = 0; i < n; ++i) {
      // PD 控制: tau = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
      double q_err = static_cast<double>(cmd->q[i]) - data_->qpos[i];
      double dq_err = static_cast<double>(cmd->dq[i]) - data_->qvel[i];
      data_->ctrl[i] = cmd->kp[i] * q_err + cmd->kd[i] * dq_err +
                       static_cast<double>(cmd->tau[i]);
    }
  } else {
    // 没有 actuator，直接施加关节力矩
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
