#include "cmd/UdpSdk.h"

#include <cstring>
#include <fcntl.h>
#include <iostream>

// 诊断计数器
static int diag_frame_count = 0;

UdpSdk::UdpSdk(int port, std::shared_ptr<LowLevelState> lowState)
    : port_(port), lowState_(lowState) {
  std::memset(&client_addr_, 0, sizeof(client_addr_));
}

UdpSdk::~UdpSdk() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool UdpSdk::init() {
  // 创建 UDP socket
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "[UdpSdk] 创建 socket 失败" << std::endl;
    return false;
  }

  // 设置非阻塞模式
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  // 设置接收超时
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = RECV_TIMEOUT_MS * 1000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // 允许端口复用
  int reuse = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // 绑定本地端口
  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&server_addr),
           sizeof(server_addr)) < 0) {
    std::cerr << "[UdpSdk] 绑定端口 " << port_ << " 失败" << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  std::cout << "[UdpSdk] SDK 指令接口已启动，监听端口: " << port_ << std::endl;
  return true;
}

bool UdpSdk::isConnected() { return has_client_.load(); }

void UdpSdk::sendRecv() {
  // 1. 尝试接收 ArmCmd
  OPENMMARM_SDK::ArmCmd recvBuf;
  ssize_t recvLen = recvfrom(socket_fd_, &recvBuf, sizeof(recvBuf), 0,
                             reinterpret_cast<struct sockaddr *>(&client_addr_),
                             &client_addr_len_);

  if (recvLen == static_cast<ssize_t>(sizeof(OPENMMARM_SDK::ArmCmd))) {
    // CRC32 校验
    uint32_t receivedCrc = recvBuf.crc;
    uint32_t calculatedCrc =
        calculateCRC32(reinterpret_cast<const uint8_t *>(&recvBuf),
                       sizeof(recvBuf) - sizeof(uint32_t));

    if (receivedCrc == calculatedCrc) {
      // 校验通过，更新 armCmd
      recvBuf.crc = receivedCrc;
      armCmd = recvBuf;
      has_client_ = true;
    } else {
      static int crc_fail_count = 0;
      ++crc_fail_count;
      if (crc_fail_count % 200 == 1) {
        std::cerr << "[UdpSdk] ArmCmd CRC 校验失败，已忽略数据包 (count="
                  << crc_fail_count << ")" << std::endl;
      }
    }
  } else if (recvLen > 0) {
    static int size_fail_count = 0;
    ++size_fail_count;
    if (size_fail_count % 200 == 1) {
      std::cerr << "[UdpSdk] ArmCmd 长度异常，收到 " << recvLen
                << " 字节，期望 " << sizeof(OPENMMARM_SDK::ArmCmd)
                << " 字节 (count=" << size_fail_count << ")" << std::endl;
    }
  }

  // 2. 从 lowState 填充 armState
  updateArmState();

  // 3. 如果有客户端连接，回传 ArmState
  if (has_client_) {
    // 填充版本号
    armState.version = PROTOCOL_VERSION;

    // 计算 CRC32
    armState.crc = calculateCRC32(reinterpret_cast<const uint8_t *>(&armState),
                                  sizeof(armState) - sizeof(uint32_t));

    sendto(socket_fd_, &armState, sizeof(armState), 0,
           reinterpret_cast<struct sockaddr *>(&client_addr_),
           client_addr_len_);
  }

  // 诊断日志：每 500 帧打印一次
  ++diag_frame_count;
  if (diag_frame_count % 500 == 0) {
    std::cout << "[UdpSdk][" << diag_frame_count << "] "
              << "has_client=" << has_client_.load()
              << " armCmd.mode=" << (int)armCmd.mode
              << " armCmd.q_d[0]=" << armCmd.q_d[0]
              << " armState.q[0]=" << armState.q[0] << std::endl;
  }
}

void UdpSdk::updateArmState() {
  if (!lowState_) {
    return;
  }

  // 从 lowState 复制传感器数据到 armState
  for (int i = 0; i < 6; ++i) {
    armState.q[i] = static_cast<double>(lowState_->q[i]);
    armState.dq[i] = static_cast<double>(lowState_->dq[i]);
    armState.tau[i] = static_cast<double>(lowState_->tau_est[i]);
    armState.temperature[i] = lowState_->temperature[i];
  }
}

uint32_t UdpSdk::calculateCRC32(const uint8_t *data, size_t len) {
  // 标准 CRC32 实现
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}
