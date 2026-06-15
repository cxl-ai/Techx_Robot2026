#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <optional>
#include <algorithm>
#include <cctype>

// Linus 风格：简单的数据结构 + 静态辅助函数
// 不搞复杂的类继承，数据结构就是数据，逻辑就是函数

// CSV 里读出来的点（最基础的结构）
struct CsvPoint {
  int floor{};
  int index{};
  geometry_msgs::msg::Point position;
  double yaw{};
};

// 带元数据的航点（用于追踪）
struct WaypointWithMeta {
  geometry_msgs::msg::Point position;
  double yaw{};
  int floor{};
  int index{};
  bool is_connector{};  // 是否是连接点（floor=G, index=1）
};

// 航点追踪器状态
struct TrackerState {
  std::vector<WaypointWithMeta> waypoints;
  size_t current_index{0};
  int robot_floor{1};
  int target_floor{1};
  bool is_tracking{false};
  size_t floor_switch_index{0};  // 楼层切换点索引（1-m点）
};

// 静态辅助函数：简单、直接、可预测

namespace waypoint_helpers {

// 两点之间的距离平方（避免sqrt，用于比较）
static inline double dist2(const geometry_msgs::msg::Point &a,
                            const geometry_msgs::msg::Point &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

// 生成 Marker ID（floor + index 编码到一个 int）
static inline int makeMarkerId(int floor, int index) {
  return ((floor & 0xFFFF) << 16) | (index & 0xFFFF);
}

// 格式化 floor-index 文本
static inline std::string floorIndexText(int floor, int index) {
  return std::to_string(floor) + "-" + std::to_string(index);
}

// 去除字符串两端空白
static inline std::string trimCopy(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

// 解析 "floor-index" 字符串
static inline std::optional<std::pair<int, int>> parseFloorIndex(
    const std::string &floor_index_raw) {
  const std::string s = trimCopy(floor_index_raw);
  const auto dash = s.find('-');
  if (dash == std::string::npos) {
    return std::nullopt;
  }
  const std::string a = trimCopy(s.substr(0, dash));
  const std::string b = trimCopy(s.substr(dash + 1));
  if (a.empty() || b.empty()) {
    return std::nullopt;
  }
  try {
    return std::make_pair(std::stoi(a), std::stoi(b));
  } catch (...) {
    return std::nullopt;
  }
}

// 从带元数据的路径中提取位置（用于可视化）
static inline std::vector<geometry_msgs::msg::Point> extractPositions(
    const std::vector<WaypointWithMeta> &path_with_meta) {
  std::vector<geometry_msgs::msg::Point> positions;
  positions.reserve(path_with_meta.size());
  for (const auto &wp : path_with_meta) {
    positions.push_back(wp.position);
  }
  return positions;
}

}  // namespace waypoint_helpers
