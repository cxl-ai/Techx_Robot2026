#pragma once

#include "waypoint_search_types.hpp"
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

// 路径规划器：纯函数，无状态
// Linus: "让函数做好一件事，参数明确，结果可预测"

namespace path_planner {

using namespace waypoint_helpers;

// 在指定楼层的航点序列里找"离 target 最近"的点
// 返回：该点在 vector 里的索引（非航点 index）
static inline std::optional<size_t> nearestWaypointPosInFloor(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    int floor,
    const geometry_msgs::msg::Point &target) {
  
  auto it = waypoints_by_floor.find(floor);
  if (it == waypoints_by_floor.end() || it->second.empty()) {
    return std::nullopt;
  }
  
  const auto &vec = it->second;
  size_t best_i = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  
  for (size_t i = 0; i < vec.size(); ++i) {
    const double d2 = dist2(vec[i].position, target);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }
  
  return best_i;
}

// 在指定楼层的航点序列里找指定 index 的点
// 返回：该点在 vector 里的位置
static inline std::optional<size_t> findWaypointPosByIndex(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    int floor,
    int index) {
  
  auto it = waypoints_by_floor.find(floor);
  if (it == waypoints_by_floor.end()) {
    return std::nullopt;
  }
  
  const auto &vec = it->second;
  for (size_t i = 0; i < vec.size(); ++i) {
    if (vec[i].index == index) {
      return i;
    }
  }
  
  return std::nullopt;
}

// 添加一段楼层内的航点序列（从 start_pos 到 end_pos）
// 会自动处理正向/反向遍历
static inline void appendFloorSegment(
    std::vector<WaypointWithMeta> &path,
    const std::vector<CsvPoint> &floor_waypoints,
    size_t start_pos,
    size_t end_pos,
    bool mark_end_as_connector = false) {
  
  if (start_pos <= end_pos) {
    // 正向：index 递增
    for (size_t i = start_pos; i <= end_pos; ++i) {
      WaypointWithMeta wp;
      wp.position = floor_waypoints[i].position;
      wp.yaw = floor_waypoints[i].yaw;
      wp.floor = floor_waypoints[i].floor;
      wp.index = floor_waypoints[i].index;
      wp.is_connector = (mark_end_as_connector && i == end_pos);
      path.push_back(wp);
    }
  } else {
    // 反向：index 递减（从 start_pos 到 end_pos，包含两端）
    for (size_t i = start_pos + 1; i-- > end_pos;) {
      WaypointWithMeta wp;
      wp.position = floor_waypoints[i].position;
      wp.yaw = floor_waypoints[i].yaw;
      wp.floor = floor_waypoints[i].floor;
      wp.index = floor_waypoints[i].index;
      wp.is_connector = (mark_end_as_connector && i == end_pos);
      path.push_back(wp);
    }
    // 反向最后一个点
    WaypointWithMeta wp_last;
    wp_last.position = floor_waypoints[end_pos].position;
    wp_last.yaw = floor_waypoints[end_pos].yaw;
    wp_last.floor = floor_waypoints[end_pos].floor;
    wp_last.index = floor_waypoints[end_pos].index;
    wp_last.is_connector = mark_end_as_connector;
    path.push_back(wp_last);
  }
}

// 场景1：同楼层内导航
// 路径：start -> A-nearest -> ... -> A-goal_nearest -> goal
static inline std::vector<WaypointWithMeta> planWithinSameFloor(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    int floor,
    const geometry_msgs::msg::Point &start,
    const CsvPoint &goal,
    std::string &why) {
  
  std::vector<WaypointWithMeta> path;
  why.clear();
  
  auto it = waypoints_by_floor.find(floor);
  if (it == waypoints_by_floor.end() || it->second.empty()) {
    why = "floor=" + std::to_string(floor) + " 的 waypoints 为空";
    return {};
  }
  
  const auto &floor_waypoints = it->second;
  
  // 找离起点最近的航点
  const auto start_pos = nearestWaypointPosInFloor(waypoints_by_floor, floor, start);
  if (!start_pos) {
    why = "无法在 floor=" + std::to_string(floor) + " 找到起点附近的航点";
    return {};
  }
  
  // 找离目标最近的航点
  const auto goal_pos = nearestWaypointPosInFloor(waypoints_by_floor, floor, goal.position);
  if (!goal_pos) {
    why = "无法在 floor=" + std::to_string(floor) + " 找到目标附近的航点";
    return {};
  }
  
  // 起点：机器人当前位置
  WaypointWithMeta wp_start;
  wp_start.position = start;
  wp_start.yaw = 0.0;
  wp_start.floor = floor;
  wp_start.index = 0;
  wp_start.is_connector = false;
  path.push_back(wp_start);
  
  // 沿着 index 连续的路径
  appendFloorSegment(path, floor_waypoints, *start_pos, *goal_pos, false);
  
  // 终点：目标点本身
  WaypointWithMeta wp_goal;
  wp_goal.position = goal.position;
  wp_goal.yaw = goal.yaw;
  wp_goal.floor = goal.floor;
  wp_goal.index = goal.index;
  wp_goal.is_connector = false;
  path.push_back(wp_goal);
  
  why = "ok: 同楼层内导航 floor=" + std::to_string(floor);
  return path;
}

// 场景2：跨楼层导航（必须通过 floor=1 中转）
// 路径：A-current -> A-1 -> 1-m -> 1-n -> B-1 -> B-goal
static inline std::vector<WaypointWithMeta> planCrossFloor(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    int current_floor,
    const geometry_msgs::msg::Point &start,
    const CsvPoint &goal,
    std::string &why) {
  
  std::vector<WaypointWithMeta> path;
  why.clear();
  
  const int target_floor = goal.floor;
  
  // === 阶段1: 从当前楼层到连接点 ===
  if (current_floor != 1) {
    // 找到当前楼层的连接点 A-1
    const auto connector_pos = findWaypointPosByIndex(waypoints_by_floor, current_floor, 1);
    if (!connector_pos) {
      why = "当前楼层 floor=" + std::to_string(current_floor) + " 缺少连接点 (index=1)";
      return {};
    }
    
    // 起点
    WaypointWithMeta wp_start;
    wp_start.position = start;
    wp_start.yaw = 0.0;
    wp_start.floor = current_floor;
    wp_start.index = 0;
    wp_start.is_connector = false;
    path.push_back(wp_start);
    
    // 找离起点最近的航点
    const auto start_pos = nearestWaypointPosInFloor(waypoints_by_floor, current_floor, start);
    if (!start_pos) {
      why = "floor=" + std::to_string(current_floor) + " 的 waypoints 为空";
      return {};
    }
    
    // 当前楼层：start -> A-nearest -> ... -> A-1
    const auto &wA = waypoints_by_floor.at(current_floor);
    appendFloorSegment(path, wA, *start_pos, *connector_pos, true);
  } else {
    // 从 floor=1 出发
    WaypointWithMeta wp_start;
    wp_start.position = start;
    wp_start.yaw = 0.0;
    wp_start.floor = 1;
    wp_start.index = 0;
    wp_start.is_connector = false;
    path.push_back(wp_start);
  }
  
  // === 阶段2: floor=1 中转 ===
  if (current_floor != 1 && target_floor != 1) {
    // 两端都不是 floor=1，需要中转
    const auto connector_A_pos = findWaypointPosByIndex(waypoints_by_floor, current_floor, 1);
    const auto connector_B_pos = findWaypointPosByIndex(waypoints_by_floor, target_floor, 1);
    
    if (!connector_A_pos || !connector_B_pos) {
      why = "缺少连接点";
      return {};
    }
    
    const CsvPoint &conn_A = waypoints_by_floor.at(current_floor)[*connector_A_pos];
    const CsvPoint &conn_B = waypoints_by_floor.at(target_floor)[*connector_B_pos];
    
    // 1-m（离 A-1 最近）
    const auto m_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, conn_A.position);
    // 1-n（离 B-1 最近）
    const auto n_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, conn_B.position);
    
    if (!m_pos || !n_pos) {
      why = "floor=1 的 waypoints 不足";
      return {};
    }
    
    // floor=1: 1-m -> 1-n
    const auto &w1 = waypoints_by_floor.at(1);
    appendFloorSegment(path, w1, *m_pos, *n_pos, false);
    
  } else if (current_floor == 1 && target_floor != 1) {
    // 从 floor=1 到 floor=n
    const auto connector_B_pos = findWaypointPosByIndex(waypoints_by_floor, target_floor, 1);
    if (!connector_B_pos) {
      why = "目标楼层 floor=" + std::to_string(target_floor) + " 缺少连接点";
      return {};
    }
    const CsvPoint &conn_B = waypoints_by_floor.at(target_floor)[*connector_B_pos];
    
    // 1-current -> 1-n（离 B-1 最近）
    const auto start_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, start);
    const auto n_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, conn_B.position);
    
    if (!start_pos || !n_pos) {
      why = "floor=1 的 waypoints 不足";
      return {};
    }
    
    const auto &w1 = waypoints_by_floor.at(1);
    appendFloorSegment(path, w1, *start_pos, *n_pos, false);
    
  } else if (current_floor != 1 && target_floor == 1) {
    // 从 floor=n 到 floor=1（已经在阶段1处理了到达 A-1）
    const auto connector_A_pos = findWaypointPosByIndex(waypoints_by_floor, current_floor, 1);
    if (!connector_A_pos) {
      why = "当前楼层缺少连接点";
      return {};
    }
    const CsvPoint &conn_A = waypoints_by_floor.at(current_floor)[*connector_A_pos];
    
    // 1-m（离 A-1 最近）-> 1-goal
    const auto m_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, conn_A.position);
    const auto goal_pos = nearestWaypointPosInFloor(waypoints_by_floor, 1, goal.position);
    
    if (!m_pos || !goal_pos) {
      why = "floor=1 的 waypoints 不足";
      return {};
    }
    
    const auto &w1 = waypoints_by_floor.at(1);
    appendFloorSegment(path, w1, *m_pos, *goal_pos, false);
  }
  
  // === 阶段3: 目标楼层连接点 -> 目标 ===
  if (target_floor != 1) {
    const auto connector_B_pos = findWaypointPosByIndex(waypoints_by_floor, target_floor, 1);
    if (!connector_B_pos) {
      why = "目标楼层 floor=" + std::to_string(target_floor) + " 缺少连接点";
      return {};
    }
    
    // B-1 -> B-nearest_to_goal
    const auto goal_pos = nearestWaypointPosInFloor(waypoints_by_floor, target_floor, goal.position);
    if (!goal_pos) {
      why = "目标楼层 waypoints 不足";
      return {};
    }
    
    const auto &wB = waypoints_by_floor.at(target_floor);
    appendFloorSegment(path, wB, *connector_B_pos, *goal_pos, false);
  }
  
  // 终点：目标点本身
  WaypointWithMeta wp_goal;
  wp_goal.position = goal.position;
  wp_goal.yaw = goal.yaw;
  wp_goal.floor = goal.floor;
  wp_goal.index = goal.index;
  wp_goal.is_connector = false;
  path.push_back(wp_goal);
  
  why = "ok: 跨楼层导航 " + std::to_string(current_floor) + " -> " + std::to_string(target_floor);
  return path;
}

// 主入口：根据当前楼层和目标楼层自动选择规划策略
static inline std::vector<WaypointWithMeta> planPath(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    int current_floor,
    const geometry_msgs::msg::Point &start,
    const CsvPoint &goal,
    std::string &why) {
  
  // 同楼层：直接规划
  if (current_floor == goal.floor) {
    return planWithinSameFloor(waypoints_by_floor, current_floor, start, goal, why);
  }
  
  // 跨楼层：必须经过 floor=1 中转
  return planCrossFloor(waypoints_by_floor, current_floor, start, goal, why);
}

}  // namespace path_planner

