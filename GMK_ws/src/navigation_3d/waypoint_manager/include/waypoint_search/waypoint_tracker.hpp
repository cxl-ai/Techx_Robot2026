#pragma once

#include "waypoint_search_types.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

// 航点追踪器：管理航点序列的执行状态
// Linus: "状态机要简单直接，别搞复杂的继承"

namespace waypoint_tracker {

using namespace waypoint_helpers;

// 检查机器人是否到达指定航点
// 使用 XY 平面距离和 Z 轴距离分别判断
// 返回：true 表示到达（XY 距离 < xy_threshold 且 Z 距离 < z_threshold）
static inline bool hasReachedWaypoint(
    const geometry_msgs::msg::Point &robot_pos,
    const geometry_msgs::msg::Point &waypoint_pos,
    double xy_threshold,
    double z_threshold) {
  
  const double dx = waypoint_pos.x - robot_pos.x;
  const double dy = waypoint_pos.y - robot_pos.y;
  const double dz = waypoint_pos.z - robot_pos.z;
  
  const double dist_xy = std::sqrt(dx * dx + dy * dy);
  const double dist_z = std::abs(dz);
  
  return dist_xy < xy_threshold && dist_z < z_threshold;
}

// 距离信息结构
struct DistanceInfo {
  double xy{0.0};  // XY 平面距离
  double z{0.0};   // Z 轴距离（绝对值）
};

// 计算到航点的距离（用于日志输出和判断）
static inline DistanceInfo distanceToWaypoint(
    const geometry_msgs::msg::Point &robot_pos,
    const geometry_msgs::msg::Point &waypoint_pos) {
  
  const double dx = waypoint_pos.x - robot_pos.x;
  const double dy = waypoint_pos.y - robot_pos.y;
  const double dz = waypoint_pos.z - robot_pos.z;
  
  DistanceInfo info;
  info.xy = std::sqrt(dx * dx + dy * dy);
  info.z = std::abs(dz);
  return info;
}

// 初始化追踪器状态（开始新的追踪任务）
static inline void startTracking(
    TrackerState &state,
    const std::vector<WaypointWithMeta> &waypoints,
    int target_floor) {
  
  state.waypoints = waypoints;
  state.current_index = 0;
  state.target_floor = target_floor;
  // robot_floor 不重置，保持当前状态
  state.is_tracking = true;
}

// 停止追踪
static inline void stopTracking(TrackerState &state) {
  state.is_tracking = false;
  state.waypoints.clear();
  state.current_index = 0;
}

// 追踪器状态更新（核心逻辑）
// 返回：是否切换到了下一个航点
struct UpdateResult {
  bool switched_waypoint{false};    // 是否切换航点
  bool floor_changed{false};        // 是否切换楼层
  int prev_floor{1};                // 之前的楼层
  int new_floor{1};                 // 新的楼层
  bool all_completed{false};        // 是否所有航点完成
  DistanceInfo distance;            // 到当前航点的距离信息
};

static inline UpdateResult updateTracking(
    TrackerState &state,
    const geometry_msgs::msg::Point &robot_pos,
    double xy_threshold,
    double z_threshold,
    rclcpp::Logger logger) {
  
  UpdateResult result;
  
  // 检查状态
  if (!state.is_tracking) {
    return result;
  }
  
  if (state.current_index >= state.waypoints.size()) {
    state.is_tracking = false;
    result.all_completed = true;
    return result;
  }
  
  // 当前目标航点
  const auto &current_wp = state.waypoints[state.current_index];
  
  // 计算距离
  result.distance = distanceToWaypoint(robot_pos, current_wp.position);
  
  // 判断是否到达（XY 平面距离 + Z 轴距离分离判断）
  if (hasReachedWaypoint(robot_pos, current_wp.position, xy_threshold, z_threshold)) {
    RCLCPP_INFO(logger, 
                "到达航点 [%zu/%zu] (floor=%d, index=%d), XY距离: %.2fm, Z距离: %.2fm", 
                state.current_index + 1, 
                state.waypoints.size(),
                current_wp.floor, 
                current_wp.index,
                result.distance.xy,
                result.distance.z);
    
    // 切换到下一个航点
    state.current_index++;
    result.switched_waypoint = true;
    
    // 检查是否全部完成
    if (state.current_index >= state.waypoints.size()) {
      state.is_tracking = false;
      result.all_completed = true;
      RCLCPP_INFO(logger, "所有航点追踪完成！当前楼层：%d", state.robot_floor);
      return result;
    }
    
    // 检查楼层变化（离开上一个航点的楼层，进入新航点的楼层）
    const auto &next_wp = state.waypoints[state.current_index];
    if (next_wp.floor != current_wp.floor) {
      result.floor_changed = true;
      result.prev_floor = state.robot_floor;
      result.new_floor = next_wp.floor;
      
      state.robot_floor = next_wp.floor;
      
      RCLCPP_INFO(logger, 
                  "楼层状态切换: %d -> %d (跟踪点: %d-%d -> %d-%d)", 
                  result.prev_floor, 
                  result.new_floor,
                  current_wp.floor, 
                  current_wp.index,
                  next_wp.floor, 
                  next_wp.index);
    }
  }
  
  return result;
}

// 获取当前目标航点（用于发布）
static inline std::optional<WaypointWithMeta> getCurrentWaypoint(
    const TrackerState &state) {
  
  if (!state.is_tracking || 
      state.current_index >= state.waypoints.size()) {
    return std::nullopt;
  }
  
  return state.waypoints[state.current_index];
}

// 获取追踪进度（用于日志和可视化）
struct TrackingProgress {
  size_t current{0};       // 当前索引
  size_t total{0};         // 总数
  int current_floor{1};    // 当前楼层
  int target_floor{1};     // 目标楼层
  bool is_active{false};   // 是否活跃
};

static inline TrackingProgress getProgress(const TrackerState &state) {
  TrackingProgress prog;
  prog.current = state.current_index;
  prog.total = state.waypoints.size();
  prog.current_floor = state.robot_floor;
  prog.target_floor = state.target_floor;
  prog.is_active = state.is_tracking;
  return prog;
}

}  // namespace waypoint_tracker

