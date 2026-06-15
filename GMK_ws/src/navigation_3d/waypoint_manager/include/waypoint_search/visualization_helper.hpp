#pragma once

#include "waypoint_search_types.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/time.hpp>
#include <optional>
#include <string>
#include <unordered_map>

// 可视化辅助函数：RViz Marker 生成
// Linus: "简单的工具函数，输入清晰，输出明确"

namespace visualization_helper {

using namespace waypoint_helpers;

// 创建一个基础 Marker（设置通用字段）
static inline visualization_msgs::msg::Marker makeBaseMarker(
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    const std::string &ns,
    int id) {
  
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  
  return m;
}

// 设置 Marker 的颜色
static inline void setMarkerColor(
    visualization_msgs::msg::Marker &m,
    float r, float g, float b, float a = 1.0f) {
  
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

// 设置 Marker 的尺寸
static inline void setMarkerScale(
    visualization_msgs::msg::Marker &m,
    double x, double y, double z) {
  
  m.scale.x = x;
  m.scale.y = y;
  m.scale.z = z;
}

// 创建航点球体列表（红色）
static inline visualization_msgs::msg::Marker makeWaypointsMarker(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  auto m = makeBaseMarker(frame_id, stamp, "waypoints", 0);
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  
  setMarkerScale(m, 0.75, 0.75, 0.75);
  setMarkerColor(m, 1.0f, 0.2f, 0.1f, 1.0f);  // 红色
  
  for (const auto &[floor, vec] : waypoints_by_floor) {
    for (const auto &p : vec) {
      m.points.push_back(p.position);
    }
  }
  
  return m;
}

// 创建目标点球体列表（蓝色）
static inline visualization_msgs::msg::Marker makeGoalpointsMarker(
    const std::unordered_map<int, std::unordered_map<int, CsvPoint>> &goalpoints_by_floor_index,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  auto m = makeBaseMarker(frame_id, stamp, "goalpoints", 0);
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  
  setMarkerScale(m, 0.65, 0.65, 0.65);
  setMarkerColor(m, 0.1f, 0.3f, 1.0f, 1.0f);  // 蓝色
  
  for (const auto &[floor, mp] : goalpoints_by_floor_index) {
    for (const auto &[idx, p] : mp) {
      (void)floor;
      (void)idx;
      m.points.push_back(p.position);
    }
  }
  
  return m;
}

// 创建文本标签（显示 floor-index）
static inline visualization_msgs::msg::Marker makeTextMarker(
    const CsvPoint &point,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    const std::string &ns,
    float r, float g, float b,
    double z_offset = 0.8) {
  
  auto m = makeBaseMarker(frame_id, stamp, ns, makeMarkerId(point.floor, point.index));
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  
  m.pose.position = point.position;
  m.pose.position.z += z_offset;
  
  setMarkerScale(m, 0, 0, 1.0);  // TEXT 只用 scale.z
  setMarkerColor(m, r, g, b, 1.0f);
  
  m.text = floorIndexText(point.floor, point.index);
  
  return m;
}

// 创建选中目标的高亮球体（绿色）
static inline visualization_msgs::msg::Marker makeSelectedGoalMarker(
    const std::optional<CsvPoint> &selected_goal,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  auto m = makeBaseMarker(frame_id, stamp, "selected_goal", 0);
  
  if (selected_goal) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position = selected_goal->position;
    
    setMarkerScale(m, 0.9, 0.9, 0.9);
    setMarkerColor(m, 0.2f, 1.0f, 0.2f, 0.9f);  // 绿色
  } else {
    m.action = visualization_msgs::msg::Marker::DELETE;
  }
  
  return m;
}

// 创建当前追踪航点的高亮球体（黄色）
static inline visualization_msgs::msg::Marker makeCurrentTrackingMarker(
    const std::optional<WaypointWithMeta> &current_wp,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  auto m = makeBaseMarker(frame_id, stamp, "current_tracking", 0);
  
  if (current_wp) {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position = current_wp->position;
    
    setMarkerScale(m, 1.0, 1.0, 1.0);
    setMarkerColor(m, 1.0f, 1.0f, 0.0f, 0.95f);  // 黄色
  } else {
    m.action = visualization_msgs::msg::Marker::DELETE;
  }
  
  return m;
}

// 创建完整的 MarkerArray（所有可视化元素）
static inline visualization_msgs::msg::MarkerArray makeFullMarkerArray(
    const std::unordered_map<int, std::vector<CsvPoint>> &waypoints_by_floor,
    const std::unordered_map<int, std::unordered_map<int, CsvPoint>> &goalpoints_by_floor_index,
    const std::optional<CsvPoint> &selected_goal,
    const std::optional<WaypointWithMeta> &current_tracking_wp,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  visualization_msgs::msg::MarkerArray arr;
  
  // 1) 航点球体
  arr.markers.push_back(makeWaypointsMarker(waypoints_by_floor, frame_id, stamp));
  
  // 2) 目标点球体
  arr.markers.push_back(makeGoalpointsMarker(goalpoints_by_floor_index, frame_id, stamp));
  
  // 3) 航点文本标签（黄色）
  for (const auto &[floor, vec] : waypoints_by_floor) {
    for (const auto &p : vec) {
      arr.markers.push_back(makeTextMarker(p, frame_id, stamp, "waypoint_ids", 
                                           1.0f, 1.0f, 0.0f, 0.8));
    }
  }
  
  // 4) 目标点文本标签（浅蓝色）
  for (const auto &[floor, mp] : goalpoints_by_floor_index) {
    for (const auto &[idx, p] : mp) {
      (void)floor;
      (void)idx;
      arr.markers.push_back(makeTextMarker(p, frame_id, stamp, "goalpoint_ids", 
                                           0.8f, 0.9f, 1.0f, 1.2));
    }
  }
  
  // 5) 选中目标高亮（绿色）
  arr.markers.push_back(makeSelectedGoalMarker(selected_goal, frame_id, stamp));
  
  // 6) 当前追踪航点高亮（黄色）
  arr.markers.push_back(makeCurrentTrackingMarker(current_tracking_wp, frame_id, stamp));
  
  return arr;
}

// 创建路径消息（nav_msgs/Path）
static inline nav_msgs::msg::Path makePathMessage(
    const std::vector<geometry_msgs::msg::Point> &path_points,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = stamp;
  path_msg.header.frame_id = frame_id;
  path_msg.poses.reserve(path_points.size());
  
  for (const auto &p : path_points) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position = p;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }
  
  return path_msg;
}

// 创建航点消息（geometry_msgs/PointStamped）
static inline geometry_msgs::msg::PointStamped makeWaypointMessage(
    const WaypointWithMeta &wp,
    const std::string &frame_id,
    const rclcpp::Time &stamp) {
  
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.point = wp.position;
  
  return msg;
}

}  // namespace visualization_helper

