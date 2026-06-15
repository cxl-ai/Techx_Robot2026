// 航点搜索节点：基于已记录的航点进行目标选择和路径规划
// 
// 设计哲学：Linus 风格
// - 简单的数据结构 + 静态函数（拆分到各个 .hpp）
// - 节点本身只做"胶水"工作：订阅、发布、服务、定时器
// - 核心逻辑全部在独立模块里，易于测试和复用

#include "waypoint_search/waypoint_search_types.hpp"
#include "waypoint_search/csv_loader.hpp"
#include "waypoint_search/path_planner.hpp"
#include "waypoint_search/waypoint_tracker.hpp"
#include "waypoint_search/visualization_helper.hpp"
#include "waypoint_search/polygon_reprojector.hpp"
#include "waypoint_search/goal_sequence_runner.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>

#include <waypoint_manager/msg/goal_point.hpp>
#include <waypoint_manager/srv/select_goal.hpp>
#include <waypoint_manager/srv/set_floor_state.hpp>

#include <filesystem>
#include <mutex>
#include <limits>
#include <cmath>
#include <cstdlib>

namespace fs = std::filesystem;

using namespace waypoint_helpers;
using namespace polygon_reprojector;

// ----------------------------------------------------------------------------
// 路径展开：支持 "~/" 与 "${HOME}"
// 说明：ROS2 参数不会自动展开 ~，C++ std::filesystem 也不会；因此在这里做一次轻量处理。
// ----------------------------------------------------------------------------
static inline std::string expandUserPath(std::string s) {
  const char *home = std::getenv("HOME");
  const std::string home_str = home ? std::string(home) : std::string();

  // "~" or "~/..."
  if (!home_str.empty() && !s.empty() && s[0] == '~') {
    if (s.size() == 1) {
      s = home_str;
    } else if (s[1] == '/') {
      s = home_str + s.substr(1);
    }
  }

  // "${HOME}"
  if (!home_str.empty()) {
    const std::string key = "${HOME}";
    size_t pos = 0;
    while ((pos = s.find(key, pos)) != std::string::npos) {
      s.replace(pos, key.size(), home_str);
      pos += home_str.size();
    }
  }

  return s;
}

// ============================================================================
// 主节点类：只保留 ROS2 接口逻辑，业务逻辑全部委托给独立模块
// ============================================================================

class WaypointSearchNode : public rclcpp::Node {
public:
  WaypointSearchNode() : rclcpp::Node("waypoint_search") {
    // 参数声明
    declareParameters();
    
    // 加载 CSV 数据
    loadCsvData();
    
    // 创建发布者
    goal_pub_ = create_publisher<waypoint_manager::msg::GoalPoint>(
        goal_topic_, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10));
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        waypoint_topic_, rclcpp::QoS(10));
    poly_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        poly_viz_topic_, rclcpp::QoS(10));

    // 创建订阅者
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        std::bind(&WaypointSearchNode::onOdom, this, std::placeholders::_1));
    terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        terrain_map_topic_, rclcpp::QoS(2),
        std::bind(&WaypointSearchNode::onTerrainMap, this, std::placeholders::_1));
    
    // 创建服务
    select_goal_srv_ = create_service<waypoint_manager::srv::SelectGoal>(
        service_name_, 
        std::bind(&WaypointSearchNode::onSelectGoal, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    set_floor_state_srv_ = create_service<waypoint_manager::srv::SetFloorState>(
        "set_floor_state", 
        std::bind(&WaypointSearchNode::onSetFloorState, this, 
                  std::placeholders::_1, std::placeholders::_2));

    advance_waypoint_srv_ = create_service<std_srvs::srv::Trigger>(
        advance_service_name_,
        std::bind(&WaypointSearchNode::onAdvanceWaypoint, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 目标到达事件发布
    goal_reached_pub_ = create_publisher<std_msgs::msg::String>(
        goal_reached_topic_, rclcpp::QoS(10));
    
    // 定时器：可视化刷新
    viz_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WaypointSearchNode::onVizTimer, this));
    
    // 定时器：航点追踪检查
    const int tracker_period_ms = static_cast<int>(1000.0 / tracker_check_rate_);
    tracker_timer_ = create_wall_timer(
        std::chrono::milliseconds(tracker_period_ms),
        std::bind(&WaypointSearchNode::onTrackerTimer, this));
    
    // 启动后立即发布一次可视化
    publishVisualization();
    
    // 打印配置信息
    printConfiguration();
  }

private:
  enum class ActiveGoalSource {
    None = 0,
    Manual,
    AutoSequence,
  };

  // --------------------------------------------------------------------------
  // 参数声明
  // --------------------------------------------------------------------------
  void declareParameters() {
    use_to_nav_dir_ = declare_parameter<std::string>(
        "use_to_nav_dir", "~/nav_ws/project/data/use_to_nav");
    waypoints_csv_param_ = declare_parameter<std::string>("waypoints_csv", "");
    goalpoints_csv_param_ = declare_parameter<std::string>("goalpoints_csv", "");

    // 展开 "~/" 与 "${HOME}"（否则自动选 CSV 会失败）
    use_to_nav_dir_ = expandUserPath(use_to_nav_dir_);
    waypoints_csv_param_ = expandUserPath(waypoints_csv_param_);
    goalpoints_csv_param_ = expandUserPath(goalpoints_csv_param_);
    
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    terrain_map_topic_ = declare_parameter<std::string>("terrain_map_topic", "/terrain_map");
    
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_point_ext");
    marker_topic_ = declare_parameter<std::string>("viz_marker_topic", "/goal_search_markers");
    path_topic_ = declare_parameter<std::string>("path_topic", "/goal_search_path");
    poly_viz_topic_ = declare_parameter<std::string>("poly_viz_topic", "/waypoint_polygons");
    service_name_ = declare_parameter<std::string>("select_goal_service", "select_goal");
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/way_point");
    goal_reached_topic_ = declare_parameter<std::string>("goal_reached_topic", "goal_reached");
    
    // 自动 goal 序列（简化版：floor_indices 非空则启用；为空等同 Completed）
    goal_sequence_floor_indices_ =
        declare_parameter<std::vector<std::string>>("goal_sequence_floor_indices", {});
    goal_sequence_next_delay_sec_ =
        declare_parameter<double>("goal_sequence_next_delay_sec", 3.0);
    
    waypoint_reach_xy_threshold_ = declare_parameter<double>("waypoint_reach_xy_threshold", 1.5);
    waypoint_reach_z_threshold_ = declare_parameter<double>("waypoint_reach_z_threshold", 2.0);
    advance_service_name_ = declare_parameter<std::string>("advance_waypoint_service", "advance_waypoint");
    tracker_check_rate_ = declare_parameter<double>("tracker_check_rate", 10.0);

    poly_grid_range_ = declare_parameter<double>("poly_grid_range", 8.0);
    poly_grid_resolution_ = declare_parameter<double>("poly_grid_resolution", 0.2);
    poly_inflate_cells_ = declare_parameter<int>("poly_inflate_cells", 1);
    poly_min_area_ = declare_parameter<double>("poly_min_area", 0.5);
    poly_free_radius_ = declare_parameter<double>("poly_free_radius", 0.6);
    poly_intensity_thred_ = declare_parameter<double>("poly_intensity_thred", 2.0);
    poly_height_lower_ = declare_parameter<double>("poly_height_lower", 0.5);  // Z轴下方过滤范围（米）
    poly_height_upper_ = declare_parameter<double>("poly_height_upper", 2.0);  // Z轴上方过滤范围（米）
    poly_wall_angle_thred_ = declare_parameter<double>("poly_wall_angle_thred", 8.0);  // 墙体角度阈值（度，推荐5-10）

    // 初始化 runner（纯逻辑）
    goal_sequence_runner_ = goal_sequence_runner::GoalSequenceRunner(goal_sequence_floor_indices_);
  }
  
  // --------------------------------------------------------------------------
  // CSV 数据加载
  // --------------------------------------------------------------------------
  void loadCsvData() {
    // 选择 waypoints CSV（显式指定 or 自动选最新）
    if (!waypoints_csv_param_.empty()) {
      waypoints_csv_ = waypoints_csv_param_;
    } else {
      const auto best = csv_loader::findLatestCsvByPrefix(
          fs::path(use_to_nav_dir_), "waypoints-");
      waypoints_csv_ = best ? best->string() : "";
    }
    
    // 选择 goalpoints CSV
    if (!goalpoints_csv_param_.empty()) {
      goalpoints_csv_ = goalpoints_csv_param_;
    } else {
      const auto best = csv_loader::findLatestCsvByPrefix(
          fs::path(use_to_nav_dir_), "goalpoints-");
      goalpoints_csv_ = best ? best->string() : "";
    }
    
    // 读取并构建索引
    waypoints_by_floor_.clear();
    goalpoints_by_floor_index_.clear();
    
    if (!waypoints_csv_.empty()) {
      const auto pts = csv_loader::readCsvPoints(waypoints_csv_, get_logger());
      waypoints_by_floor_ = csv_loader::buildFloorIndex(pts);
    }
    
    if (!goalpoints_csv_.empty()) {
      const auto pts = csv_loader::readCsvPoints(goalpoints_csv_, get_logger());
      goalpoints_by_floor_index_ = csv_loader::buildFloorIndexMap(pts);
    }
  }
  
  // --------------------------------------------------------------------------
  // 打印配置信息
  // --------------------------------------------------------------------------
  void printConfiguration() {
    RCLCPP_INFO(get_logger(), "=== Waypoint Search Node 配置 ===");
    RCLCPP_INFO(get_logger(), "use_to_nav_dir: %s", use_to_nav_dir_.c_str());
    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "goalpoints_csv: %s", goalpoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "goal_topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "waypoint_topic: %s", waypoint_topic_.c_str());
    RCLCPP_INFO(get_logger(), "goal_reached_topic: %s", goal_reached_topic_.c_str());
    RCLCPP_INFO(get_logger(), "goal_sequence_floor_indices: %zu",
                goal_sequence_floor_indices_.size());
    RCLCPP_INFO(get_logger(), "goal_sequence_next_delay_sec: %.2f",
                goal_sequence_next_delay_sec_);
    RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "terrain_map_topic: %s", terrain_map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "poly_viz_topic: %s", poly_viz_topic_.c_str());
    RCLCPP_INFO(get_logger(), "waypoint_reach_xy_threshold: %.2f m (已停用)", waypoint_reach_xy_threshold_);
    RCLCPP_INFO(get_logger(), "waypoint_reach_z_threshold: %.2f m (已停用)", waypoint_reach_z_threshold_);
    RCLCPP_INFO(get_logger(), "tracker_check_rate: %.1f Hz", tracker_check_rate_);
    
    // 多边形检测参数
    RCLCPP_INFO(get_logger(), "--- 多边形检测参数（保持原始形状 + 顶点凹凸性分析 + 墙体过滤）---");
    RCLCPP_INFO(get_logger(), "poly_grid_range: %.1f m", poly_grid_range_);
    RCLCPP_INFO(get_logger(), "poly_grid_resolution: %.2f m", poly_grid_resolution_);
    RCLCPP_INFO(get_logger(), "poly_inflate_cells: %d", poly_inflate_cells_);
    RCLCPP_INFO(get_logger(), "poly_min_area: %.2f m²", poly_min_area_);
    RCLCPP_INFO(get_logger(), "poly_free_radius: %.2f m", poly_free_radius_);
    RCLCPP_INFO(get_logger(), "poly_intensity_thred: %.1f", poly_intensity_thred_);
    RCLCPP_INFO(get_logger(), "poly_height_lower: %.2f m (下方过滤)", poly_height_lower_);
    RCLCPP_INFO(get_logger(), "poly_height_upper: %.2f m (上方过滤)", poly_height_upper_);
    RCLCPP_INFO(get_logger(), "poly_wall_angle_thred: %.1f°", poly_wall_angle_thred_);
    
    const auto full_srv = rclcpp::expand_topic_or_service_name(
        service_name_, get_name(), get_namespace());
    RCLCPP_INFO(get_logger(), 
                "服务调用示例: ros2 service call %s waypoint_manager/srv/SelectGoal "
                "\"{floor_index: '2-3'}\"", 
                full_srv.c_str());

    const auto full_advance_srv = rclcpp::expand_topic_or_service_name(
        advance_service_name_, get_name(), get_namespace());
    RCLCPP_INFO(get_logger(),
                "航点切换服务: ros2 service call %s std_srvs/srv/Trigger \"{}\"",
                full_advance_srv.c_str());
  }
  
  // --------------------------------------------------------------------------
  // 里程计回调：更新车辆位置
  // --------------------------------------------------------------------------
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    bool first_odom = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      first_odom = !last_odom_position_.has_value();
      last_odom_position_ = msg->pose.pose.position;
    }

    if (first_odom) {
      have_first_odom_ = true;
      goal_sequence_runner_.onFirstOdom();
      // 若启用了自动序列，首次 odom 到来后立刻触发第一个 goal
      maybeDispatchAutoGoal("first_odom");
    }
  }

  // --------------------------------------------------------------------------
  // 地形点云回调：更新局部凸多边形
  // --------------------------------------------------------------------------
  void onTerrainMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const auto center_opt = getVehicleCenterPosition();
    if (!center_opt) {
      return;
    }
    buildConvexPolygonsFromTerrain(*center_opt, *msg);
    publishPolygonVisualization();  // 发布多边形可视化
  }

  // --------------------------------------------------------------------------
  // 获取车辆中心位置（来自里程计）
  // --------------------------------------------------------------------------
  std::optional<geometry_msgs::msg::Point> getVehicleCenterPosition() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!last_odom_position_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "尚未收到里程计消息，无法计算路径");
      return std::nullopt;
    }
    return last_odom_position_;
  }

  // --------------------------------------------------------------------------
  // 构建局部多边形（委托给 polygon_reprojector）
  // --------------------------------------------------------------------------
  void buildConvexPolygonsFromTerrain(const geometry_msgs::msg::Point &center,
                                      const sensor_msgs::msg::PointCloud2 &cloud) {
    // 准备参数
    PolygonBuildParams params;
    params.grid_range = poly_grid_range_;
    params.grid_resolution = poly_grid_resolution_;
    params.inflate_cells = poly_inflate_cells_;
    params.min_area = poly_min_area_;
    params.intensity_thred = poly_intensity_thred_;
    params.height_lower = poly_height_lower_;
    params.height_upper = poly_height_upper_;
    params.wall_angle_thred = poly_wall_angle_thred_;
    
    // 调用 polygon_reprojector 构建多边形
    auto polys_result = polygon_reprojector::buildConvexPolygonsFromTerrain(center, cloud, params, warned_no_intensity_);
    
    // 如果第一次遇到无 intensity 字段，输出警告
    if (warned_no_intensity_ && !polygon_reprojector::hasField(cloud, "intensity")) {
      RCLCPP_WARN(get_logger(), "terrain_map 无 intensity 字段，跳过强度阈值过滤");
    }
    
    // 更新成员变量
    {
      std::lock_guard<std::mutex> lock(poly_mutex_);
      convex_polygons_ = std::move(polys_result);
    }
  }

  // --------------------------------------------------------------------------
  // 目标点在多边形内时，沿"航点→机器人"连线在边界外推（委托给 polygon_reprojector）
  // --------------------------------------------------------------------------
  bool reprojectPointOutsidePolygons(geometry_msgs::msg::Point &point) {
    // 获取机器人位置
    std::optional<geometry_msgs::msg::Point> robot_pos_opt;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      robot_pos_opt = last_odom_position_;
    }
    
    // 获取多边形列表
    std::vector<ConvexPolygon> polys;
    {
      std::lock_guard<std::mutex> lock(poly_mutex_);
      polys = convex_polygons_;
    }
    
    // 准备外推参数
    ReprojectionParams params;
    params.free_radius = poly_free_radius_;
    
    // 调用 polygon_reprojector 进行外推
    return polygon_reprojector::reprojectPointOutsidePolygons(point, robot_pos_opt, polys, params);
  }
  
  // --------------------------------------------------------------------------
  // 发布多边形可视化（轻量级：仅边界线）
  // --------------------------------------------------------------------------
  void publishPolygonVisualization() {
    std::vector<ConvexPolygon> polys;
    {
      std::lock_guard<std::mutex> lock(poly_mutex_);
      polys = convex_polygons_;
    }
    
    if (polys.empty()) {
      return;
    }
    
    visualization_msgs::msg::MarkerArray poly_marker_array;
    visualization_msgs::msg::Marker polygon_edges;
    
    // 多边形边界（LINE_LIST）- 仅显示边界，减少数据量
    polygon_edges.header.frame_id = frame_id_;
    polygon_edges.header.stamp = now();
    polygon_edges.ns = "polygon_edges";
    polygon_edges.id = 0;
    polygon_edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    polygon_edges.action = visualization_msgs::msg::Marker::ADD;
    polygon_edges.scale.x = 0.08;  // 线宽
    polygon_edges.color.r = 1.0f;
    polygon_edges.color.g = 0.5f;
    polygon_edges.color.b = 0.0f;  // 橙色
    polygon_edges.color.a = 0.8f;
    polygon_edges.pose.orientation.w = 1.0;
    
    geometry_msgs::msg::Point robot_pos;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (last_odom_position_) {
        robot_pos = *last_odom_position_;
      }
    }
    
    // 遍历所有多边形，仅绘制边界线
    for (const auto &cpoly : polys) {
      const int n = static_cast<int>(cpoly.vertices.size());
      if (n < 3) continue;
      
      // 绘制多边形边界（连接所有顶点）
      for (int i = 0; i < n; ++i) {
        const auto &v1 = cpoly.vertices[i];
        const auto &v2 = cpoly.vertices[(i + 1) % n];
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = v1.position.x;
        p1.y = v1.position.y;
        p1.z = robot_pos.z;  // 使用机器人当前高度
        p2.x = v2.position.x;
        p2.y = v2.position.y;
        p2.z = robot_pos.z;
        
        polygon_edges.points.push_back(p1);
        polygon_edges.points.push_back(p2);
      }
    }
    
    // 发布 MarkerArray（仅包含边界线）
    poly_marker_array.markers.push_back(polygon_edges);
    poly_viz_pub_->publish(poly_marker_array);
  }
  
  // --------------------------------------------------------------------------
  // 可视化定时器
  // --------------------------------------------------------------------------
  void onVizTimer() {
    publishVisualization();
  }
  
  // --------------------------------------------------------------------------
  // 航点追踪定时器
  // --------------------------------------------------------------------------
  void onTrackerTimer() {
    // 切换逻辑由 local_planner 的到达信号触发，定时器不再自动判断距离
    return;
  }

  // --------------------------------------------------------------------------
  // 服务：推进到下一个航点（由 local_planner 触发）
  // --------------------------------------------------------------------------
  void onAdvanceWaypoint(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    
    bool all_completed = false;
    bool floor_changed = false;
    int prev_floor = tracker_state_.robot_floor;
    int new_floor = tracker_state_.robot_floor;
    int final_floor = tracker_state_.robot_floor;
    std::optional<WaypointWithMeta> prev_wp;
    std::optional<WaypointWithMeta> next_wp;
    
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      
      if (!tracker_state_.is_tracking ||
          tracker_state_.current_index >= tracker_state_.waypoints.size()) {
        response->success = false;
        response->message = "当前没有活跃航点追踪任务";
        return;
      }
      
      prev_wp = tracker_state_.waypoints[tracker_state_.current_index];
      tracker_state_.current_index++;
      
      if (tracker_state_.current_index >= tracker_state_.waypoints.size()) {
        tracker_state_.is_tracking = false;
        all_completed = true;
        final_floor = tracker_state_.robot_floor;
      } else {
        next_wp = tracker_state_.waypoints[tracker_state_.current_index];
        if (next_wp->floor != prev_wp->floor) {
          floor_changed = true;
          prev_floor = tracker_state_.robot_floor;
          new_floor = next_wp->floor;
          tracker_state_.robot_floor = new_floor;
        }
      }
    }
    
    if (all_completed) {
      std_msgs::msg::String reached_msg;
      if (selected_goal_) {
        reached_msg.data = floorIndexText(selected_goal_->floor, selected_goal_->index);
      } else {
        reached_msg.data = "unknown";
        RCLCPP_WARN(get_logger(), "目标完成但 selected_goal_ 为空");
      }
      goal_reached_pub_->publish(reached_msg);

      // 自动序列推进：只有在“完成事件”到来时才前进；完成后延时 N 秒发布下一个
      handleGoalCompleted(reached_msg.data);

      response->success = true;
      response->message = "所有航点追踪完成";
      publishVisualization();
      RCLCPP_INFO(get_logger(), "所有航点追踪完成！当前楼层：%d, goal=%s",
                  final_floor, reached_msg.data.c_str());
      return;
    }
    
    if (next_wp) {
      publishCurrentWaypoint();
      publishVisualization();
      
      if (floor_changed && prev_wp) {
        RCLCPP_INFO(get_logger(), 
                    "楼层状态切换: %d -> %d (跟踪点: %d-%d -> %d-%d)", 
                    prev_floor, 
                    new_floor,
                    prev_wp->floor, 
                    prev_wp->index,
                    next_wp->floor, 
                    next_wp->index);
      }
      
      response->success = true;
      response->message = "已切换到下一个航点";
      return;
    }
    
    response->success = false;
    response->message = "航点切换失败：无有效下一个航点";
  }
  
  // --------------------------------------------------------------------------
  // 服务：选择目标点
  // --------------------------------------------------------------------------
  void onSelectGoal(
      const std::shared_ptr<waypoint_manager::srv::SelectGoal::Request> request,
      std::shared_ptr<waypoint_manager::srv::SelectGoal::Response> response) {
    std::string msg;
    const bool ok = selectGoalByFloorIndex(request->floor_index, /*is_manual=*/true, msg);
    response->success = ok;
    response->message = msg;
  }

  // --------------------------------------------------------------------------
  // 统一入口：按 floor-index 选择目标点（手动 / 自动序列复用）
  // --------------------------------------------------------------------------
  bool selectGoalByFloorIndex(const std::string &floor_index,
                              bool is_manual,
                              std::string &out_message) {
    out_message.clear();

    // 解析 floor-index
    const auto parsed = parseFloorIndex(floor_index);
    if (!parsed) {
      out_message = "floor_index 解析失败，期望格式如 \"2-3\"";
      return false;
    }

    const int floor = parsed->first;
    const int index = parsed->second;

    // 查找目标点
    auto it_floor = goalpoints_by_floor_index_.find(floor);
    if (it_floor == goalpoints_by_floor_index_.end()) {
      out_message = "goalpoints 中不存在 floor=" + std::to_string(floor);
      return false;
    }

    auto it_goal = it_floor->second.find(index);
    if (it_goal == it_floor->second.end()) {
      out_message = "goalpoints 中不存在 " + floorIndexText(floor, index);
      return false;
    }

    const CsvPoint goal = it_goal->second;

    // 手动插队：只有在手动 select 成功时才暂停自动序列
    if (is_manual) {
      goal_sequence_runner_.onManualGoalSelected();
      cancelGoalSequenceTimer();
      auto_goal_active_ = false;  // 手动覆盖自动目标
      active_goal_source_ = ActiveGoalSource::Manual;
    } else {
      active_goal_source_ = ActiveGoalSource::AutoSequence;
      auto_goal_active_ = true;
    }

    // 发布目标点
    publishGoal(goal);

    // 路径规划
    std::string why;
    const auto path_with_meta = computePathToGoal(goal, why);

    // 更新选中目标（用于可视化）
    selected_goal_ = goal;
    publishVisualization();

    if (!path_with_meta.empty()) {
      // 发布路径可视化
      publishPath(extractPositions(path_with_meta));

      // 启动航点追踪
      {
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        waypoint_tracker::startTracking(tracker_state_, path_with_meta, goal.floor);
      }
      publishCurrentWaypoint();

      RCLCPP_INFO(get_logger(),
                  "开始航点追踪，共 %zu 个点，当前楼层：%d，目标楼层：%d",
                  path_with_meta.size(),
                  tracker_state_.robot_floor,
                  goal.floor);
    } else {
      // 对手动：允许发布 goalpoint，但提示路径为空
      // 对自动序列：路径为空会导致无法推进（local_planner 只看 /way_point），因此后续会在调度处 skip
      RCLCPP_WARN(get_logger(), "路径规划为空：%s", why.c_str());
    }

    out_message = "已发布目标点 " + floorIndexText(floor, index) + "；path=" + why;

    // 若是自动序列但路径为空，立即标记为“失败并跳过”，避免卡死
    if (!is_manual && path_with_meta.empty()) {
      out_message += "；auto_seq: empty_path -> skip";
      auto_goal_active_ = false;
      active_goal_source_ = ActiveGoalSource::None;
      scheduleSkipAndNext("empty_path");
    }

    return true;
  }

  void cancelGoalSequenceTimer() {
    if (goal_sequence_timer_) {
      goal_sequence_timer_->cancel();
      goal_sequence_timer_.reset();
    }
  }

  void scheduleSkipAndNext(const std::string &why) {
    if (!goal_sequence_runner_.enabled() || goal_sequence_runner_.completed()) {
      return;
    }
    // skip 当前项
    goal_sequence_runner_.skipCurrentAutoGoal();
    if (goal_sequence_runner_.completed()) {
      return;
    }
    scheduleNextAutoGoal("skip:" + why);
  }

  void scheduleNextAutoGoal(const std::string &reason) {
    if (!goal_sequence_runner_.enabled() || goal_sequence_runner_.completed()) {
      return;
    }
    if (!have_first_odom_) {
      return;
    }
    // 当前处于手动插队则不调度
    if (active_goal_source_ == ActiveGoalSource::Manual) {
      return;
    }
    // 若当前已有自动目标在跑，不重复发布
    if (auto_goal_active_) {
      return;
    }

    cancelGoalSequenceTimer();

    const auto delay_ms =
        std::max(0, static_cast<int>(goal_sequence_next_delay_sec_ * 1000.0));
    goal_sequence_timer_ = create_wall_timer(
        std::chrono::milliseconds(delay_ms),
        [this, reason]() {
          // one-shot
          cancelGoalSequenceTimer();
          maybeDispatchAutoGoal(reason);
        });
  }

  void maybeDispatchAutoGoal(const std::string &reason) {
    if (!goal_sequence_runner_.enabled() || goal_sequence_runner_.completed()) {
      return;
    }
    if (!have_first_odom_) {
      return;
    }
    if (active_goal_source_ == ActiveGoalSource::Manual) {
      return;
    }
    if (auto_goal_active_) {
      return;
    }

    const auto next = goal_sequence_runner_.currentAutoFloorIndex();
    if (!next) {
      return;
    }

    std::string msg;
    const bool ok = selectGoalByFloorIndex(*next, /*is_manual=*/false, msg);
    if (!ok) {
      RCLCPP_WARN(get_logger(),
                  "auto_seq dispatch failed (%s): %s",
                  reason.c_str(), msg.c_str());
      // 解析/查找失败：跳过该项，延时后继续下一个
      auto_goal_active_ = false;
      active_goal_source_ = ActiveGoalSource::None;
      scheduleSkipAndNext("dispatch_failed");
      return;
    }

    RCLCPP_INFO(get_logger(), "auto_seq dispatch (%s): %s", reason.c_str(), msg.c_str());
  }

  void handleGoalCompleted(const std::string &reached_floor_index) {
    (void)reached_floor_index;

    if (!goal_sequence_runner_.enabled()) {
      active_goal_source_ = ActiveGoalSource::None;
      auto_goal_active_ = false;
      return;
    }

    if (active_goal_source_ == ActiveGoalSource::AutoSequence) {
      auto_goal_active_ = false;
      goal_sequence_runner_.onAutoGoalCompleted();
      active_goal_source_ = ActiveGoalSource::None;
      if (!goal_sequence_runner_.completed()) {
        scheduleNextAutoGoal("auto_goal_completed");
      }
      return;
    }

    if (active_goal_source_ == ActiveGoalSource::Manual) {
      goal_sequence_runner_.onManualGoalCompleted();
      active_goal_source_ = ActiveGoalSource::None;
      // 手动完成后恢复自动序列（如果还有剩余）
      if (!goal_sequence_runner_.completed()) {
        scheduleNextAutoGoal("manual_goal_completed");
      }
      return;
    }
  }
  
  // --------------------------------------------------------------------------
  // 服务：手动设置楼层状态
  // --------------------------------------------------------------------------
  void onSetFloorState(
      const std::shared_ptr<waypoint_manager::srv::SetFloorState::Request> req,
      std::shared_ptr<waypoint_manager::srv::SetFloorState::Response> resp) {
    
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    resp->previous_floor = tracker_state_.robot_floor;
    tracker_state_.robot_floor = req->floor;
    resp->success = true;
    resp->message = "楼层状态已更新：" + std::to_string(resp->previous_floor) 
                    + " -> " + std::to_string(req->floor);
    
    RCLCPP_INFO(get_logger(), "手动设置楼层状态: %d -> %d", 
                resp->previous_floor, req->floor);
  }
  
  // --------------------------------------------------------------------------
  // 计算到目标的路径（带元数据）
  // --------------------------------------------------------------------------
  std::vector<WaypointWithMeta> computePathToGoal(
      const CsvPoint &goal, 
      std::string &why) {
    
    why.clear();
    
    // 通过 TF 获取车辆中心位置
    const auto vehicle_pos_opt = getVehicleCenterPosition();
    if (!vehicle_pos_opt) {
      why = "无法获取里程计位置";
      return {};
    }
    
    // 获取当前楼层
    int current_floor;
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      current_floor = tracker_state_.robot_floor;
    }
    
    // 委托给路径规划器
    return path_planner::planPath(
        waypoints_by_floor_, 
        current_floor, 
        *vehicle_pos_opt, 
        goal, 
        why);
  }
  
  // --------------------------------------------------------------------------
  // 发布目标点消息
  // --------------------------------------------------------------------------
  void publishGoal(const CsvPoint &goal) {
    waypoint_manager::msg::GoalPoint msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.floor = goal.floor;
    msg.index = goal.index;
    msg.pose.position = goal.position;
    msg.pose.orientation = yawToQuaternion(goal.yaw);
    
    goal_pub_->publish(msg);
  }
  
  // --------------------------------------------------------------------------
  // 发布路径可视化
  // --------------------------------------------------------------------------
  void publishPath(const std::vector<geometry_msgs::msg::Point> &path_points) {
    const auto path_msg = visualization_helper::makePathMessage(
        path_points, frame_id_, now());
    path_pub_->publish(path_msg);
  }
  
  // --------------------------------------------------------------------------
  // 发布当前航点（带迭代外推缓存机制）
  // --------------------------------------------------------------------------
  void publishCurrentWaypoint() {
    const auto current_wp = waypoint_tracker::getCurrentWaypoint(tracker_state_);
    if (!current_wp) {
      return;
    }
    
    // 检测航点索引变化，清空缓存
    if (!last_reprojected_waypoint_index_ || 
        *last_reprojected_waypoint_index_ != tracker_state_.current_index) {
      last_reprojected_waypoint_index_.reset();
      last_reprojected_position_.reset();
    }
    
    auto msg = visualization_helper::makeWaypointMessage(
        *current_wp, frame_id_, now());
    
    // 使用缓存的外推位置（若存在）
    if (last_reprojected_position_) {
      msg.point.x = last_reprojected_position_->x;
      msg.point.y = last_reprojected_position_->y;
    }
    
    // 迭代检测与外推：检测当前位置（原始或缓存）是否还在多边形内
    const bool adjusted = reprojectPointOutsidePolygons(msg.point);
    if (adjusted) {
      // 更新缓存：记录外推后的位置
      last_reprojected_waypoint_index_ = tracker_state_.current_index;
      last_reprojected_position_ = cv::Point2f(msg.point.x, msg.point.y);
    }
    
    waypoint_pub_->publish(msg);
    
    RCLCPP_INFO(get_logger(), 
                "发布航点 [%zu/%zu]: (%.2f, %.2f, %.2f), floor=%d, index=%d%s",
                tracker_state_.current_index + 1,
                tracker_state_.waypoints.size(),
                msg.point.x, 
                msg.point.y, 
                msg.point.z,
                current_wp->floor, 
                current_wp->index,
                current_wp->is_connector ? " [连接点]" : "");
    if (adjusted) {
      RCLCPP_INFO(get_logger(), "航点在多边形内，已沿连线方向外推到安全位置");
    }
  }
  
  // --------------------------------------------------------------------------
  // 发布完整可视化
  // --------------------------------------------------------------------------
  void publishVisualization() {
    std::optional<WaypointWithMeta> current_tracking_wp;
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      current_tracking_wp = waypoint_tracker::getCurrentWaypoint(tracker_state_);
    }
    
    const auto arr = visualization_helper::makeFullMarkerArray(
        waypoints_by_floor_,
        goalpoints_by_floor_index_,
        selected_goal_,
        current_tracking_wp,
        frame_id_,
        now());
    
    marker_pub_->publish(arr);
  }
  
  // --------------------------------------------------------------------------
  // 辅助函数：yaw 转四元数
  // --------------------------------------------------------------------------
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

private:
  // 参数
  std::string use_to_nav_dir_;
  std::string waypoints_csv_param_;
  std::string goalpoints_csv_param_;
  std::string waypoints_csv_;
  std::string goalpoints_csv_;
  std::string frame_id_;
  std::string odom_topic_;
  std::string terrain_map_topic_;
  std::string goal_topic_;
  std::string marker_topic_;
  std::string path_topic_;
  std::string poly_viz_topic_;
  std::string service_name_;
  std::string advance_service_name_;
  std::string waypoint_topic_;
  std::string goal_reached_topic_;
  std::vector<std::string> goal_sequence_floor_indices_;
  double goal_sequence_next_delay_sec_{3.0};
  double waypoint_reach_xy_threshold_{1.5};
  double waypoint_reach_z_threshold_{2.0};
  double tracker_check_rate_{10.0};
  double poly_grid_range_{8.0};
  double poly_grid_resolution_{0.2};
  int poly_inflate_cells_{1};
  double poly_min_area_{0.5};
  double poly_free_radius_{0.6};
  double poly_intensity_thred_{2.0};
  double poly_height_lower_{0.5};  // Z轴下方过滤范围（米）
  double poly_height_upper_{2.0};  // Z轴上方过滤范围（米）
  double poly_wall_angle_thred_{8.0};  // 墙体角度阈值（度，推荐5-10）
  
  // 数据
  std::unordered_map<int, std::vector<CsvPoint>> waypoints_by_floor_;
  std::unordered_map<int, std::unordered_map<int, CsvPoint>> goalpoints_by_floor_index_;
  
  // 里程计
  std::mutex odom_mutex_;
  std::optional<geometry_msgs::msg::Point> last_odom_position_;
  std::mutex poly_mutex_;
  std::vector<ConvexPolygon> convex_polygons_;  // 带凹凸性分析的多边形
  bool warned_no_intensity_{false};
  
  // 外推缓存（用于迭代外推，直到切换航点）
  std::optional<size_t> last_reprojected_waypoint_index_;  // 上次外推的航点索引
  std::optional<cv::Point2f> last_reprojected_position_;    // 上次外推后的位置
  
  // 状态
  std::optional<CsvPoint> selected_goal_;
  TrackerState tracker_state_;
  std::mutex tracker_mutex_;

  // 自动 goal 序列（纯逻辑 + 定时器）
  goal_sequence_runner::GoalSequenceRunner goal_sequence_runner_;
  bool have_first_odom_{false};
  bool auto_goal_active_{false};
  ActiveGoalSource active_goal_source_{ActiveGoalSource::None};
  rclcpp::TimerBase::SharedPtr goal_sequence_timer_;
  
  // ROS2 接口
  rclcpp::Publisher<waypoint_manager::msg::GoalPoint>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr poly_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_reached_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Service<waypoint_manager::srv::SelectGoal>::SharedPtr select_goal_srv_;
  rclcpp::Service<waypoint_manager::srv::SetFloorState>::SharedPtr set_floor_state_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr advance_waypoint_srv_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
  rclcpp::TimerBase::SharedPtr tracker_timer_;
};

// ============================================================================
// main 函数
// ============================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointSearchNode>());
  rclcpp::shutdown();
  return 0;
}