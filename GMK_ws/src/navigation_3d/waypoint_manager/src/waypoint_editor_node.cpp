#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
 
#include <waypoint_manager/srv/list_floor_index.hpp>
#include <waypoint_manager/srv/select_point.hpp>
#include <waypoint_manager/srv/set_axis_control.hpp>
 
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
 
namespace fs = std::filesystem;
 
struct CsvPoint {
  int floor{};
  int index{};
  geometry_msgs::msg::Point position;
  double yaw{};  // rad
};
 
static std::string trimCopy(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}
 
static std::optional<std::pair<int, int>> parseFloorIndex(const std::string &floor_index_raw) {
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
 
static std::string floorIndexText(int floor, int index) {
  return std::to_string(floor) + "-" + std::to_string(index);
}
 
static int makeMarkerId(int floor, int index) {
  return ((floor & 0xFFFF) << 16) | (index & 0xFFFF);
}
 
static std::optional<fs::path> findLatestCsvByPrefix(const fs::path &dir,
                                                     const std::string &prefix) {
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return std::nullopt;
  }
 
  bool found = false;
  fs::path best;
  fs::file_time_type best_time{};
  for (const auto &entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto p = entry.path();
    if (p.extension() != ".csv") {
      continue;
    }
    const auto name = p.filename().string();
    if (name.rfind(prefix, 0) != 0) {  // not starts_with
      continue;
    }
    const auto t = entry.last_write_time();
    if (!found || t > best_time) {
      found = true;
      best_time = t;
      best = p;
    }
  }
  if (!found) {
    return std::nullopt;
  }
  return best;
}
 
static std::vector<CsvPoint> readCsvPoints(const std::string &csv_path, rclcpp::Logger logger) {
  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(logger, "打开CSV失败: %s", csv_path.c_str());
    return {};
  }
 
  std::vector<CsvPoint> out;
  std::string line;
 
  if (!std::getline(ifs, line)) {
    return {};
  }
 
  auto try_parse = [&](const std::string &ln, CsvPoint &p) -> bool {
    std::stringstream ss(ln);
    std::string cell;
    std::vector<std::string> cols;
    while (std::getline(ss, cell, ',')) {
      cols.push_back(trimCopy(cell));
    }
    if (cols.size() < 6) {
      return false;
    }
    try {
      p.floor = std::stoi(cols[0]);
      p.index = std::stoi(cols[1]);
      p.position.x = std::stod(cols[2]);
      p.position.y = std::stod(cols[3]);
      p.position.z = std::stod(cols[4]);
      p.yaw = std::stod(cols[5]);
      return true;
    } catch (...) {
      return false;
    }
  };
 
  CsvPoint p;
  if (try_parse(line, p)) {
    out.push_back(p);
  }
 
  while (std::getline(ifs, line)) {
    CsvPoint q;
    if (!try_parse(line, q)) {
      RCLCPP_WARN(logger, "跳过无法解析的CSV行: %s", line.c_str());
      continue;
    }
    out.push_back(q);
  }
  return out;
}
 
static bool writeCsvPoints(const std::string &csv_path, const std::vector<CsvPoint> &points,
                           rclcpp::Logger logger) {
  std::ofstream ofs(csv_path, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(logger, "写CSV失败(无法打开): %s", csv_path.c_str());
    return false;
  }
  ofs << "floor,index,x,y,z,yaw\n";
  for (const auto &p : points) {
    ofs << p.floor << "," << p.index << "," << p.position.x << "," << p.position.y << ","
        << p.position.z << "," << p.yaw << "\n";
  }
  ofs.flush();
  return true;
}
 
class WaypointEditorNode : public rclcpp::Node {
public:
  WaypointEditorNode() : rclcpp::Node("waypoint_editor") {
    use_to_nav_dir_ = declare_parameter<std::string>(
        "use_to_nav_dir", "~/nav_ws/project/data/use_to_nav");
    waypoints_csv_param_ = declare_parameter<std::string>("waypoints_csv", "");
    goalpoints_csv_param_ = declare_parameter<std::string>("goalpoints_csv", "");
 
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/waypoint_editor_markers");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);

    // 连续控制：最大速度（米/秒、弧度/秒）
    max_x_per_sec_ = declare_parameter<double>("max_x_per_sec", 0.8);
    max_y_per_sec_ = declare_parameter<double>("max_y_per_sec", 0.8);
    max_z_per_sec_ = declare_parameter<double>("max_z_per_sec", 0.6);
    max_yaw_per_sec_ = declare_parameter<double>("max_yaw_per_sec", 1.0);
 
    loadCsvData();
 
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(10));
 
    list_srv_ = create_service<waypoint_manager::srv::ListFloorIndex>(
        "list_floor_index",
        std::bind(&WaypointEditorNode::onList, this, std::placeholders::_1, std::placeholders::_2));
    select_srv_ = create_service<waypoint_manager::srv::SelectPoint>(
        "select_point",
        std::bind(&WaypointEditorNode::onSelect, this, std::placeholders::_1, std::placeholders::_2));
    control_srv_ = create_service<waypoint_manager::srv::SetAxisControl>(
        "set_axis_control",
        std::bind(&WaypointEditorNode::onSetAxisControl, this, std::placeholders::_1,
                  std::placeholders::_2));
 
    publishMarkers();  // immediate
    viz_timer_ = create_wall_timer(std::chrono::milliseconds(std::max(50, publish_period_ms_)),
                                   std::bind(&WaypointEditorNode::publishMarkers, this));

    // 连续控制积分：固定 50Hz，保证滑杆手感平滑
    last_control_time_ = now();
    control_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                       std::bind(&WaypointEditorNode::onControlTimer, this));
 
    RCLCPP_INFO(get_logger(), "use_to_nav_dir: %s", use_to_nav_dir_.c_str());
    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "goalpoints_csv: %s", goalpoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "marker_topic: %s", marker_topic_.c_str());
    RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id_.c_str());
 
    const auto full_list =
        rclcpp::expand_topic_or_service_name("list_floor_index", get_name(), get_namespace());
    const auto full_select =
        rclcpp::expand_topic_or_service_name("select_point", get_name(), get_namespace());
    const auto full_control =
        rclcpp::expand_topic_or_service_name("set_axis_control", get_name(), get_namespace());
    RCLCPP_INFO(get_logger(),
                "示例(list): ros2 service call %s waypoint_manager/srv/ListFloorIndex "
                "\"{is_goalpoint: false}\"",
                full_list.c_str());
    RCLCPP_INFO(get_logger(),
                "示例(select): ros2 service call %s waypoint_manager/srv/SelectPoint "
                "\"{is_goalpoint: false, floor_index: '1-1'}\"",
                full_select.c_str());
    RCLCPP_INFO(get_logger(),
                "示例(slider x=0.5): ros2 service call %s waypoint_manager/srv/SetAxisControl "
                "\"{axis: 0, value: 0.5}\"",
                full_control.c_str());
  }
 
  ~WaypointEditorNode() override {
    // 退出时一次性写回 CSV：运行时不刷盘，避免 I/O 影响实时性。
    flushDirtyToCsvOnExit();
  }

private:
  struct Snapshot {
    std::string frame_id;
    rclcpp::Time stamp;
    std::vector<CsvPoint> waypoints;
    std::vector<CsvPoint> goalpoints;
    bool have_selection{false};
    bool selected_is_goalpoint{false};
    int selected_floor{0};
    int selected_index{0};
    std::optional<geometry_msgs::msg::Point> selected_position;
  };

  Snapshot makeSnapshotLocked() {
    Snapshot s;
    s.frame_id = frame_id_;
    s.stamp = now();
    s.waypoints = waypoints_;
    s.goalpoints = goalpoints_;
    s.have_selection = have_selection_;
    s.selected_is_goalpoint = selected_is_goalpoint_;
    s.selected_floor = selected_floor_;
    s.selected_index = selected_index_;
    if (s.have_selection) {
      const auto pos = findPointPosLocked(s.selected_is_goalpoint, s.selected_floor, s.selected_index);
      if (pos) {
        const auto &p = (s.selected_is_goalpoint ? goalpoints_[*pos] : waypoints_[*pos]);
        s.selected_position = p.position;
      }
    }
    return s;
  }

  void publishSelectedOnly(const Snapshot &s) {
    visualization_msgs::msg::MarkerArray arr;

    // selected highlight: fixed id 0 (cover/DELETE stable)
    visualization_msgs::msg::Marker sel;
    sel.header.frame_id = s.frame_id;
    sel.header.stamp = s.stamp;
    sel.ns = "selected_point";
    sel.id = 0;
    if (s.selected_position) {
      sel.type = visualization_msgs::msg::Marker::SPHERE;
      sel.action = visualization_msgs::msg::Marker::ADD;
      sel.pose.position = *s.selected_position;
      sel.pose.orientation.w = 1.0;
      sel.scale.x = 0.95;
      sel.scale.y = 0.95;
      sel.scale.z = 0.95;
      sel.color.r = 0.2f;
      sel.color.g = 1.0f;
      sel.color.b = 0.2f;
      sel.color.a = 0.85f;
    } else {
      sel.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(sel);

    // 清理历史遗留的“选中点文本”（早期版本发布过，避免 RViz 里残留造成看到两个文本）
    visualization_msgs::msg::Marker sel_text_del;
    sel_text_del.header.frame_id = s.frame_id;
    sel_text_del.header.stamp = s.stamp;
    sel_text_del.ns = "selected_point_id";
    sel_text_del.id = 0;
    sel_text_del.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(sel_text_del);

    marker_pub_->publish(arr);
  }

  void loadCsvData() {
    if (!waypoints_csv_param_.empty()) {
      waypoints_csv_ = waypoints_csv_param_;
    } else {
      const auto best = findLatestCsvByPrefix(fs::path(use_to_nav_dir_), "waypoints-");
      waypoints_csv_ = best ? best->string() : "";
    }
 
    if (!goalpoints_csv_param_.empty()) {
      goalpoints_csv_ = goalpoints_csv_param_;
    } else {
      const auto best = findLatestCsvByPrefix(fs::path(use_to_nav_dir_), "goalpoints-");
      goalpoints_csv_ = best ? best->string() : "";
    }
 
    std::lock_guard<std::mutex> lk(mu_);
    waypoints_.clear();
    goalpoints_.clear();
    wp_index_.clear();
    gp_index_.clear();
 
    if (!waypoints_csv_.empty()) {
      waypoints_ = readCsvPoints(waypoints_csv_, get_logger());
      for (size_t i = 0; i < waypoints_.size(); ++i) {
        wp_index_[waypoints_[i].floor][waypoints_[i].index] = i;
      }
    }
 
    if (!goalpoints_csv_.empty()) {
      goalpoints_ = readCsvPoints(goalpoints_csv_, get_logger());
      for (size_t i = 0; i < goalpoints_.size(); ++i) {
        gp_index_[goalpoints_[i].floor][goalpoints_[i].index] = i;
      }
    }
  }
 
  std::vector<std::string> listFloorIndicesLocked(bool is_goalpoint) const {
    std::vector<std::string> out;
    const auto &v = is_goalpoint ? goalpoints_ : waypoints_;
    out.reserve(v.size());
    for (const auto &p : v) {
      out.push_back(floorIndexText(p.floor, p.index));
    }
    std::sort(out.begin(), out.end(), [](const std::string &a, const std::string &b) {
      // sort by floor then index if possible
      const auto pa = parseFloorIndex(a);
      const auto pb = parseFloorIndex(b);
      if (pa && pb) {
        if (pa->first != pb->first) return pa->first < pb->first;
        return pa->second < pb->second;
      }
      return a < b;
    });
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }
 
  std::optional<size_t> findPointPosLocked(bool is_goalpoint, int floor, int index) const {
    const auto &mp = is_goalpoint ? gp_index_ : wp_index_;
    auto it_f = mp.find(floor);
    if (it_f == mp.end()) return std::nullopt;
    auto it_i = it_f->second.find(index);
    if (it_i == it_f->second.end()) return std::nullopt;
    return it_i->second;
  }
 
  bool writeBackLocked(bool is_goalpoint) {
    auto vec = is_goalpoint ? goalpoints_ : waypoints_;
    std::sort(vec.begin(), vec.end(), [](const CsvPoint &a, const CsvPoint &b) {
      if (a.floor != b.floor) return a.floor < b.floor;
      return a.index < b.index;
    });
    const std::string path = is_goalpoint ? goalpoints_csv_ : waypoints_csv_;
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "CSV路径为空，无法写回");
      return false;
    }
    return writeCsvPoints(path, vec, get_logger());
  }
 
  void onList(const std::shared_ptr<waypoint_manager::srv::ListFloorIndex::Request> req,
              std::shared_ptr<waypoint_manager::srv::ListFloorIndex::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    resp->success = true;
    resp->message = "ok";
    resp->floor_indices = listFloorIndicesLocked(req->is_goalpoint);
  }
 
  void onSelect(const std::shared_ptr<waypoint_manager::srv::SelectPoint::Request> req,
                std::shared_ptr<waypoint_manager::srv::SelectPoint::Response> resp) {
    const auto parsed = parseFloorIndex(req->floor_index);
    if (!parsed) {
      resp->success = false;
      resp->message = "floor_index 解析失败，期望格式如 \"2-3\"";
      return;
    }
 
    Snapshot snap;
    {
      std::lock_guard<std::mutex> lk(mu_);
      const auto pos = findPointPosLocked(req->is_goalpoint, parsed->first, parsed->second);
      if (!pos) {
        resp->success = false;
        resp->message = "未找到点 " + req->floor_index;
        return;
      }
      selected_is_goalpoint_ = req->is_goalpoint;
      selected_floor_ = parsed->first;
      selected_index_ = parsed->second;
      have_selection_ = true;

      snap = makeSnapshotLocked();
    }

    resp->success = true;
    resp->message = "已选择 " + std::string(req->is_goalpoint ? "goalpoint " : "waypoint ") +
                    req->floor_index;

    // 锁外发布，避免递归加锁死锁
    publishMarkersFromSnapshot(snap);
  }
 
  void onSetAxisControl(
      const std::shared_ptr<waypoint_manager::srv::SetAxisControl::Request> req,
      std::shared_ptr<waypoint_manager::srv::SetAxisControl::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    if (req->axis < 0 || req->axis > 3) {
      resp->success = false;
      resp->message = "axis 只能是 0:x 1:y 2:z 3:yaw";
      return;
    }
    const float v = std::max(-1.0f, std::min(1.0f, req->value));
    control_[static_cast<size_t>(req->axis)] = v;
    resp->success = true;
    resp->message = "ok";
  }

  void onControlTimer() {
    Snapshot snap;
    bool moved = false;
    bool moved_is_goalpoint = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!have_selection_) {
        last_control_time_ = now();
        snap = makeSnapshotLocked();
      } else {
        const auto pos = findPointPosLocked(selected_is_goalpoint_, selected_floor_, selected_index_);
        if (!pos) {
          have_selection_ = false;
          last_control_time_ = now();
          snap = makeSnapshotLocked();
        } else {
          const rclcpp::Time t = now();
          double dt = (t - last_control_time_).seconds();
          last_control_time_ = t;
          if (!(dt > 0.0 && dt < 0.5)) {  // 防止时间跳变
            dt = 0.02;
          }

          CsvPoint &p = (selected_is_goalpoint_ ? goalpoints_[*pos] : waypoints_[*pos]);
          const double dx = static_cast<double>(control_[0]) * max_x_per_sec_ * dt;
          const double dy = static_cast<double>(control_[1]) * max_y_per_sec_ * dt;
          const double dz = static_cast<double>(control_[2]) * max_z_per_sec_ * dt;
          const double dyaw = static_cast<double>(control_[3]) * max_yaw_per_sec_ * dt;

          if (std::abs(dx) + std::abs(dy) + std::abs(dz) + std::abs(dyaw) >= 1e-12) {
            p.position.x += dx;
            p.position.y += dy;
            p.position.z += dz;
            p.yaw += dyaw;
            moved_is_goalpoint = selected_is_goalpoint_;
            if (moved_is_goalpoint) {
              dirty_goalpoints_ = true;
            } else {
              dirty_waypoints_ = true;
            }
            moved = true;
          }
          snap = makeSnapshotLocked();
        }
      }
    }

    // 控制中：高频只发布“选中点高亮”，避免每 20ms 重建全量 MarkerArray
    if (moved) {
      publishSelectedOnly(snap);
    }
  }
 
  void publishMarkersFromSnapshot(const Snapshot &s) {
    visualization_msgs::msg::MarkerArray arr;
 
    // waypoints: red SPHERE_LIST
    visualization_msgs::msg::Marker wp_points;
    wp_points.header.frame_id = s.frame_id;
    wp_points.header.stamp = s.stamp;
    wp_points.ns = "waypoints";
    wp_points.id = 0;
    wp_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    wp_points.action = visualization_msgs::msg::Marker::ADD;
    wp_points.scale.x = 0.75;
    wp_points.scale.y = 0.75;
    wp_points.scale.z = 0.75;
    wp_points.color.r = 1.0f;
    wp_points.color.g = 0.2f;
    wp_points.color.b = 0.1f;
    wp_points.color.a = 1.0f;
 
    // goalpoints: blue SPHERE_LIST
    visualization_msgs::msg::Marker gp_points;
    gp_points.header.frame_id = s.frame_id;
    gp_points.header.stamp = s.stamp;
    gp_points.ns = "goalpoints";
    gp_points.id = 0;
    gp_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    gp_points.action = visualization_msgs::msg::Marker::ADD;
    gp_points.scale.x = 0.65;
    gp_points.scale.y = 0.65;
    gp_points.scale.z = 0.65;
    gp_points.color.r = 0.1f;
    gp_points.color.g = 0.3f;
    gp_points.color.b = 1.0f;
    gp_points.color.a = 1.0f;
 
    // waypoint ids
    for (const auto &p : s.waypoints) {
      wp_points.points.push_back(p.position);
 
      visualization_msgs::msg::Marker text;
      text.header.frame_id = s.frame_id;
      text.header.stamp = s.stamp;
      text.ns = "waypoint_ids";
      text.id = makeMarkerId(p.floor, p.index);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = p.position;
      text.pose.position.z += 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 1.0;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 0.0f;
      text.color.a = 1.0f;
      text.text = floorIndexText(p.floor, p.index);
      arr.markers.push_back(text);
    }
 
    // goalpoint ids
    for (const auto &p : s.goalpoints) {
      gp_points.points.push_back(p.position);
 
      visualization_msgs::msg::Marker text;
      text.header.frame_id = s.frame_id;
      text.header.stamp = s.stamp;
      text.ns = "goalpoint_ids";
      text.id = makeMarkerId(p.floor, p.index);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = p.position;
      text.pose.position.z += 1.2;
      text.pose.orientation.w = 1.0;
      text.scale.z = 1.0;
      text.color.r = 0.8f;
      text.color.g = 0.9f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = floorIndexText(p.floor, p.index);
      arr.markers.push_back(text);
    }
 
    arr.markers.push_back(wp_points);
    arr.markers.push_back(gp_points);
 
    // selected highlight: fixed id 0 (cover/DELETE stable)
    visualization_msgs::msg::Marker sel;
    sel.header.frame_id = s.frame_id;
    sel.header.stamp = s.stamp;
    sel.ns = "selected_point";
    sel.id = 0;
    if (s.selected_position) {
        sel.type = visualization_msgs::msg::Marker::SPHERE;
        sel.action = visualization_msgs::msg::Marker::ADD;
        sel.pose.position = *s.selected_position;
        sel.pose.orientation.w = 1.0;
        sel.scale.x = 0.95;
        sel.scale.y = 0.95;
        sel.scale.z = 0.95;
        sel.color.r = 0.2f;
        sel.color.g = 1.0f;
        sel.color.b = 0.2f;
        sel.color.a = 0.85f;
    } else {
      sel.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(sel);

    // 同上：确保不再显示额外“选中点文本”
    visualization_msgs::msg::Marker sel_text_del;
    sel_text_del.header.frame_id = s.frame_id;
    sel_text_del.header.stamp = s.stamp;
    sel_text_del.ns = "selected_point_id";
    sel_text_del.id = 0;
    sel_text_del.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(sel_text_del);
 
    marker_pub_->publish(arr);
  }

  void publishMarkers() {
    Snapshot snap;
    {
      std::lock_guard<std::mutex> lk(mu_);
      snap = makeSnapshotLocked();
    }
    publishMarkersFromSnapshot(snap);
  }

  void flushDirtyToCsvOnExit() {
    // 锁内复制数据与路径，锁外写盘（避免退出时卡住其他回调/死锁）。
    bool need_wp = false;
    bool need_gp = false;
    std::string wp_path;
    std::string gp_path;
    std::vector<CsvPoint> wp;
    std::vector<CsvPoint> gp;
    {
      std::lock_guard<std::mutex> lk(mu_);
      need_wp = dirty_waypoints_;
      need_gp = dirty_goalpoints_;
      wp_path = waypoints_csv_;
      gp_path = goalpoints_csv_;
      if (need_wp) {
        wp = waypoints_;
      }
      if (need_gp) {
        gp = goalpoints_;
      }
      // 不在这里清 dirty：若写盘失败，至少日志能提示；进程即将退出无需再纠结状态。
    }

    auto save = [&](const std::string &path, std::vector<CsvPoint> vec, const char *tag) {
      if (path.empty()) {
        RCLCPP_ERROR(get_logger(), "[%s] CSV路径为空，无法写回", tag);
        return;
      }
      std::sort(vec.begin(), vec.end(), [](const CsvPoint &a, const CsvPoint &b) {
        if (a.floor != b.floor) return a.floor < b.floor;
        return a.index < b.index;
      });
      if (!writeCsvPoints(path, vec, get_logger())) {
        RCLCPP_ERROR(get_logger(), "[%s] 写回CSV失败: %s", tag, path.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "[%s] 已写回CSV: %s", tag, path.c_str());
    };

    if (need_wp) {
      save(wp_path, std::move(wp), "waypoints");
    }
    if (need_gp) {
      save(gp_path, std::move(gp), "goalpoints");
    }
  }
 
private:
  std::string use_to_nav_dir_;
  std::string waypoints_csv_param_;
  std::string goalpoints_csv_param_;
  std::string waypoints_csv_;
  std::string goalpoints_csv_;
 
  std::string frame_id_;
  std::string marker_topic_;
  int publish_period_ms_{200};
 
  double max_x_per_sec_{0.8};
  double max_y_per_sec_{0.8};
  double max_z_per_sec_{0.6};
  double max_yaw_per_sec_{1.0};
 
  std::mutex mu_;
  std::vector<CsvPoint> waypoints_;
  std::vector<CsvPoint> goalpoints_;
  std::unordered_map<int, std::unordered_map<int, size_t>> wp_index_;
  std::unordered_map<int, std::unordered_map<int, size_t>> gp_index_;
 
  bool have_selection_{false};
  bool selected_is_goalpoint_{false};
  int selected_floor_{0};
  int selected_index_{0};

  std::array<float, 4> control_{{0.0f, 0.0f, 0.0f, 0.0f}};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool dirty_waypoints_{false};
  bool dirty_goalpoints_{false};
 
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<waypoint_manager::srv::ListFloorIndex>::SharedPtr list_srv_;
  rclcpp::Service<waypoint_manager::srv::SelectPoint>::SharedPtr select_srv_;
  rclcpp::Service<waypoint_manager::srv::SetAxisControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointEditorNode>());
  rclcpp::shutdown();
  return 0;
}

