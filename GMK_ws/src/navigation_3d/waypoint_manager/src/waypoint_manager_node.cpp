#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <waypoint_manager/srv/record_points.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <ctime>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <functional>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace fs = std::filesystem;

// 这里用一个统一的数据结构存两类点（waypoint / goalpoint），避免复制粘贴两套逻辑。
struct RecordPoint {
    int floor{};                 // 楼层
    int index{};                 // 楼层内索引（从1开始）
    geometry_msgs::msg::Point position; // 位置
    double yaw{};                // 航向角(弧度)
};

class WaypointManagerNode : public rclcpp::Node {
public:
    WaypointManagerNode() : rclcpp::Node("waypoint_manager") {
        // 从参数服务器获取配置参数
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        marker_topic_ = declare_parameter<std::string>("marker_topic", "waypoint_markers");
        waypoint_service_name_ = declare_parameter<std::string>("record_service", "record_waypoint");
        goal_service_name_ = declare_parameter<std::string>("record_goal_service", "record_goalpoint");
        frame_id_override_ = declare_parameter<std::string>("frame_id", "");
        output_dir_ = declare_parameter<std::string>("output_dir", defaultOutputDir());
        waypoint_file_prefix_ = declare_parameter<std::string>("file_prefix", "waypoints-");
        goal_file_prefix_ = declare_parameter<std::string>("goal_file_prefix", "goalpoints-");

        // 确保输出目录存在
        if (output_dir_.empty()) {
            output_dir_ = defaultOutputDir();
        }
        fs::create_directories(output_dir_);

        // 两个CSV使用同一个时间戳：方便对齐、也避免“同秒创建多个文件名撞车”的尴尬。
        timestamp_ = getFileTimestamp();

        // 生成基于现实时间的CSV文件名 (waypoints-MM-DD-HH-MM-SS.csv / goalpoints-MM-DD-HH-MM-SS.csv)
        if (waypoint_file_prefix_.empty()) {
            waypoint_file_prefix_ = "waypoints-";
        }
        if (goal_file_prefix_.empty()) {
            goal_file_prefix_ = "goalpoints-";
        }
        waypoint_csv_path_ =
            (fs::path(output_dir_) / (waypoint_file_prefix_ + timestamp_ + ".csv")).string();
        goal_csv_path_ = (fs::path(output_dir_) / (goal_file_prefix_ + timestamp_ + ".csv")).string();
        
        // 回调组：把高频 odom 和 service 分开，避免单线程 executor 下 service 被饿死
        odom_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 订阅Odometry话题
        rclcpp::SubscriptionOptions odom_opt;
        odom_opt.callback_group = odom_cb_group_;
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(50),
            std::bind(&WaypointManagerNode::onOdom, this, std::placeholders::_1),
            odom_opt
        );

        // 发布可视化标记（waypoint + goalpoint 共用同一个 MarkerArray 话题，简单直接）
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, rclcpp::QoS(10)
        );

        // 两个服务都用同一个 srv 类型（request 里带 floor），调用方式一致。
        record_waypoint_srv_ = create_service<waypoint_manager::srv::RecordPoints>(
            waypoint_service_name_,
            std::bind(&WaypointManagerNode::onRecordWaypointService, this, std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default,
            srv_cb_group_
        );
        record_goal_srv_ = create_service<waypoint_manager::srv::RecordPoints>(
            goal_service_name_,
            std::bind(&WaypointManagerNode::onRecordGoalService, this, std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default,
            srv_cb_group_
        );

        // 启动后台工作线程处理文件I/O和可视化
        worker_thread_ = std::thread(&WaypointManagerNode::workerThread, this);

        // 打印初始化信息
        RCLCPP_INFO(get_logger(), "订阅Odometry话题: %s", odom_topic_.c_str());
        RCLCPP_INFO(get_logger(), "记录waypoint服务: %s", waypoint_service_name_.c_str());
        RCLCPP_INFO(get_logger(), "记录goalpoint服务: %s", goal_service_name_.c_str());
        RCLCPP_INFO(get_logger(), "waypoint CSV: %s", waypoint_csv_path_.c_str());
        RCLCPP_INFO(get_logger(), "goalpoint CSV: %s", goal_csv_path_.c_str());
        RCLCPP_INFO(get_logger(), "可视化标记话题: %s", marker_topic_.c_str());

        // “调试信息最后输出示例”：把用户最关心、最常复制的命令放最后，方便上手。
        const auto full_waypoint_srv =
            rclcpp::expand_topic_or_service_name(waypoint_service_name_, get_name(), get_namespace());
        const auto full_goal_srv =
            rclcpp::expand_topic_or_service_name(goal_service_name_, get_name(), get_namespace());
        RCLCPP_INFO(get_logger(),
                    "示例(waypoint): ros2 service call %s waypoint_manager/srv/RecordPoints "
                    "\"{floor: 1}\"",
                    full_waypoint_srv.c_str());
        RCLCPP_INFO(get_logger(),
                    "示例(goalpoint): ros2 service call %s waypoint_manager/srv/RecordPoints "
                    "\"{floor: 1}\"",
                    full_goal_srv.c_str());
    }

    ~WaypointManagerNode() {
        // 通知工作线程退出
        shutdown_flag_.store(true);
        work_queue_cv_.notify_one();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        // 关闭文件流
        if (waypoint_csv_stream_.is_open()) {
            waypoint_csv_stream_.close();
        }
        if (goal_csv_stream_.is_open()) {
            goal_csv_stream_.close();
        }
    }

private:
    // 默认输出目录：～/.ros/waypoints
    static std::string defaultOutputDir() {
        const char* home = std::getenv("HOME");
        if (home == nullptr) {
            return (fs::temp_directory_path() / "waypoints").string();
        }
        return (fs::path(home) / ".ros" / "waypoints").string();
    }

    // 生成基于现实时间的文件名 (MM-DD-HH-MM-SS)
    std::string getFileTimestamp() const {
        std::time_t now = std::time(nullptr);
        std::tm *local = std::localtime(&now);
        
        char buffer[20];
        // 格式化为：MM-DD-HH-MM-SS
        std::strftime(buffer, sizeof(buffer), "%m-%d-%H-%M-%S", local);
        return std::string(buffer);
    }

    // 从四元数计算航向角(yaw)
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q_msg) {
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    // 让 Marker 的 id 在“同一个 ns 内”唯一：否则楼层内索引重置会互相覆盖。
    static int makeMarkerId(int floor, int index) {
        // floor/index 都用 16 bit 足够用；简单拼接，稳定可预测。
        return ((floor & 0xFFFF) << 16) | (index & 0xFFFF);
    }

    static std::string floorIndexText(int floor, int index) {
        return std::to_string(floor) + "-" + std::to_string(index);
    }

    // 处理Odometry消息
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = *msg;
        if (!have_odom_.load()) {
            RCLCPP_INFO(get_logger(), "首次收到Odometry消息，来源帧: %s", msg->header.frame_id.c_str());
        }
        have_odom_.store(true);
        
        // 使用覆盖帧ID或原始帧ID
        latest_frame_id_ = frame_id_override_.empty() ? msg->header.frame_id : frame_id_override_;
    }

    // 记录 waypoint：通过 service 传入 floor，楼层内 index 从 1 开始。
    void onRecordWaypointService(
        const std::shared_ptr<waypoint_manager::srv::RecordPoints::Request> request,
        std::shared_ptr<waypoint_manager::srv::RecordPoints::Response> response
    ) {
        if (!have_odom_.load()) {
            RCLCPP_WARN(get_logger(), "未收到Odometry消息，无法记录waypoint");
            response->success = false;
            response->message = "未收到Odometry消息";
            return;
        }

        // 快速拿到 odom 快照，避免与高频 odom 回调长时间争锁
        nav_msgs::msg::Odometry odom_snapshot;
        std::string frame_snapshot;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_snapshot = latest_odom_;
            frame_snapshot = latest_frame_id_;
        }

        const int floor = request->floor;
        int index = 0;
        {
            std::lock_guard<std::mutex> lock(points_mutex_);
            index = ++floor_to_waypoint_count_[floor];
        }

        RecordPoint wp;
        wp.floor = floor;
        wp.index = index;
        wp.position = odom_snapshot.pose.pose.position;
        wp.yaw = yawFromQuaternion(odom_snapshot.pose.pose.orientation);

        // INFO 日志在某些环境会产生明显抖动（stdio/console flush），默认降级为 DEBUG
        RCLCPP_DEBUG(get_logger(), "record waypoint %s frame=%s",
                     floorIndexText(wp.floor, wp.index).c_str(), frame_snapshot.c_str());

        {
            std::lock_guard<std::mutex> lock(points_mutex_);
            waypoints_.push_back(wp);
        }
        
        // 立即返回响应
        response->success = true;
        response->message =
            "已记录waypoint，floor=" + std::to_string(wp.floor) + ", index=" + std::to_string(wp.index);

        // 将耗时操作放入后台队列
        {
            std::lock_guard<std::mutex> work_lock(work_queue_mutex_);
            work_queue_.push([this, wp]() {
                appendWaypointCsvLine(wp);
                publishMarkers();
            });
        }
        work_queue_cv_.notify_one();

        // 调试信息最后输出示例：复制粘贴友好。
        const auto full_srv =
            rclcpp::expand_topic_or_service_name(waypoint_service_name_, get_name(), get_namespace());
        RCLCPP_INFO(get_logger(),
                    "示例(waypoint): ros2 service call %s waypoint_manager/srv/RecordPoints "
                    "\"{floor: %d}\"",
                    full_srv.c_str(), wp.floor);
    }

    // 记录 goalpoint：同样通过 service 传入 floor，楼层内 index 从 1 开始。
    void onRecordGoalService(
        const std::shared_ptr<waypoint_manager::srv::RecordPoints::Request> request,
        std::shared_ptr<waypoint_manager::srv::RecordPoints::Response> response
    ) {
        if (!have_odom_.load()) {
            RCLCPP_WARN(get_logger(), "未收到Odometry消息，无法记录goalpoint");
            response->success = false;
            response->message = "未收到Odometry消息";
            return;
        }

        nav_msgs::msg::Odometry odom_snapshot;
        std::string frame_snapshot;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_snapshot = latest_odom_;
            frame_snapshot = latest_frame_id_;
        }

        const int floor = request->floor;
        int index = 0;
        {
            std::lock_guard<std::mutex> lock(points_mutex_);
            index = ++floor_to_goal_count_[floor];
        }

        RecordPoint gp;
        gp.floor = floor;
        gp.index = index;
        gp.position = odom_snapshot.pose.pose.position;
        gp.yaw = yawFromQuaternion(odom_snapshot.pose.pose.orientation);

        RCLCPP_DEBUG(get_logger(), "record goalpoint %s frame=%s",
                     floorIndexText(gp.floor, gp.index).c_str(), frame_snapshot.c_str());

        {
            std::lock_guard<std::mutex> lock(points_mutex_);
            goalpoints_.push_back(gp);
        }
        
        // 立即返回响应
        response->success = true;
        response->message =
            "已记录goalpoint，floor=" + std::to_string(gp.floor) + ", index=" + std::to_string(gp.index);

        // 将耗时操作放入后台队列
        {
            std::lock_guard<std::mutex> work_lock(work_queue_mutex_);
            work_queue_.push([this, gp]() {
                appendGoalCsvLine(gp);
                publishMarkers();
            });
        }
        work_queue_cv_.notify_one();

        // 调试信息最后输出示例：复制粘贴友好。
        const auto full_srv =
            rclcpp::expand_topic_or_service_name(goal_service_name_, get_name(), get_namespace());
        RCLCPP_INFO(get_logger(),
                    "示例(goalpoint): ros2 service call %s waypoint_manager/srv/RecordPoints "
                    "\"{floor: %d}\"",
                    full_srv.c_str(), gp.floor);
    }

    void appendWaypointCsvLine(const RecordPoint& wp) {
        if (!waypoint_csv_initialized_) {
            waypoint_csv_stream_.open(waypoint_csv_path_);
            waypoint_csv_stream_ << "floor,index,x,y,z,yaw\n";
            waypoint_csv_stream_.flush();
            waypoint_csv_initialized_ = true;
        }
        waypoint_csv_stream_ << wp.floor << "," << wp.index << ","
            << wp.position.x << "," << wp.position.y << "," << wp.position.z << ","
            << wp.yaw << "\n";
        waypoint_csv_stream_.flush();
        RCLCPP_DEBUG(get_logger(), "saved waypoint csv %s", floorIndexText(wp.floor, wp.index).c_str());
    }

    void appendGoalCsvLine(const RecordPoint& gp) {
        if (!goal_csv_initialized_) {
            goal_csv_stream_.open(goal_csv_path_);
            goal_csv_stream_ << "floor,index,x,y,z,yaw\n";
            goal_csv_stream_.flush();
            goal_csv_initialized_ = true;
        }
        goal_csv_stream_ << gp.floor << "," << gp.index << ","
            << gp.position.x << "," << gp.position.y << "," << gp.position.z << ","
            << gp.yaw << "\n";
        goal_csv_stream_.flush();
        RCLCPP_DEBUG(get_logger(), "saved goalpoint csv %s", floorIndexText(gp.floor, gp.index).c_str());
    }

    // 后台工作线程：处理文件I/O和可视化发布
    void workerThread() {
        while (!shutdown_flag_.load()) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(work_queue_mutex_);
                work_queue_cv_.wait(lock, [this] {
                    return !work_queue_.empty() || shutdown_flag_.load();
                });
                
                if (shutdown_flag_.load() && work_queue_.empty()) {
                    break;
                }
                
                if (!work_queue_.empty()) {
                    task = work_queue_.front();
                    work_queue_.pop();
                }
            }
            
            if (task) {
                task();
            }
        }
    }

    // 发布航点可视化标记
    void publishMarkers() {
        // 先快照数据，避免长时间占用锁
        std::string frame_snapshot;
        std::vector<RecordPoint> waypoints_snapshot;
        std::vector<RecordPoint> goalpoints_snapshot;
        {
            std::lock_guard<std::mutex> lock1(odom_mutex_);
            frame_snapshot = latest_frame_id_.empty() ? "map" : latest_frame_id_;
        }
        {
            std::lock_guard<std::mutex> lock2(points_mutex_);
            waypoints_snapshot = waypoints_;
            goalpoints_snapshot = goalpoints_;
        }
        
        visualization_msgs::msg::MarkerArray arr;
        
        // waypoint 点：红色
        visualization_msgs::msg::Marker waypoint_points;
        waypoint_points.header.frame_id = frame_snapshot;
        waypoint_points.header.stamp = now();
        waypoint_points.ns = "waypoints";
        waypoint_points.id = 0;
        waypoint_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoint_points.action = visualization_msgs::msg::Marker::ADD;
        waypoint_points.scale.x = 0.75;
        waypoint_points.scale.y = 0.75;
        waypoint_points.scale.z = 0.75;
        waypoint_points.color.r = 1.0f;
        waypoint_points.color.g = 0.2f;
        waypoint_points.color.b = 0.1f;
        waypoint_points.color.a = 1.0f;
        waypoint_points.points.reserve(waypoints_snapshot.size());
        for (const auto& wp : waypoints_snapshot) {
            waypoint_points.points.push_back(wp.position);
        }
        arr.markers.push_back(waypoint_points);

        // goalpoint 点：蓝色（跟 waypoint 视觉区分）
        visualization_msgs::msg::Marker goal_points;
        goal_points.header.frame_id = frame_snapshot;
        goal_points.header.stamp = now();
        goal_points.ns = "goalpoints";
        goal_points.id = 0;
        goal_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        goal_points.action = visualization_msgs::msg::Marker::ADD;
        goal_points.scale.x = 0.65;
        goal_points.scale.y = 0.65;
        goal_points.scale.z = 0.65;
        goal_points.color.r = 0.1f;
        goal_points.color.g = 0.3f;
        goal_points.color.b = 1.0f;
        goal_points.color.a = 1.0f;
        goal_points.points.reserve(goalpoints_snapshot.size());
        for (const auto& gp : goalpoints_snapshot) {
            goal_points.points.push_back(gp.position);
        }
        arr.markers.push_back(goal_points);
        
        // waypoint 文本：显示 floor-index
        for (const auto& wp : waypoints_snapshot) {
            visualization_msgs::msg::Marker text;
            text.header.frame_id = frame_snapshot;
            text.header.stamp = now();
            text.ns = "waypoint_ids";
            text.id = makeMarkerId(wp.floor, wp.index);
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position = wp.position;
            text.pose.position.z += 1.0;
            text.pose.orientation.w = 1.0;
            text.scale.z = 1.0;
            text.color.r = 1.0f;
            text.color.g = 1.0f;
            text.color.b = 0.0f;
            text.color.a = 1.0f;
            text.text = floorIndexText(wp.floor, wp.index);
            arr.markers.push_back(text);
        }

        // goalpoint 文本：显示 floor-index
        for (const auto& gp : goalpoints_snapshot) {
            visualization_msgs::msg::Marker text;
            text.header.frame_id = frame_snapshot;
            text.header.stamp = now();
            text.ns = "goalpoint_ids";
            text.id = makeMarkerId(gp.floor, gp.index);
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position = gp.position;
            text.pose.position.z += 1.2;
            text.pose.orientation.w = 1.0;
            text.scale.z = 1.0;
            text.color.r = 0.8f;
            text.color.g = 0.9f;
            text.color.b = 1.0f;
            text.color.a = 1.0f;
            text.text = floorIndexText(gp.floor, gp.index);
            arr.markers.push_back(text);
        }
        
        // 发布标记
        marker_pub_->publish(arr);
    }

private:
    // 配置参数
    std::string odom_topic_;
    std::string marker_topic_;
    std::string waypoint_service_name_;
    std::string goal_service_name_;
    std::string frame_id_override_;
    std::string output_dir_;
    std::string waypoint_file_prefix_;
    std::string goal_file_prefix_;
    std::string timestamp_;
    std::string waypoint_csv_path_;
    std::string goal_csv_path_;
    
    // 状态标志
    bool waypoint_csv_initialized_{false};
    bool goal_csv_initialized_{false};
    std::atomic<bool> have_odom_{false};
    
    // 文件流缓冲（在后台线程中使用）
    std::ofstream waypoint_csv_stream_;
    std::ofstream goal_csv_stream_;
    
    // 机器人状态
    nav_msgs::msg::Odometry latest_odom_;
    std::string latest_frame_id_;
    
    // 记录点存储（只做“简单存储 + 可视化”，不搞复杂结构）
    std::vector<RecordPoint> waypoints_;
    std::vector<RecordPoint> goalpoints_;

    // 每个楼层独立计数：floor -> count
    std::unordered_map<int, int> floor_to_waypoint_count_;
    std::unordered_map<int, int> floor_to_goal_count_;
    
    // ROS2订阅和发布
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<waypoint_manager::srv::RecordPoints>::SharedPtr record_waypoint_srv_;
    rclcpp::Service<waypoint_manager::srv::RecordPoints>::SharedPtr record_goal_srv_;
    
    // 回调组
    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

    // 后台工作线程相关
    std::thread worker_thread_;
    std::mutex work_queue_mutex_;
    std::condition_variable work_queue_cv_;
    std::queue<std::function<void()>> work_queue_;
    std::atomic<bool> shutdown_flag_{false};
    std::mutex odom_mutex_;    // 保护 latest_odom_/latest_frame_id_
    std::mutex points_mutex_;  // 保护 waypoints_/goalpoints_/计数
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointManagerNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}