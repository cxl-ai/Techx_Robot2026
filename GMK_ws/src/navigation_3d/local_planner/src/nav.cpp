//
// created by jinhu
// migrated to ROS2 Humble
//
#include "planning.hpp"
#include "control.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZINormal PointType;

rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_stairs_sub;
std::shared_ptr<Planning> planner;
std::shared_ptr<Controller> controller_ptr;
std::list<std::pair<Eigen::Vector3d, bool>> local_path_list;
rclcpp::Node::SharedPtr nh;

double last_goal_x = -1;
double last_goal_y = -1;
double stairs_speed_adjustment = -0.2;  // 从配置读取

void stairsCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    if (controller_ptr) {
        if (msg->data) {
            // 上楼梯：应用速度调整
            controller_ptr->AdjustSpeed(stairs_speed_adjustment);
        } else {
            // 正常地面：恢复原速度
            controller_ptr->AdjustSpeed(0.0);
        }
    }
}

void localPathCallback(const nav_msgs::msg::Path::ConstSharedPtr pathIn) {
    int pathSize = pathIn->poses.size();

    // 参考 CMU pathFollower: pathSize <= 1 时停止
    if (pathSize <= 1) {
        planner->m_b_stop = true;
        planner->PubZero();  // 发布零速度命令
        return;
    }

    // 从路径第一个点的 orientation 中获取目标点（local_planner 的约定）
    double cur_goal_x = pathIn->poses[0].pose.orientation.x;
    double cur_goal_y = pathIn->poses[0].pose.orientation.y;
 
    double delta_x = cur_goal_x - last_goal_x;
    double delta_y = cur_goal_y - last_goal_y;
    double dist = delta_x * delta_x + delta_y * delta_y;
    dist = sqrt(dist);

    bool b_new = false;
    if (dist > 0.01) {
        b_new = true;
        last_goal_x = cur_goal_x;
        last_goal_y = cur_goal_y;
    }

    local_path_list.clear();
    for (int i = 0; i < pathSize; i++) {
        bool b_plan = true;
        double p[4];
        
        p[0] = pathIn->poses[i].pose.position.x;
        p[1] = pathIn->poses[i].pose.position.y;

        if (i < pathSize - 1) {
            double delta_x = pathIn->poses[i+1].pose.position.x - p[0];
            double delta_y = pathIn->poses[i+1].pose.position.y - p[1];
            p[2] = atan2(delta_y, delta_x);
        } else {
            double delta_x = p[0] - pathIn->poses[i - 1].pose.position.x;
            double delta_y = p[1] - pathIn->poses[i - 1].pose.position.y;
            p[2] = atan2(delta_y, delta_x);
        }

        Eigen::Vector3d local_path_point(p[0], p[1], p[2]);
        local_path_list.emplace_back(local_path_point, b_plan);
    }

    planner->m_path_iter = planner->m_path_list.end();
    planner->m_path_list = local_path_list;
    planner->m_path_iter = planner->m_path_list.begin();
    planner->m_is_get_path = true;
    planner->m_b_stop = false;

    if (b_new) {
        planner->m_ref_state = planner->m_path_iter->first;
        planner->m_path_iter++;
        std::cout << "nav get new goal" << std::endl;
    }
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_) {
    Eigen::Matrix3d R_wb;
    Eigen::Vector3d twb;
    Eigen::Quaterniond q;
    
    q.x() = odom_->pose.pose.orientation.x;
    q.y() = odom_->pose.pose.orientation.y;
    q.z() = odom_->pose.pose.orientation.z;
    q.w() = odom_->pose.pose.orientation.w;
    R_wb = q.toRotationMatrix();
    Eigen::Vector3d angle = LogSO3(R_wb);
    
    twb(0) = odom_->pose.pose.position.x;
    twb(1) = odom_->pose.pose.position.y;
    twb(2) = odom_->pose.pose.position.z;

    Eigen::Vector3d pose(twb(0), twb(1), angle(2));
    double l = -0.3;
    double c = cos(angle(2));
    double s = sin(angle(2));

    pose(0) += l * c;
    pose(1) += l * s;

    if (!planner->m_is_get_path) {
        return;
    }
    
    if (planner->m_path_list.size() <= 10) {

    } else {
        planner->SetState(pose, twb);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("nav_node");
    
    // 读取楼梯速度调整参数
    stairs_speed_adjustment = nh->declare_parameter<double>("control.stairs_speed_adjustment", -0.2);
    
    auto controller = std::make_shared<Controller>(nh);
    controller_ptr = controller;
    planner = std::make_shared<Planning>(nh, controller, false);

    local_path_sub = nh->create_subscription<nav_msgs::msg::Path>(
        "/path", 5, localPathCallback);
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10000, odomCallback);
    is_stairs_sub = nh->create_subscription<std_msgs::msg::Bool>(
        "/is_stairs", 5, stairsCallback);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
