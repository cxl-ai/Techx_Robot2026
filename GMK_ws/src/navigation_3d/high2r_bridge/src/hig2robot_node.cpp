/*
 * @Author: linzhuyue && lzyue@mae.cuhk.edu.hk
 * @Date: 2023-11-20 15:21:51
 * @LastEditors: linzhuyue 
 * @LastEditTime: 2025-12-30 (migrated to ROS2 Humble)
 * @FilePath: /nav_ws/src/high2r_bridge/src/hig2robot_node.cpp
 * @Description: Writed In ERB 106 CUHK. Non-Commercial Used. Migrated to ROS2.
 * 
 * Copyright (c) 2023 by LinzhuYue, All Rights Reserved. 
 */

#include <ltr/robot/channel/channel_factory.hpp>
#include <ltr/robot/channel/channel_publisher.hpp>
#include <ltr/msg/dds_/VelCmd_.hpp>
#include <ltr/msg/dds_/SportModeCmd_.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#include <thread>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace ltr::robot;
using namespace ltr::msg::dds_;

#define TOPIC_VELCMD "rt/ltr/vel_cmd"
#define TOPIC_SPORTMODECMD "rt/ltr/sportmode_cmd"

// Device id for highlevel SDK control path
constexpr uint8_t DEVICE_ID_HIGHLEVEL_SDK = 2;

enum class HighLevelMode : uint8_t {
    PASSIVE = 0,
    DAMPING,
    SITDOWN,
    STANDUP,
    NORMAL_WALK,
    LION_DANCE,
    LION_DANCE_OFF,
};

class High2RBridgeNode : public rclcpp::Node {
public:
    High2RBridgeNode() : Node("high2r_node") {
        // Declare parameters
        this->declare_parameter<std::string>("dds_interface", "eth0");
        this->declare_parameter<std::string>("cmd_topic", "/high2r_cmd");
        this->declare_parameter<double>("max_vx", 1.0);
        this->declare_parameter<double>("max_vy", 0.5);
        this->declare_parameter<double>("max_yaw_rate", 2.0);
        this->declare_parameter<double>("smoothing_alpha", 0.3);  // 一阶滤波系数

        // Get parameters
        std::string network_interface = this->get_parameter("dds_interface").as_string();
        std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
        max_vx_ = this->get_parameter("max_vx").as_double();
        max_vy_ = this->get_parameter("max_vy").as_double();
        max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
        smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();

        RCLCPP_INFO(this->get_logger(), "DDS Interface: %s", network_interface.c_str());
        RCLCPP_INFO(this->get_logger(), "Command Topic: %s", cmd_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Velocity limits: vx=%.2f, vy=%.2f, yaw=%.2f", 
                    max_vx_, max_vy_, max_yaw_rate_);
        RCLCPP_INFO(this->get_logger(), "Smoothing alpha: %.2f", smoothing_alpha_);

        // Initialize DDS channel factory/publisher
        ChannelFactory::Instance()->Init(0, network_interface);
        velcmd_publisher_.InitChannel();
        sportmode_publisher_.InitChannel();

        // Start CLI thread for sport mode switching
        sportmode_cli_thread_ = std::thread(&High2RBridgeNode::sportmode_cli_loop, this);
        sportmode_cli_thread_.detach();

        // Create timer (200Hz)
        double dt = 0.005;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&High2RBridgeNode::plan_timer_callback, this));

        // Create subscriber
        dog_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 10,
            std::bind(&High2RBridgeNode::cmd_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "high2r_node initialized successfully");
    }

private:
    std::string ModeToString(HighLevelMode mode) {
        switch (mode) {
            case HighLevelMode::PASSIVE: return "PASSIVE";
            case HighLevelMode::DAMPING: return "DAMPING";
            case HighLevelMode::SITDOWN: return "SITDOWN";
            case HighLevelMode::STANDUP: return "STANDUP";
            case HighLevelMode::NORMAL_WALK: return "NORMAL_WALK";
            case HighLevelMode::LION_DANCE: return "LION_DANCE";
            case HighLevelMode::LION_DANCE_OFF: return "LION_DANCE_OFF";
        }
        return "UNKNOWN";
    }

    void sportmode_cli_loop() {
        std::cout << "[SportMode CLI] 输入模式编号并回车以切换 rt/ltr/sportmode_cmd" << std::endl;
        std::cout << "[SportMode CLI] 0:PASSIVE 1:DAMPING 2:SITDOWN 3:STANDUP 4:NORMAL_WALK 5:LION_DANCE 6:LION_DANCE_OFF" << std::endl;
        std::cout << "[SportMode CLI] 输入 q 退出 CLI（节点继续运行）" << std::endl;
        std::string line;
        while (rclcpp::ok() && std::getline(std::cin, line)) {
            if (line == "q" || line == "Q") {
                std::cout << "[SportMode CLI] 退出 CLI，保持当前模式" << std::endl;
                break;
            }
            try {
                int value = std::stoi(line);
                if (value < 0 || value > 6) {
                    std::cout << "[SportMode CLI] 无效编号，请输入 0-6" << std::endl;
                    continue;
                }
                SportModeCmd_ msg;
                msg.device_id(DEVICE_ID_HIGHLEVEL_SDK);
                msg.mode(static_cast<uint8_t>(value));
                sportmode_publisher_.Write(msg);
                auto mode = static_cast<HighLevelMode>(value);
                std::cout << "[SportMode CLI] 已发送模式 " << value << " (" << ModeToString(mode) << ")" << std::endl;
            } catch (const std::exception&) {
                std::cout << "[SportMode CLI] 输入不是数字，请输入 0-6 或 q" << std::endl;
            }
        }
    }

    void plan_timer_callback() {
        if (init_) {
            VelCmd_ cmd;
            cmd.lin_vx(static_cast<float>(dog_cmd_[0]));
            cmd.lin_vy(static_cast<float>(dog_cmd_[1]));
            cmd.lin_vz(0.0f);
            cmd.ang_wx(0.0f);
            cmd.ang_wy(0.0f);
            cmd.ang_wz(static_cast<float>(dog_cmd_[2]));
            velcmd_publisher_.Write(cmd);
        }
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        init_ = false;
        
        // 使用参数化的速度限幅
        double vx = std::clamp(msg->linear.x, -max_vx_, max_vx_);
        double vy = std::clamp(msg->linear.y, -max_vy_, max_vy_);
        double yaw_dot = std::clamp(msg->angular.z, -max_yaw_rate_, max_yaw_rate_);

        // 一阶速度平滑滤波: output = alpha * input + (1 - alpha) * prev_output
        // alpha 越小越平滑，alpha=1 时无平滑
        smoothed_cmd_[0] = smoothing_alpha_ * vx + (1.0 - smoothing_alpha_) * smoothed_cmd_[0];
        smoothed_cmd_[1] = smoothing_alpha_ * vy + (1.0 - smoothing_alpha_) * smoothed_cmd_[1];
        smoothed_cmd_[2] = smoothing_alpha_ * yaw_dot + (1.0 - smoothing_alpha_) * smoothed_cmd_[2];

        dog_cmd_[0] = smoothed_cmd_[0];
        dog_cmd_[1] = smoothed_cmd_[1];
        dog_cmd_[2] = smoothed_cmd_[2];
        
        init_ = true;
    }

    // Member variables
    bool init_ = false;
    ChannelPublisher<VelCmd_> velcmd_publisher_{TOPIC_VELCMD};
    ChannelPublisher<SportModeCmd_> sportmode_publisher_{TOPIC_SPORTMODECMD};
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dog_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> dog_cmd_{3, 0.0};
    std::thread sportmode_cli_thread_;
    
    // 速度限幅参数
    double max_vx_{1.0};
    double max_vy_{0.5};
    double max_yaw_rate_{2.0};
    
    // 一阶滤波参数
    double smoothing_alpha_{0.3};  // 滤波系数：0-1，越小越平滑
    std::vector<double> smoothed_cmd_{3, 0.0};  // 平滑后的速度
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<High2RBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
