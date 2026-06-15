//
// created by jinhu
// migrated to ROS2 Humble
//
#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

const double k1 = 0.01;
const double k2 = 8;
const double k3 = 4;
const double k4 = 16;
const double pi = 3.1415926535;

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R_);

class Controller {
public:
    double m_max_vel;
    double m_max_ang_vel;
    double m_vd = 0;
    double m_wd = 0;

    double m_max_v;
    double m_max_w;
    double m_base_max_v;  // 新增：存储基础最大速度
    bool m_b_nest = false;

    Eigen::Vector3d m_state;
    Eigen::Vector3d m_refer_state;
    Eigen::Vector2d m_u;        

    Controller(const rclcpp::Node::SharedPtr& nh_);

    void ControlLaw();

    void SetRefState(const Eigen::Vector3d &ref_state_);

    void SetState(const Eigen::Vector3d &state_);
    
    void AdjustSpeed(double adjustment);  // 新增：绝对值速度调整（加减固定值）
};
