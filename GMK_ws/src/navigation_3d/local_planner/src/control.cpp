//
// created by jinhu
// migrated to ROS2 Humble
//
#include "control.hpp"

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R_) {
    const double tr = R_(0, 0) + R_(1, 1) + R_(2, 2);
    Eigen::Vector3d w;
    
    w << (R_(2, 1) - R_(1, 2)) / 2, (R_(0, 2) -  R_(2, 0)) / 2, (R_(1, 0) - R_(0, 1)) / 2;
    const double costheta = (tr - 1.0) * 0.5;

    if(costheta > 1 || costheta < -1) {
        return w;
    }

    const double theta = acos(costheta);
    const double s = sin(theta);

    if(fabs(s) < 1e-5) {
        return w;
    } else {
        return theta * w / s;
    }
}

Controller::Controller(const rclcpp::Node::SharedPtr& nh_) {
    // ROS2 参数读取：declare_parameter 会自动从 yaml 文件读取参数值
    // 如果 yaml 中没有该参数，则使用这里提供的默认值
    m_max_v = nh_->declare_parameter<double>("control.max_v", 0.35);
    m_max_w = nh_->declare_parameter<double>("control.max_w", 0.8);
    m_b_nest = nh_->declare_parameter<bool>("control.nested", true);
    
    m_base_max_v = m_max_v;  // 保存基础速度
    
    std::cout << "[Controller] Loaded parameters - max_v: " << m_max_v << ", max_w: " << m_max_w << ", nested: " << m_b_nest << std::endl; 
}

void Controller::ControlLaw() {
    double x_e, y_e, theta_e, e1, e2; 
    double angular_vel = 0;
    double linear_vel = 0;
    double cur_yaw = m_state(2);

    Eigen::Vector3d ref_delta = m_refer_state - m_state;
    double pos_er = ref_delta.block<2, 1>(0, 0).norm();

    x_e = ref_delta(0);
    y_e = ref_delta(1);
    theta_e = ref_delta(2);
    
    e1 =  x_e * cos(cur_yaw) + y_e * sin(cur_yaw);
    e2 = -x_e * sin(cur_yaw) + y_e * cos(cur_yaw);

    if(theta_e > pi) {
        theta_e = theta_e - 2 * pi;
    }

    if(theta_e < - pi) {
        theta_e = 2 * pi + theta_e;
    }

    m_vd = m_max_v;
    m_wd = theta_e / (pos_er / m_vd);

    double vddot = 0.0;
    double wddot = 0.0;

    angular_vel = k2 * e2 * m_vd * cos(theta_e / 2) * cos(theta_e / 4) + k4 * sin(theta_e / 4) + m_wd;
    double dot_angular_vel = k2 * ((-angular_vel * e1 + m_vd * sin(theta_e)) * m_vd + e2 * vddot) * cos(theta_e / 2) * cos(theta_e / 4)
            - (k2 * e2 * m_vd * sin(theta_e / 4) * (0.5 + 0.75 * cos(theta_e / 2)) - 0.25 * k4 * cos(theta_e / 4))*(m_wd - angular_vel) + wddot;
    double e1_bar = e1 - k1 * atan(angular_vel) * e2;

    linear_vel  = m_vd * cos(theta_e) - k1 * e2 / (1 + angular_vel * angular_vel) * dot_angular_vel
            - k1 * atan(angular_vel) * (- angular_vel * e1 + m_vd * sin(theta_e)) + k3 * e1_bar;

    double factor1 = 0;
    double factor2 = 0;

    if(abs(angular_vel) > m_max_w) {
        factor1 = abs(angular_vel) / m_max_w;
    }

    if(abs(linear_vel) > m_max_v) {
        factor2 = abs(linear_vel) / m_max_v;
    }

    if (m_b_nest) {
        if(factor1 != 0 || factor2 != 0) {
            double factor;
            
            if(factor1 > factor2) factor = factor1;
            else factor = factor2;
        
            angular_vel =  angular_vel / factor;
            linear_vel  = linear_vel / factor;
        }
    } else {
        if (factor1 != 0) {
            angular_vel =  angular_vel / factor1;
        }

        if (factor2 != 0) {
            linear_vel  = linear_vel / factor2;
        }
    }

    m_u(0) = linear_vel;
    m_u(1) = angular_vel;
}

void Controller::SetRefState(const Eigen::Vector3d &ref_state_) {
    m_refer_state = ref_state_;
}

void Controller::SetState(const Eigen::Vector3d &state_) {
    m_state = state_;
}

void Controller::AdjustSpeed(double adjustment) {
    m_max_v = m_base_max_v + adjustment;  // 直接加减
    
    // 限制速度范围
    if (m_max_v < 0.1) {
        m_max_v = 0.1;  // 最低速度 0.1 m/s
    }
}
