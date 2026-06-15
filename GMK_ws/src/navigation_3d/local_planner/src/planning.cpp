//
// created by jinhu
// migrated to ROS2 Humble
//
#include "planning.hpp"
#include <iostream>
#include <fstream>
#include <geometry_msgs/msg/twist_stamped.hpp>

Planning::Planning(const rclcpp::Node::SharedPtr& nh_, const std::shared_ptr<Controller> controller_, const bool &b_load_,
                   const double &offset_) {
    m_nh = nh_;
    m_controller = controller_;
    pub_cmd = m_nh->create_publisher<geometry_msgs::msg::Twist>("/high2r_cmd", 100000);
    m_offset = offset_;

    // ROS2 参数读取：declare_parameter 直接返回参数值（从 yaml 或默认值）
    m_resolution = m_nh->declare_parameter<double>("planning.resolution", 0.05);
    m_grid_num = m_nh->declare_parameter<int>("planning.map_voxel_num", 400);
    m_dist_static_x_th = m_nh->declare_parameter<double>("planning.dist_static_x_th", 0.5);
    m_dist_static_y_th = m_nh->declare_parameter<double>("planning.dist_static_y_th", 0.25);
    m_dist_th = m_nh->declare_parameter<double>("planning.dist_th", 0.3);
    m_time_th = m_nh->declare_parameter<double>("planning.time_th", 3.0);
    m_arrive_th = m_nh->declare_parameter<double>("planning.arrive_th", 0.15);
    m_dyn_check_num = m_nh->declare_parameter<int>("planning.dyn_check_num", 10);
    m_run_time = m_nh->declare_parameter<double>("planning.run_time", 3.0);
    m_stop_time = m_nh->declare_parameter<double>("planning.stop_time", 0.8);

    std::cout << "dist th: " << m_dist_th << " arrive th: " << m_arrive_th << " " << m_time_th << "  " << offset_ << std::endl;
    std::cout << m_run_time << "   " << m_stop_time << std::endl;

    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (i == 0 && j == 0) {
                continue;
            }
            m_neigh_vec.emplace_back(i, j);
        }
    }

    if (b_load_) {
        std::string path_name = m_nh->declare_parameter<std::string>("planning.path", "");
        std::cout << "path str: " << path_name << std::endl;
        LoadPathFromFile(path_name);
    }
}

void Planning::PubRotate(const double w_z) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = w_z;
    pub_cmd->publish(cmd);
}

void Planning::SetState(const Eigen::Vector3d &state_, const Eigen::Vector3d &pos_) {
    m_state = state_; m_pos = pos_;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;

    if (m_path_iter == m_path_list.end()) {
        pub_cmd->publish(cmd);
        pub_cmd->publish(cmd);
        return;
    }

    if (m_b_stop) {
        pub_cmd->publish(cmd);
        return;
    }

    bool b_select = CheckSelect();
    if (b_select) {
        m_controller->SetRefState(m_ref_state);
    }

    m_controller->SetState(m_state);
    m_controller->ControlLaw();

    cmd.linear.x = m_controller->m_u(0);
    cmd.angular.z = m_controller->m_u(1);

    pub_cmd->publish(cmd);
}

bool Planning::CheckSelect() {
    double time = m_nh->now().seconds();
    bool b_select = false;

    if (m_last_time != 0) {
        if (time - m_last_time > m_time_th) {
            m_last_time = time;
            b_select = true;
        } 
        
        double delta_x = m_ref_state(0) - m_state(0);
        double delta_y = m_ref_state(1) - m_state(1);

        double s_theta = sin(m_ref_state(2));
        double c_theta = cos(m_ref_state(2));

        if (delta_x * c_theta + delta_y * s_theta <= m_arrive_th) {
            b_select = true;
        }
    } else {
        b_select = true;
        m_last_time = time;
    }       
    
    if (b_select) {
        if (m_path_iter == m_path_list.end()) {
            geometry_msgs::msg::Twist cmd;

            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.linear.z = 0;

            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = 0;
            pub_cmd->publish(cmd);

            std::cout << "empty" << std::endl;
            return false;
        }

        while (true) {
            Eigen::Vector3d ref_state = m_path_iter->first;
            m_b_plan = m_path_iter->second;

            double delta_x = ref_state(0) - m_state(0);
            double delta_y = ref_state(1) - m_state(1);

            double s_theta = sin(ref_state(2));
            double c_theta = cos(ref_state(2));
            double dist = delta_x * delta_x + delta_y * delta_y;

            // 90.7733 -19.667 
            if (abs(ref_state(0) - 90.7733) < 0.01 && abs(ref_state(1) + 19.667) < 0.01 && !m_b_enable_sus) {
                m_b_enable_sus = true;
            }
            
            dist = sqrt(dist);
            if (abs(delta_x * c_theta + delta_y * s_theta) > m_dist_th || dist > m_dist_th) {
                m_ref_state = ref_state;
                std::cout << "set new ref: " << m_ref_state.transpose() << std::endl;

                m_path_iter = m_path_list.erase(m_path_iter);
                break;
            }
            
            m_path_iter = m_path_list.erase(m_path_iter);

            if (m_path_iter == m_path_list.end()) {
                m_ref_state = ref_state;
                return true;
            }
        }

        std::cout << "set new ref: " << m_ref_state.transpose() << std::endl;
        return true;
    } else {
        return false;
    }
}

void Planning::LoadPathFromFile(const std::string &path_) {
    std::ifstream ifs1(path_, std::ios::in);
    
    bool b_plan = false;
    double l = -m_offset;

    while (!ifs1.eof()) {
        double p[4];
        ifs1 >> p[0] >> p[1] >> p[2];
    
        double c = cos(p[2]);
        double s = sin(p[2]);

        p[0] += l * c;
        p[1] += l * s;
        Eigen::Vector3d path_point(p[0], p[1], p[2]);

        if (p[3] == 1) {     
            if (b_plan) {
                b_plan = false;
            } else {
                b_plan = true;
            }
        }

        m_path_list.emplace_back(path_point, b_plan);
    }

    m_path_iter = m_path_list.begin();
    std::cout << "end load" << std::endl;
}

void Planning::PubZero() {
    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    pub_cmd->publish(cmd);
}

bool Planning::CheckSafety(const std::vector<Eigen::Vector2d> &pc_vec_) {
    auto it_pc = pc_vec_.begin();
    auto it_end_pc = pc_vec_.end();
    int num = 0;

    for (; it_pc != it_end_pc; it_pc++) {
        auto p = *it_pc;
        
        if (p(0) < m_dist_static_x_th + 0.1 && p(1) > -m_dist_static_y_th && p(1) < m_dist_static_y_th) {
            num++;
        }

        if (num > 0) {
            geometry_msgs::msg::Twist cmd;

            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.linear.z = 0;

            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = 0;
            pub_cmd->publish(cmd);

            return false;
        }
    }

    return true;
}

int sign(double num) {
    if (num > 0)
        return 1;
    else if (num < 0)
        return -1;
    else
        return 0;
}

Eigen::Vector2i Planning::PcToIdx2d(const Eigen::Vector2d &p_) {
    double x = p_(0); double x_abs = abs(x);
    double y = p_(1); double y_abs = abs(y);

    Eigen::Vector2i idx;
    double solution_half = m_resolution * 0.5;

    if (x_abs < solution_half) {
        idx(0) = 0;
    } else {
        idx(0) = 1 + (int)((x_abs - solution_half) / m_resolution);
        idx(0) *= sign(x);
    }

    if (y_abs < solution_half) {
        idx(1) = 0;
    } else {
        idx(1) = 1 + (int)((y_abs - solution_half) / m_resolution);
        idx(1) *= sign(y);
    }

    idx(0) += m_half_grid_num;
    idx(1) += m_half_grid_num;

    return idx;
}

Eigen::Vector2d Planning::Idx2dToPc(const Eigen::Vector2i &idx_) {
    Eigen::Vector2i idx = idx_;
    idx(0) -= m_half_grid_num; idx(1) -= m_half_grid_num;

    Eigen::Vector2d p;
    p(0) = abs(idx(0)) * m_resolution * sign(idx(0));
    p(1) = abs(idx(1)) * m_resolution * sign(idx(1));

    return p;
}
