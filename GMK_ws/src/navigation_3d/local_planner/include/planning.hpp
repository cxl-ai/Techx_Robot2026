//
// created by jinhu
// migrated to ROS2 Humble
//
#pragma once

#include <list>
#include <geometry_msgs/msg/twist.hpp>
#include "control.hpp"

struct Grid {
    Eigen::Vector2d m_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2i m_idx = Eigen::Vector2i::Zero();
    bool m_b_good = true;

    double m_dist = 0;
    double m_heu_dist = 0;
    double m_total_dist = 0;  

    std::shared_ptr<Grid> m_parent_grid = nullptr;
};

class NodeComparator {
public:
    bool operator()(std::shared_ptr<Grid> node1, std::shared_ptr<Grid> node2) {
        return node1->m_total_dist > node2->m_total_dist;
    }
};

class Planning {
public:
    rclcpp::Node::SharedPtr m_nh;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;

    std::vector<std::vector<std::shared_ptr<Grid>>> m_grid_vec;
    std::vector<std::pair<int, int>> m_neigh_vec;
    std::vector<std::pair<Eigen::Vector3d, bool>> m_path_vec;

    std::list<std::pair<Eigen::Vector3d, bool>> m_path_list;
    std::list<std::pair<Eigen::Vector3d, bool>> m_local_path_list;
    std::list<std::pair<Eigen::Vector3d, bool>>::iterator m_path_iter;
    std::shared_ptr<Controller> m_controller;

    int m_grid_num = 0;
    int m_half_grid_num = 0;
    int m_global_idx = 0;
    int m_local_idx = 0;
    int m_dyn_check_num = 15;

    bool m_b_plan = false;
    bool m_b_stop = false;
    bool m_b_select_new = false;
    bool m_b_select_local = false;
    bool m_is_get_path = false;
    bool m_b_suspend = false;
    bool m_b_enable_sus = false;

    double m_resolution = 0.1;
    double m_dist_x_th = 0.5;
    double m_dist_y_th = 0.5;
    double m_dist_z_th = 3;

    double m_dist_dyn_th = 0.5;
    double m_dist_static_x_th = 0.3;
    double m_dist_static_y_th = 0.1;

    double m_dist_th = 0.3;
    double m_arrive_th = 0.1;
    double m_time_th = 1;
    double m_last_time = 0;
    double m_offset = 0;

    double m_run_time = 5;
    double m_stop_time = 0.5;
    double m_last_stop_time = -1;
    double m_suspend_time = -1;

    Eigen::Vector3d m_state;
    Eigen::Vector3d m_ref_state;
    Eigen::Vector3d m_pos;
    Eigen::Vector2d m_goal;

    Eigen::Vector3d m_base_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_end = Eigen::Vector3d::Zero();

    Planning(const rclcpp::Node::SharedPtr& nh_, const std::shared_ptr<Controller> controller_, const bool &b_load_,
             const double &offset_ = 0);
    void PubZero();
    void PubRotate(const double w_z);
    
    bool Replan(const Eigen::Vector3d &start_, const Eigen::Vector3d &end_);

    void SetState(const Eigen::Vector3d &state_, const Eigen::Vector3d &pos_);

    void LoadPathFromFile(const std::string &path_);

    void SelectNew();

    bool CheckSafety(const std::vector<Eigen::Vector2d> &pc_vec_);

    bool CheckSelect();

    Eigen::Vector2i PcToIdx2d(const Eigen::Vector2d &p_);

    Eigen::Vector2d Idx2dToPc(const Eigen::Vector2i &idx_);
};
