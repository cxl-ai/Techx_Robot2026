// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ikd-Tree/ikd_Tree.h>
#include <common_lib.h>
#include "preprocess.h"

#define NUM_MATCH_POINTS    (5)

using namespace std;

std::shared_ptr<KD_TREE<PointType>> mp_kd_tree;

// ROS2 Node
rclcpp::Node::SharedPtr node;

// map
int cur_x, cur_y;
std::string map_path;
double map_size;
double init_x = 0., init_y = 0., init_z = 0.;
double angle_size = 0., max_angle = 0., init_angle = 0.;

string lid_topic, imu_topic;

// lidar
bool bImuInit = false;
bool bRelocInitDone = false;
std::atomic<bool> got_lidar(false);
std::atomic<bool> got_imu(false);
std::atomic<bool> warned_wait(false);

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;

Eigen::Vector3d acc_sum(0, 0, 0);
int acc_num = 0;

Eigen::Matrix3d mRwg;
Eigen::Matrix3d init_R;
Eigen::Vector3d init_t;

shared_ptr<Preprocess> p_pre(new Preprocess());

// Keep the same tiling rule as relocalization/src/mapping.cpp
static void CalculateIdx(int &x_, int &y_, const double &pose_x_, const double &pose_y_) {
    double half_map_size = map_size * 0.5;
    x_ = static_cast<int>((abs(pose_x_) - half_map_size) / map_size) + 1;
    y_ = static_cast<int>((abs(pose_y_) - half_map_size) / map_size) + 1;

    if (abs(pose_x_) < half_map_size) {
        x_ = 0;
    } else {
        if (pose_x_ < 0) {
            x_ = -x_;
        }
    }

    if (abs(pose_y_) < half_map_size) {
        y_ = 0;
    } else {
        if (pose_y_ < 0) {
            y_ = -y_;
        }
    }
}

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R_) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3d ExpSO3(const double &x_, const double &y_, const double &z_) {
    const double d2 = x_ * x_ + y_ * y_ + z_ * z_;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z_, y_, z_, 0.0, -x_, -y_, x_, 0.0;

    if(d < 1e-5) {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
        return NormalizeRotation(res);
    } else {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
        return NormalizeRotation(res);
    }
}

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w_) {
    return ExpSO3(w_[0], w_[1], w_[2]);
}

void Registration(const Eigen::Matrix3d &delta_R_, double &error_num_, double &error_, Eigen::Matrix3d &R_, Eigen::Vector3d &t_) {
    int feats_down_size = feats_down_body->points.size();
    std::vector<bool> point_selected_surf(feats_down_size, false);
    std::vector<float> res_last(feats_down_size, 0.0);
    PointCloudXYZI::Ptr normvec(new PointCloudXYZI(feats_down_size, 1));
    PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(feats_down_size, 1));
    PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(feats_down_size, 1));
    std::vector<PointVector> Nearest_Points(feats_down_size);
    
    Eigen::Matrix3d Rwb = mRwg.transpose() * delta_R_ * init_R;
    Eigen::Vector3d twb = init_t;
    
    Eigen::Matrix3d Rbl = Eigen::Matrix3d::Identity();
    Eigen::Vector3d tbl(-0.011, -0.02329, 0.04412);

    for (int k = 0; k < 100; k++) {
        error_ = 0;
        error_num_ = 0;

        // 第一步：搜索最近邻点并计算平面参数
        for (int i = 0; i < feats_down_size; i++) {
            PointType &point_body = feats_down_body->points[i];
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(Rwb * (Rbl * p_body + tbl) + twb);
            
            PointType point_world;
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_selected_surf[i] = false;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            auto &points_near = Nearest_Points[i];

            mp_kd_tree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            
            if (points_near.size() < NUM_MATCH_POINTS || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) {
                continue;
            }

            // 计算协方差矩阵
            Eigen::Vector3d centerVec = Eigen::Vector3d::Zero();
            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                centerVec(0) += points_near[j].x;
                centerVec(1) += points_near[j].y;
                centerVec(2) += points_near[j].z;
            }
            centerVec = centerVec / static_cast<double>(NUM_MATCH_POINTS);

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                Eigen::Vector3d p;
                p(0) = points_near[j].x - centerVec(0);
                p(1) = points_near[j].y - centerVec(1);
                p(2) = points_near[j].z - centerVec(2);
                covMat += p * p.transpose();
            }
            covMat = covMat / static_cast<double>(NUM_MATCH_POINTS);

            // SVD 分解得到平面法向量
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d matrixV = svd.matrixV();
            Eigen::Vector3d n = matrixV.block<3, 1>(0, 2);

            centerVec(0) = points_near[0].x;
            centerVec(1) = points_near[0].y;
            centerVec(2) = points_near[0].z;
            double b = -n.dot(centerVec);

            // 验证平面质量
            VF(4) pabcd;
            if (!esti_plane(pabcd, points_near, 0.1f)) {
                continue;
            }

            pabcd(0) = n(0);
            pabcd(1) = n(1);
            pabcd(2) = n(2);
            pabcd(3) = b;
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            
            // 评分机制：只接受高质量的点
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
                error_ += abs(pd2);
                error_num_++;
            }
        }

        // 第二步：收集有效特征点
        int effct_feat_num = 0;
        for (int i = 0; i < feats_down_size; i++) {
            if (point_selected_surf[i]) {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                effct_feat_num++;
            }
        }

        if (effct_feat_num < 50) {
            std::cout << "error num is:" << effct_feat_num << std::endl;
            break;
        }

        // 第三步：构建优化问题
        Eigen::MatrixXd Jac;
        Eigen::MatrixXd Er;
        Jac.resize(effct_feat_num, 6);
        Er.resize(effct_feat_num, 1);
        Jac.setZero();
        Er.setZero();

        for (int i = 0; i < effct_feat_num; i++) {
            const PointType &laser_p = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            V3D point_this = Rbl * point_this_be + tbl;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            V3D C(Rwb.transpose() * norm_vec);
            V3D A(point_crossmat * C);

            Jac.block<1, 6>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A);
            Er(i) = -norm_p.intensity;
        }

        Eigen::Matrix<double, 6, 6> Hess = Jac.transpose() * Jac;
        Eigen::Matrix<double, 6, 1> Res = Jac.transpose() * Er;
        Eigen::Matrix<double, 6, 1> x = Hess.ldlt().solve(Res);

        twb += x.block<3, 1>(0, 0);
        Rwb = Rwb * ExpSO3(x(3), x(4), x(5));
        Rwb = NormalizeRotation(Rwb);

        double delta_t_norm = x(0, 0) * x(0, 0) + x(1, 0) * x(1, 0) + x(2, 0) * x(2, 0);
        if (delta_t_norm < 1e-5) {
            break;
        }
    }

    R_ = Rwb;
    t_ = twb;
}

void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in) {    
    got_imu.store(true, std::memory_order_relaxed);

    // Step 1) estimate gravity direction from IMU average
    if (!bImuInit) {
        acc_sum(0) += msg_in->linear_acceleration.x;
        acc_sum(1) += msg_in->linear_acceleration.y;
        acc_sum(2) += msg_in->linear_acceleration.z;
        acc_num++;

        if (acc_num >= 200) {
            Eigen::Vector3d acc_avg = acc_sum / acc_num;
            double acc_norm = acc_avg.norm();
            if (acc_norm < 1e-6) {
                RCLCPP_WARN(node->get_logger(), "IMU acc norm too small (%.3e), keep waiting...", acc_norm);
                acc_sum.setZero();
                acc_num = 0;
                return;
            }
            Eigen::Vector3d dirG = -acc_avg / acc_norm;

            Eigen::Vector3d gI(0.0, 0.0, -1.0);
            Eigen::Vector3d v = gI.cross(dirG);
            double nv = v.norm();
            double cosg = gI.dot(dirG);
            if (cosg > 1.0) cosg = 1.0;
            if (cosg < -1.0) cosg = -1.0;
            double ang = acos(cosg);
            if (nv < 1e-9) {
                // nearly aligned: no rotation needed
                mRwg.setIdentity();
            } else {
                Eigen::Vector3d vzg = v * ang / nv;
                mRwg = ExpSO3(vzg);
            }

            bImuInit = true;
            std::cout << mRwg << std::endl;
        }
    }

    // Step 2) run alignment once when both IMU init and LiDAR data are ready
    if (bImuInit && !bRelocInitDone) {
        const bool lidar_ready = got_lidar.load(std::memory_order_relaxed) && feats_down_body && feats_down_body->points.size() >= 200;
        if (!lidar_ready) {
            if (!warned_wait.exchange(true, std::memory_order_relaxed)) {
                RCLCPP_WARN(node->get_logger(),
                            "Waiting for LiDAR pointcloud before initialization (need >=200 points). "
                            "If you just started the sensor, this is normal.");
            }
            return;
        }

            double min_error = 1000000;
            Eigen::Matrix3d opt_R = init_R;
            Eigen::Vector3d opt_t = init_t;

            for (double angle = init_angle - max_angle; angle <= init_angle + max_angle; angle += angle_size) {
                Eigen::Matrix3d delta_R;
                delta_R.setIdentity();
                delta_R(0, 0) = cos(angle);
                delta_R(0, 1) = -sin(angle);
                delta_R(1, 0) = sin(angle);
                delta_R(1, 1) = cos(angle);

                Eigen::Matrix3d cur_R;
                Eigen::Vector3d cur_t;
                double error = 0;
                double error_num = 0;
                Registration(delta_R, error_num, error, cur_R, cur_t);
                std::cout << error << "  " << error_num << std::endl;
                
                if (error_num < 200) {
                    continue;
                }
                
                error = error / error_num;
                std::cout << angle << "  " << error << std::endl;
                std::cout << cur_t.transpose() << std::endl;
                
                if (error < min_error) {
                    min_error = error;
                    opt_R = cur_R;
                    opt_t = cur_t;
                }
            }

            std::string file_name = map_path + "/pose.txt";
            ofstream ofs(file_name, ios::out);
            ofs << opt_R(0, 0) << " " << opt_R(0, 1) << " " << opt_R(0, 2) << endl;
            ofs << opt_R(1, 0) << " " << opt_R(1, 1) << " " << opt_R(1, 2) << endl;
            ofs << opt_R(2, 0) << " " << opt_R(2, 1) << " " << opt_R(2, 2) << endl;
            ofs << opt_t(0) << " " << opt_t(1) << " " << opt_t(2) << endl;
            ofs.close();
            
            RCLCPP_INFO(node->get_logger(), "Init done! Pose saved to %s", file_name.c_str());
            RCLCPP_INFO(node->get_logger(), "Position: [%.4f, %.4f, %.4f]", opt_t(0), opt_t(1), opt_t(2));
            bRelocInitDone = true;
        }
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 移除 bImuInit 检查，保持接收点云数据
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    
    downSizeFilterSurf.setInputCloud(ptr);
    downSizeFilterSurf.filter(*feats_down_body);
    got_lidar.store(true, std::memory_order_relaxed);
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    // 移除 bImuInit 检查，保持接收点云数据
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    downSizeFilterSurf.setInputCloud(ptr);
    downSizeFilterSurf.filter(*feats_down_body);
    got_lidar.store(true, std::memory_order_relaxed);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("init");

    // Declare and get parameters
    node->declare_parameter<std::string>("map_path", "");
    node->declare_parameter<double>("init_x", 0.0);
    node->declare_parameter<double>("init_y", 0.0);
    node->declare_parameter<double>("init_z", 0.1);
    node->declare_parameter<double>("angle_size", 0.05);
    node->declare_parameter<double>("max_angle", 0.1);
    node->declare_parameter<double>("init_angle", 0.0);
    node->declare_parameter<std::string>("lid_topic", "/livox/lidar");
    node->declare_parameter<std::string>("imu_topic", "/livox/imu");
    node->declare_parameter<double>("filter_size_surf", 0.2);
    node->declare_parameter<double>("filter_size_map", 0.5);
    node->declare_parameter<double>("map_size", 50.0);
    node->declare_parameter<bool>("load_neighbor_tiles", false);
    node->declare_parameter<int>("neighbor_tile_range", 0);
    node->declare_parameter<int>("preprocess.lidar_type", 1);
    node->declare_parameter<int>("preprocess.scan_line", 6);
    node->declare_parameter<int>("preprocess.timestamp_unit", 2);
    node->declare_parameter<int>("preprocess.scan_rate", 10);
    node->declare_parameter<double>("preprocess.blind", 0.01);
    node->declare_parameter<int>("point_filter_num", 2);
    node->declare_parameter<bool>("feature_extract_enable", false);

    map_path = node->get_parameter("map_path").as_string();
    init_x = node->get_parameter("init_x").as_double();
    init_y = node->get_parameter("init_y").as_double();
    init_z = node->get_parameter("init_z").as_double();
    angle_size = node->get_parameter("angle_size").as_double();
    max_angle = node->get_parameter("max_angle").as_double();
    init_angle = node->get_parameter("init_angle").as_double();
    lid_topic = node->get_parameter("lid_topic").as_string();
    imu_topic = node->get_parameter("imu_topic").as_string();
    double filter_size_surf = node->get_parameter("filter_size_surf").as_double();
    double filter_size_map = node->get_parameter("filter_size_map").as_double();
    map_size = node->get_parameter("map_size").as_double();
    bool load_neighbor_tiles = node->get_parameter("load_neighbor_tiles").as_bool();
    int neighbor_tile_range = node->get_parameter("neighbor_tile_range").as_int();
    if (neighbor_tile_range < 0) neighbor_tile_range = 0;
    p_pre->lidar_type = node->get_parameter("preprocess.lidar_type").as_int();
    p_pre->N_SCANS = node->get_parameter("preprocess.scan_line").as_int();
    p_pre->time_unit = node->get_parameter("preprocess.timestamp_unit").as_int();
    p_pre->SCAN_RATE = node->get_parameter("preprocess.scan_rate").as_int();
    p_pre->blind = node->get_parameter("preprocess.blind").as_double();
    p_pre->point_filter_num = node->get_parameter("point_filter_num").as_int();
    p_pre->feature_enabled = node->get_parameter("feature_extract_enable").as_bool();

    init_R.setIdentity();
    init_R(0, 0) = cos(init_angle);
    init_R(0, 1) = -sin(init_angle);
    init_R(1, 0) = sin(init_angle);
    init_R(1, 1) = cos(init_angle);
    
    init_t.setZero();
    init_t(0) = init_x;
    init_t(1) = init_y;
    init_t(2) = init_z;

    downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);

    // Select which tile to load based on the initial position guess.
    CalculateIdx(cur_x, cur_y, init_x, init_y);
    int cx = cur_x;
    int cy = cur_y;

    int range = load_neighbor_tiles ? neighbor_tile_range : 0;
    pcl::PointCloud<pcl::PointXYZI> cloud_xyzi_merged;
    cloud_xyzi_merged.clear();
    cloud_xyzi_merged.points.reserve(1000000);

    std::vector<std::string> tried_files;
    int loaded_tiles = 0;
    for (int dx = -range; dx <= range; dx++) {
        for (int dy = -range; dy <= range; dy++) {
            int tx = cx + dx;
            int ty = cy + dy;
            std::string file_name = map_path + "/" + std::to_string(tx) + "_" + std::to_string(ty) + ".pcd";
            tried_files.push_back(file_name);
            if (::access(file_name.c_str(), F_OK) != 0) {
                continue;
            }
            pcl::PointCloud<pcl::PointXYZI> tmp;
            if (pcl::io::loadPCDFile(file_name, tmp) != 0) {
                RCLCPP_WARN(node->get_logger(), "Failed to load tile: %s", file_name.c_str());
                continue;
            }
            loaded_tiles++;
            cloud_xyzi_merged += tmp;
        }
    }

    mp_kd_tree = std::make_shared<KD_TREE<PointType>>();
    mp_kd_tree->set_downsample_param(filter_size_map);
    mp_kd_tree->PCL_Storage.reserve(1000000);
    double t_load1 = omp_get_wtime();

    if (loaded_tiles == 0) {
        RCLCPP_ERROR(node->get_logger(),
                     "Cannot load any map tiles for init pose [%.2f, %.2f]. "
                     "Calculated tile=(%d,%d), range=%d, map_path=%s",
                     init_x, init_y, cx, cy, range, map_path.c_str());
        for (const auto &f : tried_files) {
            RCLCPP_ERROR(node->get_logger(), "Tried: %s", f.c_str());
        }
        return -1;
    }

    mp_kd_tree->PCL_Storage.reserve(cloud_xyzi_merged.points.size());
    for (const auto &q : cloud_xyzi_merged.points) {
        PointType p_;
        p_.x = q.x; p_.y = q.y; p_.z = q.z;
        p_.intensity = q.intensity;
        p_.normal_x = 0.0f; p_.normal_y = 0.0f; p_.normal_z = 0.0f;
        p_.curvature = 0.0f;
        mp_kd_tree->PCL_Storage.push_back(p_);
    }
    std::cout<<mp_kd_tree->PCL_Storage.size()<<std::endl;
    mp_kd_tree->Build(mp_kd_tree->PCL_Storage);
    std::cout<<"mp loaded"<<std::endl;
    double t_load2 = omp_get_wtime();
    std::cout<<"load time: "<<t_load2 - t_load1<<std::endl;

    /*** ROS2 subscribe initialization ***/
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard;
    
    // Use sensor-data QoS to avoid typical publisher/subscriber QoS mismatch
    auto qos_sensor = rclcpp::SensorDataQoS();

    if (p_pre->lidar_type == 1) { // AVIA
        sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, qos_sensor, livox_pcl_cbk);
    } else {
        sub_pcl_standard = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, qos_sensor, standard_pcl_cbk);
    }
    
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor, ImuCallBack);

    RCLCPP_INFO(node->get_logger(), "Init node started. Collecting IMU data...");
    RCLCPP_INFO(node->get_logger(), "Initial position guess: [%.2f, %.2f, %.2f]", init_x, init_y, init_z);
    RCLCPP_INFO(node->get_logger(), "Selected tile=(%d,%d), loaded tiles=%d (range=%d), map_size=%.2f",
                cx, cy, loaded_tiles, range, map_size);
    RCLCPP_INFO(node->get_logger(), "Angle search range: [%.3f, %.3f] with step %.3f", 
                init_angle - max_angle, init_angle + max_angle, angle_size);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
