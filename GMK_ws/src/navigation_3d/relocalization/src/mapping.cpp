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
#include <algorithm>
#include <limits>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ikd-Tree/ikd_Tree.h>
#include <common_lib.h>
#include "preprocess.h"

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// ROS2 Node
rclcpp::Node::SharedPtr node;

// Sync Policy - use ConstSharedPtr for ROS2 message_filters
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;

// map
int cur_x = 1000, cur_y = 1000;
std::string map_path;
double map_size = 50;
double sample_size = 0.1;
std::map<int, std::map<int, std::shared_ptr<KD_TREE<PointType>>>> sub_map_db;

// path
std::vector<Eigen::Vector3d> path_vec;

PointCloudXYZI::Ptr latest_cloud(new PointCloudXYZI());

bool bwrite = false;
bool save_tile_stats = false;
std::mutex save_mtx;

Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d &R_) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R_) {
    double d = 0.5*(R_(0,0)+R_(1,1)+R_(2,2)-1);
    Eigen::Vector3d omega;
    Eigen::Vector3d dR = Eigen::Vector3d(R_(2,1)-R_(1,2), R_(0,2)-R_(2,0), R_(1,0)-R_(0,1));
    if(d>0.9999) {
        omega = 0.5*dR;
    } else {
        double theta = acos(d);
        omega = theta*dR/(2*sqrt(1-d*d));
    }
    return omega;
}

void CalculateIdx(int &x_, int &y_, const double &pose_x_, const double &pose_y_) {
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
            y_ = - y_;
        }
    }
}

// ROS2 message_filters callback uses ConstSharedPtr
void DepthOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_, 
                       const nav_msgs::msg::Odometry::ConstSharedPtr &odom_) {
    double pose_x = odom_->pose.pose.position.x;
    double pose_y = odom_->pose.pose.position.y;
    double pose_z = odom_->pose.pose.position.z;
    (void)pose_z; // unused

    Eigen::Quaterniond q;
    q.x() = odom_->pose.pose.orientation.x;
    q.y() = odom_->pose.pose.orientation.y;
    q.z() = odom_->pose.pose.orientation.z;
    q.w() = odom_->pose.pose.orientation.w;

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d angle = LogSO3(R);
    path_vec.push_back(Eigen::Vector3d(pose_x, pose_y, angle(2)));

    pcl::fromROSMsg(*pc_, *latest_cloud);

    int pose_index_x, pose_index_y;
    CalculateIdx(pose_index_x, pose_index_y, pose_x, pose_y);

    if (pose_index_x != cur_x || pose_index_y != cur_y) {
        // create 3x3 submap
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                int cur_index_x = pose_index_x + i;
                int cur_index_y = pose_index_y + j;

                if (sub_map_db.find(cur_index_x) == sub_map_db.end() ||
                    sub_map_db[cur_index_x].find(cur_index_y) == sub_map_db[cur_index_x].end()) {
                    auto cur_sub_map = std::make_shared<KD_TREE<PointType>>();
                    cur_sub_map->set_downsample_param(sample_size);
                    sub_map_db[cur_index_x][cur_index_y] = cur_sub_map;
                }
            }
        }

        cur_x = pose_index_x;
        cur_y = pose_index_y;
    }

    // add points to 3x3 submap
    std::map<int, std::map<int, PointVector>> need_to_add;
    for (size_t i = 0; i < latest_cloud->points.size(); i++) {
        PointType &p = latest_cloud->points[i];
        int p_index_x, p_index_y;
        CalculateIdx(p_index_x, p_index_y, p.x, p.y);

        // add to 3x3 neighbor
        for (int ii = -1; ii < 2; ii++) {
            for (int jj = -1; jj < 2; jj++) {
                need_to_add[p_index_x + ii][p_index_y + jj].push_back(p);
            }
        }
    }

    for (auto &x_map : need_to_add) {
        int idx_x = x_map.first;
        for (auto &y_map : x_map.second) {
            int idx_y = y_map.first;
            PointVector &points = y_map.second;

            if (sub_map_db.find(idx_x) != sub_map_db.end() &&
                sub_map_db[idx_x].find(idx_y) != sub_map_db[idx_x].end()) {
                auto &kd_tree = sub_map_db[idx_x][idx_y];
                
                if (kd_tree->Root_Node == nullptr) {
                    kd_tree->Build(points);
                } else {
                    kd_tree->Add_Points(points, true);
                }
            }
        }
    }
}

static bool SaveMapOnce(std::string *message_out = nullptr)
{
    std::lock_guard<std::mutex> lk(save_mtx);

    if (!node) {
        if (message_out) *message_out = "node is null";
        return false;
    }

    if (bwrite) {
        if (message_out) *message_out = "already saved";
        return true;
    }

    if (map_path.empty()) {
        if (message_out) *message_out = "map_path is empty";
        RCLCPP_ERROR(node->get_logger(), "map_path is empty, cannot save.");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Saving map to %s ...", map_path.c_str());

    // save all submaps
    for (auto &x_map : sub_map_db) {
        int idx_x = x_map.first;
        for (auto &y_map : x_map.second) {
            int idx_y = y_map.first;
            auto &kd_tree = y_map.second;

            std::string file_name = map_path + "/" + std::to_string(idx_x) + "_" + std::to_string(idx_y) + ".pcd";

            // - do NOT reuse kd_tree->PCL_Storage as an output buffer (it can be mutated by KD-tree internals)
            // - use DELETE_POINTS_REC to keep behavior consistent
            PointVector storage;
            storage.reserve(std::max(0, kd_tree->validnum()));
            kd_tree->flatten(kd_tree->Root_Node, storage, DELETE_POINTS_REC);

            pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
            cloud_xyzi.points.reserve(storage.size());

            float min_x = std::numeric_limits<float>::infinity();
            float min_y = std::numeric_limits<float>::infinity();
            float min_z = std::numeric_limits<float>::infinity();
            float max_x = -std::numeric_limits<float>::infinity();
            float max_y = -std::numeric_limits<float>::infinity();
            float max_z = -std::numeric_limits<float>::infinity();

            for (const auto &p : storage) {
                pcl::PointXYZI q;
                q.x = p.x; q.y = p.y; q.z = p.z;
                q.intensity = p.intensity;
                cloud_xyzi.points.push_back(q);

                min_x = std::min(min_x, q.x); min_y = std::min(min_y, q.y); min_z = std::min(min_z, q.z);
                max_x = std::max(max_x, q.x); max_y = std::max(max_y, q.y); max_z = std::max(max_z, q.z);
            }
            cloud_xyzi.width = static_cast<uint32_t>(cloud_xyzi.points.size());
            cloud_xyzi.height = 1;
            cloud_xyzi.is_dense = false;

            if (save_tile_stats) {
                RCLCPP_INFO(node->get_logger(),
                            "Tile[%d,%d] points=%zu bbox=[%.2f %.2f %.2f]->[%.2f %.2f %.2f] file=%s",
                            idx_x, idx_y, cloud_xyzi.points.size(),
                            min_x, min_y, min_z, max_x, max_y, max_z,
                            file_name.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "Saving %s with %zu points",
                           file_name.c_str(), cloud_xyzi.points.size());
            }

            int ret = pcl::io::savePCDFileBinaryCompressed(file_name, cloud_xyzi);
            if (ret != 0) {
                RCLCPP_ERROR(node->get_logger(), "Failed to save %s (ret=%d)", file_name.c_str(), ret);
            }
        }
    }

    // save path
    {
        std::string file_name = map_path + "/path.txt";
        ofstream ofs(file_name, ios::out);
        for (auto &pose : path_vec) {
            ofs << pose(0) << " " << pose(1) << " " << pose(2) << "\n";
        }
        ofs.close();
    }

    RCLCPP_INFO(node->get_logger(), "Map saved successfully! Total poses: %zu", path_vec.size());
    bwrite = true;
    if (message_out) *message_out = "saved";
    return true;
}

static void MapSaveServiceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::string msg;
    bool ok = SaveMapOnce(&msg);
    res->success = ok;
    res->message = msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("mapping");

    // Declare and get parameters
    node->declare_parameter<std::string>("map_path", "");
    node->declare_parameter<std::string>("pose_topic", "/Odometry");
    node->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    node->declare_parameter<double>("map_size", 50.0);
    node->declare_parameter<double>("sample_size", 0.1);
    node->declare_parameter<bool>("debug.save_tile_stats", false);

    map_path = node->get_parameter("map_path").as_string();
    std::string pose_topic = node->get_parameter("pose_topic").as_string();
    std::string cloud_topic = node->get_parameter("cloud_topic").as_string();
    map_size = node->get_parameter("map_size").as_double();
    sample_size = node->get_parameter("sample_size").as_double();
    save_tile_stats = node->get_parameter("debug.save_tile_stats").as_bool();

    RCLCPP_INFO(node->get_logger(), "Mapping node started");
    RCLCPP_INFO(node->get_logger(), "Map path: %s", map_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Cloud topic: %s", cloud_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Pose topic: %s", pose_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Map size: %.1f m, Sample size: %.2f m", map_size, sample_size);

    // Message filter subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud(node.get(), cloud_topic);
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom(node.get(), pose_topic);

    Sync sync(SyncPolicy(10), sub_cloud, sub_odom);
    sync.registerCallback(&DepthOdomCallback);

    // Save service: ros2 service call /map_save std_srvs/srv/Trigger {}
    auto map_save_srv = node->create_service<std_srvs::srv::Trigger>(
        "map_save", &MapSaveServiceCallback);

    // Ctrl+C / shutdown hook
    rclcpp::on_shutdown([]() {
        (void)SaveMapOnce(nullptr);
    });

    RCLCPP_INFO(node->get_logger(), "Waiting for data... Ctrl+C will save once on exit.");
    RCLCPP_INFO(node->get_logger(), "You can also save by calling: ros2 service call /map_save std_srvs/srv/Trigger {}");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
