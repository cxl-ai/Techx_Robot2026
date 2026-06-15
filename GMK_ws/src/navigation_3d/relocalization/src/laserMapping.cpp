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
#include <iostream>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <common_lib.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, time_sync_en = false, extrinsic_est_en = true, path_en = true, map_pub_en = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = "";
string lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;
std::shared_ptr<KD_TREE<PointType>> mp_kd_tree;
std::shared_ptr<KD_TREE<PointType>> mp_kd_tree_new;
int cur_x, cur_y;
std::string map_path;
double map_size;
bool bloaded = false;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
Eigen::Matrix3d mRwg;

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// ROS2 Node pointer
rclcpp::Node::SharedPtr node;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// void SigHandle(int sig)
// {
//     flg_exit = true;
//     RCLCPP_WARN(node->get_logger(), "catch sig %d", sig);
//     sig_buffer.notify_all();
// }

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(mRwg.transpose() * state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + mRwg.transpose() * state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(mRwg.transpose() * state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + mRwg.transpose() * state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
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

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    double msg_time = get_time_sec(msg->header.stamp);
    if (msg_time < last_timestamp_lidar)
    {
        RCLCPP_ERROR(node->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg_time);
    last_timestamp_lidar = msg_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    double msg_time = get_time_sec(msg->header.stamp);
    if (msg_time < last_timestamp_lidar)
    {
        RCLCPP_ERROR(node->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) 
{
    std::cout.precision(17);
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = get_ros_time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(node->get_logger(), "imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            RCLCPP_WARN(node->get_logger(), "Too few input point cloud!");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */

        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "world";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar_base";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "world";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    PointCloudXYZI::Ptr map_cloud(new PointCloudXYZI());
    if (mp_kd_tree && !mp_kd_tree->PCL_Storage.empty()) {
        map_cloud->points = mp_kd_tree->PCL_Storage;
        map_cloud->width = map_cloud->points.size();
        map_cloud->height = 1;
    }
    pcl::toROSMsg(*map_cloud, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = "world";
    pubLaserCloudMap->publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{    
    Eigen::Quaterniond q;
    q.x() = geoQuat.x;
    q.y() = geoQuat.y;
    q.z() = geoQuat.z;
    q.w() = geoQuat.w;

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t(state_point.pos(0), state_point.pos(1), state_point.pos(2));
    Eigen::Matrix3d R_new = mRwg.transpose() * R;
    t = mRwg.transpose()  * t;

    Eigen::Quaterniond q_new(R_new);
    out.pose.position.x = t(0);
    out.pose.position.y = t(1);
    out.pose.position.z = t(2);
    
    out.pose.orientation.x = q_new.x();
    out.pose.orientation.y = q_new.y();
    out.pose.orientation.z = q_new.z();
    out.pose.orientation.w = q_new.w();
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "lidar_base";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    
    odomAftMapped.twist.twist.linear.x = state_point.vel(0);
    odomAftMapped.twist.twist.linear.y = state_point.vel(1);
    odomAftMapped.twist.twist.linear.z = state_point.vel(2);
    pubOdomAftMapped->publish(odomAftMapped);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odomAftMapped.header.stamp;
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "lidar_base";
    transform_stamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform_stamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform_stamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform_stamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    tf_broadcaster->sendTransform(transform_stamped);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "world";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    double error = 0;
    int errorNum = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;
        point_selected_surf[i] = false;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];
        Eigen::Vector3d n;
        double b;
        
        if (ekfom_data.converge || true)
        {
            /** Find the closest surfaces in the map **/
            mp_kd_tree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s_val > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
                error += abs(pd2);
                errorNum ++;
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        RCLCPP_WARN(node->get_logger(), "No Effective Points!");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
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

bool b_start = false;
bool b_end_thread = false;
bool b_end_finish = false;

void LoadNewMp() {
    while (!b_end_thread) {
        if (b_start) {
            b_start = false;
            std::string file_name = map_path + "/" + std::to_string(cur_x) + "_" + std::to_string(cur_y) + ".pcd";

            pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
            if (pcl::io::loadPCDFile(file_name, cloud_xyzi) == 0) {
                std::cout << file_name << std::endl;

                mp_kd_tree_new = std::make_shared<KD_TREE<PointType>>();
                mp_kd_tree_new->set_downsample_param(filter_size_map_min);
                mp_kd_tree_new->PCL_Storage.reserve(cloud_xyzi.points.size());

                for (const auto &q : cloud_xyzi.points) {
                    PointType p_;
                    p_.x = q.x; p_.y = q.y; p_.z = q.z;
                    p_.intensity = q.intensity;
                    p_.normal_x = 0.0f; p_.normal_y = 0.0f; p_.normal_z = 0.0f;
                    p_.curvature = 0.0f;
                    mp_kd_tree_new->PCL_Storage.push_back(p_);
                }

                std::cout << mp_kd_tree_new->PCL_Storage.size() << std::endl;
                mp_kd_tree_new->Build(mp_kd_tree_new->PCL_Storage);
                std::cout << "mp loaded" << std::endl;
                bloaded = true;
            } else {
                std::cout << "failed to load " << file_name << std::endl;
            }
        } else {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    b_end_finish = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("localization");

    node->declare_parameter<std::string>("root_dir", "");
    root_dir = node->get_parameter("root_dir").as_string();
    if (root_dir.empty()) {
        try {
            root_dir = ament_index_cpp::get_package_share_directory("relocalization");
        } catch (...) {
            root_dir = ".";
        }
    }
    
    // Initialize TF broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

    // Declare and get parameters
    node->declare_parameter<bool>("publish.path_en", true);
    node->declare_parameter<bool>("publish.scan_publish_en", true);
    node->declare_parameter<bool>("publish.dense_publish_en", true);
    node->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    node->declare_parameter<bool>("publish.map_en", false);
    node->declare_parameter<int>("max_iteration", 4);
    node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    node->declare_parameter<std::string>("map_path", "");
    node->declare_parameter<bool>("common.time_sync_en", false);
    node->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    node->declare_parameter<double>("filter_size_corner", 0.5);
    node->declare_parameter<double>("filter_size_surf", 0.5);
    node->declare_parameter<double>("filter_size_map", 0.5);
    node->declare_parameter<double>("cube_side_length", 200.0);
    node->declare_parameter<double>("map_size", 50.0);
    node->declare_parameter<float>("mapping.det_range", 300.0f);
    node->declare_parameter<double>("mapping.fov_degree", 180.0);
    node->declare_parameter<double>("mapping.gyr_cov", 0.1);
    node->declare_parameter<double>("mapping.acc_cov", 0.1);
    node->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    node->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    node->declare_parameter<double>("preprocess.blind", 0.01);
    node->declare_parameter<int>("preprocess.lidar_type", AVIA);
    node->declare_parameter<int>("preprocess.scan_line", 16);
    node->declare_parameter<int>("preprocess.timestamp_unit", US);
    node->declare_parameter<int>("preprocess.scan_rate", 10);
    node->declare_parameter<int>("point_filter_num", 2);
    node->declare_parameter<bool>("feature_extract_enable", false);
    node->declare_parameter<bool>("runtime_pos_log_enable", true);
    node->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    // NOTE: `pcd_save.*` was a leftover from FAST_LIO-style map saving.
    // This relocalization `localization` node does not save PCD files here.
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());

    path_en = node->get_parameter("publish.path_en").as_bool();
    scan_pub_en = node->get_parameter("publish.scan_publish_en").as_bool();
    dense_pub_en = node->get_parameter("publish.dense_publish_en").as_bool();
    scan_body_pub_en = node->get_parameter("publish.scan_bodyframe_pub_en").as_bool();
    map_pub_en = node->get_parameter("publish.map_en").as_bool();
    NUM_MAX_ITERATIONS = node->get_parameter("max_iteration").as_int();
    lid_topic = node->get_parameter("common.lid_topic").as_string();
    imu_topic = node->get_parameter("common.imu_topic").as_string();
    map_path = node->get_parameter("map_path").as_string();
    time_sync_en = node->get_parameter("common.time_sync_en").as_bool();
    time_diff_lidar_to_imu = node->get_parameter("common.time_offset_lidar_to_imu").as_double();
    filter_size_corner_min = node->get_parameter("filter_size_corner").as_double();
    filter_size_surf_min = node->get_parameter("filter_size_surf").as_double();
    filter_size_map_min = node->get_parameter("filter_size_map").as_double();
    cube_len = node->get_parameter("cube_side_length").as_double();
    map_size = node->get_parameter("map_size").as_double();
    DET_RANGE = node->get_parameter("mapping.det_range").as_double();
    fov_deg = node->get_parameter("mapping.fov_degree").as_double();
    gyr_cov = node->get_parameter("mapping.gyr_cov").as_double();
    acc_cov = node->get_parameter("mapping.acc_cov").as_double();
    b_gyr_cov = node->get_parameter("mapping.b_gyr_cov").as_double();
    b_acc_cov = node->get_parameter("mapping.b_acc_cov").as_double();
    p_pre->blind = node->get_parameter("preprocess.blind").as_double();
    p_pre->lidar_type = node->get_parameter("preprocess.lidar_type").as_int();
    p_pre->N_SCANS = node->get_parameter("preprocess.scan_line").as_int();
    p_pre->time_unit = node->get_parameter("preprocess.timestamp_unit").as_int();
    p_pre->SCAN_RATE = node->get_parameter("preprocess.scan_rate").as_int();
    p_pre->point_filter_num = node->get_parameter("point_filter_num").as_int();
    p_pre->feature_enabled = node->get_parameter("feature_extract_enable").as_bool();
    runtime_pos_log = node->get_parameter("runtime_pos_log_enable").as_bool();
    extrinsic_est_en = node->get_parameter("mapping.extrinsic_est_en").as_bool();
    // `pcd_save.*` removed (was unused/misleading)
    extrinT = node->get_parameter("mapping.extrinsic_T").as_double_array();
    extrinR = node->get_parameter("mapping.extrinsic_R").as_double_array();
    
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    cur_x = 0; cur_y = 0;
    std::string file_name = map_path + "/" + std::to_string(cur_x) + "_" + std::to_string(cur_y) + ".pcd";
    mp_kd_tree = std::make_shared<KD_TREE<PointType>>();
    mp_kd_tree->set_downsample_param(filter_size_map_min);
    mp_kd_tree->PCL_Storage.reserve(1000000);
    
    double t_load1 = omp_get_wtime();
    pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
    if (pcl::io::loadPCDFile(file_name, cloud_xyzi) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load submap: %s", file_name.c_str());
        return 1;
    }

    mp_kd_tree->PCL_Storage.reserve(cloud_xyzi.points.size());
    for (const auto &q : cloud_xyzi.points) {
        PointType p_;
        p_.x = q.x; p_.y = q.y; p_.z = q.z;
        p_.intensity = q.intensity;
        p_.normal_x = 0.0f; p_.normal_y = 0.0f; p_.normal_z = 0.0f;
        p_.curvature = 0.0f;
        mp_kd_tree->PCL_Storage.push_back(p_);
    }
    
    std::thread load_mp_thread(&LoadNewMp);
    std::cout<<mp_kd_tree->PCL_Storage.size()<<std::endl;
    mp_kd_tree->Build(mp_kd_tree->PCL_Storage);
    std::cout<<"mp loaded"<<std::endl;
    double t_load2 = omp_get_wtime();
    std::cout<<"load time"<<t_load2 - t_load1<<std::endl;

    // 初始化 mRwg 为单位矩阵（定位模式）
    mRwg = Eigen::Matrix3d::Identity();
    
    Eigen::Matrix3d init_R; Eigen::Vector3d init_t;
    file_name = map_path + "/" + "pose.txt";
    ifstream ifs1(file_name, ios::in);
    for (int i = 0; i < 4; i++) {
        double p[3];
        ifs1>>p[0]>>p[1]>>p[2];

        if (i < 3) {
            init_R(i, 0) = p[0];
            init_R(i, 1) = p[1];
            init_R(i, 2) = p[2];
        } else {
            init_t(0) = p[0];
            init_t(1) = p[1];
            init_t(2) = p[2];
        }
    }
    ifs1.close();

    path.header.stamp = node->now();
    path.header.frame_id ="world";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->m_init_R = init_R;
    p_imu->m_init_t = init_t;

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    state_ikfom st = kf.get_x();

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open((root_dir + "/Log/mat_pre.txt"),ios::out);
    fout_out.open((root_dir + "/Log/mat_out.txt"),ios::out);
    fout_dbg.open((root_dir + "/Log/dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~"<<root_dir<<" file opened" << endl;
    else
        cout << "~"<<root_dir<<" doesn't exist" << endl;

    /*** ROS2 subscribe initialization ***/
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_standard;
    
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::QoS(200000), livox_pcl_cbk);
    } else {
        sub_pcl_standard = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::QoS(200000), standard_pcl_cbk);
    }
    
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(200000), imu_cbk);
    
    auto pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 100000);
    auto pubLaserCloudFull_body = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", 100000);
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_effected", 100000);
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/Laser_map_local", 100000);
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>(
        "/Odometry", 100000);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>(
        "/path", 100000);

    // signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    
    while (rclcpp::ok())
    {
        if (flg_exit) break;
        rclcpp::spin_some(node);
        if(sync_packages(Measures)) 
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            Eigen::Vector3d accAvg = kf.get_x().grav.get_vect();
            Eigen::Vector3d dirG = accAvg / accAvg.norm();

            Eigen::Vector3d gI(0.0, 0.0, -1.0);
            Eigen::Vector3d v = gI.cross(dirG);
            double nv = v.norm();
            double cosg = gI.dot(dirG);
            double ang = acos(cosg);
            Eigen::Vector3d vzg = v * ang / nv;
            mRwg = ExpSO3(vzg);

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            /*** downsample the feature points in a scan ***/

            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true;
             
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();

            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            int x_, y_;
            CalculateIdx(x_, y_, pos_lid(0), pos_lid(1));

            if(x_ != cur_x || y_ != cur_y) {
                cur_x = x_;
                cur_y = y_;
                b_start = true;
                std::cout<<"need change new mp"<<std::endl;
            }
            
            if (bloaded) {
                mp_kd_tree = mp_kd_tree_new;
                std::cout<<"change map"<<std::endl;
                bloaded = false;
            }

            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];
            publish_odometry(pubOdomAftMapped);

            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en)                    publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            
            // 发布地图点云（每50帧发布一次以减少带宽）
            if (map_pub_en) {
                static int map_pub_cnt = 0;
                if (++map_pub_cnt % 50 == 0) publish_map(pubLaserCloudMap);
            }
        }

        rate.sleep();
    }

    /**************** save map ****************/
    b_end_thread = true;
    while (!b_end_finish) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    load_mp_thread.join();
    
    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    rclcpp::shutdown();
    return 0;
}
