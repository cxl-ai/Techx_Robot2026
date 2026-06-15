#!/usr/bin/env python3
# -*- coding: utf8 -*-

import os
import os.path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('fast_lio')             # fast_lio包路径
    livox_pkg_path = get_package_share_directory('livox_ros_driver2')  # livox驱动包

    default_cfg_dir = os.path.join(pkg_path, 'config')             # 配置目录
    default_rviz_cfg = os.path.expanduser('~/nav_ws/project/rviz/fastlio.rviz')  # RViz配置

    use_sim_time = LaunchConfiguration('use_sim_time')             # 是否使用sim time
    config_path = LaunchConfiguration('config_path')               # 配置目录
    config_file = LaunchConfiguration('config_file')               # 配置文件
    rviz_use = LaunchConfiguration('rviz')                         # RViz开关
    rviz_cfg = LaunchConfiguration('rviz_cfg')                     # RViz配置
    use_ros2bag = LaunchConfiguration('use_ros2bag')               # ros2bag回放开关
    ros2bag_path = LaunchConfiguration('ros2bag_path')             # ros2bag包路径
    ros2bag_rate = LaunchConfiguration('ros2bag_rate')             # ros2bag倍速
    
    declare_use_ros2bag_cmd = DeclareLaunchArgument('use_ros2bag', default_value='false')          # ros2bag回放
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value=use_ros2bag)    # 仿真时间开关
    declare_config_path_cmd = DeclareLaunchArgument('config_path', default_value=default_cfg_dir)  # 配置目录
    declare_config_file_cmd = DeclareLaunchArgument('config_file', default_value='odom.yaml')      # 配置文件
    declare_rviz_cmd = DeclareLaunchArgument('rviz', default_value='false')                        # RViz开关
    declare_rviz_config_path_cmd = DeclareLaunchArgument('rviz_cfg', default_value=default_rviz_cfg)  # RViz配置
    declare_ros2bag_path_cmd = DeclareLaunchArgument('ros2bag_path', default_value='/home/lhl/rosbag2_XXX')             # ros2bag包
    declare_ros2bag_rate_cmd = DeclareLaunchArgument('ros2bag_rate', default_value='1.0')          # ros2bag倍速

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )  # Fast-LIO节点

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )  # RViz节点

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(livox_pkg_path, 'launch', 'msg_MID360_launch.py')
        ]),
        condition=UnlessCondition(use_ros2bag)
    )  # Livox驱动（回放时关闭）

    ros2bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            ros2bag_path,
            '--clock',
            '--rate', ros2bag_rate
        ],
        output='screen',
        condition=IfCondition(use_ros2bag)
    )  # ros2bag回放（带--clock）

    ld = LaunchDescription()
    ld.add_action(declare_use_ros2bag_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_ros2bag_path_cmd)
    ld.add_action(declare_ros2bag_rate_cmd)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(livox_launch)
    ld.add_action(ros2bag_play)

    return ld