#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # 1. 声明启动参数
    # ------------------------------------------------------------------ #
    map_path_arg    = DeclareLaunchArgument('map_path',
                                            default_value=os.path.join(os.path.expanduser('~'), 'nav_ws/project/map'))
    pose_topic_arg  = DeclareLaunchArgument('pose_topic',   default_value='/Odometry')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic',  default_value='/cloud_registered')
    map_size_arg    = DeclareLaunchArgument('map_size',     default_value='50.0')
    sample_size_arg = DeclareLaunchArgument('sample_size',  default_value='0.25')

    # ------------------------------------------------------------------ #
    # 2. 建图节点
    #    负责接收位姿和点云数据，生成并保存地图
    #    保存触发：
    #      - Ctrl+C 退出时自动保存一次
    #      - 或调用 service：ros2 service call /map_save std_srvs/srv/Trigger {}
    # ------------------------------------------------------------------ #
    mapping_node = Node(
        package='relocalization',
        executable='mapping',
        name='mapping',                    # 节点名称
        output='screen',                   # 日志输出到终端
        parameters=[{
            'map_path':    LaunchConfiguration('map_path'),     # 地图保存路径
            'pose_topic':  LaunchConfiguration('pose_topic'),   # 位姿话题（来自 FAST_LIO）
            'cloud_topic': LaunchConfiguration('cloud_topic'),  # 点云话题（来自 FAST_LIO）
            'map_size':    LaunchConfiguration('map_size'),     # 地图块大小（m）
            'sample_size': LaunchConfiguration('sample_size'),  # 点云采样分辨率（m）
            'debug.save_tile_stats': False,
        }]
    )

    # ------------------------------------------------------------------ #
    # 3. 返回启动描述
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        map_path_arg,
        pose_topic_arg,
        cloud_topic_arg,
        map_size_arg,
        sample_size_arg,
        mapping_node,
    ])
