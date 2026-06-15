#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # 1. 声明启动参数
    # ------------------------------------------------------------------ #
    map_path   = DeclareLaunchArgument('map_path',   default_value=os.path.join(os.path.expanduser('~'), 'nav_ws/project/map'))
    map_size   = DeclareLaunchArgument('map_size',   default_value='50.0')
    init_x     = DeclareLaunchArgument('init_x',     default_value='0.0')
    init_y     = DeclareLaunchArgument('init_y',     default_value='0.0')
    init_z     = DeclareLaunchArgument('init_z',     default_value='0.0')
    angle_size = DeclareLaunchArgument('angle_size', default_value='0.10')  # 单位: 弧度 (rad)
    max_angle  = DeclareLaunchArgument('max_angle',  default_value='3.14')  # 单位: 弧度 (rad)
    init_angle = DeclareLaunchArgument('init_angle', default_value='0.0')  # 单位: 弧度 (rad)
    lid_topic  = DeclareLaunchArgument('lid_topic',  default_value='/livox/lidar')
    imu_topic  = DeclareLaunchArgument('imu_topic',  default_value='/livox/imu')
    launch_livox = DeclareLaunchArgument('launch_livox', default_value='true')

    # ------------------------------------------------------------------ #
    # 2. Livox MID360 driver2
    # ------------------------------------------------------------------ #
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch',
                'msg_MID360_launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_livox'))
    )

    # ------------------------------------------------------------------ #
    # 3. 重定位初始化节点
    #    负责加载地图、接收 LiDAR/IMU 数据并完成初始位姿估计
    # ------------------------------------------------------------------ #
    init_node = Node(
        package='relocalization',
        executable='init',
        name='init',                    # 节点名称
        output='screen',                # 日志输出到终端
        parameters=[{

            'map_path':   LaunchConfiguration('map_path'),
            'map_size':   LaunchConfiguration('map_size'),
            'init_x':     LaunchConfiguration('init_x'),
            'init_y':     LaunchConfiguration('init_y'),
            'init_z':     LaunchConfiguration('init_z'),
            'angle_size': LaunchConfiguration('angle_size'),
            'max_angle':  LaunchConfiguration('max_angle'),
            'init_angle': LaunchConfiguration('init_angle'),
            'lid_topic':  LaunchConfiguration('lid_topic'),
            'imu_topic':  LaunchConfiguration('imu_topic'),
        }]
    )

    # ------------------------------------------------------------------ #
    # 4. 返回启动描述
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        map_path,
        map_size,
        init_x,
        init_y,
        init_z,
        angle_size,
        max_angle,
        init_angle,
        lid_topic,
        imu_topic,
        launch_livox,
        livox_launch,
        init_node,
    ])