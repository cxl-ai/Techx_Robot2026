#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('local_planner')
    
    # -------------------- 配置文件路径 -------------------- #
    config_file = os.path.join(pkg_share, 'config', 'local_planner.yaml')

    # -------------------- 参数声明 -------------------- #
    # 这里只保留需要在 launch 中动态配置的参数（如用于 TF 发布的传感器偏移）
    args = [
        DeclareLaunchArgument('sensorOffsetX', default_value='0.35'),      # 激光X偏移（用于TF发布，可从yaml读取）
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),        # 激光Y偏移（用于TF发布，可从yaml读取）
        DeclareLaunchArgument('checkTerrainConn', default_value='true'),   # 地形连通检查
    ]

    # -------------------- 包含外部 launch -------------------- #
    terrain_analysis_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('terrain_analysis'),
                'launch',
                'terrain_analysis.launch'
            )
        )
    )

    terrain_analysis_ext_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('terrain_analysis_ext'),
                'launch',
                'terrain_analysis_ext.launch'
            )
        ),
        launch_arguments={'checkTerrainConn': LaunchConfiguration('checkTerrainConn')}.items()
    )

    # -------------------- 局部规划器节点 -------------------- #
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='local_planner',
        output='screen',
        parameters=[
            config_file,  # 从 yaml 加载所有参数
            {
                # pathFolder 通过代码计算，保留在 launch 中
                'pathFolder': os.path.join(pkg_share, 'paths'),
                'sensorOffsetX': LaunchConfiguration('sensorOffsetX'),
                'sensorOffsetY': LaunchConfiguration('sensorOffsetY'),
            }
        ]
    )

    # -------------------- 静态 TF -------------------- #
    # 从 lidar_base 到 base 的变换，使用传感器偏移参数
    # 对应 ROS1 版本中的 /sensor 到 /vehicle 变换
    base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baseTransPublisher',
        arguments=[
            '--x', PythonExpression(['-float(', LaunchConfiguration('sensorOffsetX'), ')']),  # 负的传感器X偏移
            '--y', PythonExpression(['-float(', LaunchConfiguration('sensorOffsetY'), ')']),  # 负的传感器Y偏移
            '--z', '0',                                                                       # Z方向无偏移（cameraOffsetZ用于相机变换）
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'lidar_base',
            '--child-frame-id', 'base'
        ]
    )

    return LaunchDescription([
        *args,
        terrain_analysis_launch,
        terrain_analysis_ext_launch,
        local_planner_node,
        base_tf_node,
    ])