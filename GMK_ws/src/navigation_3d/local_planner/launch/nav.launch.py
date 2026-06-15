#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    local_planner_share = get_package_share_directory('local_planner')
    fast_lio_share = get_package_share_directory('fast_lio')
    relocalization_share = get_package_share_directory('relocalization')
    high2r_bridge_share = get_package_share_directory('high2r_bridge')

    # -------------------- 参数声明 -------------------- #
    rviz_enable_arg = DeclareLaunchArgument('rviz_enable', default_value='false')  # 是否启用RViz可视化
    use_relocalization_arg = DeclareLaunchArgument(
        'use_relocalization', 
        default_value='false', description='true: 有图重定位模式(relocalization), false: 无图导航模式(odom)'
    )

    # -------------------- 配置文件路径 -------------------- #
    config_file = os.path.join(local_planner_share, 'config', 'speed_control.yaml')
    rviz_config_file = os.path.expanduser('~/nav_ws/project/rviz/vehicle_simulator.rviz')

    # -------------------- 包含外部 launch -------------------- #
    # 有图重定位模式：使用 relocalization
    relocalization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(relocalization_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('use_relocalization'))
    )

    # 无图导航模式：使用 FAST_LIO odom
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'odom.launch.py')
        ),
        launch_arguments={'config_file': 'odom.yaml'}.items(),
        condition=UnlessCondition(LaunchConfiguration('use_relocalization'))
    )

    # 局部规划器节点
    local_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_planner_share, 'launch', 'local_planner.launch.py')
        )
    )

    # High2R 桥接节点
    high2r_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(high2r_bridge_share, 'launch', 'high2r_bridge.launch.py')
        )
    )

    # -------------------- 导航节点 -------------------- #
    nav_node = Node(
        package='local_planner',
        executable='nav_node',
        name='nav_node',
        output='screen',
        parameters=[config_file]
    )

    # -------------------- RViz 可视化节点 -------------------- #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='viz_real',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz_enable'))
    )

    return LaunchDescription([
        rviz_enable_arg,
        use_relocalization_arg,
        relocalization_launch,
        fast_lio_launch,
        local_planner_launch,
        high2r_bridge_launch,
        nav_node,
        rviz_node,
    ])
