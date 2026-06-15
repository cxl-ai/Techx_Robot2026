#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # 1. 声明启动参数
    # ------------------------------------------------------------------ #
    relocalization_share = get_package_share_directory('relocalization')
    livox_ros_driver2_share = get_package_share_directory('livox_ros_driver2')
    config_file = os.path.join(relocalization_share, 'config', 'localization.yaml')

    default_map_path = os.path.join(os.path.expanduser('~'), 'nav_ws/project/map')
    
    rviz_arg      = DeclareLaunchArgument('rviz',           default_value='true')
    map_path_arg  = DeclareLaunchArgument('map_path',       default_value=default_map_path)
    config_file_arg = DeclareLaunchArgument('config_file',  default_value=config_file)

    # ------------------------------------------------------------------ #
    # 2. 定位节点
    #    负责基于地图进行实时定位
    # ------------------------------------------------------------------ #
    localization_node = Node(
        package='relocalization',
        executable='localization',
        name='localization',               # 节点名称
        output='screen',                   # 日志输出到终端
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # map_path 可以通过 launch 参数覆盖 yaml 中的默认值
                'map_path': LaunchConfiguration('map_path'),
            }
        ]
    )

    # ------------------------------------------------------------------ #
    # 3. RViz 可视化节点（可选）
    #    根据 rviz 参数决定是否启动
    # ------------------------------------------------------------------ #
    rviz_config_file = os.path.expanduser('~/nav_ws/project/rviz/fastlio.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ------------------------------------------------------------------ #
    # 4. Livox MID360 驱动节点
    #    启动 livox_ros_driver2 的 msg_MID360_launch.py
    # ------------------------------------------------------------------ #
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_ros_driver2_share, 'launch', 'msg_MID360_launch.py')
        )
    )

    # ------------------------------------------------------------------ #
    # 5. 返回启动描述
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        rviz_arg,
        map_path_arg,
        config_file_arg,
        livox_driver_launch,
        localization_node,
        rviz_node,
    ])
