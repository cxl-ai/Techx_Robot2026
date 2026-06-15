#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('high2r_bridge')
    config_file = os.path.join(pkg_share, 'config', 'high2r.yaml')

    high2r_node = Node(
        package='high2r_bridge',
        executable='high2r_bridge_node',
        name='high2r_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([high2r_node])
