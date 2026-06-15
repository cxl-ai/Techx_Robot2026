import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("waypoint_manager")
    param_file = os.path.join(pkg_share, "config", "waypoint_search.yaml")
    return LaunchDescription(
        [
            Node(
                package="waypoint_manager",
                executable="waypoint_search_node",
                name="waypoint_search",
                output="screen",
                parameters=[
                    param_file
                ],
            )
        ]
    )