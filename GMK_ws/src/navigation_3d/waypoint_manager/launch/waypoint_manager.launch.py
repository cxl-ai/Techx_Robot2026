import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="waypoint_manager",
                executable="waypoint_manager_node",
                name="waypoint_manager",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/Odometry",
                        "marker_topic": "waypoint_markers",
                        "record_service": "record_waypoint",
                        "record_goal_service": "record_goalpoint",
                        "frame_id": "",   # 为空则使用里程计 header.frame_id
                        "output_dir": os.path.join(os.path.expanduser('~'), 'nav_ws/project/data/use_to_nav'),
                        "file_prefix": "waypoints-",
                        "goal_file_prefix": "goalpoints-",
                    }
                ],
            )
        ]
    )

