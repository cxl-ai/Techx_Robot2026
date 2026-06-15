import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_to_nav_dir = os.path.join(os.path.expanduser("~"), "nav_ws/project/data/use_to_nav")
    return LaunchDescription(
        [
            Node(
                package="waypoint_manager",
                executable="waypoint_editor_node",
                name="waypoint_editor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,

                        # CSV 目录：不指定 waypoints_csv/goalpoints_csv 时，会自动取目录下最新一对
                        "use_to_nav_dir": use_to_nav_dir,
                        "waypoints_csv": os.path.join(use_to_nav_dir, "waypoints-01-27-17-01-20.csv"),
                        "goalpoints_csv": os.path.join(use_to_nav_dir, "goalpoints-01-27-17-01-20.csv"),

                        "frame_id": "world",
                        "marker_topic": "/waypoint_editor_markers",
                        "publish_period_ms": 500,

                        # 连续控制：最大速度（米/秒、弧度/秒）
                        "max_x_per_sec": 0.8,
                        "max_y_per_sec": 0.8,
                        "max_z_per_sec": 0.6,
                        "max_yaw_per_sec": 1.0,
                    },
                ],
            )
        ]
    )

