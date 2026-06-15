"""
navigation.launch.py
────────────────────
启动 nav2 导航全栈（map_server + 规划/控制/行为 + lifecycle 管理）。

前置条件（需独立启动）：
  ros2 launch open3d_loc localization_3d.launch.py
  （该 launch 会同时拉起 FAST-LIO 和 open3d_loc，构建完整 TF 树）

本文件负责：
  1. map_server    — 加载 PGM 静态地图，发布 /map（OccupancyGrid）
  2. planner_server  — NavFn A* 全局规划器
  3. controller_server — DWB 局部规划器
  4. smoother_server  — 路径平滑器
  5. behavior_server  — 恢复行为（旋转/后退/等待等）
  6. bt_navigator    — 行为树导航执行器
  7. waypoint_follower — 航点跟随
  8. velocity_smoother — 速度平滑输出
  两个 lifecycle_manager 分别管理地图节点和导航节点。

用法：
  ros2 launch navigation navigation.launch.py
  ros2 launch navigation navigation.launch.py map:=/other/path/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # ── 包路径 ────────────────────────────────────────────────────────────────
    nav_pkg_dir = get_package_share_directory('navigation')

    # ── 启动参数 ──────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    params_file  = LaunchConfiguration('params_file')
    map_yaml     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟（实机运行保持 false）')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='lifecycle 节点自动激活')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml'),
        description='nav2 参数文件路径')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/lhl/nav_ws/config/map/map.yaml',
        description='2D 占用栅格地图 YAML 路径')

    # ── 参数替换：将 use_sim_time 和 map 路径注入 yaml ────────────────────────
    # map_server 的 yaml_filename 在 nav2_params.yaml 中为空，这里动态填入
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # ── 日志设置 ──────────────────────────────────────────────────────────────
    set_log_buf = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # ── lifecycle 管理的节点列表 ──────────────────────────────────────────────
    lifecycle_nodes_map = ['map_server']
    lifecycle_nodes_nav = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # ── 节点定义 ──────────────────────────────────────────────────────────────

    # map_server：加载静态 PGM 地图，发布 /map（nav2_costmap 的 static_layer 订阅此话题）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    # planner_server：NavFn A* 全局规划器
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    )

    # controller_server：DWB 局部规划器，输出速度指令到 /cmd_vel_nav
    # velocity_smoother 再将 /cmd_vel_nav → /cmd_vel（最终指令）
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # smoother_server：路径平滑器
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
    )

    # behavior_server：恢复行为（原地旋转、后退、等待等）
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    )

    # bt_navigator：行为树执行引擎，协调规划→执行→恢复全流程
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    )

    # waypoint_follower：多航点依序导航
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
    )

    # velocity_smoother：对 /cmd_vel_nav 做速度平滑，输出到 /cmd_vel
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # lifecycle_manager_map：管理 map_server 的生命周期
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes_map,
        }],
    )

    # lifecycle_manager_navigation：管理全部导航节点的生命周期
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes_nav,
        }],
    )

    # ── 汇总 ──────────────────────────────────────────────────────────────────
    return LaunchDescription([
        set_log_buf,
        # 参数声明
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        # 地图服务
        map_server_node,
        lifecycle_manager_map,
        # 导航栈
        planner_server_node,
        controller_server_node,
        smoother_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_nav,
    ])
