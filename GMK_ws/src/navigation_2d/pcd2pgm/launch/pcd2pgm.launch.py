from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ========== 输入输出 ==========
        DeclareLaunchArgument('pcd_file',   default_value='/home/lhl/nav_ws/config/map/map.pcd'),  # 输入 PCD 文件路径
        DeclareLaunchArgument('output_dir', default_value='/home/lhl/nav_ws/config/map'),  # 输出目录
        DeclareLaunchArgument('map_name',   default_value='map'),  # 输出文件名前缀

        # ========== 体素/栅格分辨率 ==========
        DeclareLaunchArgument('resolution', default_value='0.05'),  # 栅格分辨率（米/像素）

        # ========== 高度切片（投影为 2D 地图） ==========
        DeclareLaunchArgument('minh', default_value='-1.0'),  # 高度滤波下限（米）
        DeclareLaunchArgument('maxh', default_value='0.5'),  # 高度滤波上限（米）

        # ========== 地图边界留白 ==========
        DeclareLaunchArgument('padding', default_value='1.0'),  # 地图四周向外扩展的留白（米）

        # ========== 体素降采样（可选） ==========
        DeclareLaunchArgument('enable_voxel_filter', default_value='false' ),  # 是否启用体素降采样
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.05' ),  # 体素大小（米）

        # ========== 半径离群点去除（可选） ==========
        DeclareLaunchArgument('enable_radius_filter', default_value='true' ),  # 是否启用半径离群点去除
        DeclareLaunchArgument('radius_filter', default_value='0.3' ),  # 搜索半径（米）
        DeclareLaunchArgument('min_neighbors', default_value='15' ),  # 半径内最小邻居点数

        # ========== 非占用栅格默认值 ==========
        DeclareLaunchArgument('default_cell', default_value='free' ),  # "free"=白色(254)，"unknown"=灰色(205)

        # ========== YAML 阈值（供 nav2_map_server 读取） ==========
        DeclareLaunchArgument('occupied_thresh', default_value='0.65' ),  # 占用概率阈值
        DeclareLaunchArgument('free_thresh', default_value='0.196' ),  # 自由概率阈值

        # ========== 节点 ==========
        Node(
            package='pcd2pgm',
            executable='pcd2pgm_node',
            name='pcd2pgm',
            output='screen',
            parameters=[{
                'pcd_file': LaunchConfiguration('pcd_file'),
                'output_dir': LaunchConfiguration('output_dir'),
                'map_name': LaunchConfiguration('map_name'),
                'resolution': LaunchConfiguration('resolution'),
                'minh': LaunchConfiguration('minh'),
                'maxh': LaunchConfiguration('maxh'),
                'padding': LaunchConfiguration('padding'),
                'enable_voxel_filter': PythonExpression(["'", LaunchConfiguration('enable_voxel_filter'), "' == 'true'"]),
                'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                'enable_radius_filter': PythonExpression(["'", LaunchConfiguration('enable_radius_filter'), "' == 'true'"]),
                'radius_filter': LaunchConfiguration('radius_filter'),
                'min_neighbors': LaunchConfiguration('min_neighbors'),
                'default_cell': LaunchConfiguration('default_cell'),
                'occupied_thresh': LaunchConfiguration('occupied_thresh'),
                'free_thresh': LaunchConfiguration('free_thresh'),
            }]
        ),
])