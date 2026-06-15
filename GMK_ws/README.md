# GMK_ws - ROBOCON 2026 主工控机 ROS 2 工作空间

本工作空间运行在 **GMK 主工控机（Ubuntu 22.04 + ROS 2 Humble）**，负责整个机器人的中枢调度、多机通讯、HMI 战术交互、激光雷达建图与 3D 自主局部避障规划。

---

## 一、 目录结构与功能包说明

GMK_ws 采用模块化子系统划分，所有包均位于 src/ 下：

`	ext
GMK_ws/
├── project/                     # 统一配置、地图与部署资源
│   ├── config/                  # 激光雷达网络配置 (MID360_config.json)
│   ├── map/                     # 场地全局 3D PCD 点云与子图
│   ├── data/use_to_nav/         # 赛段巡航航点与目标点 CSV 数据
│   ├── rviz/                    # RViz 可视化方案配置
│   └── scripts/                 # 自启动服务与系统脚本
│
└── src/                         # ROS 2 功能包源码 (按子系统划分)
    ├── vision_bridge/           # 【1. 视觉通讯桥】
    │   └── techx_vision_bridge  # 接收 Jetson UDP 数据包，转为 ROS 2 话题并进行 TF 坐标系转换
    │
    ├── hmi/                     # 【2. 战术指挥系统】
    │   ├── r2_hmi_core          # ROS 2 Action (ExecuteStage) 与 Srv (SyncCalibration) 接口
    │   └── r2_hmi_app           # PySide6 + QML 战术指挥看板 (支持红蓝阵营、A*标定、九宫格多选)
    │
    ├── drivers/                 # 【3. 传感器驱动与交互插件】
    │   ├── livox_ros_driver2    # Livox MID-360 固态激光雷达 ROS 2 驱动
    │   ├── waypoint_rviz_plugin # RViz 航点标记可视化插件
    │   └── waypoint_editor_rviz_plugin # RViz 交互式拖拽航点编辑插件
    │
    ├── navigation_2d/           # 【4. 经典 2D 激光导航栈 (Nav2)】
    │   ├── open3d_loc           # 基于 Open3D 的点云配准初始重定位 (map -> odom)
    │   ├── pcd2pgm              # 3D PCD 点云转 2D 占用栅格地图 (map.pgm + map.yaml)
    │   └── navigation           # 经典 Nav2 导航规划器配置文件
    │
    └── navigation_3d/           # 【5. 全自主 3D 地形越障与规划栈】
        ├── FAST_LIO             # 高精度激光惯性里程计与建图
        ├── relocalization       # 3D 点云全局重定位
        ├── terrain_analysis     # 实时三维地面可通行度与点云地形分析
        ├── terrain_analysis_ext # 扩展地形分析
        ├── local_planner        # 基于 CMU 框架的 3D 动态自主避障规划器
        ├── waypoint_manager     # 巡航路标点序列管理与发布
        └── high2r_bridge        # 宇树四足/移动底盘 DDS 控制协议中继桥
`

---

## 二、 工作空间编译

`ash
cd ~/GMK_ws

# 1. 全量编译（推荐使用符号链接安装模式）
colcon build --symlink-install

# 2. 仅编译特定子系统（以加快开发迭代）
colcon build --packages-select techx_vision_bridge
colcon build --packages-select r2_hmi_core r2_hmi_app
colcon build --packages-select FAST_LIO local_planner waypoint_manager

# 3. 环境变量生效
source install/setup.bash
`

---

## 三、 运行与调试流程

### 1. 启动传感器与 3D 自主避障导航
`ash
# 启动 3D 建图或带重定位的自主导航
ros2 launch local_planner nav.launch.py use_relocalization:=true
ros2 launch waypoint_manager waypoint_search.launch.py
`

### 2. 启动视觉桥接节点
`ash
# 接收 Jetson 发送的 UDP 帧，发布 /techx/vision/frame 和坐标话题
ros2 launch techx_vision_bridge vision_bridge.launch.py
`

### 3. 启动 R2 HMI 战术指挥系统
`ash
cd ~/GMK_ws/src/hmi/r2_hmi_app
python3 main.py
# 注：无 ROS 2 节点时会自动降级为 Mock 模式，方便单机离线测试交互界面与 A* 标定算法
`

### 4. 离线建图与航点标定流程
* **离线建图 (ros2bag 回放)**：
  `ash
  ros2 launch fast_lio mapping.launch.py use_ros2bag:=true ros2bag_path:=/path/to/bag
  ros2 service call /map_save std_srvs/srv/Trigger {}
  `
* **生成 2D 栅格地图**：
  `ash
  ros2 launch pcd2pgm pcd2pgm.launch.py pcd_file:=~/GMK_ws/project/map/map.pcd output_dir:=~/GMK_ws/project/map map_name:=map
  `
* **交互式航点编辑**：
  `ash
  ros2 launch waypoint_manager waypoint_editor.launch.py
  `

---

## 四、 关键配置与路径说明

* **雷达 IP 配置**：project/config/MID360_config.json（雷达默认 IP 192.168.1.1xx）
* **航点路标文件**：project/data/use_to_nav/waypoints-*.csv
* **开机自启动**：project/scripts/nav_ws_autostart.service 可复制到 /etc/systemd/system/ 实现通电自动运行