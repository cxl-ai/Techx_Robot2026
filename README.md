# Techx_Robot2026 - ROBOCON 2026 自动化全栈机器人系统

本项目为全国大学生机器人大赛 (ROBOCON 2026) 参赛机器人 **TechX R2 战车** 的完整全栈机载软件系统。系统采用**机载物理双机协同计算架构**，由边缘视觉节点与主控 ROS 2 机器人大脑构成，具备实时视觉伺服目标识别、战术态势指挥交互 (HMI)、3D 激光雷达 SLAM、自主越障与局部动态轨迹规划能力。

---

## 目录
- [一、 物理双机协同架构与网络拓扑](#一-物理双机协同架构与网络拓扑)
- [二、 软件系统架构](#二-软件系统架构)
- [三、 仓库组织结构](#三-仓库组织结构)
- [四、 核心子系统详解](#四-核心子系统详解)
- [五、 快速部署与操作流程](#五-快速部署与操作流程)
- [六、 现场运维与自启动服务](#六-现场运维与自启动服务)

---

## 一、 物理双机协同架构与网络拓扑

系统硬件部署在两台独立的机载计算单元上，通过千兆以太网实现点对点低延迟实时通信：

| 计算硬件 | 系统环境 | 核心职能 | 静态 IP | 通信协议 |
| :--- | :--- | :--- | :--- | :--- |
| **Jetson Orin NX** (Jetson_ws) | Linux (Ubuntu 20.04/22.04), CUDA, TensorRT | 奥比中光相机采集、YOLO 多目标识别、深度空间解算、手眼标定 | 192.168.10.101 | 发送 UDP V2 遥测数据包至 GMK (Port: 12345) |
| **GMK 主工控机** (GMK_ws) | Ubuntu 22.04 / ROS 2 (Humble) | 视觉数据坐标系解算、QML 战术指挥系统、3D 点云建图、Nav2/局部自主规划、底盘控制 | 192.168.10.100 | 接收 UDP 视觉帧，转为 ROS 2 话题分发与 Action 调度 |

`	ext
+-----------------------------------------------------------------------------------+
|                            TechX R2 战车计算与感知网络                              |
+-----------------------------------------------------------------------------------+
  [奥比中光 335L 深度相机]
            | (USB 3.0)
            v
  +-----------------------+     UDP V2 (192.168.10.100:12345)     +-----------------------+
  |    Jetson Orin NX     | ====================================> |      GMK 主工控机      |
  |  (YOLOv8 + 深度估计)   |                                       |      (ROS 2 主脑)      |
  +-----------------------+                                       +-----------------------+
                                                                             ^
  [Livox MID-360 激光雷达]                                                    |
            | (Ethernet)                                                     |
            +----------------------------------------------------------------+
                                                                             |
                                             +-------------------------------+-------------------------------+
                                             |                               |                               |
                                             v                               v                               v
                                  [HMI 战术指挥系统 (QML)]      [3D SLAM 与局部避障规划]         [底盘与机械臂执行机构]
                                  (阵营选择 / A* 标定 / 九宫格)    (FAST_LIO / local_planner)      (high2r_bridge / TF2)
`

---

## 二、 软件系统架构

`mermaid
graph TD
    subgraph Jetson_ws ["Jetson_ws (边缘感知节点)"]
        Cam["Orbbec 335L 驱动"] --> Detect["YOLO 多目标检测<br/>(武器头/真假KFS/灯带/二维码)"]
        Detect --> Depth["点云/深度空间解算"]
        Depth --> UDPSend["UDP V2 遥测组包发送<br/>(192.168.10.101:12345)"]
    end

    subgraph GMK_ws ["GMK_ws (ROS 2 全栈工控机工作空间)"]
        UDPSend -.->|"UDP 数据帧"| VB["techx_vision_bridge<br/>(TF2 坐标变换与发布)"]
        
        subgraph HMI_System ["HMI 战术交互系统"]
            HMI_App["r2_hmi_app (PySide6 + QML)"] <-->|"Action / Srv"| HMI_Core["r2_hmi_core"]
            HMI_Core -->|"战术阶段触发"| Decision["战术决策状态机"]
        end

        subgraph Navigation_Stack ["导航与定位系统"]
            Lidar["Livox MID-360"] --> Driver["livox_ros_driver2"]
            Driver --> FAST["FAST_LIO (3D里程计建图)"]
            FAST --> Terrain["terrain_analysis (地形通行度分析)"]
            Terrain --> LocalPlan["local_planner (动态避障与轨迹生成)"]
            Waypoints["waypoint_manager (比赛航点序列)"] --> LocalPlan
            FAST --> Reloc["open3d_loc / relocalization (点云重定位)"]
            Reloc --> Nav2["navigation (2D Nav2 备用链路)"]
        end

        VB --> Decision
        Decision --> High2r["high2r_bridge (底盘 DDS 控制通讯)"]
        LocalPlan --> High2r
    end

    High2r ==>|"电机驱动 / 串口 / CAN"| Chassis["移动底盘 & 机械臂执行器"]
`

---

## 三、 仓库组织结构

`	ext
Techx_Robot2026/
├── .gitignore                          # 过滤 ROS 编译产物、日志及模型缓存
├── README.md                           # 全系统总览设计文档
├── docs/                               # 场地资料、点云图纸与标定文档
│   ├── field_3d_pointcloud.jpg         # 比赛场地三维高精度激光点云图
│   ├── projection_3d_to_2d.jpg         # 空间坐标系投影图
│   └── projection_3d_to_2d_2.jpg
│
├── Jetson_ws/                          # 【设备 1: Jetson Orin NX 视觉工程】
│   ├── config.json                     # 相机/网络/检测阈值配置
│   ├── launch.py / main.py             # 视觉推理主入口
│   ├── camera/                         # 奥比中光相机 SDK 封装 (pyorbbecsdk)
│   ├── detection/                      # YOLO 目标检测、灯带识别、二维码定位
│   ├── communication/                  # UDP V2 协议封包与校验
│   ├── models/                         # 模型权重 (kfs_v3, head_v1, gesture)
│   ├── tools/                          # 手眼标定采集与外参解算工具链
│   ├── scripts/                        # 自启动服务与网络配置脚本
│   └── requirements_jetson.txt
│
└── GMK_ws/                             # 【设备 2: GMK 工控机 ROS 2 工作空间】
    ├── README.md                       # GMK 工作空间专用编译与启动手册
    ├── project/                        # 统一配置中心、地图 PCD 与 RViz 可视化
    │   ├── config/                     # 雷达配置 (MID360_config.json)
    │   ├── map/                        # 场地 3D 点云地图与子图
    │   ├── data/use_to_nav/            # 预设航点与目标序列 (waypoints / goalpoints)
    │   ├── rviz/                       # fastlio.rviz, vehicle_simulator.rviz
    │   └── scripts/                    # systemd 开机自启服务配置
    │
    └── src/                            # ROS 2 源码 (按 5 大子系统规范组织)
        ├── vision_bridge/              # [子系统 1: 视觉通讯桥]
        │   └── techx_vision_bridge     # UDP 解析、TF2 转换 (camera -> base -> arm)
        ├── hmi/                        # [子系统 2: 战术指挥系统]
        │   ├── r2_hmi_core             # ROS 2 Action/Service 协议包
        │   └── r2_hmi_app              # PySide6+QML 界面 (阵营/A*梅林标定/九宫格/急停)
        ├── drivers/                    # [子系统 3: 传感器驱动与交互插件]
        │   ├── livox_ros_driver2       # Livox MID-360 雷达 ROS 2 驱动
        │   ├── waypoint_rviz_plugin    # RViz 航点显示插件
        │   └── waypoint_editor_rviz_plugin # RViz 航点交互式编辑插件
        ├── navigation_2d/              # [子系统 4: 2D 建图与 Nav2 备用栈]
        │   ├── open3d_loc              # 基于 Open3D 的点云配准初始定位
        │   ├── pcd2pgm                 # 3D PCD 点云转 2D 栅格地图
        │   └── navigation              # Nav2 导航规划器配置
        └── navigation_3d/              # [子系统 5: 3D 自主地形避障与规划栈]
            ├── FAST_LIO                # 高精度激光惯性里程计与建图
            ├── relocalization          # 3D 全局点云重定位
            ├── terrain_analysis        # 实时地面通行度与坡度点云分析
            ├── terrain_analysis_ext    # 扩展地形特征提取
            ├── local_planner           # 3D 动态障碍物绕障轨迹规划
            ├── waypoint_manager        # 赛段巡航点序列管理
            └── high2r_bridge           # 宇树/四足底盘运动控制桥
`

---

## 四、 核心子系统详解

### 1. 视觉感知子系统 (Jetson_ws)
* **检测目标体系**：
  *  ~5：红蓝双方真假 KFS 矿石
  * 100~102：三种武器头（抓取装配）
  * 150~159：装配状态灯带颜色（识别成功装配事件）
  * 200：R1 机器人抬升定位二维码
* **三维定位解算**：基于奥比中光双目深度图与外参投影，输出准确的 3D 目标质心及抓取位姿。

### 2. 视觉桥接与坐标系转换 (GMK_ws/src/vision_bridge)
* 接收 Jetson 发送的 UDP 帧，将 camera_link 下的坐标实时解算到：
  * obot_base：机器人底盘中心系（用于底盘对齐逼近）
  * rm1_base：机械臂 1 坐标系（武器头抓取）
  * rm2_base：机械臂 2 坐标系（KFS 抓取与九宫格投递）
* 对外提供 /techx/vision/request 按需目标请求服务，降低主控计算负载。

### 3. R2 HMI 战术指挥系统 (GMK_ws/src/hmi)
* **阵营选择**：红方 / 蓝方一键切换坐标系镜像。
* **梅林区 12 宫格标定**：操作手在触控屏上标记真假 KFS，内置 **A\* 路径搜索算法** 实时规划并高亮最优绕行路线。
* **对抗区九宫格看板**：支持多选目标任务卡片下发。
* **安全保护**：全局一键硬件急停锁死。

### 4. 3D 激光雷达建图与全自主避障规划 (GMK_ws/src/navigation_3d)
* **FAST_LIO**：结合 MID-360 内部 IMU 与激光点云，提供 100Hz 高频里程计。
* **Terrain Analysis**：将点云划分为局部网格，实时解算地面凹凸度、台阶高度与障碍物。
* **Local Planner**：在动态障碍物（如对方战车、翻转矿石）突现时，实时生成无碰撞的三维样条曲线轨迹。

---

## 五、 快速部署与操作流程

### 1. Jetson 端启动
`ash
cd ~/Jetson_ws
# 场景 A: 带显示器调试
TECHX_GUI=1 bash start_jetson.sh

# 场景 B: 现场比赛无屏幕自启
bash scripts/field_start_jetson.sh
`

### 2. GMK 工控机端编译与启动
`ash
cd ~/GMK_ws
# 全量构建所有 ROS 2 功能包
colcon build --symlink-install
source install/setup.bash

# 1. 启动激光雷达驱动与 3D 自主避障导航
ros2 launch local_planner nav.launch.py use_relocalization:=true

# 2. 启动视觉通讯中继桥
ros2 launch techx_vision_bridge vision_bridge.launch.py

# 3. 启动战术指挥界面
cd ~/GMK_ws/src/hmi/r2_hmi_app
python3 main.py
`

---

## 六、 现场运维与自启动服务

* **工控机系统自启服务**：参考 GMK_ws/project/scripts/nav_ws_autostart.service，可直接注册为 Linux systemd 服务，实现开机自动上电建图与定位。
* **手眼标定与精度检验**：使用 Jetson_ws/tools/collect_chessboard_handeye.py 与 estimate_extrinsic_from_points.py 可在赛前 5 分钟内快速校验相机与机械臂坐标误差。