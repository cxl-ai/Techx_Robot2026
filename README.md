# TechX Robot 2026 - 全栈自动化机器人机载系统

> **ROBOCON 2026 全国大学生机器人大赛 TechX 战队 R2 自动化战车机载软件系统**  
> 采用 **Jetson Orin NX (边缘视觉)** 与 **GMK 工控机 (ROS 2 主脑)** 双机协同计算架构。

---

## 📌 系统架构概览

整车机载计算系统由两台物理计算设备构成，通过千兆车载交换机实现点对点低延迟实时协同：

| 硬件计算单元 | 操作系统与环境 | 核心职能 | 静态 IP 配置 | 通信接口 |
| :--- | :--- | :--- | :--- | :--- |
| **Jetson Orin NX** (`Jetson_ws`) | Ubuntu 20.04/22.04 + CUDA | 奥比中光 335L 相机驱动、YOLO 目标识别、3D 空间测距、手眼标定 | `192.168.10.101` | UDP V2 遥测推流 (`Port: 12345`) |
| **GMK 工控机** (`GMK_ws`) | Ubuntu 22.04 + ROS 2 Humble | 视觉数据坐标解算、QML 战术指挥系统、3D SLAM、自主越障与局部规划 | `192.168.10.100` | 接收 UDP 帧并解算 TF 坐标发布 |

```
                                  【TechX R2 机载双机协同拓扑】
                                  
   +-----------------------+                         +-----------------------------------+
   | 奥比中光 335L 深度相机 |                         |      Livox MID-360 固态激光雷达     |
   +-----------------------+                         +-----------------------------------+
               | (USB 3.0)                                             | (Ethernet)
               v                                                       v
   +-----------------------+      UDP V2 遥测数据包      +-----------------------------------+
   |    Jetson Orin NX     | ==========================> |           GMK 主工控机            |
   | (YOLO目标检测+深度解算)|   (192.168.10.100:12345)    |       (ROS 2 Humble 全栈主脑)      |
   +-----------------------+                             +-----------------------------------+
                                                                       |
                 +-----------------------------------------------------+---------------------------------+
                 |                                                     |                                 |
                 v                                                     v                                 v
   +---------------------------+                         +---------------------------+     +---------------------------+
   |  R2 HMI 战术指挥系统 (QML) |                         |   3D 激光雷达建图与越障规划  |     |   底盘控制与机械臂执行中继   |
   | (红蓝阵营/A*标定/九宫格)   |                         | (FAST_LIO + local_planner)|     | (high2r_bridge / TF2解算)  |
   +---------------------------+                         +---------------------------+     +---------------------------+
```

---

## 📂 仓库目录导引

本项目源码按两台物理机载设备划分为两大独立工程工作空间：

```
Techx_Robot2026/
├── Jetson_ws/                          # 【设备 1: Jetson Orin NX 边缘视觉工程】
│   ├── config.json                     # 相机/网络/检测阈值配置文件
│   ├── launch.py / main.py             # 视觉推理主入口
│   ├── camera/                         # 奥比中光相机 SDK 驱动封装
│   ├── detection/                      # YOLO 多目标识别 (武器头/真假KFS/灯带/二维码)
│   ├── communication/                  # UDP V2 协议封包与收发模块
│   ├── models/                         # YOLO 模型权重文件 (kfs_v3, head_v1 等)
│   ├── tools/                          # 手眼标定与深度精度测试工具
│   └── scripts/                        # 边缘端开机自启脚本
│
├── GMK_ws/                             # 【设备 2: GMK 主工控机 ROS 2 工作空间】
│   ├── README.md                       # GMK 端专用编译与部署指南
│   ├── project/                        # 统一配置中心 (雷达参数、地图 PCD、航点 CSV)
│   └── src/                            # ROS 2 源码 (按 5 大子系统规整)
│       ├── vision_bridge/              # [子系统 1] 视觉通讯桥: techx_vision_bridge
│       ├── hmi/                        # [子系统 2] 战术指挥系统: r2_hmi_core, r2_hmi_app
│       ├── drivers/                    # [子系统 3] 传感器驱动: livox_ros_driver2, rviz 插件
│       ├── navigation_2d/              # [子系统 4] 2D 激光导航: open3d_loc, pcd2pgm, navigation
│       └── navigation_3d/              # [子系统 5] 3D 自主规划: FAST_LIO, local_planner, high2r
│
└── docs/                               # 场地资料、3D 点云全景图与坐标变换说明
    ├── field_3d_pointcloud.jpg         # 比赛场地三维高精度激光点云图
    └── projection_3d_to_2d.jpg         # 空间坐标系投影参考图
```

---

## 🚀 核心子系统与功能

### 1. 边缘端目标感知 (`Jetson_ws`)
* **多目标并行检测**：实时识别真/假 KFS 矿石、三种不同武器头、装配成功指示灯带、以及 R1 对齐二维码。
* **空间 3D 解算**：结合相机内参矩阵与双目视差图，实时输出空间毫米级 3D 坐标。

### 2. 视觉通讯桥接 (`GMK_ws/src/vision_bridge`)
* 接收来自 Jetson 的 UDP 数据帧；
* 通过 ROS 2 TF2 坐标变换树，将目标坐标从 `camera_link` 实时投影至 `robot_base`（底盘中心系）和 `arm1_base`/`arm2_base`（机械臂基座系）；
* 提供 `/techx/vision/request` 按需请求服务，极大降低上位机总线开销。

### 3. R2 HMI 战术指挥系统 (`GMK_ws/src/hmi`)
* **技术栈**：PySide6 + QML 现代化扁平视觉；
* **阵营选择**：一键切换红/蓝双方坐标镜像；
* **梅林区 12 宫格 A* 寻路标定**：触屏标记障碍物，实时计算并金色高亮最短无碰撞路径；
* **对抗区九宫格看板**：支持多选目标任务卡片下发，配备硬件全局急停保护。

### 4. 3D 激光雷达建图与全自主避障规划 (`GMK_ws/src/navigation_3d`)
* **FAST_LIO**：结合 MID-360 内置高频 IMU 与点云，输出 100Hz 强鲁棒性高精度里程计；
* **Terrain Analysis**：实时解算点云地面可通行度与坡度；
* **Local Planner**：动态障碍物（赛场对抗车辆）突现时，实时生成 3D 绕障避障轨迹。

---

## 🛠️ 快速上手与运行指南

### 1. Jetson Orin NX 端启动
```bash
cd ~/Jetson_ws

# 场景 A: 带显示器调试
TECHX_GUI=1 bash start_jetson.sh

# 场景 B: 现场比赛无屏幕模式
bash scripts/field_start_jetson.sh
```

### 2. GMK 工控机端构建与启动
```bash
cd ~/GMK_ws

# 全量构建所有 ROS 2 功能包
colcon build --symlink-install
source install/setup.bash

# 启动 3D 建图或带重定位的自主导航
ros2 launch local_planner nav.launch.py use_relocalization:=true

# 启动视觉桥接中继节点
ros2 launch techx_vision_bridge vision_bridge.launch.py

# 启动 R2 HMI 战术指挥界面
cd ~/GMK_ws/src/hmi/r2_hmi_app && python3 main.py
```

---

## 📜 现场自启动部署
* 参考 `GMK_ws/project/scripts/nav_ws_autostart.service`，注册为 systemd 服务实现工控机上电自动拉起导航系统。
