# high2r_bridge

本包提供 **ROS2 节点**，将 ROS 速度指令（`geometry_msgs/msg/Twist`）转成 **DDS** 的 `VelCmd_`，发送到机器人侧的 DDS topic。

## 功能概览

- **订阅(ROS2)**: `/high2r_cmd`（`geometry_msgs/msg/Twist`）
  - 使用字段：`linear.x` 和 `angular.z`
  - 线速度/角速度由参数 `max_vx`、`max_vy`、`max_yaw_rate` 限幅，并可通过 `smoothing_alpha` 做一阶平滑
- **发布(DDS)**: `rt/ltr/vel_cmd`（类型：`ltr::msg::dds_::VelCmd_`）
- **发布(DDS)**: `rt/ltr/sportmode_cmd`（类型：`ltr::msg::dds_::SportModeCmd_`），通过终端交互 CLI 输入 0–6 切换模式
- **DDS 网卡选择**: 参数 `dds_interface`（默认见 [config/high2r.yaml](config/high2r.yaml)，如 `enp45s0`）

节点源码：`src/hig2robot_node.cpp`

## 依赖

- **ROS2**（Humble 或更高，ament_cmake）
  - `rclcpp`、`geometry_msgs`
- **Eigen3**
- **DDS / LTR SDK**（仓库内 `dds_comm/` 已包含预编译依赖与头文件）

> 注意：`dds_comm/thirdparty/lib/<arch>/` 下的 `libddsc.so` / `libddscxx.so` 运行时可能需要在 `LD_LIBRARY_PATH` 中可见（见下文）。

## 编译（colcon 工作空间）

本包使用 ament_cmake，需放入 ROS2 工作空间：

```bash
cd ~/nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select high2r_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

**1. 启动节点**

```bash
ros2 launch high2r_bridge high2r_bridge.launch.py
```

参数在 [config/high2r.yaml](config/high2r.yaml) 中配置，主要包含：

- `dds_interface`：DDS 网卡名（如 `enp45s0`）
- `cmd_topic`：订阅的 Twist 话题，默认 `/high2r_cmd`
- `max_vx` / `max_vy` / `max_yaw_rate`：速度限幅
- `smoothing_alpha`：速度一阶滤波系数 (0~1)

**2. 从 ROS2 侧发送速度指令（示例：前进 + 原地转）**

```bash
ros2 topic pub /high2r_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -r 50
```

### SportMode 交互 CLI（终端输入）

节点运行后，在同一终端直接输入数字切换 `rt/ltr/sportmode_cmd`（设备 ID 固定为高阶 SDK 通道）：

- 0: PASSIVE（不驱动）
- 1: DAMPING（阻尼保护）
- 2: SITDOWN（坐下）
- 3: STANDUP（站立）
- 4: NORMAL_WALK（常规行走控制）
- 5: LION_DANCE（舞狮模式）
- 6: LION_DANCE_OFF（退出舞狮）

输入 `q` 可退出 CLI（节点继续运行）。

### 推荐的启动 / 关闭顺序

- 启动：先输入 3 (STANDUP) → 再输入 4 (NORMAL_WALK)
- 关闭：先回到 3 (STANDUP) → 然后 2 (SITDOWN) → 最后 1 (DAMPING)

### 运行时库路径（常见问题）

若运行时报 `libddsc.so` / `libddscxx.so` 找不到，可把本包源码下的 DDS 第三方库加入 `LD_LIBRARY_PATH`：

```bash
# 先 source 工作空间
source install/setup.bash
# 将 <ws> 替换为你的工作空间根路径，如 ~/nav_ws
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<ws>/src/high2r_bridge/dds_comm/thirdparty/lib/$(uname -m)
```

`uname -m` 多为 `x86_64` 或 `aarch64`；若目录名不一致，请按实际 `dds_comm/thirdparty/lib/` 下的子目录名填写。
