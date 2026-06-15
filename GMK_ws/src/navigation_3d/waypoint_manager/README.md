# waypoint_manager

一个用于ROS2的航点管理工具包，支持航点记录、编辑和基于航点的路径规划。

## 包概述

`waypoint_manager` 提供了完整的航点管理解决方案，包含三个核心节点：

1. **waypoint_manager_node**：记录和可视化航点
2. **waypoint_editor_node**：交互式编辑已有航点
3. **waypoint_search_node**：基于航点的目标选择和路径规划

所有航点数据以CSV格式存储，支持多楼层场景，可在RViz2中实时可视化。

## 数据格式

### CSV文件格式

所有CSV文件遵循统一格式：

```csv
floor,index,x,y,z,yaw
```

- **floor**：楼层编号（int）
- **index**：楼层内的航点索引（int，从1开始）
- **x, y, z**：位置坐标（double，单位：米）
- **yaw**：航向角（double，单位：弧度）

### 楼层语义（针对 `project/data/use_to_nav/` 目录）

- **floor=1**：连接层（用于跨楼层连接和导航）
- **floor>=2**：实际楼层编号

### 航点类型

- **waypoints**：导航路径点，用于构建导航路径
- **goalpoints**：目标点，通常是任务目的地

## 节点详解

### 1. waypoint_manager_node（航点记录节点）

记录机器人当前位置为航点或目标点，并保存到CSV文件。

#### 功能特性

- 订阅里程计话题获取机器人位置
- 通过ROS2服务触发航点记录
- 自动生成带时间戳的CSV文件（格式：`waypoints-MM-DD-HH-MM-SS.csv`）
- 实时发布可视化标记到RViz2
- 支持多楼层航点管理（每个楼层独立索引）
- 后台线程处理文件I/O，避免阻塞服务响应

#### 参数配置

| 参数名                | 类型   | 默认值              | 说明                                 |
| --------------------- | ------ | ------------------- | ------------------------------------ |
| `odom_topic`          | string | `/odom`             | 里程计话题名称                       |
| `marker_topic`        | string | `waypoint_markers`  | 可视化标记话题                       |
| `record_service`      | string | `record_waypoint`   | 记录航点的服务名                     |
| `record_goal_service` | string | `record_goalpoint`  | 记录目标点的服务名                   |
| `frame_id`            | string | `""`                | 坐标系ID（空则使用里程计的frame_id） |
| `output_dir`          | string | `~/.ros/waypoints/` | CSV文件输出目录                      |
| `file_prefix`         | string | `waypoints-`        | 航点文件名前缀                       |
| `goal_file_prefix`    | string | `goalpoints-`       | 目标点文件名前缀                     |

#### 启动方式

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py
```

#### 服务调用示例

记录航点（waypoint）：

```bash
# 记录floor=1的航点
ros2 service call /waypoint_manager/record_waypoint waypoint_manager/srv/RecordPoints "{floor: 1}"

# 记录floor=2的航点
ros2 service call /waypoint_manager/record_waypoint waypoint_manager/srv/RecordPoints "{floor: 2}"
```

记录目标点（goalpoint）：

```bash
# 记录floor=2的目标点
ros2 service call /waypoint_manager/record_goalpoint waypoint_manager/srv/RecordPoints "{floor: 2}"
```

#### RViz2可视化

1. 添加 `MarkerArray` 显示类型
2. 话题选择 `/waypoint_markers`
3. 可视化效果：
   - 红色球体：航点（waypoints）
   - 蓝色球体：目标点（goalpoints）
   - 黄色文本：航点标签（格式：`floor-index`）
   - 浅蓝色文本：目标点标签

---

### 2. waypoint_editor_node（航点编辑节点）

加载已有的航点CSV文件，提供交互式编辑功能。

#### 功能特性

- 自动加载最新的CSV文件（或指定文件）
- 通过服务选择要编辑的航点
- 支持连续控制（滑杆式）调整航点的x、y、z、yaw
- 实时可视化编辑效果（绿色高亮显示选中点）
- 退出时自动保存修改到原CSV文件
- 高频控制更新（50Hz）保证平滑操作

#### 参数配置

| 参数名              | 类型   | 默认值                     | 说明                            |
| ------------------- | ------ | -------------------------- | ------------------------------- |
| `use_to_nav_dir`    | string | 见配置文件                 | CSV文件所在目录                 |
| `waypoints_csv`     | string | `""`                       | 指定航点CSV（空则自动选最新）   |
| `goalpoints_csv`    | string | `""`                       | 指定目标点CSV（空则自动选最新） |
| `frame_id`          | string | `world`                    | 坐标系ID                        |
| `marker_topic`      | string | `/waypoint_editor_markers` | 可视化标记话题                  |
| `publish_period_ms` | int    | 200                        | 可视化发布周期（毫秒）          |
| `max_x_per_sec`     | double | 0.8                        | X轴最大移动速度（米/秒）        |
| `max_y_per_sec`     | double | 0.8                        | Y轴最大移动速度（米/秒）        |
| `max_z_per_sec`     | double | 0.6                        | Z轴最大移动速度（米/秒）        |
| `max_yaw_per_sec`   | double | 1.0                        | 航向角最大旋转速度（弧度/秒）   |

#### 启动方式

```bash
ros2 launch waypoint_manager waypoint_editor.launch.py
```

#### 服务调用示例

列出所有可用的航点：

```bash
# 列出所有waypoints
ros2 service call /waypoint_editor/list_floor_index waypoint_manager/srv/ListFloorIndex "{is_goalpoint: false}"

# 列出所有goalpoints
ros2 service call /waypoint_editor/list_floor_index waypoint_manager/srv/ListFloorIndex "{is_goalpoint: true}"
```

选择要编辑的航点：

```bash
# 选择waypoint 1-5
ros2 service call /waypoint_editor/select_point waypoint_manager/srv/SelectPoint "{is_goalpoint: false, floor_index: '1-5'}"

# 选择goalpoint 2-3
ros2 service call /waypoint_editor/select_point waypoint_manager/srv/SelectPoint "{is_goalpoint: true, floor_index: '2-3'}"
```

控制选中航点的移动：

```bash
# X轴正方向移动（滑杆值：-1.0 到 1.0）
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 0, value: 0.5}"

# Y轴移动
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 1, value: -0.3}"

# Z轴移动
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 2, value: 0.2}"

# Yaw旋转
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 3, value: 0.8}"

# 停止移动（松开滑杆）
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 0, value: 0.0}"
```

#### RViz2可视化

1. 添加 `MarkerArray` 显示类型
2. 话题选择 `/waypoint_editor_markers`
3. 可视化效果：
   - 红色/蓝色球体：所有航点
   - **绿色球体**：当前选中的航点（高亮）
   - 文本标签：显示floor-index

#### 配合RViz插件使用

推荐配合 `waypoint_editor_rviz_plugin` 使用，可在RViz面板中使用滑杆可视化操作，无需手动调用服务。

---

### 3. waypoint_search_node（航点搜索节点）

基于已记录的航点，进行目标选择和路径规划。

#### 功能特性

- 自动加载最新的waypoints和goalpoints CSV文件
- 通过floor-index选择目标点
- **任意楼层路径规划**：支持从任意楼层到任意楼层的路径规划
- 发布自定义目标点消息（包含floor、index、pose）
- 发布路径可视化（nav_msgs/Path）
- 支持多楼层路径拼接（通过floor=1连接层）
- 实时可视化所有航点和当前目标
- **航点顺序追踪**：自动按顺序逐个发布航点到`/way_point`话题
- **智能切换**：根据机器人与当前航点的距离自动切换到下一个航点
- **动态楼层状态管理**：实时追踪机器人当前所在楼层，自动更新楼层状态
- **手动状态修改**：提供服务接口手动设置楼层状态，防止意外情况

#### 路径规划策略

支持从任意楼层到任意楼层的路径规划，具体策略如下：

**场景1：同楼层内导航（当前楼层 == 目标楼层）**

```
当前位置 -> A-nearest -> ... -> A-nearest_to_goal -> goal
```

直接在同楼层内规划，沿着index连续的路径，不经过连接点。

**场景2：跨楼层导航（必须通过floor=1中转）**

```
A-current -> A-nearest -> ... -> A-1 -> 1-m -> 1-n -> B-1 -> ... -> B-goal
```

具体步骤：
1. **当前楼层 A → 连接点 A-1**：从当前位置到连接点
2. **floor=1 中转**：1-m（离A-1最近）→ 1-n（离B-1最近）
3. **目标楼层 B-1 → goal**：从连接点到目标点

**特殊情况：**
- 从floor=1到floor=n：`1-current → 1-n → n-1 → n-goal`
- 从floor=n到floor=1：`n-current → n-1 → 1-m → 1-goal`

**Index连续性**：在同一楼层内的航点序列保证index连续（递增或递减）

#### 参数配置

| 参数名                     | 类型   | 默认值                 | 说明                                  |
| -------------------------- | ------ | ---------------------- | ------------------------------------- |
| `use_to_nav_dir`           | string | 见launch文件           | CSV文件所在目录                       |
| `waypoints_csv`            | string | `""`                   | 指定航点CSV（空则自动选最新）         |
| `goalpoints_csv`           | string | `""`                   | 指定目标点CSV（空则自动选最新）       |
| `odom_topic`               | string | `/Odometry`            | 里程计话题名称                        |
| `frame_id`                 | string | `world`                | 发布消息的坐标系                      |
| `goal_topic`               | string | `/goal_point_ext`      | 目标点发布话题                        |
| `viz_marker_topic`         | string | `/goal_search_markers` | 可视化标记话题                        |
| `path_topic`               | string | `/goal_search_path`    | 路径发布话题                          |
| `select_goal_service`      | string | `select_goal`          | 目标选择服务名                        |
| `waypoint_topic`           | string | `/way_point`           | 航点追踪发布话题                      |
| `advance_waypoint_service` | string | `advance_waypoint`     | 航点切换服务名（由local_planner调用） |
| `tracker_check_rate`       | double | `10.0`                 | 追踪检查频率（Hz）                    |
| `goal_sequence_floor_indices` | string[] | `[]`                | 自动目标序列列表（非空即启用，按顺序执行） |
| `goal_sequence_next_delay_sec` | double | `3.0`              | 到达一个目标点后等待 N 秒再发布下一个 |

#### 启动方式

```bash
ros2 launch waypoint_manager waypoint_search.launch.py
```

#### 自动目标序列（按参数顺序发布 goalpoints）

如果你希望节点**自动按顺序导航多个 goalpoint**，直接在参数里填入 `goal_sequence_floor_indices`：

- 列表为空：**不自动发布**（等同“已完成”状态），节点其他功能不受影响
- 列表非空：等待首次收到 `/Odometry` 后发布第一个；每到达一个目标点，等待 `goal_sequence_next_delay_sec` 秒再发布下一个
- 自动运行中如手动调用 `select_goal`：自动序列会暂停；手动目标完成后会继续自动序列

#### 服务调用示例

选择目标点并生成路径：

```bash
# 选择floor=2, index=3的目标点
ros2 service call /waypoint_search/select_goal waypoint_manager/srv/SelectGoal "{floor_index: '2-3'}"

# 选择floor=3, index=1的目标点
ros2 service call /waypoint_search/select_goal waypoint_manager/srv/SelectGoal "{floor_index: '3-1'}"
```

服务响应会包含路径规划的详细信息。

手动设置楼层状态（防止意外情况）：

```bash
# 手动设置机器人当前在 floor=2
ros2 service call /waypoint_search/set_floor_state waypoint_manager/srv/SetFloorState "{floor: 2}"

# 重置到 floor=1
ros2 service call /waypoint_search/set_floor_state waypoint_manager/srv/SetFloorState "{floor: 1}"
```

手动触发航点切换（一般由local_planner自动调用）：

```bash
ros2 service call /waypoint_search/advance_waypoint std_srvs/srv/Trigger "{}"
```

**注意**：正常情况下楼层状态会自动维护，只有在以下情况需要手动设置：
- 机器人位置传感器失效导致楼层追踪失误
- 手动移动机器人跨楼层后重新启动导航
- 调试和测试需求

#### RViz2可视化

1. 添加 `MarkerArray` 显示类型，话题选择 `/goal_search_markers`
   - 红色球体：所有航点
   - 蓝色球体：所有目标点
   - **绿色球体**：当前选中的目标点
   - **黄色球体**：正在追踪的航点（动态更新）
   - 文本标签：floor-index

2. 添加 `Path` 显示类型，话题选择 `/goal_search_path`
   - 显示从当前位置到目标的完整路径

**可视化特色**：黄色高亮球体会动态跟随机器人进度，实时显示当前正在追踪的航点，让你一眼看出机器人的导航状态。

#### 发布的消息

- **目标点**：`waypoint_manager/msg/GoalPoint` @ `/goal_point_ext`
  - 包含：header、floor、index、pose
- **路径**：`nav_msgs/msg/Path` @ `/goal_search_path`
  - 包含从当前位置到目标的所有路径点
- **当前航点**：`geometry_msgs/msg/PointStamped` @ `/way_point`
  - 航点追踪器逐个发布的当前目标航点
  - 下游local_planner节点订阅此话题进行导航

#### 航点追踪功能说明

当通过`select_goal`服务选择目标后，节点会自动：

1. **生成完整路径**：计算从当前位置到目标的所有航点序列
2. **启动追踪器**：开始逐个发布航点到`/way_point`话题
3. **等待到达信号**：由`local_planner`根据`goalThre`判断到达
4. **服务触发切换**：`local_planner`调用`advance_waypoint`服务切换到下一个航点
5. **动态楼层状态管理**：
   - 实时追踪机器人当前所在楼层
   - 当切换到不同楼层的航点时，自动更新楼层状态
   - 支持从任意楼层到任意楼层的路径规划
   - 楼层切换日志会在终端输出

**工作流程示例（从floor=1到floor=2）**：

```
当前楼层：1，选择目标 2-3
  ↓
生成路径：odom -> 1-n -> 1-m -> 2-1 -> 2-n -> goal
  ↓
发布航点1 (odom位置)，楼层状态：1
  ↓
到达 1-m 点，发布航点 (2-1)
  ↓
楼层状态切换：1 → 2（离开1-m，进入2-1）
  ↓
继续追踪 floor=2 的航点
  ↓
所有航点完成，当前楼层：2
```

**跨楼层示例（从floor=2到floor=3）**：

```
当前楼层：2，选择目标 3-2
  ↓
生成路径：odom -> 2-n -> 2-1 -> 1-m -> 1-n -> 3-1 -> 3-n -> goal
  ↓
追踪 2-n...2-1（楼层：2）
  ↓
到达 2-1，发布航点 (1-m)
  ↓
楼层状态切换：2 → 1
  ↓
追踪 1-m...1-n（楼层：1）
  ↓
到达 1-n，发布航点 (3-1)
  ↓
楼层状态切换：1 → 3
  ↓
所有航点完成，当前楼层：3
```

**配置建议**：

- `local_planner` 的 `goalThre`：作为唯一到达阈值（单位m）
  - 高精度定位：可设置为0.2~0.5m
  - 一般情况：0.8~1.5m
  - 定位误差较大：2.0m或更大
- `tracker_check_rate`：建议保持10Hz，平衡响应速度和CPU占用

---

## 消息和服务接口

### 消息类型

#### GoalPoint.msg

```
std_msgs/Header header
int32 floor
int32 index
geometry_msgs/Pose pose
```

### 服务类型

#### RecordPoints.srv

用于记录航点。

```
int32 floor
---
bool success
string message
```

#### SelectGoal.srv

用于选择导航目标点。

```
string floor_index    # 格式："floor-index"，例如 "2-3"
---
bool success
string message
```

#### ListFloorIndex.srv

用于列出所有可用的航点。

```
bool is_goalpoint     # true=列出goalpoints，false=列出waypoints
---
bool success
string message
string[] floor_indices    # 返回的floor-index列表
```

#### SelectPoint.srv

用于选择要编辑的航点。

```
bool is_goalpoint
string floor_index
---
bool success
string message
```

#### SetAxisControl.srv

用于连续控制航点位置。

```
int8 axis        # 0:x, 1:y, 2:z, 3:yaw
float32 value    # 范围：[-1.0, 1.0]
---
bool success
string message
```

#### SetFloorState.srv

用于手动设置机器人当前楼层状态。

```
int32 floor    # 要设置的楼层状态
---
bool success
string message
int32 previous_floor    # 之前的楼层状态
```

**使用场景**：
- 机器人手动移动后重新启动导航
- 楼层追踪失误需要手动纠正
- 调试和测试

---

## 完整工作流程示例

### 场景：记录、编辑、导航一体化

#### 步骤1：记录航点

```bash
# 启动记录节点
ros2 launch waypoint_manager waypoint_manager.launch.py

# 驾驶机器人到各个位置，记录航点
# floor=1（连接层）
ros2 service call /waypoint_manager/record_waypoint waypoint_manager/srv/RecordPoints "{floor: 1}"
# ... 继续记录多个floor=1的航点 ...

# floor=2（实际楼层）
ros2 service call /waypoint_manager/record_waypoint waypoint_manager/srv/RecordPoints "{floor: 2}"
# ... 记录floor=2的航点 ...

# 记录目标点
ros2 service call /waypoint_manager/record_goalpoint waypoint_manager/srv/RecordPoints "{floor: 2}"
```

记录完成后，CSV文件会保存在 `data/` 目录（或配置的输出目录）。

#### 步骤2：编辑航点（可选）

将生成的CSV文件复制到 `project/data/use_to_nav/` 目录：

```bash
cp data/waypoints-01-12-15-30-45.csv ~/nav_ws/project/data/use_to_nav/
cp data/goalpoints-01-12-15-30-45.csv ~/nav_ws/project/data/use_to_nav/
```

启动编辑节点：

```bash
ros2 launch waypoint_manager waypoint_editor.launch.py
```

列出并选择航点进行微调：

```bash
# 列出所有航点
ros2 service call /waypoint_editor/list_floor_index waypoint_manager/srv/ListFloorIndex "{is_goalpoint: false}"

# 选择并编辑
ros2 service call /waypoint_editor/select_point waypoint_manager/srv/SelectPoint "{is_goalpoint: false, floor_index: '1-3'}"

# 微调位置（使用RViz插件或命令行）
ros2 service call /waypoint_editor/set_axis_control waypoint_manager/srv/SetAxisControl "{axis: 0, value: 0.5}"
```

编辑完成后，直接关闭节点，修改会自动保存。

#### 步骤3：基于航点导航

确保CSV文件在 `project/data/use_to_nav/` 目录，启动搜索节点：

```bash
ros2 launch waypoint_manager waypoint_search.launch.py
```

选择目标点：

```bash
ros2 service call /waypoint_search/select_goal waypoint_manager/srv/SelectGoal "{floor_index: '2-3'}"
```

节点会：
1. 发布目标点到 `/goal_point_ext`（下游导航节点可订阅）
2. 发布路径到 `/goal_search_path`（在RViz中可视化）
3. 在RViz中高亮显示选中的目标点

---

## 编译与安装

### 依赖项

- ROS2 Humble 或更新版本
- rclcpp
- nav_msgs
- visualization_msgs
- geometry_msgs
- tf2
- tf2_geometry_msgs

### 编译

```bash
cd ~/nav_ws
colcon build --packages-select waypoint_manager --symlink-install
source install/setup.bash
```

### 验证安装

```bash
# 检查可执行文件
ros2 pkg executables waypoint_manager

# 检查接口
ros2 interface list | grep waypoint_manager
```

---

## 文件结构

```
waypoint_manager/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── waypoint_editor.yaml          # 编辑器配置
├── data/
│   ├── waypoints-*.csv                # 记录的航点（时间戳）
│   ├── goalpoints-*.csv               # 记录的目标点（时间戳）
│   └── use_to_nav/                    # 用于导航的航点（手动放置）
│       ├── waypoints-*.csv
│       └── goalpoints-*.csv
├── launch/
│   ├── waypoint_manager.launch.py     # 记录节点启动文件
│   ├── waypoint_editor.launch.py      # 编辑节点启动文件
│   └── waypoint_search.launch.py      # 搜索节点启动文件
├── msg/
│   └── GoalPoint.msg                  # 目标点消息定义
├── srv/
│   ├── RecordPoints.srv               # 记录服务
│   ├── SelectGoal.srv                 # 目标选择服务
│   ├── ListFloorIndex.srv             # 列出航点服务
│   ├── SelectPoint.srv                # 选择航点服务
│   └── SetAxisControl.srv             # 控制轴服务
└── src/
    ├── waypoint_manager_node.cpp      # 记录节点
    ├── waypoint_editor_node.cpp       # 编辑节点
    └── waypoint_search_node.cpp       # 搜索节点
```

---

## 常见问题

### Q1: CSV文件保存在哪里？

- **waypoint_manager_node**：默认保存在 `~/.ros/waypoints/`，可通过 `output_dir` 参数修改
- **waypoint_editor_node** 和 **waypoint_search_node**：从 `use_to_nav_dir` 参数指定的目录加载

### Q2: 如何选择特定的CSV文件？

在launch文件中设置参数：

```python
"waypoints_csv": "/path/to/specific/waypoints.csv",
"goalpoints_csv": "/path/to/specific/goalpoints.csv",
```

如果不指定（留空），节点会自动选择目录下最新的文件。

### Q3: 编辑后的航点何时保存？

`waypoint_editor_node` 在节点正常退出时（Ctrl+C）自动保存修改到原CSV文件。运行期间不会频繁写盘，以保证实时性。

### Q4: 路径规划失败怎么办？

检查以下几点：
- 是否收到里程计消息（`/Odometry`）
- 目标点的floor是否>=2
- CSV文件中是否存在连接点（floor=G, index=1）
- CSV文件中是否有足够的航点

查看节点日志获取详细错误信息。

### Q5: 可视化标记不显示？

1. 确认RViz订阅的话题名称正确
2. 检查坐标系（Fixed Frame）是否匹配
3. 确认节点已启动并正常运行

### Q6: 如何支持更多楼层？

只需在记录时使用不同的floor值即可。路径规划逻辑会自动适应floor编号，只要遵循：
- floor=1为连接层
- floor>=2为实际楼层
- 每个楼层的连接点index=1