# Waypoint Search 模块

遵循 Linus 哲学的简单工程化设计：
- 数据结构就是数据，逻辑就是函数
- 每个模块独立且职责单一
- 易于测试和复用

## 模块列表

### `waypoint_search_types.hpp`
基础数据类型和辅助函数
- `CsvPoint`: CSV 文件中的点数据
- `WaypointWithMeta`: 带元数据的航点
- `TrackerState`: 航点追踪器状态

### `csv_loader.hpp`
CSV 文件加载器
- `loadWaypointsFromCsv()`: 加载航点
- `loadGoalpointsFromCsv()`: 加载目标点

### `path_planner.hpp`
路径规划
- `computePathToGoal()`: 计算到目标的路径（带楼层切换逻辑）

### `waypoint_tracker.hpp`
航点追踪器
- `hasReachedWaypoint()`: 判断是否到达航点
- `updateTrackerState()`: 更新追踪状态
- `getCurrentWaypoint()`: 获取当前航点

### `visualization_helper.hpp`
RViz 可视化辅助
- `makeFullMarkerArray()`: 创建完整可视化
- `makeWaypointMessage()`: 创建航点消息

### `polygon_reprojector.hpp` 
多边形障碍物检测与航点外推
- `PolygonBuildParams`: 多边形构建参数
- `ConvexPolygon`: 多边形结构（带凹凸性分析）
- `buildConvexPolygonsFromTerrain()`: 从地形点云构建障碍物多边形
- `reprojectPointOutsidePolygons()`: 将落入障碍物的航点外推到安全位置

**特性**：
- 基于 far_planner 思路：凹凸性分析 + 墙体过滤
- 外推策略：沿"航点→机器人"连线在边界处外推
- 迭代外推：持续检测直到航点切换

## 使用示例

```cpp
// 在 waypoint_search_node.cpp 中
#include "waypoint_search/polygon_reprojector.hpp"

using namespace polygon_reprojector;

// 构建多边形
PolygonBuildParams params;
params.grid_range = 5.0;
params.grid_resolution = 0.05;
// ... 设置其他参数
auto polys = buildConvexPolygonsFromTerrain(center, cloud, params, warned);

// 外推航点
ReprojectionParams reproject_params;
reproject_params.free_radius = 0.1;
bool adjusted = reprojectPointOutsidePolygons(waypoint, robot_pos, polys, reproject_params);
```
