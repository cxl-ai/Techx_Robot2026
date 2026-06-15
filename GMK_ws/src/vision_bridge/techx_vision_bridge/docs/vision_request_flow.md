# techx_vision_bridge 单节点请求流程

当前 GMK 视觉功能包正常运行时只有一个节点：

```text
vision_bridge_node
```

这个节点同时负责两件事：

```text
1. 接收 Jetson UDP V2，发布完整视觉帧 /techx/vision/frame
2. 接收 /techx/vision/request，发布请求对应结果 /techx/vision/selected
```

所以使用者不需要启动 `vision_selector_node`，也不需要理解两个节点之间的内部转发。

---

## 话题

| 话题 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/techx/vision/frame` | `VisionFrame` | bridge -> 其他包 | 完整最新视觉帧，包含所有目标 |
| `/techx/vision/request` | `VisionRequest` | 决策/上层包 -> bridge | 告诉 bridge 当前想要哪类目标 |
| `/techx/vision/selected` | `VisionSelection` | bridge -> 决策/上层包 | 根据 request 从最新 frame 中选出的最佳目标 |

`/frame` 是完整事实；`/selected` 是按请求筛出来的便捷结果。

---

## 典型流程

```text
Jetson 持续发 UDP V2
        ↓
vision_bridge_node 持续发布 /techx/vision/frame
        ↓
决策包发布 /techx/vision/request，例如 class_id=100
        ↓
vision_bridge_node 从最新 frame 里筛 class_id=100
        ↓
vision_bridge_node 发布 /techx/vision/selected
        ↓
决策包读取 selected.target，生成底盘/机械臂命令
```

---

## 请求示例

### 请求拳头武器头

```text
VisionRequest:
  request_seq: 1
  target_type: 1
  zone_id: 1
  use_class_id: true
  class_id: 100
  require_control_xyz: true
  min_confidence: 0.4
  max_frame_age_sec: 0.2
```

底盘使用 `selected.target.robot_x/y/z`，机械臂1使用 `selected.target.arm1_x/y/z`。

### 请求红方 R2 真 KFS

```text
VisionRequest:
  request_seq: 2
  target_type: 2
  zone_id: 2
  use_class_id: true
  class_id: 2
  require_control_xyz: true
  min_confidence: 0.4
  max_frame_age_sec: 0.2
```

底盘使用 `selected.target.robot_x/y/z`，机械臂2使用 `selected.target.arm2_x/y/z`。

### 请求二维码

```text
VisionRequest:
  request_seq: 3
  target_type: 3
  zone_id: 3
  use_class_id: true
  class_id: 200
  require_control_xyz: false
  min_confidence: 0.3
  max_frame_age_sec: 0.2
```

二维码水平对齐可先用 `selected.target.align_err_x/y`。如果要靠近距离，必须确认三维坐标有效。

---

## `VisionSelection.status`

```text
0 STATUS_OK
1 STATUS_NO_REQUEST
2 STATUS_NO_FRAME
3 STATUS_NO_MATCH
4 STATUS_FRAME_STALE
5 STATUS_REQUEST_STALE
```

决策包只能在下面条件同时满足时使用数据：

```text
has_match == true
status == STATUS_OK
```

如果要三维控制，还必须满足：

```text
selected.target.valid_control_xyz == true
```

---

## 注意

`/request` 不会命令 Jetson 只识别某个目标。Jetson 仍持续识别并发送完整视觉帧。`/request` 只是在 GMK 端从最新视觉帧里筛选目标。
