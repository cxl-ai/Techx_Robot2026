# UDP V2 视觉帧下发协议

Jetson 视觉端负责识别、深度测距和 `camera_link` 相机坐标系下的 3D 解算。GMK 端负责把相机坐标变换到机器人本体、机械臂1、机械臂2等控制坐标系。

关键原则：

```text
Jetson 下发的 x/y/z 永远是 camera_link 相机坐标。
不要在 Jetson 里写死机械臂1、机械臂2、机器人本体外参。
GMK 的 techx_vision_bridge 根据外参参数完成坐标转换。
决策包不直接解析 UDP；它订阅 /techx/vision/frame，或使用 /techx/vision/request + /techx/vision/selected。
```

---

## 1. V2 包发送规则

`UdpSender.send_many(targets, timestamp=frame_ts)` 每个新推理结果发送一个 V2 包：

```text
count=0
  本帧没有 fresh 目标，但 Jetson 视觉链路在线。

count>0
  本帧有 fresh 目标。每个目标包含类别、置信度、颜色、像素中心和 camera_link 坐标。

目标 z=0
  识别到了目标，但本帧没有有效 3D 深度。GMK 只能使用类别/置信度/u/v，不能使用 x/y/z 控制。
```

旧 29 字节包默认不发送。只有设置环境变量才会开启：

```bash
TECHX_SEND_LEGACY_UDP=1
```

比赛主链路应使用 V2，不建议使用旧包。

---

## 2. V2 包格式

字节序：小端序 little-endian。

Header：

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 0x55AB
    uint8_t  version;    // 2
    uint8_t  flags;      // 当前为 0
    uint32_t seq;        // uint32 递增，溢出回绕
    double   timestamp;  // 相机帧时间戳，秒；count=0 时仍有效
    uint8_t  count;      // target 数量，0~16
} VisionV2Header;
```

Python struct：

```python
"<H B B I d B"
```

Header 长度：17 字节。

Target Entry：

```c
typedef struct __attribute__((packed)) {
    uint8_t track_id;    // 追踪 ID，低 8 位
    uint8_t class_id;    // 全局目标类别 ID，见第 3 节
    uint8_t color;       // 0=unknown, 1=red, 2=blue
    float   confidence;  // 置信度，0.0~1.0
    float   u;           // 像素中心 u
    float   v;           // 像素中心 v
    float   x;           // camera_link X，单位 m；z=0 时忽略
    float   y;           // camera_link Y，单位 m；z=0 时忽略
    float   z;           // camera_link Z，单位 m；z>0 表示 3D 有效
} VisionV2Target;
```

Python struct：

```python
"<B B B f 2f 3f"
```

单个 Target 长度：27 字节。

CRC：

```text
crc16_ccitt(header + target[count])
```

完整长度：

```text
17 + count * 27 + 2
```

---

## 3. 全局 class_id 分配

`class_id` 是整个比赛视觉系统的**全局语义编号**，不是某个模型内部的本地类别编号。换模型时必须用 `class_id_map` 或 `class_id_offset` 映射回下表，保证 GMK 和决策包不需要跟着模型类别顺序变化。

### 3.1 武器头 Head，机械臂1使用

| class_id | class_name | 中文含义 | GMK target_type | 推荐控制坐标系 |
|---:|---|---|---:|---|
| 100 | `weapon_head_fist` | 拳头 | 1 | arm1_base |
| 101 | `weapon_head_palm` | 掌 | 1 | arm1_base |
| 102 | `weapon_head_spear` | 矛头 | 1 | arm1_base |

保留：`103~149` 给后续新增武器头或对接特征。

### 3.2 KFS，机械臂2使用

| class_id | class_name | color | 中文含义 | GMK target_type | 推荐控制坐标系 |
|---:|---|---:|---|---:|---|
| 0 | `kfs_red_r1` | 1 | 红方 R1 KFS | 2 | arm2_base |
| 1 | `kfs_red_r2_fake` | 1 | 红方 R2 假 KFS | 2 | arm2_base |
| 2 | `kfs_red_r2_true` | 1 | 红方 R2 真 KFS | 2 | arm2_base |
| 3 | `kfs_blue_r1` | 2 | 蓝方 R1 KFS | 2 | arm2_base |
| 4 | `kfs_blue_r2_fake` | 2 | 蓝方 R2 假 KFS | 2 | arm2_base |
| 5 | `kfs_blue_r2_true` | 2 | 蓝方 R2 真 KFS | 2 | arm2_base |

决策包在梅花林阶段一般只选择 `class_id=2` 或 `class_id=5`，具体取决于当前己方/目标颜色策略。不要只用 `target_type=2`，因为那会把 R1、R2 假、R2 真都混在一起。

### 3.3 QR，机器人本体使用

| class_id | class_name | 中文含义 | GMK target_type | 推荐控制坐标系 |
|---:|---|---|---:|---|
| 200 | `qr_code` | 二维码 | 3 | robot_base |

当前 V2 只传二维码中心、置信度、距离和坐标，不传二维码字符串内容。如果后续要读取二维码内容或任务编号，需要 V3/TLV 或额外字段。

---

## 4. Jetson 配置原则

### 4.1 KFS 模型

KFS 模型必须输出六类，或通过 `class_id_map` 映射成六类：

```json
"class_id_map": {
  "0": 0,
  "1": 1,
  "2": 2,
  "3": 3,
  "4": 4,
  "5": 5,
  "kfs_red_r1": 0,
  "kfs_red_r2_fake": 1,
  "kfs_red_r2_true": 2,
  "kfs_blue_r1": 3,
  "kfs_blue_r2_fake": 4,
  "kfs_blue_r2_true": 5
}
```

### 4.2 武器头模型

武器头模型必须输出三类，或通过 `class_id_map` 映射成三类：

```json
"class_id_map": {
  "0": 100,
  "1": 101,
  "2": 102,
  "weapon_head_fist": 100,
  "weapon_head_palm": 101,
  "weapon_head_spear": 102
}
```

### 4.3 QR 检测

QR 由 `CodeMarkerDetector` 输出：

```text
class_id = 200
class_name = qr_code
color = 0 unknown
```

---

## 5. 坐标系与标定职责

Jetson 端必须保证：

```text
1. RGB-D 相机内参正确。
2. RGB 图和深度图已经对齐，或者深度查询逻辑与彩色框一致。
3. x/y/z 是 camera_link 坐标，单位 m。
4. QR 对齐可使用 u/v；QR 距离可使用 z。
```

GMK 端负责：

```text
T_robot_camera_xyz_rpy
T_arm1_robot_xyz_rpy
T_arm2_robot_xyz_rpy
```

也就是：

```text
point_robot = T_robot_camera * point_camera
point_arm1  = T_arm1_robot  * point_robot
point_arm2  = T_arm2_robot  * point_robot
```

如果相机固定在机器人前方，这主要是相机到机器人本体/机械臂基座的外参标定，不建议把这些外参写死在 Jetson 仓库里。

---

## 6. 新增识别模型/新物体的扩展流程

### 6.1 Jetson 侧

新增模型时，不需要改 UDP 协议。只需要：

```text
1. 把模型放到 models/<new_model_folder>/best.pt、best.onnx 或 best.engine。
2. 在 config.json 的 models[] 增加一项。
3. 给新模型分配不冲突的 class_id 范围。
4. 用 class_id_offset 或 class_id_map 把模型本地类别映射到全局 class_id。
```

示例：新增一个 docking_marker 模型，映射到 class_id 150~159：

```json
{
  "name": "Docking marker detection",
  "folder": "dock_marker_v1",
  "enabled": true,
  "class_id_offset": 150,
  "class_confs": {
    "dock_marker": 0.35
  }
}
```

如果模型类别顺序不稳定，建议使用 `class_id_map` 精确映射。

### 6.2 GMK 侧

GMK 不需要改 C++。只需要在 `vision_bridge.yaml` 的 `class_rules` 增加规则：

```yaml
class_rules:
  - "0-5:2:2:4:0.0"       # KFS -> arm2_base
  - "100-102:1:1:3:0.0"   # Head -> arm1_base
  - "200:3:3:2:0.0"       # QR -> robot_base
  - "150-159:10:10:2:0.0" # new custom object -> robot_base
```

规则格式：

```text
"class_or_range:zone_id:target_type:control_frame:priority_bias"
```

`control_frame` 取值：

```text
1 camera_link
2 robot_base
3 arm1_base
4 arm2_base
```

---

## 7. 决策包应该使用哪些数据

决策包不需要向 Jetson 请求数据。Jetson 每个视觉周期主动下发完整帧，GMK 发布 `/techx/vision/frame`。

决策包可以直接订阅 `/techx/vision/frame`，也可以发布 `/techx/vision/request` 后订阅 `/techx/vision/selected`。无论哪种方式，最终选择目标都必须看全局 `class_id`。

建议每个阶段使用的数据：

| 阶段 | 过滤条件 | 主要控制数据 | 备注 |
|---|---|---|---|
| Head 获取 | `target_type=1` 且 `class_id=100/101/102` | `control_x/y/z` | control_frame=arm1_base |
| Head/R1 对接 | `target_type=3` 或后续自定义 marker | `align_err_x/y`, `control_z` | 若要确认拼接成功，建议新增专用 class_id |
| KFS 梅花林 | `target_type=2` 且 `class_id=2/5` | `control_x/y/z`, `priority` | control_frame=arm2_base |
| QR 对齐靠近 | `target_type=3`, `class_id=200` | `align_err_x/y`, `control_z` | control_frame=robot_base |

多目标选择不要按接收顺序，建议按：

```text
任务阶段匹配 > class_id/color 匹配 > valid_control_xyz > confidence > priority > 距离/中心误差
```
