# TECHx_vision UDP 下发数据协议 v2.0

## 帧格式 (29 字节, 小端序)

```
┌────────┬──────────┬───────────┬──────────┬──────────────────────┬───────┐
│ Magic  │ Sequence │ Timestamp │ Track ID │  Xc    Yc    Zc      │ CRC16 │
│ uint16 │  uint32  │  float64  │  uint8   │ f32    f32    f32    │uint16 │
│ 0x55AA │  自增    │   秒      │  追踪ID  │  米    米    米      │ 校验  │
│ [0:2]  │  [2:6]   │  [6:14]   │  [14]    │ [15:27]             │[27:29]│
└────────┴──────────┴───────────┴──────────┴──────────────────────┴───────┘
```

| 偏移 | 字节 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 2 | uint16 | magic | 帧头魔数 `0x55AA` |
| 2 | 4 | uint32 | sequence | 序列号 (自增, 2³² 绕回) |
| 6 | 8 | float64 | timestamp | 相机曝光时间戳 (Unix 秒, Chrony 同步) |
| 14 | 1 | uint8 | track_id | 追踪 ID (0–255, 0=未追踪) |
| 15 | 4 | float32 | Xc | 相机坐标系 X (米, 右为正) |
| 19 | 4 | float32 | Yc | 相机坐标系 Y (米, 下为正) |
| 23 | 4 | float32 | Zc | 相机坐标系 Z (米, 前为正) |
| 27 | 2 | uint16 | crc16 | CCITT-CRC16 校验 (前 27 字节) |

Python struct: `'<H I d B 3f H'` = 29 bytes

## 发送行为

| 特性 | 说明 |
|------|------|
| 触发条件 | 检测到目标 且 深度有效 (Zc > 0) |
| 频率 | 每帧发送 (通常 3–50 FPS, 取决于推理速度) |
| 优先级 | 最近目标优先 (Zc 最小) |
| 容错 | 非阻塞 socket, sendto 失败静默丢弃 |

## CRC16-CCITT 校验

- 多项式: `0x1021` (x^16 + x^12 + x^5 + 1)
- 初始值: `0xFFFF`
- 校验范围: 前 27 字节
- 与 STM32 HAL_CRC 完全一致

### GMK 端 C 实现

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
```

### GMK 端接收示例 (Python)

```python
import socket, struct

FRAME_FMT = '<H I d B 3f H'
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 12345))

while True:
    data, addr = sock.recvfrom(64)
    if len(data) != 29: continue
    # CRC16 校验
    crc_calc = crc16_ccitt(data[:27])
    magic, seq, ts, tid, xc, yc, zc, crc = struct.unpack(FRAME_FMT, data)
    if magic != 0x55AA or crc != crc_calc: continue
    # ts 可直接用作 ROS msg.header.stamp
    print(f"[#{seq}] Track{tid}: ({xc:.4f}, {yc:.4f}, {zc:.4f}) @ {ts:.6f}s")
```

## 坐标系统

```
        Yc (下)
         ↑
         │
    ─────┼─────→ Xc (右)
         │
         ↓
        Zc (前/深度)
```

- **相机坐标系 (Xc, Yc, Zc)**: 右手坐标系，原点在相机光心
- 若启用手眼标定，UDP 发送的仍是相机坐标 (GMK 自行做坐标变换)
- Zc > 0 表示有效深度，Zc ≤ 0 表示无效

## 时间戳说明

- `timestamp` 为相机硬件曝光时刻的系统时间 (Unix 秒, float64)
- 通过 Chrony 与 GMK 主控同步 (通常 offset < 5ms)
- GMK 可直接将 timestamp 用作 ROS `msg.header.stamp` 进行 TF2 查询
- 时间戳精度: 毫秒级 (Orbbec SDK `get_system_timestamp()` 提供)

## 监控工具

```bash
# 实时监听 UDP 数据
python tools/kfs_monitor.py --port 12345

# 保存到 CSV
python tools/kfs_monitor.py --port 12345 --save output.csv

# JSON 格式输出
python tools/kfs_monitor.py --port 12345 --json
```
