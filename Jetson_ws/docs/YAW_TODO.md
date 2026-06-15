# 武器头 yaw(朝向)输出 —— 后续要加时按这个做（铺垫，未实现）

目标：武器头朝向会变时，从 **seg 掩膜**算出图像平面角度(yaw)，经 **UDP V2 传到 GMK**。
原则：**追加字段、默认 0、用 flags 位门控**，不破坏现有 27 字节目标帧（老 GMK 仍可解 27 字节帧）。

> 前提：先把 git 理顺（本地重新 clone 一份干净的，和队友的流水线版本合并），再加。
> 因为 `types/tracker/yolo` 正在被改，让**改流水线的人**加下面的 Jetson 部分最干净。

## 协议约定（一处）
- V2 头里的 `flags` 字节：**bit0 = 1** 表示“每个 27 字节目标后面追加 4 字节 float yaw（角度，单位度）”。
- GMK 解码时：读 `flags`，bit0=1 → 每个目标 stride = 27+4，读完 27 字节结构体再读 4 字节 yaw；bit0=0 → 老行为(27 字节，yaw=0)。

## Jetson 改 5 处（都是追加，默认 0，不影响现有逻辑）
1. `interface/types.py`：给 `Detection`、`Track`、`Target3D` 各加 `angle: float = 0.0`。
2. `detection/yolo.py`（顶部 `import cv2`），`_parse_results` 里循环每个目标时算角度：
   ```python
   masks_obj = getattr(results[0], "masks", None)
   # ...循环内：
   angle = 0.0
   if masks_obj is not None:
       try:
           poly = masks_obj.xy[i]
           if poly is not None and len(poly) >= 3:
               angle = float(cv2.minAreaRect(np.asarray(poly, dtype=np.float32))[2])
       except Exception:
           angle = 0.0
   raw.append(Detection(box=..., cls_id=..., cls_name=..., conf=..., angle=angle))
   ```
3. `detection/tracker.py` `update()` 两处：新建 Track 的构造里加 `angle=det.angle`；匹配到旧轨迹那支加 `tr.angle = float(det.angle)`。
4. `pipeline/solver.py` `solve()` 构造 `Target3D(...)` 时加 `angle=track.angle`（`common` 字典里加一项即可）。
5. `communication/udp.py` `_send_v2`：
   - 头里 `flags` 传 `1`（原来是 0）。
   - 每个目标 `payload += struct.pack(_V2_TARGET_FMT, ...)` 之后追加 `payload += struct.pack("<f", float(getattr(target, "angle", 0.0)))`。
   - 帧长自检改成 `expected = _V2_HEADER_SIZE + count*(_V2_TARGET_SIZE + 4) + 2`。
   - 建议加环境开关 `TECHX_SEND_YAW`（默认 1），关掉时 flags=0、不追加，退回老行为。

## GMK 改 2 处
1. `msg/VisionObject.msg`：加一行 `float32 yaw`（放在 `priority` 附近即可）。
2. `src/vision_frame_bridge_node.cpp` `decode_v2`：
   ```cpp
   const bool has_yaw = (h.flags & 0x01) != 0;
   const size_t tsize = sizeof(V2Target) + (has_yaw ? sizeof(float) : 0);
   const size_t expected = sizeof(V2Header) + static_cast<size_t>(h.count) * tsize + sizeof(uint16_t);
   // ...循环内：memcpy(&raw, data+off, sizeof(V2Target));
   float yaw = 0.0f;
   if (has_yaw) std::memcpy(&yaw, data + off + sizeof(V2Target), sizeof(float));
   off += tsize;
   t.yaw = yaw;   // DecodedTarget 加 float yaw{0.0f};
   ```
   并在 `to_object` 里 `msg.yaw = t.yaw;`。`V2Target` 结构体**保持 27 字节不变**（yaw 是结构体外追加的，不动 static_assert）。

## 验证
- 改完跑 `tools` 下的 UDP 契约自测（扩展成带 yaw）确认两端字节对齐。
- GMK `colcon build` 必须重编。
- 现场：摆一个转了角度的武器头，看 GMK 收到的 `yaw` 跟着变。

## 注意
- `cv2.minAreaRect` 的角度约定随 OpenCV 版本不同（±90° 区间不一样），现场**示教一次基准角**即可，别纠结绝对值。
- 这是图像平面内的旋转 = 夹爪绕光轴需要转的角度（前视/俯视时）。
