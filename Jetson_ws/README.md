# TECHX Jetson Vision 说明

本仓库运行在 **Jetson Orin NX**，负责相机采集、目标识别、深度计算、目标质量判断、调试记录和 UDP V2 下发。Jetson 只输出 `camera_link` 下的视觉坐标；`robot_base / arm1_base / arm2_base` 坐标转换由 GMK 仓库完成。

> 📘 **完整现场手册**（比赛流程、四坐标系、手眼标定全流程、深度精度校准、CENTER/GRASP、调试与备份）见
> [`docs/FIELD_OPERATIONS.md`](docs/FIELD_OPERATIONS.md)。GMK 侧见 GMK 仓库 `docs/FIELD_OPERATIONS.md`。

## 比赛视觉流程

```text
1. R2 导航到武器头区，Jetson 识别武器头，GMK/决策用视觉精调底盘，机械臂1抓取武器头。
2. R2 到 R1 拼接武器头，R1 拼接成功后亮灯带，Jetson 识别灯带/拼接成功事件。
3. R2 进入梅林，Jetson 识别真 R2 KFS，视觉精调后机械臂2抓取 KFS。
4. R2 到三区武馆区，Jetson 识别 R1 二维码，R2 边走边对齐进入 R1 抬升机构。
5. R1 抬升 R2，R2 将 KFS 放入九宫格。
```

## 目标类别

```text
0-5     KFS
100-102 武器头
150-159 灯带颜色 + 拼接成功事件（颜色可配置，橙色为主用；152=assembly_success）
200     二维码
```

灯带颜色在 `config.json` 的 `assembly_light.colors` 配置（橙=150 主用，红/蓝预留，可加其它色用于决策）。

## 固定网络

```text
Jetson = 192.168.10.101
GMK    = 192.168.10.100
UDP    = 12345
```

`config.json` 中 UDP 默认发往 GMK：

```json
"udp": {
  "local_ip": "192.168.10.101",
  "target_ip": "192.168.10.100",
  "target_port": 12345
}
```

## 启动（三种方式 + 开机自启）

| 场景 | 命令 |
|---|---|
| **Windows 验模型**（接 335L 出深度） | `set TECHX_UDP_OPTIONAL=1` 后 `start_windows.bat`（同一份 config.json，单机不报 UDP 错） |
| **Jetson 带 UI**（调试，需显示器） | `TECHX_GUI=1 bash start_jetson.sh` |
| **Jetson 无 UI**（比赛/无显示器） | `bash start_jetson.sh` 或 `bash scripts/field_start_jetson.sh` |

分阶段：`TECHX_STAGE=all|head|kfs|assembly|qr bash scripts/field_start_jetson.sh`。启动脚本会检查 Python、cv2、numpy、pyorbbecsdk、模型、网络、GMK 连通性，并打印电源/性能预检。

**网络一次性设好（持久，重启不丢）：**
```bash
sudo bash scripts/setup_field_network.sh jetson   # 建持久 NetworkManager 连接 = 192.168.10.101
```

**开机自启（无显示屏部署，带开关）：**
```bash
sudo bash scripts/install_vision_service.sh        # 装一次 systemd 服务
# 开关在 scripts/autostart.env：TECHX_AUTOSTART=true 上电自动跑 / =false 正常手动（默认）
```
详见 [`docs/FIELD_OPERATIONS.md`](docs/FIELD_OPERATIONS.md) §6。

## 棋盘格手眼标定

推荐赛前用 A4 棋盘格标定 `T_robot_camera`：

```bash
TECHX_CHESSBOARD_COLS=9 \
TECHX_CHESSBOARD_ROWS=6 \
TECHX_CHESSBOARD_SQUARE_M=0.025 \
TECHX_CALIB_SAMPLES=9 \
TECHX_CALIB_DIR=runs/calib_robot_camera \
bash scripts/calibrate_chessboard_jetson.sh
```

输出：

```text
runs/calib_robot_camera/robot_camera_points.csv
runs/calib_robot_camera/gmk_robot_camera.yaml
runs/calib_robot_camera/gmk_robot_camera_report.json
runs/calib_robot_camera/gmk_robot_camera_residuals.csv
runs/calib_robot_camera/overlays/
```

说明：Jetson 能自动识别棋盘格并得到 `camera_link` 坐标，但每个采样点仍需要输入该棋盘格参考点在 `robot_base` 下的真实坐标。

## 调试记录

默认开启 recorder：

```bash
TECHX_DEBUG_RECORDER=1
TECHX_DEBUG_DIR=runs/field_001
TECHX_DEBUG_FRAME_EVERY=10
```

输出：

```text
runs/field_001/perf.csv
runs/field_001/tracks.csv
runs/field_001/targets.csv
runs/field_001/depth.csv        # 含 z_raw_m(修正前) 与 z(修正后)
runs/field_001/overlay_frames/
```

深度精度校准（`z_corrected_mm = z_raw_mm * scale_corr + offset_mm`，可在 `config.json` 的 `depth` 段配置）：

```bash
# 已知真值距离下统计深度偏差并给出 offset_mm/scale_corr 建议
python3 tools/depth_accuracy_report.py --depth-csv runs/field_001/depth.csv --truth-m 0.50 --class-id 2
```

灯带调试：

```bash
TECHX_STAGE=assembly TECHX_LIGHT_DEBUG_DIR=runs/field_001/light bash scripts/field_start_jetson.sh
```

## 决策使用规则

```text
搜索/居中：require_control_xyz=false，使用 u/v 或 align_err_x/y。
抓取阶段：require_control_xyz=true，必须使用 GMK 输出的 control_x/y/z。
灯带/二维码：主要做事件或图像对齐，不作为机械臂抓取坐标。
```

## 现场验收

```text
1. Jetson 能启动并向 GMK 发送 UDP。
2. recorder 能生成 perf/tracks/targets/depth CSV。
3. 棋盘格标定能生成 gmk_robot_camera.yaml。
4. GMK /techx/vision/selected 能输出有效 control_x/y/z。
5. 武器头、KFS、灯带、二维码四个阶段分别实测通过。
```
