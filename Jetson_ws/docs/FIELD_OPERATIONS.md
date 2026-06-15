# TECHX 现场作业手册（视觉 + GMK）

本手册覆盖比赛流程、两仓库分工、四个坐标系、手眼标定、深度精度校准、启动命令、
CENTER/GRASP 用法、现场调试与备份。Jetson 端命令在 Jetson(Linux/L4T)上运行，
GMK 端命令在 GMK(Linux/ROS2)上运行。Windows 只能做静态自检。

---

## 1. 两仓库分工

| 仓库 | 角色 | 运行平台 |
|---|---|---|
| `TECHX_RobotCode_vision`（本仓库） | 相机识别 → 输出 **camera_link** 坐标 → UDP V2 下发 | Jetson + Gemini 335L |
| `TECHX_Robocon_GMK_Rx` | 收 UDP V2 → 外参变换到机体/机械臂坐标系 → guarded 话题给决策包 | GMK(ROS2 Humble/Foxy) |

**Jetson 只产出 camera_link 坐标。** robot_base / arm1_base / arm2_base 一律由 GMK 在收到 UDP 后变换得到。

## 2. 四个坐标系

| frame_id | 坐标系 | 谁产生 | 用途 |
|---|---|---|---|
| 1 | `camera_link` | Jetson（相机光心） | 灯带事件；所有 3D 的源 |
| 2 | `robot_base` | GMK（`T_robot_camera`） | 底盘定位、二维码对齐 |
| 3 | `arm1_base` | GMK（`T_arm1_robot`） | 机械臂1抓武器头 |
| 4 | `arm2_base` | GMK（`T_arm2_robot`） | 机械臂2抓 KFS |

变换链：`camera_link --T_robot_camera--> robot_base --T_arm1_robot/T_arm2_robot--> arm*_base`。

class_id → 用途/坐标系（GMK `class_rules`）：

| class_id | 物体 | 用途 | 控制坐标系 |
|---|---|---|---|
| 0–5 | KFS（真/假/红蓝） | 抓取 | arm2_base |
| 100–102 | 武器头（拳/掌/矛） | 抓取 | arm1_base |
| 200 | 二维码 | 对齐（align_err，不要 3D） | robot_base |
| 150–152 | 灯带红/蓝/拼接成功 | 事件触发（不走抓取闸） | camera_link |

## 3. 比赛流程（精简）

R2 全自动：导航到武器头区(±5cm) → Jetson 识别武器头 → 视觉精调底盘进入抓取区 →
GMK 把武器头坐标转 arm1_base → 决策包确认 `valid_control_xyz=true` → 机械臂1 抓 →
到 R1 拼接 → R1 灯带亮 → R2 识别灯带颜色/`assembly_success` → 进梅林区 →
识别 KFS(真/假/红蓝) → 视觉精调 → GMK 转真 R2 KFS 到 arm2_base → 机械臂2 抓真 KFS →
到三区武馆 → 识别 R1 二维码 → 用 `align_err_x/y` 边走边对齐 → 进 R1 抬升 → R2 放 KFS 入九宫格。

## 4. 手眼标定流程（T_robot_camera）

> 目标：求 camera_link → robot_base 的刚体变换。±1cm 抓取要求 RMSE 尽量 < 10mm。

### 4.1 准备
- A4 棋盘格一张，量准方格边长（米）和内角点行列数（例：9×6 内角点、0.025m）。
- 机器人静止，相机能稳定看到棋盘格，棋盘格在 robot_base 下的参考点位置可人工测量。

### 4.2 采集 + 估计（一条龙脚本）
```bash
cd TECHX_RobotCode_vision
# 阈值已收紧到 rmse<=0.010m, point<=0.020m；早期联调可放宽：
#   TECHX_CALIB_MAX_RMSE=0.020 TECHX_CALIB_MAX_POINT_ERROR=0.040 \
TECHX_CHESSBOARD_COLS=9 TECHX_CHESSBOARD_ROWS=6 TECHX_CHESSBOARD_SQUARE_M=0.025 \
  bash scripts/calibrate_chessboard_jetson.sh
```
脚本会：打开相机 → 检测棋盘格(子像素角点 + solvePnP) → 自动拒绝模糊/贴边/深度差的样本 →
每个被接受的样本提示你输入“该参考点在 robot_base 下的 X Y Z(米)” → 存点对 + overlay 图 →
最后 `export_handeye_yaml.py` 估计外参并产出：
- `runs/calib_robot_camera/gmk_robot_camera.yaml`（给 GMK）
- `..._report.json`（含 `accepted / rmse_m / max_error_m`）
- `..._residuals.csv`（逐点残差）
- `overlays/`（每个采样帧的可视化）

也可手动分两步（采集脚本 + `tools/export_handeye_yaml.py`），见各脚本头部注释。

### 4.3 标定后如何确保“准”（必须做）
1. **看 report.json**：`accepted` 必须为 `true`；`rmse_m` 越小越好，±1cm 目标建议 < 0.010。
   - 标定失败时导出的 YAML 里 `*_calibrated` 一定是 `false`（即使加了 `--allow-poor-fit`），
     GMK 不会放行抓取坐标——这是有意的安全设计。
2. **看 residuals.csv**：找 `err_norm` 异常大的点（通常是某个采样深度差或人工测量错），删掉重采。
3. **点要分散**：≥6 点、覆盖近/远/左/右/高/低，不要共面。`export` 对退化几何会报错。
4. **复验**：标定后再取 2–3 个“没参与标定”的新点，人工测 robot_base 真值，对比 GMK 输出，
   误差应与 RMSE 同量级。
5. **深度自检**：采集时脚本记录了 `depth_pnp_delta_m`（深度中位数 vs solvePnP 的 Z），
   普遍偏大说明深度有系统性偏差，先做第 5 节深度校准再标手眼。

### 4.4 机械臂外参（T_arm1_robot / T_arm2_robot）
这是 **机械** 标定（robot_base → arm*_base），不经过相机：测量同一组点在 robot_base 和
arm*_base 下的坐标，用 `tools/export_handeye_yaml.py --name T_arm1_robot`（或 arm2）估计。
未做之前对应 `*_calibrated=false`，机械臂抓取请求会得到 NO_MATCH（安全）。

### 4.5 把标定写入 GMK
在 GMK 上（见 GMK 手册 `docs/FIELD_OPERATIONS.md`）：
```bash
python3 src/techx_vision_bridge/tools/apply_calibration_yaml.py --snippet /path/gmk_robot_camera.yaml
python3 src/techx_vision_bridge/tools/check_vision_bridge_config.py   # 必须 OK
```
`apply` 会把同一外参写进 `vision_bridge_node` 和 `calibration_guard_node` 两处并打开 flag；
`check` 会校验两处逐值一致、且 `calibrated=true` 时外参非全零。

## 5. 深度精度校准（±1cm 的关键）

深度修正模型（`pipeline/solver.py`）：`z_corrected_mm = z_raw_mm * scale_corr + offset_mm`。
两个参数现在可在 `config.json` 的 `depth` 段配置：
```json
"depth": { "offset_mm": -50.0, "scale_corr": 1.0, "min_mm": 200.0, "max_mm": 8000.0 }
```
启动日志会打印当前 `offset/scale/范围`；`depth.csv` 同时记录 `z_raw_m`(修正前) 与 `z`(修正后)。

### 校准步骤
1. 把一个目标放在**已知距离**（例 0.50m），开录制跑一会：
   ```bash
   TECHX_DEBUG_RECORDER=1 TECHX_DEBUG_DIR=runs/d050 TECHX_STAGE=kfs \
     bash scripts/field_start_jetson.sh
   ```
2. 用工具算偏差并得到推荐值：
   ```bash
   # 单距离（只修 offset）
   python3 tools/depth_accuracy_report.py --depth-csv runs/d050/depth.csv --truth-m 0.50 --class-id 2
   # 多距离（同时修 scale 和 offset，建议 0.4/0.8/1.2m）
   python3 tools/depth_accuracy_report.py \
     --sample 0.40=runs/d040/depth.csv --sample 0.80=runs/d080/depth.csv --sample 1.20=runs/d120/depth.csv
   ```
3. 把推荐的 `offset_mm / scale_corr` 写回 `config.json` 的 `depth` 段，重启，再在**新距离**复验偏差应接近 0。

> 距离与精度：±1cm 实际只在近距离(≤约0.6m)且手眼/深度都标好时可行；深度误差随距离≈二次增长，
> >1m 很难保证 ±1cm。流程上应让底盘在 CENTER 阶段把目标精调到近处再 GRASP。

## 6. 启动方式（三种 + 开机自启）

| 场景 | 命令 | UI |
|---|---|---|
| **Windows 验模型** | `start_windows.bat` 或 `python launch.py`（接 335L 出深度；单机加 `TECHX_UDP_OPTIONAL=1` 不报 UDP 错） | 有 UI |
| **Jetson 带 UI**（调试，需接显示器/X） | `TECHX_GUI=1 bash start_jetson.sh` | 有 UI |
| **Jetson 无 UI**（比赛/无显示器） | `bash start_jetson.sh` 或 `bash scripts/field_start_jetson.sh` | 无 UI |

```bash
cd TECHX_RobotCode_vision
python3 tools/check_vision_config.py --config config.json   # 赛前静态自检（会报缺权重/灯带ROI等）
TECHX_DEBUG_RECORDER=1 bash scripts/field_start_jetson.sh    # 现场启动(headless)+录制
```
- 默认 `TECHX_STAGE=all`（KFS+武器头+灯带+二维码）。单阶段：`TECHX_STAGE=kfs|head|assembly|qr`。
- **模型缺权重**：`all` 阶段若启用了某模型但 `models/<folder>/` 没有 `best.engine/onnx/pt`，
  且 `strict_detector_load=true`，会**带明确提示退出(2)**；`TECHX_STAGE=kfs` 不会因为缺 head 权重而失败。
- 只在 config.json 里的模型才会启用；`models/` 里多余的文件夹会被**自动禁用**（避免 class_id 串号）。

### 开机自启（最终无显示屏部署，带 true/false 开关）
让 Orin NX 上电即自动跑视觉（无需登录/显示器），用 systemd 服务。**装一次,之后用一个开关控制**：
```bash
sudo bash scripts/setup_field_network.sh jetson   # 一次性：把 192.168.10.101 设成持久静态 IP
sudo bash scripts/install_vision_service.sh        # 一次性：安装服务（服务一直在，靠开关控制）
```
**开关在 `scripts/autostart.env`**（不用每次 systemctl）：
```ini
TECHX_AUTOSTART=false   # 上电正常开机，视觉不自动跑（默认，手动启动）
TECHX_AUTOSTART=true    # 上电自动跑视觉
# 可选：TECHX_STAGE=all  TECHX_DEBUG_RECORDER=1
```
改完 `sudo systemctl restart techx-vision`（或直接重启）。
- `false` 时服务在开机会被拉起但**立刻干净退出**(不跑、不重启循环)；`true` 时才真正运行。
- 服务以**登录用户**身份运行（自带 conda 环境 + 相机权限），`After=network-online`，崩溃自动重启（120s 内最多 10 次）。
- 看日志：`journalctl -u techx-vision -f`；立即手动跑一次：`sudo systemctl start techx-vision`。
- conda 路径不标准时：在 `autostart.env` 里加 `PYTHON=/path/to/python` 或 `TECHX_CONDA_SH=/path/conda.sh`。

## 7. CENTER 与 GRASP（决策包侧）

- **CENTER**：底盘精调，只用像素中心 `u/v` 和 `align_err_x/y`，**不要求 3D**（`require_control_xyz=false`）。
  视觉对“可见但不可抓”的目标仍输出 class/conf/u/v（质量门只清空 XYZ），所以 CENTER 照常能用。
- **GRASP**：机械臂抓取，**必须** `require_control_xyz=true`，GMK 只会选 `valid_control_xyz=true` 的目标；
  未标定/无深度/质量不达标的目标一律选不中（NO_MATCH）。
- 决策包必须订 **guarded** 话题 `/techx/vision/frame`、`/techx/vision/selected`，不要订 `_raw`。

## 8. 现场调试（主要在 Jetson 上看日志）

- **画面叠加**（UI 或 overlay 帧）：框颜色 **绿=可抓 / 橙=可见不可抓 / 灰=无深度**，
  标签带深度和不可抓原因；左上角 HUD 显示 fps、推理 ms、可抓数/有效深度数。
- **HEALTH 日志**（每 2s）：相机/推理/UDP 帧率、深度有效率、温度、内存、队列丢帧、前几个目标。
- **录制 CSV**（`runs/<dir>/`）：
  - `targets.csv`：camera_x/y/z、depth_m、can_grab、reject_reason、quality、edge/center、depth_spread。
  - `depth.csv`：`z_raw_m`(修正前) / `z`(修正后)、valid_ratio、p10/median/p90、roi、fallback。
  - `perf.csv`、`overlay_frames/`。
- **日志速读**：`python3 tools/log_summary.py logs/techx_current.log`。
- **GMK 侧**：`/tmp/techx_gmk_selected_debug.csv` 的 `reason` 列解释为什么 selected 没输出
  （NO_MATCH_CLASS/TYPE/ZONE/COLOR、LOW_CONFIDENCE、UNCALIBRATED、NO_VALID_CONTROL_XYZ…）。

## 9. 备份（赛前必做）

- **Jetson 系统镜像**（整盘 dd/clone）一份。
- `models/`（KFS、武器头权重）、`config.json`、本仓库代码（含分支/commit）。
- 标定产物：`runs/calib_*/` 的 `gmk_*.yaml / *_report.json / *_residuals.csv / overlays/`。
- GMK 侧：`vision_bridge.yaml`（含已写入的外参 + flag）、`install/` 可选、ROS2 工作区代码。
- 把“已验证可用”的一组 config + 标定打一个 tag/压缩包，便于现场快速回滚。

## 10. 启动顺序（比赛当天）

1. 赛前一次性：网络(101/100) → 手眼标定 + 深度校准 → apply 到 GMK → 两端 checker 必须 OK。
2. 先起 **GMK**（等 `vision bridge ready` 和三个 `calibrated`）。
3. 再起 **Jetson**（GMK 应打出 `first UDP frame received`）。
4. 决策包只订 guarded 话题，按 CENTER→GRASP 顺序发请求。

相关文档：UDP V2 协议见 `docs/udp_v2_protocol.md`；GMK 侧见 GMK 仓库 `docs/FIELD_OPERATIONS.md`。
