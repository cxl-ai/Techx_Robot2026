"""标注绘制器 — 将track和深度叠加层渲染到帧上。

纯渲染模块 — 不含业务逻辑，不持有状态。 接收帧 + track，返回
带标注的副本。 设计为可替换（不同比赛可切换不同的叠加层样式）。
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

from interface.types import Track
from tuning.tuning import TuningParams
from utils.logger import get_logger

log = get_logger(__name__)


class Visualizer:
    """绘制边界框、标签、深度叠加层和最近目标标记。"""

    def __init__(self, tuning: Optional[TuningParams] = None):
        """初始化可视化器。

        Args:
            tuning: 可调参数实例，若为None则使用默认值
        """
        self._tuning = tuning or TuningParams()
        self._frame_w = 640
        self._frame_h = 480

        # 颜色定义（BGR格式）
        self.COLOR_GREEN  = (0, 255, 0)    # 绿色 — 可抓取目标
        self.COLOR_YELLOW = (0, 255, 255)  # 黄色 — 最近目标/十字准星
        self.COLOR_CYAN   = (255, 255, 0)  # 青色 — ROI半透明叠加
        self.COLOR_WHITE  = (255, 255, 255)# 白色
        self.COLOR_RED    = (0, 0, 255)    # 红色
        self.COLOR_BLUE   = (255, 0, 0)    # 蓝色
        self.COLOR_ORANGE = (0, 165, 255)  # 橙色 — 可见但不可抓（质量门拦下）
        self.COLOR_GRAY   = (170, 170, 170)# 灰色 — 无有效深度

    def set_frame_size(self, w: int, h: int) -> None:
        """设置当前帧的宽高尺寸（像素）。

        Args:
            w: 帧宽度（像素）
            h: 帧高度（像素）
        """
        self._frame_w = w
        self._frame_h = h

    # ── 深度热力图 ────────────────────────────────────────────────

    def draw_depth_overlay(self, ann: np.ndarray, depth_mm: Optional[np.ndarray]) -> np.ndarray:
        """将JET色彩空间的深度热力图以指定alpha值叠加到图像上。

        步骤：
        1. 将深度值钳位至 [200, 5000] 毫米有效范围。
        2. 归一化到 [0, 255]。
        3. 应用JET色彩映射表。
        4. 以 alpha 透明度与原始图像融合。

        Args:
            ann:      待叠加的BGR图像（H×W×3）
            depth_mm: 深度图（float32，单位毫米），可为None

        Returns:
            融合后的BGR图像（若depth_mm为None则原样返回）
        """
        if depth_mm is None:
            return ann

        p = self._tuning
        # 钳位深度值到有效显示范围
        clipped = np.clip(depth_mm, 200.0, 5000.0).astype(np.float32)
        # 归一化到 [0, 255] 并转为8位
        normalised = cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # 应用JET色彩映射表（伪彩色）
        coloured = cv2.applyColorMap(normalised, cv2.COLORMAP_JET)
        # 以指定alpha值融合
        return cv2.addWeighted(ann, 1.0 - p.heatmap_alpha, coloured, p.heatmap_alpha, 0)

    # ── 完整标注 ──────────────────────────────────────────────────

    def draw(
        self,
        frame: np.ndarray,
        tracks: List[Track],
        show_depth_overlay: bool = False,
        depth_mm: Optional[np.ndarray] = None,
        confirm_frames: int = 1,
        targets: Optional[list] = None,
    ) -> np.ndarray:
        """在 *frame* 的副本上绘制所有标注。

        绘制内容：
        1. 可选的深度热力图叠加层。
        2. 每个已确认track的边界框、ROI、标签。
        3. 最近目标的黄色高亮标记（粗框 + 十字准星 + 圆圈）。

        Args:
            frame:              BGR图像（H×W×3）
            tracks:             带有深度和颜色的活跃track列表
            show_depth_overlay: 是否显示深度热力图
            depth_mm:           深度图（用于热力图）
            confirm_frames:     绘制所需的最低确认帧数

        Returns:
            带标注的BGR图像（副本），原始frame未被修改
        """
        ann = frame.copy()

        # 深度叠加层
        if show_depth_overlay:
            ann = self.draw_depth_overlay(ann, depth_mm)

        # 过滤出已确认的track
        visible = [
            t for t in tracks
            if t.confirm_count >= confirm_frames
            and t.is_fresh
            and float(t.conf) >= float(self._tuning.output_min_confidence)
        ]
        if not visible:
            return ann

        # 把 Target3D 的抓取质量(can_grab/reject_reason/z)按 track_id 映射回来,
        # 这样画面上一眼就能看出"可抓(绿)/可见不可抓(橙)/无深度(灰)"。
        target_by_id = {}
        for tg in (targets or []):
            try:
                target_by_id[int(tg.track_id)] = tg
            except (TypeError, AttributeError):
                continue

        # 为每个已确认track绘制标注
        for t in visible:
            self._draw_track(ann, t, target_by_id.get(int(t.track_id)))

        # 查找并高亮最近目标
        nearest = self._find_nearest(visible)
        if nearest is not None:
            self._draw_nearest(ann, nearest)

        return ann

    # ── 单track绘制 ────────────────────────────────────────────────

    def _draw_track(self, ann: np.ndarray, t: Track, target=None) -> None:
        """绘制单个track的标注信息。

        颜色编码（便于现场观察抓取可行性）：
        - 绿色：可抓取（can_grab 且有有效深度）
        - 橙色：可见但不可抓（质量门拦下，仍用于 CENTER 居中）
        - 灰色：无有效深度
        标签追加深度和不可抓原因，方便一眼定位问题。

        Args:
            ann:    待绘制的BGR图像（原地修改）
            t:      需要绘制的Track对象
            target: 对应的 Target3D（含 can_grab/reject_reason/z），可为 None
        """
        x1, y1, x2, y2 = map(int, t.box)

        # 绘制中心ROI的半透明叠加层（30%边界框大小）
        p = self._tuning
        rw = max(5, int((x2 - x1) * 0.3))
        rh = max(5, int((y2 - y1) * 0.3))
        cx_r, cy_r = (x1 + x2) // 2, (y1 + y2) // 2
        rx1 = max(0, cx_r - rw // 2)
        ry1 = max(0, cy_r - rh // 2)
        rx2 = min(self._frame_w - 1, rx1 + rw)
        ry2 = min(self._frame_h - 1, ry1 + rh)

        overlay = ann.copy()
        cv2.rectangle(overlay, (rx1, ry1), (rx2, ry2), (255, 255, 0), -1)  # 青色填充
        cv2.addWeighted(overlay, p.roi_overlay_alpha, ann, 1.0 - p.roi_overlay_alpha, 0, dst=ann)

        # 根据抓取质量选择边界框颜色
        z = float(getattr(target, "z", t.z) if target is not None else t.z)
        can_grab = bool(getattr(target, "can_grab", True)) if target is not None else (z > 0)
        reject = str(getattr(target, "reject_reason", "") or "") if target is not None else ""
        if z <= 0:
            color = self.COLOR_GRAY
        elif can_grab:
            color = self.COLOR_GREEN
        else:
            color = self.COLOR_ORANGE
        cv2.rectangle(ann, (x1, y1), (x2, y2), color, 2)

        # 组装并绘制标签文字
        name = t.cls_name
        if t.color:
            name = name.replace("_red", "").replace("_blue", "") + "_" + t.color

        label = f"#{t.track_id} {name} {t.conf:.2f}"
        if z > 0:
            label += f" | {z:.2f}m"
        if z > 0 and not can_grab and reject:
            label += f" | {reject.split('+')[0]}"  # 第一个拦截原因即可
        cv2.putText(ann, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def _find_nearest(self, tracks: List[Track]) -> Optional[Track]:
        """在所有已确认track中找出最近目标。

        优先级规则：
        1. 有深度值的目标优先于无深度的目标。
        2. 若同为有深度，取Z值最小的（最近的）。
        3. 若同为无深度，取边界框面积最大的（通常最近的目标在图像中最大）。

        Args:
            tracks: 已确认track列表（confirm_count ≥ 阈值）

        Returns:
            最近目标的Track对象，若列表为空则返回None
        """
        nearest: Optional[Track] = None
        nearest_has_depth = False

        for t in tracks:
            has_depth = t.z > 0
            if nearest is None:
                # 首个track直接作为候选
                nearest = t
                nearest_has_depth = has_depth
            elif has_depth and not nearest_has_depth:
                # 当前有深度而候选无深度 → 当前胜出
                nearest = t
                nearest_has_depth = True
            elif has_depth == nearest_has_depth:
                # 深度状态相同 → 按具体指标比较
                if has_depth:
                    # 都有深度 → 取Z值更小的（更近的）
                    if t.z < nearest.z:
                        nearest = t
                else:
                    # 都无深度 → 取边界框面积更大的
                    if (t.box[2] - t.box[0]) * (t.box[3] - t.box[1]) > \
                       (nearest.box[2] - nearest.box[0]) * (nearest.box[3] - nearest.box[1]):
                        nearest = t
        return nearest

    def _draw_nearest(self, ann: np.ndarray, t: Track) -> None:
        """用黄色样式高亮绘制最近目标。

        绘制内容：
        - 黄色粗边界框（线宽3）
        - 标签文字："... <- nearest"
        - 中心十字准星（8像素水平/垂直线 + 半径4像素圆圈）

        Args:
            ann: 待绘制的BGR图像（原地修改）
            t:   最近目标的Track对象
        """
        x1, y1, x2, y2 = map(int, t.box)

        # 黄色加粗边界框
        cv2.rectangle(ann, (x1, y1), (x2, y2), self.COLOR_YELLOW, 3)

        # 组装标签文字
        name = t.cls_name
        if t.color:
            name = name.replace("_red", "").replace("_blue", "") + "_" + t.color
        ds = f"{t.z:.2f}m" if t.z > 0 else "near"
        cv2.putText(ann, f"#{t.track_id} {name} {t.conf:.2f} | {ds} <- nearest",
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOR_YELLOW, 2)

        # 中心十字准星 + 圆圈
        u, v = int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))
        cv2.line(ann, (u - 8, v), (u + 8, v), self.COLOR_YELLOW, 1)  # 水平线
        cv2.line(ann, (u, v - 8), (u, v + 8), self.COLOR_YELLOW, 1)  # 垂直线
        cv2.circle(ann, (u, v), 4, self.COLOR_YELLOW, 1)             # 圆圈

    # ── FPS / 状态栏 ──────────────────────────────────────────────

    def draw_status_bar(
        self, ann: np.ndarray,
        fps: float, infer_ms: float, track_count: int,
        graspable: Optional[int] = None, valid_xyz: Optional[int] = None,
    ) -> np.ndarray:
        """在图像左上角绘制 FPS / 延迟 / 目标数 + 抓取状态与图例。

        Args:
            ann:         待绘制的BGR图像（原地修改）
            fps:         当前帧率
            infer_ms:    上次推理耗时（毫秒）
            track_count: 当前活跃track数量
            graspable:   可抓取目标数（可选）
            valid_xyz:   有有效深度的目标数（可选）

        Returns:
            同一图像（已原地修改）
        """
        text = f"{fps:.0f} det/s | {infer_ms:.0f}ms | {track_count} obj"
        if graspable is not None or valid_xyz is not None:
            text += f" | grab={graspable if graspable is not None else '-'} validXYZ={valid_xyz if valid_xyz is not None else '-'}"
        cv2.putText(ann, text, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLOR_GREEN, 2)
        # 颜色图例：绿=可抓 橙=可见不可抓 灰=无深度
        cv2.putText(ann, "green=grab  orange=visible-only  gray=no-depth", (8, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.COLOR_WHITE, 1)
        return ann
