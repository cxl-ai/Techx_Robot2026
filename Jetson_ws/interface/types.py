"""TECHx_vision 流水线的不可变/轻量数据容器。

所有模块通过这些类型交换数据，避免使用结构不明确的 dict/tuple。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Frame:
    """单帧传感器数据。"""

    bgr: np.ndarray
    depth_mm: Optional[np.ndarray] = None
    timestamp: float = 0.0
    frame_id: int = 0

    @property
    def shape(self) -> Tuple[int, int, int]:
        return self.bgr.shape

    @property
    def has_depth(self) -> bool:
        return self.depth_mm is not None


@dataclass
class Detection:
    """单次目标检测结果。"""

    box: np.ndarray
    cls_id: int
    cls_name: str = ""
    conf: float = 0.0

    @property
    def center(self) -> Tuple[float, float]:
        return (
            (self.box[0] + self.box[2]) / 2.0,
            (self.box[1] + self.box[3]) / 2.0,
        )

    @property
    def area(self) -> float:
        return float((self.box[2] - self.box[0]) * (self.box[3] - self.box[1]))

    @property
    def width(self) -> float:
        return float(self.box[2] - self.box[0])

    @property
    def height(self) -> float:
        return float(self.box[3] - self.box[1])


@dataclass
class Track:
    """跨帧追踪目标。

    matched_this_frame 与 stale_age 是控制链路安全字段：
    - matched_this_frame=True 表示该轨迹在当前检测帧中真实匹配到检测框；
    - stale_age>0 表示只是为了显示/短暂遮挡保留的旧轨迹，不能直接发送给机械臂。
    """

    track_id: int
    box: np.ndarray
    cls_id: int
    cls_name: str = ""
    conf: float = 0.0
    z: float = 0.0
    last_seen: int = 0
    confirm_count: int = 0
    color: Optional[str] = None
    color_hist: List[str] = field(default_factory=list)
    cls_scores: Dict[int, float] = field(default_factory=dict)
    cls_names: Dict[int, str] = field(default_factory=dict)
    smooth_box: Optional[np.ndarray] = None
    smoothed_conf: float = 0.0
    matched_this_frame: bool = True
    stale_age: int = 0

    @property
    def center(self) -> Tuple[float, float]:
        return (
            (self.box[0] + self.box[2]) / 2.0,
            (self.box[1] + self.box[3]) / 2.0,
        )

    @property
    def is_fresh(self) -> bool:
        """是否由当前帧检测结果更新。"""
        return self.matched_this_frame and self.stale_age == 0

    @property
    def is_confirmed(self, min_frames: int = 1) -> bool:
        return self.confirm_count >= min_frames

    @property
    def has_valid_depth(self) -> bool:
        return self.z > 0.0


@dataclass
class Target3D:
    """最终视觉目标输出。

    quality/can_grab 字段只用于 Jetson 侧安全门槛和日志，不改变 UDP V2
    基础协议字段。质量不达标的抓取类目标仍保留 class/conf/u/v 用于搜索和居中，
    但 solver 会清空输出 XYZ，使 GMK 的 require_control_xyz 抓取请求无法选中它。
    """

    track_id: int
    class_name: str
    timestamp: float
    pixel_uv: Tuple[float, float]
    camera_xyz: Tuple[float, float, float]
    base_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    depth_m: float = 0.0
    is_calibrated: bool = False
    class_id: int = -1
    confidence: float = 0.0
    color: Optional[str] = None
    raw_confidence: float = 0.0
    can_grab: bool = True
    reject_reason: str = ""
    quality_score: float = 1.0
    edge_margin_px: float = 9999.0
    center_error_norm: float = 0.0
    depth_valid_ratio: float = 0.0
    depth_spread_m: float = 0.0
    depth_fallback_used: bool = False

    @property
    def x(self) -> float:
        return self.camera_xyz[0]

    @property
    def y(self) -> float:
        return self.camera_xyz[1]

    @property
    def z(self) -> float:
        return self.camera_xyz[2]

    def __repr__(self) -> str:
        xc, yc, zc = self.camera_xyz
        q = "OK" if self.can_grab else self.reject_reason or "REJECT"
        return (
            f"Target3D(#{self.track_id} cls={self.class_id}:{self.class_name} "
            f"conf={self.confidence:.2f}/{self.raw_confidence:.2f} color={self.color or '-'} "
            f"cam=({xc:.3f},{yc:.3f},{zc:.3f}) q={q} score={self.quality_score:.2f} "
            f"ts={self.timestamp:.6f}s)"
        )
