"""基于 IoU + 中心距离 + 简单速度预测的多目标追踪器。"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from interface.interfaces import ITracker
from interface.types import Detection, Track
from tuning.tuning import TuningParams
from utils.geometry import box_iou
from utils.logger import get_logger

log = get_logger(__name__)


def _track_family(class_id: int) -> int:
    cid = int(class_id)
    if 0 <= cid < 100:
        return 0
    if 100 <= cid < 150:
        return 1
    return cid


def _is_fake_kfs_class(class_id: int, class_name: str = "") -> bool:
    return int(class_id) == 0 or (class_name or "").lower() == "fake_kfs"


def _is_real_kfs_class(class_id: int, class_name: str = "") -> bool:
    return 1 <= int(class_id) <= 4 or "_kfs_" in (class_name or "").lower()


def _fake_real_kind(class_id: int, class_name: str = "") -> int:
    if _is_fake_kfs_class(class_id, class_name):
        return 1
    if _is_real_kfs_class(class_id, class_name):
        return 2
    return 0


class IouTracker(ITracker):
    """贪心 IoU + 最近中心距离 + 常速度预测追踪器。

    关键安全约束：未匹配的旧轨迹只用于短暂遮挡显示和保持 ID，
    会被标记 matched_this_frame=False / stale_age>0，控制链路不得发送。

    现场意义：
    - 慢速/中速目标：优先用 IoU 保持 ID；
    - 快速移动导致相邻框 IoU 很小时：用上一帧速度预测中心点，减少换 ID；
    - 类别变化仍然不跨类匹配，避免把真/假 KFS 或武器头类别互相串掉。
    """

    def __init__(self, tuning: Optional[TuningParams] = None):
        self._tuning = tuning or TuningParams()
        self._tracks: List[Track] = []
        self._frame_idx = 0
        self._next_id = 0
        self._velocity: dict[int, tuple[float, float]] = {}

    def update(self, detections: List[Detection]) -> List[Track]:
        self._frame_idx += 1
        p = self._tuning

        # 所有旧轨迹先置为未匹配，避免 stale 轨迹被误当作当前目标。
        for tr in self._tracks:
            tr.matched_this_frame = False
            tr.stale_age = self._frame_idx - tr.last_seen

        matched_old: set[int] = set()
        updated: List[Track] = []
        active_track_ids: set[int] = set()

        # 高置信度优先匹配，减少低置信度框抢占轨迹的概率。
        for det in sorted(detections, key=lambda d: d.conf, reverse=True):
            best_ti = self._find_best_track(det, matched_old)

            if best_ti >= 0:
                matched_old.add(best_ti)
                tr = self._tracks[best_ti]
                old_cx, old_cy = tr.center
                new_cx, new_cy = det.center
                vx, vy = self._velocity.get(tr.track_id, (0.0, 0.0))
                # 指数平滑速度，避免单帧误检造成过大预测跳变。
                self._velocity[tr.track_id] = (0.7 * vx + 0.3 * (new_cx - old_cx), 0.7 * vy + 0.3 * (new_cy - old_cy))
                tr.cls_names[int(det.cls_id)] = det.cls_name
                if not tr.cls_scores:
                    tr.cls_scores = {int(tr.cls_id): max(float(tr.conf), 0.01)}
                    tr.cls_names[int(tr.cls_id)] = tr.cls_name
                old_kind = _fake_real_kind(tr.cls_id, tr.cls_name)
                new_kind = _fake_real_kind(det.cls_id, det.cls_name)
                if old_kind and new_kind and old_kind != new_kind:
                    tr.cls_scores = {int(det.cls_id): float(det.conf)}
                else:
                    for cls_id in list(tr.cls_scores.keys()):
                        tr.cls_scores[cls_id] *= 0.85
                    tr.cls_scores[int(det.cls_id)] = tr.cls_scores.get(int(det.cls_id), 0.0) + 0.15 * float(det.conf)
                tr.cls_scores = {cls_id: score for cls_id, score in tr.cls_scores.items() if score > 0.01}
                best_cls = max(tr.cls_scores, key=tr.cls_scores.get)
                tr.cls_id = int(best_cls)
                tr.cls_name = tr.cls_names.get(int(best_cls), det.cls_name)
                tr.conf = float(det.conf)
                if tr.smooth_box is None:
                    tr.smooth_box = det.box.astype(np.float32).copy()
                else:
                    tr.smooth_box = (0.7 * tr.smooth_box + 0.3 * det.box).astype(np.float32)
                tr.box = tr.smooth_box.copy()
                tr.smoothed_conf = float(tr.cls_scores[best_cls])
                tr.last_seen = self._frame_idx
                tr.confirm_count += 1
                tr.matched_this_frame = True
                tr.stale_age = 0
                updated.append(tr)
                active_track_ids.add(tr.track_id)
            else:
                self._next_id += 1
                tid = self._next_id
                self._velocity[tid] = (0.0, 0.0)
                updated.append(Track(
                    track_id=tid,
                    box=det.box,
                    cls_id=det.cls_id,
                    cls_name=det.cls_name,
                    conf=det.conf,
                    last_seen=self._frame_idx,
                    confirm_count=1,
                    cls_scores={int(det.cls_id): float(det.conf)},
                    cls_names={int(det.cls_id): det.cls_name},
                    smooth_box=det.box.astype(np.float32).copy(),
                    smoothed_conf=float(det.conf),
                    matched_this_frame=True,
                    stale_age=0,
                ))
                active_track_ids.add(tid)

        current_ids = {t.track_id for t in updated}
        for old_t in self._tracks:
            if old_t.track_id in current_ids:
                continue
            old_t.matched_this_frame = False
            old_t.stale_age = self._frame_idx - old_t.last_seen
            if old_t.stale_age < p.track_max_stale:
                updated.append(old_t)
                active_track_ids.add(old_t.track_id)

        # 清理已经彻底消失的速度状态，避免长时间运行内存增长。
        for tid in list(self._velocity.keys()):
            if tid not in active_track_ids:
                del self._velocity[tid]

        self._tracks = updated
        return self._tracks

    def _find_best_track(self, det: Detection, matched_old: set[int]) -> int:
        """为一个检测框寻找最佳旧轨迹。优先 IoU，其次预测中心距离。"""
        p = self._tuning
        best_iou = min(float(p.track_iou_threshold), 0.10)
        best_iou_idx = -1
        dcx, dcy = det.center

        center_candidates: list[tuple[float, int]] = []
        for idx, tr in enumerate(self._tracks):
            if idx in matched_old or _track_family(tr.cls_id) != _track_family(det.cls_id):
                continue

            iou = box_iou(tr.box, det.box)
            if iou > best_iou:
                best_iou = iou
                best_iou_idx = idx

            # 快速移动时 IoU 可能接近 0，此时使用上一帧速度预测的中心点。
            # 阈值随 stale_age 轻微放宽，但限制上限，避免两个相近同类目标互相抢 ID。
            if iou < 0.1:
                tcx, tcy = tr.center
                vx, vy = self._velocity.get(tr.track_id, (0.0, 0.0))
                pred_x = tcx + vx
                pred_y = tcy + vy
                dist = float(np.hypot(dcx - pred_x, dcy - pred_y))
                stale_bonus = min(2.0, max(1.0, 1.0 + 0.15 * float(tr.stale_age)))
                max_dist = min(float(p.track_center_dist) * stale_bonus, float(p.track_center_dist) * 2.0)
                if dist < max_dist:
                    center_candidates.append((dist, idx))

        if best_iou_idx >= 0:
            return best_iou_idx
        if center_candidates:
            center_candidates.sort(key=lambda item: item[0])
            return center_candidates[0][1]
        return -1

    def reset(self) -> None:
        self._tracks.clear()
        self._velocity.clear()
        self._frame_idx = 0
        self._next_id = 0
        log.debug("Tracker reset")

    @property
    def active_count(self) -> int:
        p = self._tuning
        return sum(
            1 for t in self._tracks
            if t.matched_this_frame and t.confirm_count >= p.track_confirm_frames
        )
