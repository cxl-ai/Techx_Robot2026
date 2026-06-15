"""3D coordinate solver, depth query, color classification, and target quality."""

from __future__ import annotations

from collections import Counter
from typing import Dict, List, Optional

import cv2
import numpy as np

from interface.types import Track, Target3D
from tuning.tuning import TuningParams
from utils.logger import get_logger

log = get_logger(__name__)


class CoordinateSolver:
    """Depth query + color classification + 3D coordinate solver."""

    def __init__(
        self,
        fx: float = 367.93,
        fy: float = 368.05,
        cx: float = 318.75,
        cy: float = 236.20,
        tuning: Optional[TuningParams] = None,
        transformer=None,
    ):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self._tuning = tuning or TuningParams()
        self._transformer = transformer
        self._smooth: Dict[int, float] = {}
        self._frame_w = 640
        self._frame_h = 480
        self._last_depth_debug: List[dict] = []

    def set_frame_size(self, w: int, h: int) -> None:
        self._frame_w = int(w)
        self._frame_h = int(h)

    def set_intrinsics(self, fx: float, fy: float, cx: float, cy: float) -> None:
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

    def set_transformer(self, transformer) -> None:
        self._transformer = transformer

    @property
    def last_depth_debug(self) -> List[dict]:
        return list(self._last_depth_debug)

    @property
    def is_calibrated(self) -> bool:
        if self._transformer is None:
            return False
        try:
            return (
                not np.allclose(self._transformer.R, np.eye(3))
                and not np.allclose(self._transformer.t, np.zeros(3))
            )
        except Exception:
            return False

    def _empty_depth_debug(self, track: Track) -> dict:
        x1, y1, x2, y2 = [float(v) for v in track.box[:4]]
        u, v = track.center
        return {
            "track_id": int(track.track_id),
            "class_id": int(track.cls_id),
            "valid_depth": 0,
            "depth_m": 0.0,
            "z_raw_m": 0.0,
            "bbox": "%.1f %.1f %.1f %.1f" % (x1, y1, x2, y2),
            "u": float(u),
            "v": float(v),
            "z": 0.0,
            "valid_count": 0,
            "valid_ratio": 0.0,
            "p10": 0.0,
            "median": 0.0,
            "p90": 0.0,
            "roi_x": 0,
            "roi_y": 0,
            "roi_w": 0,
            "roi_h": 0,
            "fallback_used": 0,
            "edge_margin_px": 0.0,
            "center_error_norm": 1.0,
            "depth_spread_m": 0.0,
            "quality_score": 0.0,
            "can_grab": 0,
            "reject_reason": "NO_DEPTH",
            "raw_confidence": float(track.conf),
            "output_confidence": float(track.conf),
        }

    def _depth_debug_for_track(self, track_id: int) -> Optional[dict]:
        for row in reversed(self._last_depth_debug):
            if int(row.get("track_id", -1)) == int(track_id):
                return row
        return None

    def query_depth(self, track: Track, depth_mm: Optional[np.ndarray]) -> float:
        """Query target depth in metres using progressive ROI + MAD filtering.

        Besides returning z, this records a per-track row in last_depth_debug so
        the field recorder can explain why a target had/ lacked valid depth.
        """
        debug = self._empty_depth_debug(track)
        if depth_mm is None:
            self._last_depth_debug.append(debug)
            return 0.0

        p = self._tuning
        x1, y1, x2, y2 = map(int, track.box[:4])
        h, w = depth_mm.shape[:2]
        cx_r, cy_r = (x1 + x2) // 2, (y1 + y2) // 2
        last_valid = None
        last_roi = (0, 0, 0, 0)
        fallback_used = False

        for scale in p.depth_roi_expansions:
            rw = max(5, int((x2 - x1) * scale))
            rh = max(5, int((y2 - y1) * scale))
            rx1 = max(0, cx_r - rw // 2)
            ry1 = max(0, cy_r - rh // 2)
            rx2 = min(w, rx1 + rw)
            ry2 = min(h, ry1 + rh)
            last_roi = (ry1, ry2, rx1, rx2)

            roi = depth_mm[ry1:ry2, rx1:rx2]
            valid = roi[(roi > p.depth_min_mm) & (roi <= p.depth_max_mm)]
            debug.update(self._depth_stats_for_debug(valid, rx1, ry1, rx2, ry2, roi.size))
            if len(valid) >= p.depth_roi_min_pixels:
                median = float(np.median(valid))
                inliers = valid[np.abs(valid - median) < p.depth_mad_threshold_mm]
                last_valid = float(np.median(inliers)) if len(inliers) >= 3 else median
                break

        if last_valid is None and last_roi[0] < last_roi[1]:
            ry1, ry2, rx1, rx2 = last_roi
            roi = depth_mm[ry1:ry2, rx1:rx2]
            valid = roi[(roi > p.depth_min_mm) & (roi <= p.depth_max_mm)]
            debug.update(self._depth_stats_for_debug(valid, rx1, ry1, rx2, ry2, roi.size))
            if len(valid) >= 10:
                last_valid = float(np.percentile(valid, p.depth_fallback_percentile))
                fallback_used = True

        if last_valid is None:
            debug["fallback_used"] = int(fallback_used)
            self._last_depth_debug.append(debug)
            return 0.0

        z_mm = last_valid * p.depth_scale_corr + p.depth_offset_mm
        z_m = max(0.0, z_mm) / 1000.0
        debug["valid_depth"] = int(z_m > 0.0)
        debug["depth_m"] = z_m
        debug["z"] = z_m
        debug["z_raw_m"] = max(0.0, float(last_valid)) / 1000.0  # before scale/offset correction
        debug["fallback_used"] = int(fallback_used)
        self._last_depth_debug.append(debug)
        return z_m

    @staticmethod
    def _depth_stats_for_debug(valid: np.ndarray, rx1: int, ry1: int, rx2: int, ry2: int, roi_size: int) -> dict:
        if len(valid) > 0:
            p10, med, p90 = [float(x) / 1000.0 for x in np.percentile(valid, [10, 50, 90])]
        else:
            p10 = med = p90 = 0.0
        return {
            "valid_count": int(len(valid)),
            "valid_ratio": float(len(valid)) / float(max(1, roi_size)),
            "p10": p10,
            "median": med,
            "p90": p90,
            "roi_x": int(rx1),
            "roi_y": int(ry1),
            "roi_w": int(max(0, rx2 - rx1)),
            "roi_h": int(max(0, ry2 - ry1)),
        }

    def smooth_depth(self, track_id: int, z_raw: float) -> float:
        p = self._tuning
        if z_raw <= 0:
            self._smooth.pop(track_id, None)
            return 0.0

        prev = self._smooth.get(track_id, 0.0)
        smoothed = z_raw if prev <= 0 else p.depth_smooth_alpha * z_raw + (1 - p.depth_smooth_alpha) * prev
        self._smooth[track_id] = smoothed
        return smoothed

    def apply_depth_to_tracks(
        self,
        tracks: List[Track],
        depth_mm: Optional[np.ndarray],
        manual_depth_m: float = 0.0,
    ) -> List[Track]:
        self._last_depth_debug = []
        active_ids = {t.track_id for t in tracks}
        for t in tracks:
            if depth_mm is not None:
                z_raw = self.query_depth(t, depth_mm)
            else:
                z_raw = manual_depth_m if manual_depth_m > 0 else 0.0
                debug = self._empty_depth_debug(t)
                debug["valid_depth"] = int(z_raw > 0.0)
                debug["depth_m"] = float(z_raw)
                debug["z"] = float(z_raw)
                debug["z_raw_m"] = float(z_raw)
                debug["fallback_used"] = int(manual_depth_m > 0)
                self._last_depth_debug.append(debug)
            t.z = self.smooth_depth(t.track_id, z_raw)

        for tid in list(self._smooth.keys()):
            if tid not in active_ids:
                del self._smooth[tid]
        return tracks

    def classify_color(self, track: Track, bgr: np.ndarray) -> Optional[str]:
        p = self._tuning
        x1, y1, x2, y2 = map(int, track.box[:4])
        h_img, w_img = bgr.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w_img - 1, x2), min(h_img - 1, y2)
        if x2 <= x1 or y2 <= y1:
            return None

        hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        red_mask = (
            cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
            | cv2.inRange(hsv, (160, 50, 50), (180, 255, 255))
        )
        blue_mask = cv2.inRange(hsv, (100, 50, 50), (130, 255, 255))

        r_cnt = cv2.countNonZero(red_mask)
        b_cnt = cv2.countNonZero(blue_mask)
        if r_cnt + b_cnt < p.color_min_pixels:
            return None

        color = None
        if r_cnt > b_cnt * p.color_ratio_threshold:
            color = "red"
        elif b_cnt > r_cnt * p.color_ratio_threshold:
            color = "blue"

        if color:
            track.color_hist.append(color)
        track.color_hist = track.color_hist[-p.color_hist_size:]
        if track.color_hist:
            track.color = Counter(track.color_hist).most_common(1)[0][0]
        return track.color

    def classify_colors(self, tracks: List[Track], bgr: np.ndarray) -> List[Track]:
        for t in tracks:
            self.classify_color(t, bgr)
        return tracks

    def _is_grasp_class(self, class_id: int) -> bool:
        return int(class_id) in set(int(x) for x in self._tuning.quality_gate_classes)

    def _quality_for_track(self, track: Track, z: float) -> dict:
        p = self._tuning
        x1, y1, x2, y2 = [float(v) for v in track.box[:4]]
        u, v = track.center
        edge_margin = min(x1, y1, float(self._frame_w) - x2, float(self._frame_h) - y2)
        center_error = max(
            abs(float(u) - float(self._frame_w) * 0.5) / max(1.0, float(self._frame_w) * 0.5),
            abs(float(v) - float(self._frame_h) * 0.5) / max(1.0, float(self._frame_h) * 0.5),
        )
        depth_row = self._depth_debug_for_track(track.track_id) or {}
        valid_ratio = float(depth_row.get("valid_ratio", 0.0) or 0.0)
        p10 = float(depth_row.get("p10", 0.0) or 0.0)
        p90 = float(depth_row.get("p90", 0.0) or 0.0)
        depth_spread = max(0.0, p90 - p10) if p10 > 0.0 or p90 > 0.0 else 0.0
        fallback_used = bool(int(depth_row.get("fallback_used", 0) or 0))

        reasons: List[str] = []
        if z <= 0.0:
            reasons.append("NO_DEPTH")
        if edge_margin < p.quality_edge_margin_px:
            reasons.append("TOO_CLOSE_TO_EDGE")
        if center_error > p.quality_center_error_norm:
            reasons.append("NOT_CENTERED")
        if z > 0.0 and valid_ratio < p.quality_min_depth_valid_ratio:
            reasons.append("BAD_DEPTH_RATIO")
        if z > 0.0 and p.quality_max_depth_spread_m > 0.0 and depth_spread > p.quality_max_depth_spread_m:
            reasons.append("BAD_DEPTH_SPREAD")
        if p.quality_require_no_fallback and fallback_used:
            reasons.append("DEPTH_FALLBACK")

        edge_score = min(1.0, max(0.0, edge_margin / max(1.0, float(p.quality_edge_margin_px))))
        center_score = 1.0 - min(1.0, center_error / max(1e-6, p.quality_center_error_norm))
        depth_ratio_score = min(1.0, valid_ratio / max(1e-6, p.quality_min_depth_valid_ratio))
        if p.quality_max_depth_spread_m > 0.0 and depth_spread > 0.0:
            depth_spread_score = 1.0 - min(1.0, depth_spread / p.quality_max_depth_spread_m)
        else:
            depth_spread_score = 1.0
        quality_score = max(0.0, min(1.0, min(edge_score, center_score, depth_ratio_score, depth_spread_score)))

        return {
            "edge_margin_px": float(edge_margin),
            "center_error_norm": float(center_error),
            "depth_valid_ratio": float(valid_ratio),
            "depth_spread_m": float(depth_spread),
            "depth_fallback_used": bool(fallback_used),
            "quality_score": float(quality_score),
            "can_grab": len(reasons) == 0,
            "reject_reason": "+".join(reasons),
        }

    def _apply_quality_gate(self, track: Track, target: Target3D) -> Target3D:
        """Attach grasp quality without hiding visible targets.

        Bad-quality grasp targets must remain visible to GMK/decision code for
        SEARCH/CENTER_TARGET using class/conf/u/v. To keep them out of arm motion,
        clear only the 3D output so GMK marks valid_xyz/valid_control_xyz=false;
        a GRASP request with require_control_xyz=true will then reject them.
        """
        target.raw_confidence = float(track.conf)
        quality = self._quality_for_track(track, target.z)
        for key, value in quality.items():
            setattr(target, key, value)

        gated = bool(self._tuning.quality_gate_enabled and self._is_grasp_class(target.class_id))
        if gated and not target.can_grab:
            target.camera_xyz = (0.0, 0.0, 0.0)
            target.base_xyz = (0.0, 0.0, 0.0)
            target.depth_m = 0.0
            target.is_calibrated = False
        elif not gated:
            target.can_grab = True
            target.reject_reason = ""
            target.quality_score = 1.0

        row = self._depth_debug_for_track(track.track_id)
        if row is not None:
            row.update({
                "edge_margin_px": target.edge_margin_px,
                "center_error_norm": target.center_error_norm,
                "depth_spread_m": target.depth_spread_m,
                "quality_score": target.quality_score,
                "can_grab": int(bool(target.can_grab)),
                "reject_reason": target.reject_reason,
                "raw_confidence": target.raw_confidence,
                "output_confidence": target.confidence,
                "output_valid_xyz": int(bool(target.z > 0.0)),
            })
        return target

    def solve(self, track: Track, timestamp: float) -> Target3D:
        u, v = track.center
        z = track.z
        cls_name = track.cls_name
        if track.color:
            cls_name = cls_name.replace("_red", "").replace("_blue", "") + "_" + track.color

        common = {
            "track_id": track.track_id,
            "class_name": cls_name,
            "timestamp": timestamp,
            "pixel_uv": (u, v),
            "class_id": int(track.cls_id),
            "confidence": float(track.conf),
            "raw_confidence": float(track.conf),
            "color": track.color,
        }

        if z <= 0:
            target = Target3D(
                **common,
                camera_xyz=(0.0, 0.0, 0.0),
                depth_m=0.0,
            )
            return self._apply_quality_gate(track, target)

        calibrated = False
        if self._transformer is not None and self.fx > 0:
            try:
                cam = self._transformer.pixel_to_camera(u, v, z)
                base = self._transformer.camera_to_base(cam)
                Xc, Yc, Zc = cam if cam is not None else (0.0, 0.0, 0.0)
                Xb, Yb, Zb = base if base is not None else (0.0, 0.0, 0.0)
                calibrated = self.is_calibrated
            except Exception:
                Xc = (u - self.cx) * z / self.fx if self.fx > 0 else 0.0
                Yc = (v - self.cy) * z / self.fy if self.fy > 0 else 0.0
                Zc = z
                Xb = Yb = Zb = 0.0
        else:
            Xc = (u - self.cx) * z / self.fx if self.fx > 0 else 0.0
            Yc = (v - self.cy) * z / self.fy if self.fy > 0 else 0.0
            Zc = z
            Xb = Yb = Zb = 0.0

        target = Target3D(
            **common,
            camera_xyz=(Xc, Yc, Zc),
            base_xyz=(Xb, Yb, Zb) if calibrated else (0.0, 0.0, 0.0),
            depth_m=z,
            is_calibrated=calibrated,
        )
        return self._apply_quality_gate(track, target)

    def solve_all(
        self,
        tracks: List[Track],
        timestamp: float,
        confirm_frames: int = 1,
    ) -> List[Target3D]:
        results = []
        for t in tracks:
            if t.confirm_count < confirm_frames:
                continue
            results.append(self.solve(t, timestamp))
        results.sort(key=lambda x: (x.z <= 0, x.z))
        return results
