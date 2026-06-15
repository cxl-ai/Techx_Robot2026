"""Field flight recorder for Jetson competition runs.

Enabled by environment:
    TECHX_DEBUG_RECORDER=1
    TECHX_DEBUG_DIR=runs/field_001
    TECHX_DEBUG_FRAME_EVERY=10

This module is intentionally dependency-light and fail-soft. Recorder failures must
not stop the robot pipeline once the process has already started.
"""

from __future__ import annotations

import csv
import os
import time
from typing import Any, Dict, Iterable, Optional

import cv2


_PERF_FIELDS = [
    "time", "frame_id", "avg_infer_ms", "avg_display_ms", "track_count", "raw_target_count",
    "target_count", "sender_ready", "camera_frame_age_sec", "infer_result_age_sec", "fatal_reason",
]

_TRACK_FIELDS = [
    "time", "frame_id", "track_id", "class_id", "class_name", "conf",
    "x1", "y1", "x2", "y2", "cx", "cy", "z", "confirm_count",
    "matched_this_frame", "stale_age", "color",
]

_TARGET_FIELDS = [
    "time", "frame_id", "track_id", "class_id", "class_name", "conf", "raw_conf",
    "u", "v", "camera_x", "camera_y", "camera_z", "depth_m", "color",
    "can_grab", "reject_reason", "quality_score", "edge_margin_px", "center_error_norm",
    "depth_valid_ratio", "depth_spread_m", "depth_fallback_used",
]

_DEPTH_FIELDS = [
    "time", "frame_id", "track_id", "class_id", "valid_depth", "depth_m", "z_raw_m",
    "bbox", "u", "v", "z", "valid_count", "valid_ratio", "p10", "median", "p90",
    "roi_x", "roi_y", "roi_w", "roi_h", "fallback_used",
    "edge_margin_px", "center_error_norm", "depth_spread_m", "quality_score",
    "can_grab", "reject_reason", "raw_confidence", "output_confidence", "output_valid_xyz",
]


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return default


class FieldRecorder:
    def __init__(self, enabled: bool, base_dir: str, frame_every: int = 10) -> None:
        self.enabled = bool(enabled)
        self.base_dir = base_dir
        self.frame_every = max(1, int(frame_every))
        self.overlay_dir = os.path.join(base_dir, "overlay_frames")
        self._perf_fh = None
        self._tracks_fh = None
        self._targets_fh = None
        self._depth_fh = None
        self._perf = None
        self._tracks = None
        self._targets = None
        self._depth = None
        self._last_depth_frame_id: Optional[int] = None
        self._overlay_counter = 0
        if self.enabled:
            os.makedirs(self.overlay_dir, exist_ok=True)
            self._perf_fh = open(os.path.join(base_dir, "perf.csv"), "a", newline="", encoding="utf-8")
            self._tracks_fh = open(os.path.join(base_dir, "tracks.csv"), "a", newline="", encoding="utf-8")
            self._targets_fh = open(os.path.join(base_dir, "targets.csv"), "a", newline="", encoding="utf-8")
            self._depth_fh = open(os.path.join(base_dir, "depth.csv"), "a", newline="", encoding="utf-8")
            self._perf = csv.DictWriter(self._perf_fh, fieldnames=_PERF_FIELDS)
            self._tracks = csv.DictWriter(self._tracks_fh, fieldnames=_TRACK_FIELDS)
            self._targets = csv.DictWriter(self._targets_fh, fieldnames=_TARGET_FIELDS)
            self._depth = csv.DictWriter(self._depth_fh, fieldnames=_DEPTH_FIELDS)
            for fh, writer in [
                (self._perf_fh, self._perf),
                (self._tracks_fh, self._tracks),
                (self._targets_fh, self._targets),
                (self._depth_fh, self._depth),
            ]:
                if fh.tell() == 0:
                    writer.writeheader()

    @classmethod
    def from_env(cls) -> "FieldRecorder":
        return cls(
            enabled=_env_bool("TECHX_DEBUG_RECORDER", False),
            base_dir=os.getenv("TECHX_DEBUG_DIR", "runs/field_001"),
            frame_every=_env_int("TECHX_DEBUG_FRAME_EVERY", 10),
        )

    def record_stats(self, stats: Dict[str, Any], engine: Any) -> None:
        if not self.enabled:
            return
        now = time.time()
        frame_id = int(stats.get("frame_id", 0) or 0)
        try:
            self._perf.writerow({
                "time": now,
                "frame_id": frame_id,
                "avg_infer_ms": stats.get("avg_infer_ms", 0.0),
                "avg_display_ms": stats.get("avg_display_ms", 0.0),
                "track_count": stats.get("track_count", 0),
                "raw_target_count": stats.get("raw_target_count", 0),
                "target_count": stats.get("target_count", 0),
                "sender_ready": int(bool(stats.get("sender_ready", False))),
                "camera_frame_age_sec": stats.get("camera_frame_age_sec", 0.0),
                "infer_result_age_sec": stats.get("infer_result_age_sec", 0.0),
                "fatal_reason": stats.get("fatal_reason", ""),
            })
            self._write_tracks(now, frame_id, getattr(engine, "tracks", []))
            self._write_targets(now, frame_id, getattr(engine, "targets", []))
            self._write_depth(now, frame_id, getattr(getattr(engine, "solver", None), "last_depth_debug", []))
            self._flush()
        except Exception:
            pass

    def record_frame(self, frame: Any) -> None:
        if not self.enabled or frame is None:
            return
        try:
            self._overlay_counter += 1
            if self._overlay_counter % self.frame_every != 0:
                return
            path = os.path.join(self.overlay_dir, f"overlay_{self._overlay_counter:06d}.jpg")
            cv2.imwrite(path, frame)
        except Exception:
            pass

    def _write_tracks(self, now: float, frame_id: int, tracks: Iterable[Any]) -> None:
        for t in tracks:
            x1, y1, x2, y2 = [float(v) for v in t.box[:4]]
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            self._tracks.writerow({
                "time": now,
                "frame_id": frame_id,
                "track_id": int(t.track_id),
                "class_id": int(t.cls_id),
                "class_name": t.cls_name,
                "conf": float(t.conf),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "cx": cx, "cy": cy,
                "z": float(t.z),
                "confirm_count": int(t.confirm_count),
                "matched_this_frame": int(bool(t.matched_this_frame)),
                "stale_age": int(t.stale_age),
                "color": t.color or "",
            })

    def _write_targets(self, now: float, frame_id: int, targets: Iterable[Any]) -> None:
        for t in targets:
            u, v = t.pixel_uv
            x, y, z = t.camera_xyz
            self._targets.writerow({
                "time": now,
                "frame_id": frame_id,
                "track_id": int(t.track_id),
                "class_id": int(t.class_id),
                "class_name": t.class_name,
                "conf": float(t.confidence),
                "raw_conf": float(getattr(t, "raw_confidence", t.confidence)),
                "u": float(u),
                "v": float(v),
                "camera_x": float(x),
                "camera_y": float(y),
                "camera_z": float(z),
                "depth_m": float(t.depth_m),
                "color": t.color or "",
                "can_grab": int(bool(getattr(t, "can_grab", True))),
                "reject_reason": getattr(t, "reject_reason", ""),
                "quality_score": float(getattr(t, "quality_score", 1.0)),
                "edge_margin_px": float(getattr(t, "edge_margin_px", 0.0)),
                "center_error_norm": float(getattr(t, "center_error_norm", 0.0)),
                "depth_valid_ratio": float(getattr(t, "depth_valid_ratio", 0.0)),
                "depth_spread_m": float(getattr(t, "depth_spread_m", 0.0)),
                "depth_fallback_used": int(bool(getattr(t, "depth_fallback_used", False))),
            })

    def _write_depth(self, now: float, frame_id: int, rows: Iterable[Dict[str, Any]]) -> None:
        if self._last_depth_frame_id == frame_id:
            return
        self._last_depth_frame_id = frame_id
        for row in rows or []:
            out = {k: row.get(k, "") for k in _DEPTH_FIELDS}
            out["time"] = now
            out["frame_id"] = frame_id
            self._depth.writerow(out)

    def _flush(self) -> None:
        for fh in [self._perf_fh, self._tracks_fh, self._targets_fh, self._depth_fh]:
            if fh is not None:
                fh.flush()

    def close(self) -> None:
        for fh in [self._perf_fh, self._tracks_fh, self._targets_fh, self._depth_fh]:
            try:
                if fh is not None:
                    fh.close()
            except Exception:
                pass
