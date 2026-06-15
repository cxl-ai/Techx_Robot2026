from __future__ import annotations

import csv
import os
import time
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from interface.interfaces import IDetector
from interface.types import Detection, Frame
from utils.logger import get_logger

log = get_logger(__name__)

ASSEMBLY_SUCCESS_ID = 152

# Default lightbar colors. Orange is the active R1 assembly light; red/blue are
# reserved so other decisions can key off a color later without code changes.
# Each entry: name, class_id, h_min, h_max (OpenCV hue 0-180; h_min>h_max wraps
# around 0/180, e.g. red), success. success=True also emits the generic
# assembly_success event (class 152). class_id 152 is reserved for that event.
DEFAULT_LIGHT_COLORS = [
    {"name": "assembly_light_orange", "class_id": 150, "h_min": 11, "h_max": 25, "success": True},
    {"name": "assembly_light_red", "class_id": 151, "h_min": 168, "h_max": 12, "success": False},
    {"name": "assembly_light_blue", "class_id": 153, "h_min": 95, "h_max": 135, "success": False},
]


def normalize_light_colors(raw) -> List[dict]:
    out: List[dict] = []
    seen_ids = set()
    for c in (raw or []):
        try:
            name = str(c["name"])
            class_id = int(c["class_id"])
            h_min = max(0, min(180, int(c["h_min"])))
            h_max = max(0, min(180, int(c["h_max"])))
            success = bool(c.get("success", False))
        except (KeyError, TypeError, ValueError):
            continue
        if class_id == ASSEMBLY_SUCCESS_ID or class_id in seen_ids:
            continue  # 152 reserved for success; skip duplicate class_ids
        seen_ids.add(class_id)
        out.append({"name": name, "class_id": class_id, "h_min": h_min, "h_max": h_max, "success": success})
    return out or [dict(c) for c in DEFAULT_LIGHT_COLORS]


class AssemblyLightDetector(IDetector):
    def __init__(
        self,
        enabled: bool = True,
        detect_every_n: int = 1,
        roi_xywh: Optional[Sequence[int]] = None,
        min_area_px: int = 80,
        max_area_ratio: float = 0.08,
        min_width_px: int = 8,
        min_height_px: int = 3,
        min_aspect_ratio: float = 2.0,
        max_aspect_ratio: float = 25.0,
        min_fill_ratio: float = 0.30,
        min_saturation: int = 80,
        min_brightness: int = 140,
        confidence: float = 0.85,
        event_confidence: float = 0.90,
        max_components_per_color: int = 2,
        emit_color_event: bool = True,
        emit_success_event: bool = True,
        colors: Optional[Sequence[dict]] = None,
    ):
        self.enabled = bool(enabled)
        self.detect_every_n = max(1, int(detect_every_n))
        self.roi_xywh = list(roi_xywh or [0, 0, 0, 0])[:4]
        while len(self.roi_xywh) < 4:
            self.roi_xywh.append(0)
        self.min_area_px = max(1, int(min_area_px))
        self.max_area_ratio = max(0.0, float(max_area_ratio))
        self.min_width_px = max(1, int(min_width_px))
        self.min_height_px = max(1, int(min_height_px))
        self.min_aspect_ratio = max(1.0, float(min_aspect_ratio))
        self.max_aspect_ratio = max(self.min_aspect_ratio, float(max_aspect_ratio))
        self.min_fill_ratio = max(0.01, min(1.0, float(min_fill_ratio)))
        self.min_saturation = max(0, min(255, int(min_saturation)))
        self.min_brightness = max(0, min(255, int(min_brightness)))
        self.confidence = max(0.0, min(1.0, float(confidence)))
        self.event_confidence = max(0.0, min(1.0, float(event_confidence)))
        self.max_components_per_color = max(1, int(max_components_per_color))
        self.emit_color_event = bool(emit_color_event)
        self.emit_success_event = bool(emit_success_event)
        self.colors = normalize_light_colors(colors)
        self._loaded = False
        self._counter = 0
        self._debug_dir = os.getenv("TECHX_LIGHT_DEBUG_DIR", "").strip()
        self._debug_csv_fh = None
        self._debug_csv = None
        if self._debug_dir:
            os.makedirs(self._debug_dir, exist_ok=True)
            self._debug_csv_fh = open(os.path.join(self._debug_dir, "light_debug.csv"), "a", newline="", encoding="utf-8")
            self._debug_csv = csv.DictWriter(
                self._debug_csv_fh,
                fieldnames=["time", "frame_id", "color", "x", "y", "w", "h", "area", "fill", "aspect", "conf", "roi_x", "roi_y", "roi_w", "roi_h"],
            )
            if self._debug_csv_fh.tell() == 0:
                self._debug_csv.writeheader()

    def load(self) -> bool:
        self._loaded = self.enabled
        return self._loaded

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def class_names(self) -> dict:
        names = {c["class_id"]: c["name"] for c in self.colors}
        names[ASSEMBLY_SUCCESS_ID] = "assembly_success"
        return names

    @property
    def backend_name(self) -> str:
        return "OpenCV AssemblyLightDetector"

    def detect(self, frame: Frame) -> List[Detection]:
        self._counter += 1
        if not self._loaded or frame.bgr is None:
            return []
        if (self._counter - 1) % self.detect_every_n != 0:
            return []
        bgr, offset = self._crop_roi(frame.bgr)
        if bgr.size == 0:
            return []
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        common = (s >= self.min_saturation) & (v >= self.min_brightness)
        self._save_roi_image(frame.frame_id, bgr)
        out: List[Detection] = []
        for color in self.colors:
            mask = self._color_mask(h, common, color["h_min"], color["h_max"])
            self._save_mask_image(frame.frame_id, color["name"], mask)
            out.extend(self._components(mask, offset, color["class_id"], color["name"], frame.frame_id, color["success"]))
        return out

    @staticmethod
    def _color_mask(h: np.ndarray, common: np.ndarray, h_min: int, h_max: int) -> np.ndarray:
        if h_min <= h_max:
            band = (h >= h_min) & (h <= h_max)
        else:  # wrap around the 0/180 hue boundary (e.g. red)
            band = (h >= h_min) | (h <= h_max)
        return (band & common).astype(np.uint8)

    def _crop_roi(self, image: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        h, w = image.shape[:2]
        x, y, rw, rh = [int(v) for v in self.roi_xywh]
        if rw <= 0 or rh <= 0:
            return image, (0, 0)
        x1 = max(0, min(w - 1, x))
        y1 = max(0, min(h - 1, y))
        x2 = max(x1 + 1, min(w, x1 + rw))
        y2 = max(y1 + 1, min(h, y1 + rh))
        return image[y1:y2, x1:x2], (x1, y1)

    def _components(self, mask: np.ndarray, offset: Tuple[int, int], color_id: int, color_name: str, frame_id: int, success: bool = True) -> List[Detection]:
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        roi_area = float(mask.shape[0] * mask.shape[1]) if mask.size else 1.0
        cand = []
        for i in range(1, n):
            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            area = float(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_area_px:
                continue
            if self.max_area_ratio > 0 and area / roi_area > self.max_area_ratio:
                continue
            if w < self.min_width_px or h < self.min_height_px:
                continue
            aspect = max(float(w), float(h)) / max(1.0, min(float(w), float(h)))
            if aspect < self.min_aspect_ratio or aspect > self.max_aspect_ratio:
                continue
            fill = area / max(1.0, float(w * h))
            if fill < self.min_fill_ratio:
                continue
            cand.append((area * aspect * fill, x, y, w, h, area, fill, aspect))
        cand.sort(key=lambda item: item[0], reverse=True)
        ox, oy = offset
        out: List[Detection] = []
        for _score, x, y, w, h, area, fill, aspect in cand[: self.max_components_per_color]:
            box = np.array([x + ox, y + oy, x + w + ox, y + h + oy], dtype=np.float32)
            bonus = min(0.08, area / 10000.0) + min(0.04, max(0.0, fill - self.min_fill_ratio))
            color_conf = max(0.0, min(0.99, self.confidence + bonus))
            event_conf = max(0.0, min(0.99, self.event_confidence + bonus))
            self._write_debug_row(frame_id, color_name, x + ox, y + oy, w, h, area, fill, aspect, max(color_conf, event_conf))
            if self.emit_color_event:
                out.append(Detection(box=box.copy(), cls_id=color_id, cls_name=color_name, conf=color_conf))
            if self.emit_success_event and success:
                out.append(Detection(box=box.copy(), cls_id=ASSEMBLY_SUCCESS_ID, cls_name="assembly_success", conf=event_conf))
        return out

    def _save_roi_image(self, frame_id: int, roi_raw: np.ndarray) -> None:
        if not self._debug_dir:
            return
        try:
            tag = f"{int(frame_id):06d}_{self._counter:06d}"
            cv2.imwrite(os.path.join(self._debug_dir, f"roi_raw_{tag}.jpg"), roi_raw)
        except Exception:
            pass

    def _save_mask_image(self, frame_id: int, color_name: str, mask: np.ndarray) -> None:
        if not self._debug_dir:
            return
        try:
            tag = f"{int(frame_id):06d}_{self._counter:06d}"
            cv2.imwrite(os.path.join(self._debug_dir, f"mask_{color_name}_{tag}.png"), mask * 255)
        except Exception:
            pass

    def _write_debug_row(self, frame_id: int, color_name: str, x: int, y: int, w: int, h: int, area: float, fill: float, aspect: float, conf: float) -> None:
        if self._debug_csv is None:
            return
        try:
            rx, ry, rw, rh = [int(v) for v in self.roi_xywh]
            self._debug_csv.writerow({
                "time": time.time(),
                "frame_id": frame_id,
                "color": color_name,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": area,
                "fill": fill,
                "aspect": aspect,
                "conf": conf,
                "roi_x": rx,
                "roi_y": ry,
                "roi_w": rw,
                "roi_h": rh,
            })
            self._debug_csv_fh.flush()
        except Exception:
            pass
