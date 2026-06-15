from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from interface.interfaces import IDetector
from interface.types import Detection, Frame


class CodeMarkerDetector(IDetector):
    def __init__(
        self,
        class_id: int = 200,
        class_name: str = "qr_code",
        detect_every_n: int = 1,
        min_size_px: int = 18,
        decoded_confidence: float = 1.0,
        detected_confidence: float = 0.65,
    ):
        self.class_id = int(class_id)
        self.class_name = class_name
        self.detect_every_n = max(1, int(detect_every_n))
        self.min_size_px = max(4, int(min_size_px))
        self.decoded_confidence = max(0.0, min(1.0, float(decoded_confidence)))
        self.detected_confidence = max(0.0, min(1.0, float(detected_confidence)))
        self._detector = cv2.QRCodeDetector()
        self._loaded = True
        self._counter = 0

    def load(self) -> bool:
        self._loaded = True
        return True

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def class_names(self) -> dict:
        return {self.class_id: self.class_name}

    @property
    def backend_name(self) -> str:
        return "OpenCV QRCodeDetector"

    def detect(self, frame: Frame) -> List[Detection]:
        self._counter += 1
        if not self._loaded or frame.bgr is None:
            return []
        if (self._counter - 1) % self.detect_every_n != 0:
            return []
        gray = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2GRAY)
        try:
            text, points, _ = self._detector.detectAndDecode(gray)
        except Exception:
            return []
        conf = self.decoded_confidence if text else self.detected_confidence
        det = self._points_to_detection(points, frame.bgr.shape, conf)
        return [det] if det is not None else []

    def _points_to_detection(self, points, image_shape, conf: float) -> Optional[Detection]:
        if points is None:
            return None
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        if pts.shape[0] < 4:
            return None
        h, w = image_shape[:2]
        x1 = float(np.clip(np.min(pts[:, 0]), 0, w - 1))
        y1 = float(np.clip(np.min(pts[:, 1]), 0, h - 1))
        x2 = float(np.clip(np.max(pts[:, 0]), 0, w - 1))
        y2 = float(np.clip(np.max(pts[:, 1]), 0, h - 1))
        if x2 - x1 < self.min_size_px or y2 - y1 < self.min_size_px:
            return None
        return Detection(
            box=np.array([x1, y1, x2, y2], dtype=np.float32),
            cls_id=self.class_id,
            cls_name=self.class_name,
            conf=max(0.0, min(1.0, float(conf))),
        )
