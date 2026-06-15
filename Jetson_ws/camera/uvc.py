"""UVC / USB 网络摄像头回退方案 —— ICamera 接口实现。

当 Orbbec SDK 不可用或深度相机断开连接时使用。
本相机不提供深度图 —— 管线需要回退到界面中手动输入的深度值。

支持的后端: V4L2 (Linux)、DSHOW (Windows)、默认后端 (macOS)。
"""

from __future__ import annotations

import sys
import time
from typing import Optional

import cv2
import numpy as np

from interface.interfaces import ICamera
from interface.types import Frame
from interface.exceptions import CameraNotFoundError
from utils.logger import get_logger

log = get_logger(__name__)


def _is_color_frame(frame) -> bool:
    return frame is not None and getattr(frame, "ndim", 0) == 3 and frame.shape[2] == 3


class UvcCamera(ICamera):
    """标准 UVC / USB 网络摄像头。"""

    def __init__(self, width: int = 640, height: int = 480):
        self._width = width
        self._height = height
        self._cap: Optional[cv2.VideoCapture] = None
        self._opened = False
        self._frame_id = 0
        self._backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_DSHOW

    def open(self) -> bool:
        best_cap = None
        best_idx = -1
        best_score = -1

        for idx in range(8):
            cap = cv2.VideoCapture(idx, self._backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            ret, frame = cap.read()
            if ret and _is_color_frame(frame):
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                score = (actual_w == self._width) + (actual_h == self._height)
                if score > best_score:
                    if best_cap is not None:
                        best_cap.release()
                    best_cap = cap
                    best_idx = idx
                    best_score = score
                    if score == 2:
                        self._cap = best_cap
                        self._opened = True
                        log.info("UVC camera — index=%d %dx%d (完美匹配)", idx, actual_w, actual_h)
                        return True
                else:
                    cap.release()
            else:
                cap.release()

        if best_cap is not None:
            self._cap = best_cap
            self._opened = True
            log.info(
                "UVC camera — index=%d %dx%d (最佳匹配)",
                best_idx,
                int(best_cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(best_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            )
            return True

        log.warning("No UVC color camera found at indices 0–7, trying default")
        self._cap = cv2.VideoCapture(0)
        if self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret and _is_color_frame(frame):
                self._opened = True
                return True
            self._cap.release()
            self._cap = None

        raise CameraNotFoundError("No UVC color camera found (indices 0–7)")

    def grab(self) -> Optional[Frame]:
        if not self._opened or self._cap is None:
            return None

        ret, bgr = self._cap.read()
        if not ret or not _is_color_frame(bgr):
            return None

        h, w = bgr.shape[:2]
        if w != self._width or h != self._height:
            bgr = cv2.resize(bgr, (self._width, self._height))

        self._frame_id += 1
        return Frame(
            bgr=bgr,
            depth_mm=None,
            timestamp=time.time(),
            frame_id=self._frame_id,
        )

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._opened = False
        log.info("UVC camera closed")

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def intrinsics(self) -> tuple:
        return (0.0, 0.0, 0.0, 0.0)

    @property
    def has_depth(self) -> bool:
        return False
