"""Orbbec Gemini RGB-D camera implementation."""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from interface.exceptions import CameraNotFoundError, CameraTimeoutError
from interface.interfaces import ICamera
from interface.types import Frame
from utils.logger import get_logger

log = get_logger(__name__)

_pipeline_cls = _config_cls = _context_cls = None
_ob_format = _ob_sensor_type = _ob_stream_type = None
_align_filter_cls = _format_convert_filter_cls = None
_ob_convert_format = None


def _frame_array(frame, dtype=np.uint8) -> Optional[np.ndarray]:
    try:
        data = frame.get_data()
        try:
            return np.frombuffer(data, dtype=dtype)
        except TypeError:
            return np.asanyarray(data, dtype=dtype).reshape(-1)
    except Exception:
        return None


def _reshape_or_none(arr: Optional[np.ndarray], shape: tuple) -> Optional[np.ndarray]:
    if arr is None:
        return None
    expected = int(np.prod(shape))
    if arr.size < expected:
        return None
    if arr.size > expected:
        arr = arr[:expected]
    try:
        return arr.reshape(shape)
    except ValueError:
        return None


def _orbbec_to_bgr(frame) -> Optional[np.ndarray]:
    """Convert an Orbbec color VideoFrame into an OpenCV BGR image.

    This function intentionally avoids np.resize because np.resize repeats/truncates
    data silently. A malformed or strided buffer should fail cleanly instead of
    generating a visually plausible but wrong image.
    """
    try:
        w, h = int(frame.get_width()), int(frame.get_height())
        fmt = frame.get_format()
        data = _frame_array(frame, np.uint8)

        if fmt == _ob_format.RGB:
            rgb = _reshape_or_none(data, (h, w, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb is not None else None

        if fmt == _ob_format.BGR:
            return _reshape_or_none(data, (h, w, 3))

        if fmt == _ob_format.MJPG:
            if data is None or data.size == 0:
                return None
            return cv2.imdecode(data, cv2.IMREAD_COLOR)

        if fmt in (_ob_format.YUYV, _ob_format.UYVY):
            yuv = _reshape_or_none(data, (h, w, 2))
            if yuv is None:
                return None
            code = cv2.COLOR_YUV2BGR_YUYV if fmt == _ob_format.YUYV else cv2.COLOR_YUV2BGR_UYVY
            return cv2.cvtColor(yuv, code)

        if _format_convert_filter_cls is not None:
            cf = _format_convert_filter_cls()
            cf.set_format_convert_format(_ob_convert_format.I420_TO_RGB888)
            rgb_frame = cf.process(frame)
            if rgb_frame is None:
                return None
            rgb = _reshape_or_none(_frame_array(rgb_frame, np.uint8), (h, w, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb is not None else None
    except Exception as e:
        log.debug("Orbbec color convert failed: %s", e)
    return None


class OrbbecCamera(ICamera):
    """Orbbec Gemini 335L RGB-D camera through pyorbbecsdk."""

    def __init__(self, width: int = 640, height: int = 480):
        self._width = int(width)
        self._height = int(height)
        self._pipeline = None
        self._align = None
        self._running = False
        self._opened = False
        self._queue: queue.Queue = queue.Queue(maxsize=3)
        self._thread: Optional[threading.Thread] = None
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._dbg_depth = False
        self._frame_id = 0

    def open(self) -> bool:
        sdk = None

        def _import_sdk():
            nonlocal sdk
            try:
                import pyorbbecsdk as _sdk
                sdk = _sdk
            except ImportError:
                sdk = None

        t = threading.Thread(target=_import_sdk, daemon=True)
        t.start()
        t.join(timeout=6.0)
        if sdk is None:
            log.error("Orbbec SDK import timed out or failed")
            raise CameraTimeoutError("Orbbec SDK import timed out or failed")

        self._setup_globals(sdk)
        ctx = _context_cls()
        devices = ctx.query_devices()
        if devices.get_count() == 0:
            raise CameraNotFoundError("No Orbbec device connected")

        self._pipeline = _pipeline_cls()
        cfg = _config_cls()

        color_profiles = self._pipeline.get_stream_profile_list(_ob_sensor_type.COLOR_SENSOR)
        color_profile = None
        for fmt in (_ob_format.BGR, _ob_format.RGB, _ob_format.MJPG):
            try:
                color_profile = color_profiles.get_video_stream_profile(self._width, self._height, fmt, 30)
                break
            except Exception:
                continue
        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
        cfg.enable_stream(color_profile)

        depth_profiles = self._pipeline.get_stream_profile_list(_ob_sensor_type.DEPTH_SENSOR)
        try:
            depth_profile = depth_profiles.get_video_stream_profile(self._width, self._height, _ob_format.Y16, 30)
        except Exception:
            depth_profile = depth_profiles.get_default_video_stream_profile()
        cfg.enable_stream(depth_profile)

        self._pipeline.start(cfg)
        self._align = _align_filter_cls(align_to_stream=_ob_stream_type.COLOR_STREAM)
        self._read_intrinsics()

        self._running = True
        self._opened = True
        self._thread = threading.Thread(target=self._fetch_loop, daemon=True)
        self._thread.start()
        log.info(
            "Orbbec started — %dx%d @30fps | fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            self._width, self._height, self._fx, self._fy, self._cx, self._cy,
        )
        return True

    def grab(self) -> Optional[Frame]:
        try:
            bgr, depth_mm, ts = self._queue.get_nowait()
            self._frame_id += 1
            return Frame(bgr=bgr, depth_mm=depth_mm, timestamp=ts, frame_id=self._frame_id)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        self._align = None
        self._opened = False
        log.info("Orbbec camera closed")

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
        return self._fx, self._fy, self._cx, self._cy

    @property
    def has_depth(self) -> bool:
        return True

    def _setup_globals(self, mod) -> None:
        global _pipeline_cls, _config_cls, _context_cls
        global _ob_format, _ob_sensor_type, _ob_stream_type
        global _align_filter_cls, _format_convert_filter_cls, _ob_convert_format
        _pipeline_cls = mod.Pipeline
        _config_cls = mod.Config
        _context_cls = mod.Context
        _ob_format = mod.OBFormat
        _ob_sensor_type = mod.OBSensorType
        _ob_stream_type = mod.OBStreamType
        _align_filter_cls = mod.AlignFilter
        _format_convert_filter_cls = getattr(mod, "FormatConvertFilter", None)
        _ob_convert_format = getattr(mod, "OBConvertFormat", None)

    def _read_intrinsics(self) -> None:
        for _ in range(30):
            frames = self._pipeline.wait_for_frames(1000)
            if not frames:
                continue
            cf = frames.get_color_frame()
            if not cf:
                continue
            try:
                vsp = cf.get_stream_profile().as_video_stream_profile()
                try:
                    intr = vsp.get_intrinsic()
                except AttributeError:
                    intr = vsp.get_intrinsics()
                if intr.fx > 0:
                    self._fx, self._fy = float(intr.fx), float(intr.fy)
                    self._cx, self._cy = float(intr.cx), float(intr.cy)
                    return
            except Exception:
                continue
        log.warning("Orbbec intrinsics unavailable; config/default intrinsics will be used")

    def _fetch_loop(self) -> None:
        log.info("Orbbec fetch thread started")
        while self._running:
            bgr, depth, ts = self._fetch_one()
            if bgr is not None:
                self._enqueue((bgr, depth, ts))
            else:
                time.sleep(0.001)

    def _fetch_one(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
        try:
            frames = self._pipeline.wait_for_frames(200)
            if not frames:
                return None, None, 0.0
            aligned = self._align.process(frames) if self._align is not None else frames
            cf = aligned.get_color_frame()
            df = aligned.get_depth_frame()
            if not cf or not df:
                return None, None, 0.0

            bgr = _orbbec_to_bgr(cf)
            if bgr is None:
                return None, None, 0.0

            ts = float(cf.get_system_timestamp()) / 1000.0
            dh, dw = int(df.get_height()), int(df.get_width())
            depth_raw = _frame_array(df, np.uint16)
            depth_u16 = _reshape_or_none(depth_raw, (dh, dw))
            if depth_u16 is None:
                return None, None, 0.0
            depth = depth_u16.astype(np.float32) * float(df.get_depth_scale())
            if depth.shape[:2] != bgr.shape[:2]:
                depth = cv2.resize(depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)

            if not self._dbg_depth:
                self._dbg_depth = True
                valid = (depth > 200) & (depth <= 8000)
                cy, cx = depth.shape[0] // 2, depth.shape[1] // 2
                log.info(
                    "Depth scale=%.4f centre=%.1fmm valid=%.1f%% range=%.0f-%.0fmm ts=%.6fs",
                    float(df.get_depth_scale()), float(depth[cy, cx]), float(valid.mean() * 100),
                    float(depth[valid].min()) if valid.any() else 0.0,
                    float(depth[valid].max()) if valid.any() else 0.0,
                    ts,
                )
            return bgr, depth, ts
        except Exception as e:
            log.debug("Orbbec frame fetch error: %s", e)
            return None, None, 0.0

    def _enqueue(self, item: tuple) -> None:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass
