from __future__ import annotations

import os
import queue
import threading
import time
from collections import deque
from typing import List, Optional, Tuple

import cv2
import numpy as np

from interface.interfaces import ICamera, IDetector, ITracker, ISender
from interface.types import Frame, Track, Target3D
from tuning.tuning import TuningParams
from utils.logger import get_logger

log = get_logger(__name__)
InferResult = Tuple[List[Track], Frame, float]


class PipelineEngine:
    def __init__(self, camera: ICamera, detector: IDetector, tracker: ITracker,
                 solver, visualizer, sender: Optional[ISender] = None,
                 tuning: Optional[TuningParams] = None):
        self.camera = camera
        self.detector = detector
        self.tracker = tracker
        self.solver = solver
        self.visualizer = visualizer
        self.sender = sender
        self._tuning = tuning or TuningParams()
        # Only graspable classes (KFS, weapon heads) may be dropped for lacking
        # depth. Event/alignment targets (assembly light 150-152, QR 200) must
        # survive the output filter even with z<=0 so GMK can still consume their
        # color/u/v/align_err. GMK's require_control_xyz gate handles grasp safety.
        self._grasp_classes = set(int(x) for x in self._tuning.quality_gate_classes)
        self._infer_queue: queue.Queue = queue.Queue(maxsize=self._tuning.infer_queue_maxsize)
        self._result_queue: queue.Queue = queue.Queue(maxsize=2)
        self._infer_running = False
        self._infer_thread: Optional[threading.Thread] = None
        self._running = False
        self._paused = False
        self._show_depth = False
        self._frame_count = 0
        self._infer_times: deque = deque(maxlen=self._tuning.stat_window)
        self._display_times: deque = deque(maxlen=self._tuning.stat_window)
        self._last_infer_ms = 0.0
        self._latest_tracks: List[Track] = []
        self._latest_targets: List[Target3D] = []
        self._latest_ts = 0.0
        self._manual_depth_m = 1.2
        self._last_frame: Optional[np.ndarray] = None
        self._last_depth: Optional[np.ndarray] = None
        now = time.monotonic()
        self._last_camera_frame_monotonic = now
        self._last_infer_result_monotonic = now
        self._fatal_no_camera_frame_timeout_sec = 600.0
        self._fatal_no_infer_result_timeout_sec = 600.0
        self._fatal_exit_code = 2
        self._fatal_reason: Optional[str] = None
        self._health_interval_sec = self._read_env_float("TECHX_HEALTH_LOG_INTERVAL", 2.0)
        self._infer_every_n = max(1, int(self._read_env_float("TECHX_INFER_EVERY_N", 1)))
        self._last_health_log_monotonic = now
        self._camera_frames_since_health = 0
        self._infer_results_since_health = 0
        self._send_packets_since_health = 0
        self._infer_queue_drops_since_health = 0
        self._result_queue_drops_since_health = 0
        self._last_sent_target_count = 0
        self._last_raw_target_count = 0
        self._last_output_filter_log_monotonic = now
        self.on_target: Optional[callable] = None
        self.on_stats: Optional[callable] = None
        self.on_frame: Optional[callable] = None

    def set_runtime_timeouts(self, no_camera_frame_sec: float = 600.0, no_infer_result_sec: float = 600.0,
                             fatal_exit_code: int = 2) -> None:
        self._fatal_no_camera_frame_timeout_sec = float(no_camera_frame_sec)
        self._fatal_no_infer_result_timeout_sec = float(no_infer_result_sec)
        self._fatal_exit_code = int(fatal_exit_code)

    @property
    def fatal_reason(self) -> Optional[str]:
        return self._fatal_reason

    def start(self) -> bool:
        log.info("流水线启动中…")
        if not self.camera.is_open and not self.camera.open():
            log.error("相机打开失败")
            return False
        if self.camera.has_depth:
            fx, fy, cx, cy = self.camera.intrinsics
            if fx > 0:
                self.solver.set_intrinsics(fx, fy, cx, cy)
        self.solver.set_frame_size(self.camera.width, self.camera.height)
        self.visualizer.set_frame_size(self.camera.width, self.camera.height)
        now = time.monotonic()
        self._last_camera_frame_monotonic = now
        self._last_infer_result_monotonic = now
        self._last_health_log_monotonic = now
        self._last_output_filter_log_monotonic = now
        self._fatal_reason = None
        self._infer_running = True
        self._running = True
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()
        log.info("流水线已启动 — %dx%d", self.camera.width, self.camera.height)
        return True

    def stop(self) -> None:
        if not self._running and not self._infer_running:
            return
        log.info("流水线停止中…")
        self._running = False
        self._infer_running = False
        try:
            self._infer_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._infer_thread is not None:
            self._infer_thread.join(timeout=2.0)
            self._infer_thread = None
        self.camera.close()
        if self.sender is not None:
            self.sender.close()
        log.info("流水线已停止")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def tracks(self) -> List[Track]:
        return self._latest_tracks

    @property
    def targets(self) -> List[Target3D]:
        return self._latest_targets

    def set_manual_depth(self, metres: float) -> None:
        self._manual_depth_m = max(0.0, metres)

    def toggle_depth_view(self) -> bool:
        self._show_depth = not self._show_depth
        return self._show_depth

    def set_paused(self, paused: bool) -> None:
        self._paused = paused

    @property
    def is_paused(self) -> bool:
        return self._paused

    def tick(self) -> Optional[np.ndarray]:
        if not self._running or self._paused:
            return None

        t_start = time.time()
        self._frame_count += 1
        current_frame = self.camera.grab()
        if current_frame is not None:
            self._last_camera_frame_monotonic = time.monotonic()
            self._camera_frames_since_health += 1
            self._last_frame = current_frame.bgr
            self._last_depth = current_frame.depth_mm
            self._latest_ts = current_frame.timestamp
            if self._frame_count % self._infer_every_n == 0:
                self._submit_for_inference(current_frame)
        elif self._last_frame is None:
            if self._check_runtime_timeouts():
                return self._blank_frame(self._fatal_reason or "Stopped")
            return self._blank_frame("No camera signal")
        elif self._check_runtime_timeouts():
            return self._blank_frame(self._fatal_reason or "Stopped")

        display_frame = current_frame or Frame(self._last_frame, self._last_depth, self._latest_ts)
        tracks = self._latest_tracks
        targets = self._latest_targets
        fresh_targets: Optional[List[Target3D]] = None
        fresh_timestamp: Optional[float] = None

        try:
            tracks, infer_frame, infer_ms = self._result_queue.get_nowait()
            self._last_infer_result_monotonic = time.monotonic()
            self._infer_results_since_health += 1
            self._last_infer_ms = infer_ms
            self._infer_times.append(infer_ms)
            self._latest_tracks = tracks
            self._latest_ts = infer_frame.timestamp
            display_frame = infer_frame

            live_tracks = [t for t in tracks if t.is_fresh]
            if live_tracks:
                live_tracks = self.solver.apply_depth_to_tracks(
                    live_tracks,
                    infer_frame.depth_mm if self.camera.has_depth else None,
                    self._manual_depth_m if not self.camera.has_depth else 0.0,
                )
                live_tracks = self.solver.classify_colors(live_tracks, infer_frame.bgr)

            raw_targets = self.solver.solve_all(
                live_tracks, infer_frame.timestamp,
                confirm_frames=self._tuning.track_confirm_frames,
            )
            live_ids = {t.track_id for t in live_tracks if t.is_fresh}
            raw_targets = [t for t in raw_targets if t.track_id in live_ids]
            targets = self._filter_output_targets(raw_targets)
            self._latest_targets = targets
            fresh_targets = targets
            fresh_timestamp = infer_frame.timestamp
        except queue.Empty:
            pass

        if self._check_runtime_timeouts():
            return self._blank_frame(self._fatal_reason or "Stopped")

        if fresh_targets is not None:
            self._send_targets(fresh_targets, timestamp=fresh_timestamp)

        ann = self.visualizer.draw(
            display_frame.bgr, tracks,
            show_depth_overlay=self._show_depth,
            depth_mm=display_frame.depth_mm,
            confirm_frames=self._tuning.track_confirm_frames,
            targets=targets,
        )
        display_ms = (time.time() - t_start) * 1000.0
        self._display_times.append(display_ms)
        avg_infer = float(np.mean(self._infer_times)) if self._infer_times else 0.0
        avg_display = float(np.mean(self._display_times)) if self._display_times else 0.0

        self._maybe_log_health_snapshot(avg_infer, avg_display, tracks, targets, display_frame.depth_mm)

        if self.on_stats:
            now_mono = time.monotonic()
            self.on_stats({
                "avg_infer_ms": avg_infer,
                "avg_display_ms": avg_display,
                "track_count": len([t for t in tracks if t.is_fresh]),
                "target_count": len(targets),
                "raw_target_count": self._last_raw_target_count,
                "frame_id": self._frame_count,
                "sender_ready": self.sender.is_ready if self.sender else False,
                "camera_frame_age_sec": max(0.0, now_mono - self._last_camera_frame_monotonic),
                "infer_result_age_sec": max(0.0, now_mono - self._last_infer_result_monotonic),
                "fatal_reason": self._fatal_reason or "",
            })
        if self.on_target and fresh_targets:
            self.on_target(fresh_targets[0])

        detect_fps = 1000.0 / avg_infer if avg_infer > 0 else 0.0
        graspable = sum(1 for t in targets if getattr(t, "can_grab", True) and t.z > 0)
        valid_xyz = sum(1 for t in targets if t.z > 0)
        ann = self.visualizer.draw_status_bar(ann, detect_fps, avg_infer, len(tracks), graspable=graspable, valid_xyz=valid_xyz)
        if self.on_frame:
            self.on_frame(ann)
        return ann

    def _filter_output_targets(self, targets: List[Target3D]) -> List[Target3D]:
        self._last_raw_target_count = len(targets)
        p = self._tuning
        kept: List[Target3D] = []
        reject_conf = 0
        reject_depth = 0
        for t in targets:
            if t.confidence < p.output_min_confidence:
                reject_conf += 1
                continue
            # Never drop event/alignment targets (lightbar, QR) for missing depth;
            # only graspable classes are subject to the optional depth requirement.
            if p.output_require_valid_depth and t.z <= 0 and int(t.class_id) in self._grasp_classes:
                reject_depth += 1
                continue
            kept.append(t)

        if p.output_sort_by_confidence:
            # Prefer grasp-ready/valid-XYZ targets first so a crowded frame cannot
            # fill the packet with visible-only edge/clipped targets. Visible-only
            # targets are still sent after valid targets for CENTER_TARGET behavior.
            kept = sorted(
                kept,
                key=lambda t: (
                    1 if getattr(t, "can_grab", True) else 0,
                    1 if t.z > 0 else 0,
                    float(getattr(t, "quality_score", 0.0)),
                    float(t.confidence),
                ),
                reverse=True,
            )
        else:
            kept = sorted(kept, key=lambda t: (t.z <= 0, t.z if t.z > 0 else 999.0))

        truncated = 0
        if p.output_max_targets_per_frame > 0 and len(kept) > p.output_max_targets_per_frame:
            truncated = len(kept) - p.output_max_targets_per_frame
            kept = kept[:p.output_max_targets_per_frame]

        now = time.monotonic()
        changed = len(kept) != len(targets) or reject_conf > 0 or reject_depth > 0 or truncated > 0
        if p.output_log_interval_sec > 0 and (changed or now - self._last_output_filter_log_monotonic >= p.output_log_interval_sec):
            self._last_output_filter_log_monotonic = now
            best = ", ".join(
                f"{t.class_name}:{t.confidence:.2f}:z={t.z:.2f}:grab={int(getattr(t, 'can_grab', True))}"
                for t in kept[:5]
            ) or "none"
            log.info(
                "OUTPUT filter raw=%d sent=%d reject_conf=%d reject_depth=%d truncated=%d min_conf=%.2f require_depth=%s best=[%s]",
                len(targets), len(kept), reject_conf, reject_depth, truncated,
                p.output_min_confidence, "1" if p.output_require_valid_depth else "0", best,
            )
        return kept

    def _fatal_stop(self, reason: str) -> None:
        if self._fatal_reason is None:
            self._fatal_reason = reason
            log.error("运行保护触发: %s", reason)
        self.stop()
        os._exit(self._fatal_exit_code)

    def _check_runtime_timeouts(self) -> bool:
        if not self._running or self._paused:
            return False
        now_mono = time.monotonic()
        if self._fatal_no_camera_frame_timeout_sec > 0:
            age = now_mono - self._last_camera_frame_monotonic
            if age > self._fatal_no_camera_frame_timeout_sec:
                self._fatal_stop(f"no camera frame for {age:.1f}s")
                return True
        if self._fatal_no_infer_result_timeout_sec > 0:
            age = now_mono - self._last_infer_result_monotonic
            if age > self._fatal_no_infer_result_timeout_sec:
                self._fatal_stop(f"no inference result for {age:.1f}s")
                return True
        return False

    def _submit_for_inference(self, frame_obj: Frame) -> None:
        try:
            self._infer_queue.put_nowait(frame_obj)
        except queue.Full:
            self._infer_queue_drops_since_health += 1
            try:
                self._infer_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._infer_queue.put_nowait(frame_obj)
            except queue.Full:
                pass

    def _send_targets(self, targets: List[Target3D], timestamp: Optional[float] = None) -> None:
        if self.sender is None or not self.sender.is_ready:
            return
        try:
            sent = self.sender.send_many(targets, timestamp=timestamp)
            if sent > 0:
                self._send_packets_since_health += sent
                self._last_sent_target_count = len(targets)
            if sent <= 0:
                log.debug("目标/状态批量下发返回0")
        except TypeError:
            if not targets:
                return
            sent = self.sender.send_many(targets)
            if sent > 0:
                self._send_packets_since_health += sent
                self._last_sent_target_count = len(targets)
            if sent <= 0:
                log.debug("目标批量下发返回0")
        except AttributeError:
            if not targets:
                return
            nearest = targets[0]
            if nearest.z > 0 and not self.sender.send(nearest):
                log.debug("UDP发送返回False")
            else:
                self._send_packets_since_health += 1
                self._last_sent_target_count = 1

    def _blank_frame(self, text: str) -> np.ndarray:
        blank = np.zeros((self.camera.height, self.camera.width, 3), dtype=np.uint8)
        cv2.putText(blank, text, (10, self.camera.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return blank

    def _infer_loop(self) -> None:
        log.info("推理线程已启动")
        while self._infer_running:
            try:
                item = self._infer_queue.get(timeout=0.5)
                if item is None:
                    break
                frame_obj: Frame = item
                t0 = time.time()
                detections = self.detector.detect(frame_obj)
                tracks = self.tracker.update(detections)
                infer_ms = (time.time() - t0) * 1000.0
                self._put_result((self._snapshot_tracks(tracks), frame_obj, infer_ms))
            except queue.Empty:
                continue
            except Exception as e:
                log.error("推理线程异常: %s", e)
        log.info("推理线程已停止")

    def _snapshot_tracks(self, tracks: List[Track]) -> List[Track]:
        snap: List[Track] = []
        for t in tracks:
            snap.append(Track(
                track_id=t.track_id,
                box=t.box.copy(),
                cls_id=t.cls_id,
                cls_name=t.cls_name,
                conf=t.conf,
                z=t.z,
                last_seen=t.last_seen,
                confirm_count=t.confirm_count,
                color=t.color,
                color_hist=list(t.color_hist),
                cls_scores=dict(getattr(t, "cls_scores", {}) or {}),
                cls_names=dict(getattr(t, "cls_names", {}) or {}),
                smooth_box=t.smooth_box.copy() if getattr(t, "smooth_box", None) is not None else None,
                smoothed_conf=float(getattr(t, "smoothed_conf", t.conf)),
                matched_this_frame=t.matched_this_frame,
                stale_age=t.stale_age,
            ))
        return snap

    def _put_result(self, result: InferResult) -> None:
        try:
            self._result_queue.put_nowait(result)
        except queue.Full:
            self._result_queue_drops_since_health += 1
            try:
                self._result_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._result_queue.put_nowait(result)
            except queue.Full:
                pass

    def _maybe_log_health_snapshot(
        self,
        avg_infer_ms: float,
        avg_display_ms: float,
        tracks: List[Track],
        targets: List[Target3D],
        depth_mm: Optional[np.ndarray],
    ) -> None:
        if self._health_interval_sec <= 0:
            return
        now = time.monotonic()
        dt = now - self._last_health_log_monotonic
        if dt < self._health_interval_sec:
            return

        camera_fps = self._camera_frames_since_health / dt if dt > 0 else 0.0
        infer_fps = self._infer_results_since_health / dt if dt > 0 else 0.0
        udp_fps = self._send_packets_since_health / dt if dt > 0 else 0.0
        live_tracks = [t for t in tracks if t.is_fresh]
        valid_targets = [t for t in targets if t.z > 0]
        stale_tracks = [t for t in tracks if not t.is_fresh]
        queue_info = (
            f"infer_q={self._infer_queue.qsize()}/{self._infer_queue.maxsize} drops={self._infer_queue_drops_since_health} "
            f"result_q={self._result_queue.qsize()}/{self._result_queue.maxsize} drops={self._result_queue_drops_since_health}"
        )
        depth_info = "no_depth"
        if depth_mm is not None:
            valid = depth_mm[(depth_mm > self._tuning.depth_min_mm) & (depth_mm <= self._tuning.depth_max_mm)]
            ratio = float(len(valid)) / float(depth_mm.size) if depth_mm.size else 0.0
            med = float(np.median(valid)) / 1000.0 if len(valid) else 0.0
            depth_info = f"depth_valid={ratio:.3f} med={med:.2f}m"
        best = ", ".join(
            f"#{t.track_id}:{t.class_name}:{t.confidence:.2f}:z={t.z:.2f}:grab={int(getattr(t, 'can_grab', True))}"
            for t in targets[:3]
        ) or "none"
        temp_info = self._read_jetson_temp_info()
        mem_info = self._read_mem_info()
        log.info(
            "HEALTH cam_fps=%.1f infer_fps=%.1f udp_fps=%.1f infer_ms=%.1f display_ms=%.1f "
            "tracks=%d stale=%d raw_targets=%d sent_targets=%d valid_xyz=%d last_sent=%d | %s | %s | %s | %s | targets=[%s]",
            camera_fps, infer_fps, udp_fps, avg_infer_ms, avg_display_ms,
            len(live_tracks), len(stale_tracks), self._last_raw_target_count, len(targets), len(valid_targets),
            self._last_sent_target_count, queue_info, depth_info, temp_info, mem_info, best,
        )
        self._last_health_log_monotonic = now
        self._camera_frames_since_health = 0
        self._infer_results_since_health = 0
        self._send_packets_since_health = 0
        self._infer_queue_drops_since_health = 0
        self._result_queue_drops_since_health = 0

    @staticmethod
    def _read_env_float(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _read_jetson_temp_info() -> str:
        paths = [
            "/sys/devices/virtual/thermal/thermal_zone0/temp",
            "/sys/devices/virtual/thermal/thermal_zone1/temp",
            "/sys/devices/virtual/thermal/thermal_zone2/temp",
        ]
        vals = []
        for p in paths:
            try:
                with open(p, "r", encoding="utf-8") as f:
                    raw = f.read().strip()
                vals.append(float(raw) / 1000.0)
            except Exception:
                continue
        if not vals:
            return "temp=na"
        return "temp=%.1fC" % max(vals)

    @staticmethod
    def _read_mem_info() -> str:
        try:
            data = {}
            with open("/proc/meminfo", "r", encoding="utf-8") as f:
                for line in f:
                    k, v = line.split(":", 1)
                    data[k] = float(v.strip().split()[0]) / 1024.0
            total = data.get("MemTotal", 0.0)
            avail = data.get("MemAvailable", 0.0)
            used = max(0.0, total - avail)
            return "mem=%.0f/%.0fMB" % (used, total)
        except Exception:
            return "mem=na"
