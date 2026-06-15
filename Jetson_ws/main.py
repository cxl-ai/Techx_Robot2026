"""TECHX vision main entry for Jetson field runtime."""

from __future__ import annotations

import argparse
import importlib
import os
import sys
import time
from typing import Optional

import numpy as np
import numpy.core  # noqa: F401

for _n in [
    "multiarray", "umath", "numerictypes", "_internal", "shape_base",
    "fromnumeric", "arrayprint", "records", "defchararray", "getlimits",
    "einsumfunc", "function_base", "machar", "numeric", "type_check",
    "nanfunctions",
]:
    try:
        sys.modules.setdefault(f"numpy._core.{_n}", importlib.import_module(f"numpy.core.{_n}"))
    except ImportError:
        pass


def _patch_torchvision_for_jetson() -> None:
    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            if "NVIDIA Jetson" not in f.read():
                return
    except Exception:
        return
    try:
        import warnings
        import torchvision.extension as _ext

        def _patched_assert_has_ops():
            if not getattr(_ext, "_has_ops", False):
                warnings.warn("torchvision C++ ops not available on Jetson; using Python NMS fallback")

        _ext._assert_has_ops = _patched_assert_has_ops
        import torchvision.ops as _tv_ops
        from ultralytics.utils.nms import TorchNMS

        def _python_nms(boxes, scores, iou_threshold):
            return TorchNMS.nms(boxes, scores, iou_threshold)

        _tv_ops.nms = _python_nms
        _tv_ops.batched_nms = lambda boxes, scores, idxs, iou_threshold: _python_nms(boxes, scores, iou_threshold)
    except Exception:
        pass


_patch_torchvision_for_jetson()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from camera.orbbec import OrbbecCamera
from camera.uvc import UvcCamera
from communication.udp import UdpSender
from config.settings import Settings, load_settings
from detection.assembly_light import AssemblyLightDetector
from detection.code_marker import CodeMarkerDetector
from detection.composite import CompositeDetector
from detection.tracker import IouTracker
from detection.yolo import YoloDetector, optimize_jetson
from diagnostics.field_recorder import FieldRecorder
from pipeline.engine import PipelineEngine
from pipeline.solver import CoordinateSolver
from pipeline.visualizer import Visualizer
from tuning.tuning import TuningParams
from utils.logger import get_logger, setup_logging
from utils.time_sync import check_time_sync

log = get_logger(__name__)
_STAGE_CHOICES = {"all", "head", "gesture", "assembly", "kfs", "qr"}
_STAGE_CONFIRM_DEFAULTS = {"head": 3, "gesture": 3, "assembly": 2, "kfs": 3, "qr": 1}


def _resolve_stage() -> str:
    stage = os.environ.get("TECHX_STAGE", "all").strip().lower() or "all"
    if stage not in _STAGE_CHOICES:
        log.error("Invalid TECHX_STAGE=%r; expected %s", stage, sorted(_STAGE_CHOICES))
        raise SystemExit(2)
    return stage


def _apply_stage_confirm_frames(settings: Settings, stage: str) -> None:
    raw = os.environ.get("TECHX_CONFIRM_FRAMES", "").strip()
    if raw:
        try:
            settings.display.confirm_frames = max(1, int(raw))
            log.info("confirm_frames override=%d", settings.display.confirm_frames)
            return
        except ValueError:
            log.error("Invalid TECHX_CONFIRM_FRAMES=%r", raw)
            raise SystemExit(2)
    if stage in _STAGE_CONFIRM_DEFAULTS:
        settings.display.confirm_frames = _STAGE_CONFIRM_DEFAULTS[stage]
        log.info("stage=%s confirm_frames=%d", stage, settings.display.confirm_frames)


def create_camera(settings: Settings, wait: bool = True, retry_sec: float = 1.0, wait_timeout: Optional[float] = None) -> tuple:
    w, h = settings.camera.width, settings.camera.height
    deadline = time.monotonic() + wait_timeout if wait_timeout and wait_timeout > 0 else None
    while True:
        try:
            cam = OrbbecCamera(width=w, height=h)
            if cam.open():
                log.info("camera: Orbbec RGB-D")
                return cam, True
            cam.close()
        except Exception as e:
            log.debug("Orbbec init failed: %s", e)
        if not settings.runtime.require_orbbec:
            try:
                cam = UvcCamera(width=w, height=h)
                if cam.open():
                    log.warning("camera: UVC fallback")
                    return cam, False
                cam.close()
            except Exception as e:
                log.debug("UVC init failed: %s", e)
        if not wait:
            raise RuntimeError("no usable camera")
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(f"camera wait timeout ({wait_timeout:.1f}s)")
        sleep_s = retry_sec if deadline is None else max(0.05, min(retry_sec, deadline - time.monotonic()))
        log.warning("waiting for camera... retry in %.1fs", sleep_s)
        time.sleep(sleep_s)


def _create_sender(monitor, settings: Settings):
    status = monitor.check_network(settings.udp.target_ip, settings.udp.target_port)
    if status.ready:
        log.info("network ready: %s", status.detail)
    else:
        log.warning("network not ready: %s; local UDP bind is still fail-fast", status.detail)
    return UdpSender(settings.udp.target_ip, settings.udp.target_port, local_ip=settings.udp.local_ip)


def _stage_wants_model(stage: str, model) -> bool:
    folder = (model.folder or "").lower()
    name = (model.name or "").lower()
    if stage == "all":
        return bool(model.enabled)
    if stage == "head":
        return "head" in folder or "weapon" in name
    if stage == "gesture":
        return "gesture" in folder or "gesture" in name
    if stage == "kfs":
        return "kfs" in folder or "kfs" in name or "gesture" in folder or "gesture" in name
    return False


def _model_has_weight(model) -> bool:
    return any(p and os.path.exists(p) for p in (model.engine_path, model.onnx_path, model.pt_path))


def _apply_head_weights_override(settings: Settings) -> None:
    """Model switch: TECHX_HEAD_WEIGHTS=<path> points the weapon-head model at any
    .pt/.onnx/.engine without editing config or copying files. Switch models by
    changing one env var (also settable in scripts/autostart.env)."""
    weights = os.environ.get("TECHX_HEAD_WEIGHTS", "").strip()
    if not weights:
        return
    if not os.path.exists(weights):
        log.warning("TECHX_HEAD_WEIGHTS=%s not found; keeping config models", weights)
        return
    ext = os.path.splitext(weights)[1].lower()
    targeted = False
    for m in settings.models:
        if m.enabled and int(m.class_id_offset) == 100:  # the weapon-head model
            m.pt_path = weights if ext == ".pt" else ""
            m.onnx_path = weights if ext == ".onnx" else ""
            m.engine_path = weights if ext == ".engine" else ""
            log.info("model switch: weapon-head weights -> %s (TECHX_HEAD_WEIGHTS)", weights)
            targeted = True
    if not targeted:
        log.warning("TECHX_HEAD_WEIGHTS set but no enabled weapon-head model (offset=100) to override")


def _add_yolo(detectors, model, settings: Settings, tuning: TuningParams) -> None:
    detectors.append(YoloDetector(
        pt_path=model.pt_path,
        onnx_path=model.onnx_path,
        engine_path=model.engine_path,
        class_confs=model.class_confs,
        class_names=model.class_names,
        backend=settings.inference.backend,
        tuning=tuning,
        class_id_offset=model.class_id_offset,
        class_id_map=model.class_id_map,
    ))
    log.info("detector YOLO: %s folder=%s offset=%d", model.name, model.folder, model.class_id_offset)


def _add_light(detectors, settings: Settings) -> None:
    cfg = settings.assembly_light
    if list(cfg.roi_xywh)[:4] == [0, 0, 0, 0]:
        log.warning("assembly light roi_xywh=[0,0,0,0]; full-frame detection may false-trigger")
    detectors.append(AssemblyLightDetector(
        enabled=True,
        detect_every_n=cfg.detect_every_n,
        roi_xywh=cfg.roi_xywh,
        min_area_px=cfg.min_area_px,
        max_area_ratio=cfg.max_area_ratio,
        min_width_px=cfg.min_width_px,
        min_height_px=cfg.min_height_px,
        min_aspect_ratio=cfg.min_aspect_ratio,
        max_aspect_ratio=cfg.max_aspect_ratio,
        min_fill_ratio=cfg.min_fill_ratio,
        min_saturation=cfg.min_saturation,
        min_brightness=cfg.min_brightness,
        confidence=cfg.confidence,
        event_confidence=cfg.event_confidence,
        max_components_per_color=cfg.max_components_per_color,
        emit_color_event=cfg.emit_color_event,
        emit_success_event=cfg.emit_success_event,
        colors=cfg.colors or None,
    ))
    log.info("detector assembly_light enabled roi=%s colors=%d", cfg.roi_xywh, len(cfg.colors) if cfg.colors else 3)


def create_detector(settings: Settings, tuning: TuningParams, stage: str = "all") -> CompositeDetector:
    detectors = []
    log.info("TECHX_STAGE=%s", stage)
    missing_weight_models = []
    for model in settings.models:
        if _stage_wants_model(stage, model):
            if not _model_has_weight(model):
                # Only models actually selected for this stage are required. A stage
                # that excludes a model (e.g. TECHX_STAGE=kfs excludes the weapon head)
                # will never reach here, so a missing head model does not block KFS.
                missing_weight_models.append(model.folder or model.name)
                log.error(
                    "model '%s' (folder=%s) is enabled for stage=%s but has no best.engine/onnx/pt; "
                    "put weights in models/%s/ or run a stage that excludes it (e.g. TECHX_STAGE=kfs skips the weapon head)",
                    model.name, model.folder, stage, model.folder,
                )
                continue
            _add_yolo(detectors, model, settings, tuning)
        else:
            log.info("skip model: %s folder=%s stage=%s enabled=%s", model.name, model.folder, stage, model.enabled)
    if missing_weight_models and settings.runtime.strict_detector_load:
        log.error(
            "strict_detector_load=true and %d selected model(s) missing weights: %s. "
            "Fix the weights, set the model enabled=false in config.json, or use TECHX_STAGE to select only available models.",
            len(missing_weight_models), ", ".join(missing_weight_models),
        )
        sys.exit(2)
    if stage in {"all", "assembly"} and (settings.assembly_light.enabled or stage == "assembly"):
        _add_light(detectors, settings)
    if stage in {"all", "qr"} and settings.qr.enabled:
        detectors.append(CodeMarkerDetector(
            class_id=settings.qr.class_id,
            class_name=settings.qr.class_name,
            detect_every_n=settings.qr.detect_every_n,
            min_size_px=settings.qr.min_size_px,
            decoded_confidence=settings.qr.decoded_confidence,
            detected_confidence=settings.qr.detected_confidence,
        ))
        log.info("detector QR enabled class_id=%d", settings.qr.class_id)
    if not detectors:
        log.error("no detector enabled for stage=%s", stage)
        sys.exit(2)
    detector = CompositeDetector(detectors, strict_load=settings.runtime.strict_detector_load)
    if not detector.load():
        log.error("detector load failed strict=%s stage=%s", settings.runtime.strict_detector_load, stage)
        sys.exit(2)
    return detector


def main() -> None:
    parser = argparse.ArgumentParser(description="TECHX_vision field runtime")
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--log-level", type=str, default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--camera-wait-timeout", type=float, default=600.0)
    parser.add_argument("--no-camera-frame-timeout", type=float, default=600.0)
    parser.add_argument("--no-infer-result-timeout", type=float, default=600.0)
    camera_mode = parser.add_mutually_exclusive_group()
    camera_mode.add_argument("--require-orbbec", dest="require_orbbec", action="store_true", default=None)
    camera_mode.add_argument("--allow-uvc-fallback", dest="require_orbbec", action="store_false")
    args = parser.parse_args()

    import logging
    setup_logging(level=getattr(logging, args.log_level))
    stage = _resolve_stage()
    config_path = os.path.abspath(args.config or os.path.join(BASE_DIR, "config.json"))
    log.info("TECHX_vision start Python=%s config=%s stage=%s", sys.version.split()[0], config_path, stage)

    if os.environ.get("TECHX_SKIP_JETSON_OPT", "0") == "1":
        log.info("skip Jetson optimization")
    else:
        optimize_jetson()
    if os.environ.get("TECHX_SKIP_TIME_SYNC", "0") != "1":
        ts_ok, ts_msg = check_time_sync()
        (log.info if ts_ok else log.warning)("time sync: %s", ts_msg)

    settings = load_settings(args.config)
    if args.require_orbbec is not None:
        settings.runtime.require_orbbec = args.require_orbbec
    _apply_stage_confirm_frames(settings, stage)
    _apply_head_weights_override(settings)
    log.info(
        "settings: models=%d qr=%s light=%s udp_local=%s udp_target=%s:%d confirm=%d qgate=%s edge=%d center=%.2f depth_ratio=%.2f depth_spread=%.3f",
        len(settings.models), settings.qr.enabled, settings.assembly_light.enabled,
        settings.udp.local_ip, settings.udp.target_ip, settings.udp.target_port,
        settings.display.confirm_frames,
        settings.output.quality_gate_enabled,
        settings.output.quality_edge_margin_px,
        settings.output.quality_center_error_norm,
        settings.output.quality_min_depth_valid_ratio,
        settings.output.quality_max_depth_spread_m,
    )

    tuning = TuningParams(
        conf_threshold=settings.inference.conf_threshold,
        iou_threshold=settings.inference.iou_threshold,
        img_size=settings.inference.img_size,
        min_box_area=settings.inference.min_box_area,
        max_box_area=settings.inference.max_box_area,
        max_aspect_ratio=settings.inference.max_aspect_ratio,
        edge_low_confidence=settings.inference.edge_low_confidence,
        nms_iou_threshold=settings.inference.nms_iou_threshold,
        containment_threshold=settings.inference.containment_threshold,
        cross_class_nms_iou_threshold=settings.inference.cross_class_nms_iou_threshold,
        cross_class_near_iou_threshold=settings.inference.cross_class_near_iou_threshold,
        cross_class_center_px=settings.inference.cross_class_center_px,
        fake_kfs_prefer_margin=settings.inference.fake_kfs_prefer_margin,
        max_detections_per_frame=settings.inference.max_detections_per_frame,
        output_min_confidence=settings.output.min_confidence,
        output_require_valid_depth=settings.output.require_valid_depth,
        output_max_targets_per_frame=settings.output.max_targets_per_frame,
        output_sort_by_confidence=settings.output.sort_by_confidence,
        output_log_interval_sec=settings.output.log_interval_sec,
        quality_gate_enabled=settings.output.quality_gate_enabled,
        quality_edge_margin_px=settings.output.quality_edge_margin_px,
        quality_center_error_norm=settings.output.quality_center_error_norm,
        quality_min_depth_valid_ratio=settings.output.quality_min_depth_valid_ratio,
        quality_max_depth_spread_m=settings.output.quality_max_depth_spread_m,
        quality_require_no_fallback=settings.output.quality_require_no_fallback,
        quality_bad_confidence=settings.output.quality_bad_confidence,
        track_confirm_frames=settings.display.confirm_frames,
        display_every_n=settings.display.display_every_n,
        depth_offset_mm=settings.depth.offset_mm,
        depth_scale_corr=settings.depth.scale_corr,
        depth_min_mm=settings.depth.min_mm,
        depth_max_mm=settings.depth.max_mm,
    )
    log.info(
        "depth correction: z_corr_mm = z_raw_mm * %.4f + %.1fmm | valid range [%.0f, %.0f]mm (config.json depth.*)",
        tuning.depth_scale_corr, tuning.depth_offset_mm, tuning.depth_min_mm, tuning.depth_max_mm,
    )

    from utils.device_monitor import DeviceMonitor
    monitor = DeviceMonitor()
    wait_timeout = args.camera_wait_timeout if args.camera_wait_timeout > 0 else None
    try:
        camera, is_orbbec = create_camera(settings, wait=True, wait_timeout=wait_timeout)
    except RuntimeError as e:
        log.error("camera start failed: %s", e)
        sys.exit(3)

    detector = create_detector(settings, tuning, stage=stage)
    tracker = IouTracker(tuning=tuning)
    fx, fy, cx, cy = settings.camera.fx, settings.camera.fy, settings.camera.cx, settings.camera.cy
    if is_orbbec:
        cfx, cfy, ccx, ccy = camera.intrinsics
        if cfx > 0:
            fx, fy, cx, cy = cfx, cfy, ccx, ccy
    solver = CoordinateSolver(fx=fx, fy=fy, cx=cx, cy=cy, tuning=tuning, transformer=None)
    visualizer = Visualizer(tuning=tuning)
    sender = _create_sender(monitor, settings)
    recorder = FieldRecorder.from_env()
    if recorder.enabled:
        log.info("field recorder enabled dir=%s frame_every=%d", recorder.base_dir, recorder.frame_every)

    engine = PipelineEngine(camera, detector, tracker, solver, visualizer, sender=sender, tuning=tuning)
    engine.set_runtime_timeouts(args.no_camera_frame_timeout, args.no_infer_result_timeout)
    engine.on_stats = lambda stats: recorder.record_stats(stats, engine)
    engine.on_frame = recorder.record_frame
    try:
        if not engine.start():
            sys.exit(1)
        if args.headless:
            _run_headless(engine)
        else:
            _run_ui(engine, settings, tuning)
    finally:
        recorder.close()


def _run_ui(engine, settings: Settings, tuning: TuningParams) -> None:
    from ui.app import TechxVisionApp
    TechxVisionApp(engine, settings, tuning).run()


def _run_headless(engine: PipelineEngine) -> None:
    log.info("headless mode; Ctrl+C to stop")
    try:
        while engine.is_running:
            engine.tick()
            time.sleep(0.001)
        if engine.fatal_reason:
            log.error("runtime stopped: %s", engine.fatal_reason)
    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        engine.stop()


if __name__ == "__main__":
    main()
