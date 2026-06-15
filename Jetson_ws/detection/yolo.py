"""YOLO detector implementation with backend fallback and cached backend probing."""

from __future__ import annotations

import os
import time
from collections import Counter
from typing import Dict, List, Optional, Tuple

import numpy as np

from interface.exceptions import ModelNotFoundError
from interface.interfaces import IDetector
from interface.types import Detection, Frame
from tuning.tuning import TuningParams
from utils.geometry import box_iou
from utils.logger import get_logger

log = get_logger(__name__)

_BACKEND_MAP = {
    "tensorrt": "TensorRT (.engine)",
    "cuda": "PyTorch CUDA (.pt)",
    "onnx_gpu": "ONNX Runtime GPU (.onnx)",
    "onnx": "ONNX Runtime CPU (.onnx)",
    "pytorch": "PyTorch CPU (.pt)",
}
_PRIORITY = ["tensorrt", "cuda", "onnx_gpu", "onnx", "pytorch"]
_AVAILABLE_BACKENDS_CACHE: Optional[List[str]] = None


def get_available_backends(refresh: bool = False) -> List[str]:
    global _AVAILABLE_BACKENDS_CACHE
    if _AVAILABLE_BACKENDS_CACHE is not None and not refresh:
        return list(_AVAILABLE_BACKENDS_CACHE)

    backends: List[str] = []
    try:
        import tensorrt  # noqa: F401
        backends.append("tensorrt")
        log.info("TensorRT available")
    except ImportError:
        pass

    try:
        import torch
        if torch.cuda.is_available():
            backends.append("cuda")
            try:
                log.info("CUDA: %s (Compute %s)", torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0))
            except Exception:
                log.info("CUDA available")
    except ImportError:
        pass

    try:
        import onnxruntime as ort
        providers = ort.get_available_providers()
        if "CUDAExecutionProvider" in providers or "TensorrtExecutionProvider" in providers:
            backends.append("onnx_gpu")
        else:
            backends.append("onnx")
        log.info("ONNX Runtime providers: %s", providers)
    except ImportError:
        pass

    try:
        import ultralytics  # noqa: F401
        backends.append("pytorch")
    except ImportError:
        pass

    _AVAILABLE_BACKENDS_CACHE = backends
    return list(backends)


def _preferred_file(files: List[str], suffix: str) -> str:
    """Choose deployment artifact deterministically."""
    if not files:
        return ""
    names = sorted(files)
    for name in (f"best{suffix}", f"model{suffix}", f"deploy{suffix}"):
        if name in names:
            return name
    return names[0]


def scan_models_folder(base_dir: str) -> dict:
    models_dir = os.path.join(base_dir, "models")
    found: dict = {}
    if not os.path.isdir(models_dir):
        return found
    for folder in sorted(os.listdir(models_dir)):
        fp = os.path.join(models_dir, folder)
        if not os.path.isdir(fp):
            continue
        pts = [f for f in os.listdir(fp) if f.endswith(".pt")]
        onnxs = [f for f in os.listdir(fp) if f.endswith(".onnx")]
        engs = [f for f in os.listdir(fp) if f.endswith(".engine")]
        if not (pts or onnxs or engs):
            continue
        pt = _preferred_file(pts, ".pt")
        onnx = _preferred_file(onnxs, ".onnx")
        engine = _preferred_file(engs, ".engine")
        found[folder.replace("_", " ").title()] = {
            "pt": os.path.join(models_dir, folder, pt) if pt else "",
            "onnx": os.path.join(models_dir, folder, onnx) if onnx else "",
            "engine": os.path.join(models_dir, folder, engine) if engine else "",
            "folder": folder,
        }
    return found


def optimize_jetson() -> None:
    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            if "NVIDIA Jetson" not in f.read():
                return
    except Exception:
        return
    log.info("Jetson detected — applying performance optimisations")
    for cpu in range(8):
        try:
            with open(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor", "w", encoding="utf-8") as f:
                f.write("performance")
        except Exception:
            pass
    os.system("nvpmodel -m 0 2>/dev/null")
    os.system("jetson_clocks 2>/dev/null")


def _file_ok(path: str) -> bool:
    return bool(path) and os.path.exists(path)


def _mtime_label(path: str) -> str:
    if not _file_ok(path):
        return "-"
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path)))
    except Exception:
        return "?"


class YoloDetector(IDetector):
    def __init__(
        self,
        pt_path: str,
        onnx_path: str = "",
        engine_path: str = "",
        class_confs: Optional[Dict[str, float]] = None,
        class_names: Optional[Dict[int, str]] = None,
        backend: str = "auto",
        tuning: Optional[TuningParams] = None,
        class_id_offset: int = 0,
        class_id_map: Optional[Dict[str, int]] = None,
    ):
        self._pt_path = pt_path
        self._onnx_path = onnx_path
        self._engine_path = engine_path
        self._class_confs = class_confs or {}
        self._class_names = class_names or {}
        self._requested_backend = backend
        self._tuning = tuning or TuningParams()
        self._class_id_offset = int(class_id_offset)
        self._class_id_map = class_id_map or {}
        self._backend_key = "pytorch"
        self._backend_label = _BACKEND_MAP[self._backend_key]
        self._model_pt = None
        self._model_onnx = None
        self._model_engine = None
        self._loaded = False
        self._fallback_order: List[str] = []
        self._diag_frame = 0
        self._last_diag_log = 0.0

    def load(self) -> bool:
        self._patch_jetson_nms()
        get_available_backends(refresh=True)
        last_error = ""
        for backend in self._candidate_backends(self._requested_backend):
            ok, err = self._ensure_backend_loaded(backend)
            if ok:
                self._set_active_backend(backend)
                self._sync_class_names()
                self._loaded = True
                log.info(
                    "YOLO loaded — backend=%s pt=%s mtime=%s onnx=%s mtime=%s engine=%s mtime=%s classes=%d offset=%d",
                    self._backend_label,
                    os.path.basename(self._pt_path) if self._pt_path else "-",
                    _mtime_label(self._pt_path),
                    os.path.basename(self._onnx_path) if self._onnx_path else "-",
                    _mtime_label(self._onnx_path),
                    os.path.basename(self._engine_path) if self._engine_path else "-",
                    _mtime_label(self._engine_path),
                    len(self._class_names),
                    self._class_id_offset,
                )
                self._log_class_map()
                if self._class_confs:
                    log.info("YOLO class thresholds: %s", self._class_confs)
                return True
            last_error = err
            log.warning("Backend %s unavailable: %s", backend, err)
        raise ModelNotFoundError(f"No usable YOLO backend/model. Last error: {last_error}")

    def detect(self, frame: Frame) -> List[Detection]:
        if not self._loaded or frame.bgr is None:
            return []
        results = self._infer(frame.bgr)
        if results is None:
            return []
        return self._parse_results(results, frame.bgr.shape[:2])

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def class_names(self) -> dict:
        return self._class_names

    @property
    def backend_name(self) -> str:
        return self._backend_label

    def set_backend(self, backend: str) -> str:
        self._requested_backend = backend
        for candidate in self._candidate_backends(backend):
            ok, err = self._ensure_backend_loaded(candidate)
            if ok:
                self._set_active_backend(candidate)
                self._sync_class_names()
                log.info("Backend switched → %s", self._backend_label)
                return self._backend_label
            log.warning("Backend switch candidate %s failed: %s", candidate, err)
        log.warning("Backend switch failed; keeping %s", self._backend_label)
        return self._backend_label

    def get_class_conf(self, class_name: str) -> float:
        return self._class_confs.get(class_name, self._tuning.conf_threshold)

    def _candidate_backends(self, requested: str) -> List[str]:
        available = set(get_available_backends())
        file_ready = {
            "tensorrt": _file_ok(self._engine_path),
            "cuda": _file_ok(self._pt_path),
            "onnx_gpu": _file_ok(self._onnx_path),
            "onnx": _file_ok(self._onnx_path),
            "pytorch": _file_ok(self._pt_path),
        }
        order = _PRIORITY if requested == "auto" else [requested] + [b for b in _PRIORITY if b != requested]
        if os.getenv("TECHX_NO_TORCH_BACKEND", "0").strip().lower() in {"1", "true", "yes", "on"}:
            order = [b for b in order if b not in {"cuda", "pytorch"}]
        return [b for b in order if b in available and file_ready.get(b, False)]

    def _set_active_backend(self, backend: str) -> None:
        self._backend_key = backend
        self._backend_label = _BACKEND_MAP.get(backend, backend)
        self._fallback_order = [backend] + [b for b in self._candidate_backends("auto") if b != backend]

    def _ensure_backend_loaded(self, backend: str) -> Tuple[bool, str]:
        if backend == "tensorrt":
            return (True, "") if self._load_engine() else (False, f"engine missing/load failed: {self._engine_path}")
        if backend in ("onnx", "onnx_gpu"):
            return (True, "") if self._load_onnx() else (False, f"onnx missing/load failed: {self._onnx_path}")
        if backend in ("cuda", "pytorch"):
            return (True, "") if self._load_pt() else (False, f"pt missing/load failed: {self._pt_path}")
        return False, f"unknown backend: {backend}"

    def _patch_jetson_nms(self) -> None:
        try:
            import torchvision.ops as _tv_ops
            from ultralytics.utils.nms import TorchNMS
            if not hasattr(_tv_ops, "_jetson_patched"):
                _tv_ops.nms = TorchNMS.nms
                _tv_ops._jetson_patched = True
        except Exception:
            pass

    def _load_pt(self) -> bool:
        if self._model_pt is not None:
            return True
        if not _file_ok(self._pt_path):
            return False
        try:
            from ultralytics import YOLO
            self._model_pt = YOLO(self._pt_path)
            return True
        except Exception as e:
            log.error("PyTorch YOLO load failed: %s", e)
            return False

    def _load_engine(self) -> bool:
        if self._model_engine is not None:
            return True
        if not _file_ok(self._engine_path):
            return False
        try:
            from ultralytics import YOLO
            self._model_engine = YOLO(self._engine_path, task="detect")
            return True
        except Exception as e:
            log.error("TensorRT load failed: %s", e)
            return False

    def _load_onnx(self) -> bool:
        if self._model_onnx is not None:
            return True
        if not _file_ok(self._onnx_path):
            return False
        try:
            from ultralytics import YOLO
            self._model_onnx = YOLO(self._onnx_path, task="detect")
            return True
        except Exception as e:
            log.error("ONNX load failed: %s", e)
            return False

    def _sync_class_names(self) -> None:
        model_names = {}
        for model in (self._model_engine, self._model_pt, self._model_onnx):
            names = getattr(model, "names", None)
            if names:
                model_names = {int(i): str(n) for i, n in names.items()}
                break
        if not model_names:
            return
        if self._class_names:
            mismatch = [
                idx for idx, name in self._class_names.items()
                if idx not in model_names or str(model_names[idx]) != str(name)
            ]
            if mismatch or len(self._class_names) != len(model_names):
                log.warning(
                    "Configured class_names do not match YOLO model names; using model names. configured=%s model=%s",
                    self._class_names,
                    model_names,
                )
                self._class_names = model_names
            return
        self._class_names = model_names

    def _log_class_map(self) -> None:
        model = {
            "tensorrt": self._model_engine,
            "onnx": self._model_onnx,
            "onnx_gpu": self._model_onnx,
            "cuda": self._model_pt,
            "pytorch": self._model_pt,
        }.get(self._backend_key)
        model_names = getattr(model, "names", {}) if model is not None else {}
        configured = ", ".join(f"{k}:{v}" for k, v in sorted(self._class_names.items())) or "none"
        model_map = ", ".join(f"{k}:{v}" for k, v in sorted(model_names.items())) if isinstance(model_names, dict) else str(model_names)
        log.info("YOLO class map configured=[%s] model_names=[%s]", configured, model_map)

    def _infer(self, bgr: np.ndarray):
        for backend in (self._fallback_order or [self._backend_key]):
            ok, _ = self._ensure_backend_loaded(backend)
            if not ok:
                continue
            model = {
                "tensorrt": self._model_engine,
                "onnx": self._model_onnx,
                "onnx_gpu": self._model_onnx,
                "cuda": self._model_pt,
                "pytorch": self._model_pt,
            }.get(backend)
            if model is None:
                continue
            try:
                kwargs = dict(conf=self._tuning.conf_threshold, iou=self._tuning.iou_threshold,
                              verbose=False, imgsz=self._tuning.img_size)
                if backend in ("cuda", "pytorch"):
                    kwargs["device"] = 0 if backend == "cuda" else "cpu"
                result = model(bgr, **kwargs)
                if backend != self._backend_key:
                    log.warning("Inference backend fallback: %s → %s", self._backend_key, backend)
                    self._set_active_backend(backend)
                return result
            except Exception as e:
                log.error("Inference error on %s: %s", backend, e)
        return None

    def _remap_class(self, local_id: int, local_name: str) -> Tuple[int, str]:
        mapped_id = self._class_id_map.get(local_name)
        if mapped_id is None:
            mapped_id = self._class_id_map.get(str(local_id))
        if mapped_id is None:
            mapped_id = local_id + self._class_id_offset
        return int(mapped_id), local_name

    def _parse_results(self, results, image_shape: Tuple[int, int]) -> List[Detection]:
        if results is None or len(results) == 0:
            return []
        boxes_obj = results[0].boxes
        if boxes_obj is None or len(boxes_obj) == 0:
            self._log_detection_diagnostics(0, 0, 0, 0, [], [])
            return []
        raw: List[Detection] = []
        xyxy, confs, cls_ids = boxes_obj.xyxy, boxes_obj.conf, boxes_obj.cls
        for i in range(len(xyxy)):
            local_id = int(cls_ids[i])
            local_name = self._class_names.get(local_id, f"class_{local_id}")
            cls_id, cls_name = self._remap_class(local_id, local_name)
            raw.append(Detection(
                box=xyxy[i].cpu().numpy().astype(np.float32),
                cls_id=cls_id,
                cls_name=cls_name,
                conf=float(confs[i]),
            ))
        return self._filter(raw, image_shape)

    def _filter(self, detections: List[Detection], image_shape: Tuple[int, int]) -> List[Detection]:
        p = self._tuning
        img_h, img_w = image_shape
        kept: List[Detection] = []
        for d in detections:
            if d.conf < self.get_class_conf(d.cls_name):
                continue
            if d.area < p.min_box_area:
                continue
            if p.max_box_area > 0 and d.area > p.max_box_area:
                continue
            w, h = d.width, d.height
            if w <= 0 or h <= 0:
                continue
            if max(w / h, h / w) > p.max_aspect_ratio:
                continue
            cx, cy = d.center
            if cx < p.edge_margin_x or cx > img_w - p.edge_margin_x:
                continue
            if cy < p.edge_margin_y or cy > img_h - p.edge_margin_y:
                continue
            edge_box = d.box[0] <= 2 or d.box[1] <= 2 or d.box[2] >= img_w - 2 or d.box[3] >= img_h - 2
            if edge_box and d.conf < p.edge_low_confidence:
                continue
            kept.append(d)

        if len(kept) <= 1:
            self._log_detection_diagnostics(len(detections), len(kept), len(kept), len(kept), detections, kept)
            return kept

        same_nms = self._same_class_nms(kept)
        final = self._cross_class_dedup(same_nms)
        if p.max_detections_per_frame > 0 and len(final) > p.max_detections_per_frame:
            final = sorted(final, key=lambda d: d.conf, reverse=True)[:p.max_detections_per_frame]
        self._log_detection_diagnostics(len(detections), len(kept), len(same_nms), len(final), detections, final)
        return final

    def _same_class_nms(self, detections: List[Detection]) -> List[Detection]:
        p = self._tuning
        merged: List[Detection] = []
        for d in sorted(detections, key=lambda x: x.conf, reverse=True):
            duplicate = False
            for k in merged:
                if d.cls_id == k.cls_id and box_iou(d.box, k.box) > p.nms_iou_threshold:
                    duplicate = True
                    break
            if not duplicate:
                merged.append(d)
        return merged

    @staticmethod
    def _box_iou_and_containment(b1: np.ndarray, b2: np.ndarray) -> Tuple[float, float]:
        x1 = max(float(b1[0]), float(b2[0]))
        y1 = max(float(b1[1]), float(b2[1]))
        x2 = min(float(b1[2]), float(b2[2]))
        y2 = min(float(b1[3]), float(b2[3]))
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        a1 = max(0.0, float(b1[2] - b1[0])) * max(0.0, float(b1[3] - b1[1]))
        a2 = max(0.0, float(b2[2] - b2[0])) * max(0.0, float(b2[3] - b2[1]))
        iou = inter / (a1 + a2 - inter) if a1 + a2 - inter > 0 else 0.0
        containment = max(inter / a1 if a1 > 0 else 0.0, inter / a2 if a2 > 0 else 0.0)
        return float(iou), float(containment)

    @staticmethod
    def _is_fake_kfs(det: Detection) -> bool:
        return int(det.cls_id) == 0 or (det.cls_name or "").lower() == "fake_kfs"

    @staticmethod
    def _is_real_kfs(det: Detection) -> bool:
        return 1 <= int(det.cls_id) <= 4 or "_kfs_" in (det.cls_name or "").lower()

    def _prefer_duplicate(self, candidate: Detection, kept: Detection) -> bool:
        """Return True when candidate should replace an already-kept duplicate."""
        margin = float(self._tuning.fake_kfs_prefer_margin)
        candidate_fake = self._is_fake_kfs(candidate)
        kept_fake = self._is_fake_kfs(kept)
        if candidate_fake != kept_fake and (self._is_real_kfs(candidate) or self._is_real_kfs(kept)):
            if candidate_fake and candidate.conf + margin >= kept.conf:
                return True
            if kept_fake and kept.conf + margin >= candidate.conf:
                return False
        return candidate.conf > kept.conf

    def _cross_class_dedup(self, detections: List[Detection]) -> List[Detection]:
        """Suppress duplicate boxes even when the model gave different classes.

        This directly addresses the competition symptom: one physical KFS object
        appears with several class labels.  We keep the highest-confidence box
        when boxes strongly overlap or their centers are almost identical.
        """
        p = self._tuning
        if len(detections) <= 1 or p.cross_class_nms_iou_threshold <= 0:
            return detections
        kept: List[Detection] = []
        for d in sorted(detections, key=lambda x: x.conf, reverse=True):
            dcx, dcy = d.center
            is_dup = False
            for k in kept:
                iou, containment = self._box_iou_and_containment(d.box, k.box)
                kcx, kcy = k.center
                center_dist = float(np.hypot(dcx - kcx, dcy - kcy))
                strong_overlap = iou >= p.cross_class_nms_iou_threshold
                nested = containment >= p.containment_threshold
                near_center = center_dist <= p.cross_class_center_px and iou >= p.cross_class_near_iou_threshold
                if strong_overlap or nested or near_center:
                    is_dup = True
                    if self._prefer_duplicate(d, k):
                        kept.remove(k)
                        kept.append(d)
                        log.debug(
                            "cross-class duplicate replaced keep=%s %.2f drop=%s %.2f iou=%.2f containment=%.2f dist=%.1f",
                            d.cls_name, d.conf, k.cls_name, k.conf, iou, containment, center_dist,
                        )
                    elif d.cls_id != k.cls_id:
                        log.debug(
                            "cross-class duplicate suppressed keep=%s %.2f drop=%s %.2f iou=%.2f containment=%.2f dist=%.1f",
                            k.cls_name, k.conf, d.cls_name, d.conf, iou, containment, center_dist,
                        )
                    break
            if not is_dup:
                kept.append(d)
        return kept

    def _log_detection_diagnostics(
        self,
        raw_count: int,
        kept_count: int,
        same_nms_count: int,
        final_count: int,
        raw: List[Detection],
        final: List[Detection],
    ) -> None:
        """Periodically log box counts so duplicate-box problems are diagnosable."""
        self._diag_frame += 1
        now = time.monotonic()
        suppressed = kept_count - final_count
        suspicious = (
            raw_count >= 6
            or kept_count >= 6
            or final_count >= 4
            or raw_count > max(final_count, 1) * 2
            or suppressed >= 2
        )
        if not suspicious and now - self._last_diag_log < 3.0:
            return
        if now - self._last_diag_log < 0.8:
            return
        self._last_diag_log = now
        raw_summary = ", ".join(f"{k}:{v}" for k, v in Counter(d.cls_name for d in raw).most_common()) or "none"
        final_summary = ", ".join(f"{k}:{v}" for k, v in Counter(d.cls_name for d in final).most_common()) or "none"
        best = ", ".join(f"{d.cls_name}:{d.conf:.2f}" for d in sorted(final, key=lambda x: x.conf, reverse=True)[:5]) or "none"
        msg = (
            "YOLO boxes diag frame=%d backend=%s raw=%d kept=%d same_nms=%d final=%d suppressed=%d "
            "conf=%.2f iou=%.2f same_post_nms=%.2f cross_nms=%.2f center_px=%.1f raw_cls=[%s] final_cls=[%s] best=[%s]"
        )
        args = (
            self._diag_frame,
            self._backend_label,
            raw_count,
            kept_count,
            same_nms_count,
            final_count,
            suppressed,
            self._tuning.conf_threshold,
            self._tuning.iou_threshold,
            self._tuning.nms_iou_threshold,
            self._tuning.cross_class_nms_iou_threshold,
            self._tuning.cross_class_center_px,
            raw_summary,
            final_summary,
            best,
        )
        if suspicious:
            log.warning(msg, *args)
        else:
            log.info(msg, *args)
