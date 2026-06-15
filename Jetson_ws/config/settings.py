"""Configuration loader for the Jetson vision runtime."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class UdpConfig:
    local_ip: str = "192.168.10.101"
    target_ip: str = "192.168.10.100"
    target_port: int = 12345


@dataclass
class ModelEntry:
    name: str = ""
    folder: str = ""
    pt_path: str = ""
    onnx_path: str = ""
    engine_path: str = ""
    enabled: bool = True
    class_confs: Dict[str, float] = field(default_factory=dict)
    class_names: Dict[int, str] = field(default_factory=dict)
    class_id_offset: int = 0
    class_id_map: Dict[str, int] = field(default_factory=dict)


@dataclass
class InferenceConfig:
    backend: str = "auto"
    conf_threshold: float = 0.45
    iou_threshold: float = 0.45
    img_size: int = 640
    min_box_area: int = 500
    max_box_area: int = 250000
    max_aspect_ratio: float = 10.0
    edge_low_confidence: float = 0.30
    nms_iou_threshold: float = 0.35
    containment_threshold: float = 0.65
    cross_class_nms_iou_threshold: float = 0.35
    cross_class_near_iou_threshold: float = 0.20
    cross_class_center_px: float = 24.0
    fake_kfs_prefer_margin: float = 0.12
    max_detections_per_frame: int = 10


@dataclass
class OutputConfig:
    min_confidence: float = 0.55
    require_valid_depth: bool = True
    max_targets_per_frame: int = 6
    sort_by_confidence: bool = True
    log_interval_sec: float = 2.0
    quality_gate_enabled: bool = True
    quality_edge_margin_px: int = 30
    quality_center_error_norm: float = 0.18
    quality_min_depth_valid_ratio: float = 0.50
    quality_max_depth_spread_m: float = 0.015
    quality_require_no_fallback: bool = True
    quality_bad_confidence: float = 0.01


@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    fx: float = 367.93
    fy: float = 368.05
    cx: float = 318.75
    cy: float = 236.20


@dataclass
class DepthConfig:
    # z_corrected_mm = z_raw_mm * scale_corr + offset_mm. Calibrate with
    # tools/depth_accuracy_report.py against a known ground-truth distance.
    offset_mm: float = -50.0
    scale_corr: float = 1.0
    min_mm: float = 200.0
    max_mm: float = 8000.0


@dataclass
class RuntimeConfig:
    require_orbbec: bool = True
    strict_detector_load: bool = True


@dataclass
class DisplayConfig:
    confirm_frames: int = 5
    display_every_n: int = 8


@dataclass
class QrConfig:
    enabled: bool = True
    class_id: int = 200
    class_name: str = "qr_code"
    detect_every_n: int = 1
    min_size_px: int = 24
    decoded_confidence: float = 1.0
    detected_confidence: float = 0.75


@dataclass
class AssemblyLightConfig:
    enabled: bool = True
    detect_every_n: int = 1
    roi_xywh: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    min_area_px: int = 80
    max_area_ratio: float = 0.08
    min_width_px: int = 8
    min_height_px: int = 3
    min_aspect_ratio: float = 2.0
    max_aspect_ratio: float = 25.0
    min_fill_ratio: float = 0.30
    min_saturation: int = 80
    min_brightness: int = 140
    confidence: float = 0.85
    event_confidence: float = 0.90
    max_components_per_color: int = 2
    emit_color_event: bool = True
    emit_success_event: bool = True
    # Configurable lightbar colors: [{name, class_id, h_min, h_max, success}, ...].
    # Empty -> detector uses its built-in default (orange primary, red/blue reserved).
    colors: List[dict] = field(default_factory=list)


@dataclass
class Settings:
    udp: UdpConfig = field(default_factory=UdpConfig)
    models: List[ModelEntry] = field(default_factory=list)
    inference: InferenceConfig = field(default_factory=InferenceConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    depth: DepthConfig = field(default_factory=DepthConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    qr: QrConfig = field(default_factory=QrConfig)
    assembly_light: AssemblyLightConfig = field(default_factory=AssemblyLightConfig)


def _preferred_file(files: List[str], suffix: str) -> str:
    if not files:
        return ""
    names = sorted(files)
    preferred = [f"best{suffix}", f"model{suffix}", f"deploy{suffix}"]
    for name in preferred:
        if name in names:
            return name
    return names[0]


def _scan_models_dir(base_dir: str) -> Dict[str, dict]:
    models_dir = os.path.join(base_dir, "models")
    found: Dict[str, dict] = {}
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
        found[folder] = {
            "pt": os.path.join(models_dir, folder, pt) if pt else "",
            "onnx": os.path.join(models_dir, folder, onnx) if onnx else "",
            "engine": os.path.join(models_dir, folder, engine) if engine else "",
        }
    return found


def _as_int_key_map(raw: dict) -> Dict[int, str]:
    out: Dict[int, str] = {}
    for k, v in (raw or {}).items():
        try:
            out[int(k)] = str(v)
        except (TypeError, ValueError):
            continue
    return out


def _as_class_id_map(raw: dict) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for k, v in (raw or {}).items():
        try:
            out[str(k)] = int(v)
        except (TypeError, ValueError):
            continue
    return out


def _class_confs(raw: dict) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for cn, cd in (raw or {}).items():
        try:
            out[str(cn)] = float(cd if isinstance(cd, (int, float)) else cd.get("conf", 0.45))
        except (TypeError, ValueError, AttributeError):
            continue
    return out


def _model_entry_from_config(base_dir: str, scanned: Dict[str, dict], cfg: dict) -> ModelEntry:
    folder = cfg.get("folder", "")
    auto = scanned.get(folder, {})
    models_dir = os.path.join(base_dir, "models")
    return ModelEntry(
        name=cfg.get("name", folder.replace("_", " ").title()),
        folder=folder,
        pt_path=auto.get("pt", os.path.join(models_dir, folder, "best.pt")),
        onnx_path=auto.get("onnx", os.path.join(models_dir, folder, "best.onnx")),
        engine_path=auto.get("engine", os.path.join(models_dir, folder, "best.engine")),
        enabled=bool(cfg.get("enabled", True)),
        class_confs=_class_confs(cfg.get("class_confs", cfg.get("classes", {}))),
        class_names=_as_int_key_map(cfg.get("class_names", {})),
        class_id_offset=int(cfg.get("class_id_offset", 0)),
        class_id_map=_as_class_id_map(cfg.get("class_id_map", {})),
    )


def _get_hardcoded_models(base_dir: str) -> List[ModelEntry]:
    mdir = os.path.join(base_dir, "models")
    return [
        ModelEntry(
            name="KFS target detection",
            folder="kfs_v3",
            pt_path=os.path.join(mdir, "kfs_v3", "best.pt"),
            onnx_path=os.path.join(mdir, "kfs_v3", "best.onnx"),
            engine_path=os.path.join(mdir, "kfs_v3", "best.engine"),
            class_confs={
                "kfs_red_r1": 0.50,
                "kfs_red_r2_fake": 0.50,
                "kfs_red_r2_true": 0.50,
                "kfs_blue_r1": 0.50,
                "kfs_blue_r2_fake": 0.50,
                "kfs_blue_r2_true": 0.50,
            },
            class_names={0: "kfs_red_r1", 1: "kfs_red_r2_fake", 2: "kfs_red_r2_true", 3: "kfs_blue_r1", 4: "kfs_blue_r2_fake", 5: "kfs_blue_r2_true"},
            class_id_map={str(i): i for i in range(6)},
        ),
        ModelEntry(
            name="Weapon head detection",
            folder="head_v1",
            pt_path=os.path.join(mdir, "head_v1", "best.pt"),
            onnx_path=os.path.join(mdir, "head_v1", "best.onnx"),
            engine_path=os.path.join(mdir, "head_v1", "best.engine"),
            enabled=False,
            class_confs={"weapon_head_fist": 0.50, "weapon_head_palm": 0.50, "weapon_head_spear": 0.50},
            class_names={0: "weapon_head_fist", 1: "weapon_head_palm", 2: "weapon_head_spear"},
            class_id_offset=100,
            class_id_map={"0": 100, "1": 101, "2": 102},
        ),
    ]


def _bool(raw, default: bool) -> bool:
    if isinstance(raw, bool):
        return raw
    if raw is None:
        return default
    if isinstance(raw, (int, float)):
        return bool(raw)
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def _int_list(raw, default: List[int], size: int = 4) -> List[int]:
    if not isinstance(raw, list):
        raw = default
    out: List[int] = []
    for item in raw[:size]:
        try:
            out.append(int(item))
        except (TypeError, ValueError):
            out.append(0)
    while len(out) < size:
        out.append(0)
    return out


def load_settings(config_path: Optional[str] = None) -> Settings:
    if config_path is None:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(base_dir, "config.json")
    else:
        base_dir = os.path.dirname(os.path.abspath(config_path))

    cfg: dict = {}
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[Config] cannot read {config_path}: {e}; using defaults")

    udp_cfg = cfg.get("udp", {})
    inf_cfg = cfg.get("inference", {})
    out_cfg = cfg.get("output", {})
    cam_cfg = cfg.get("camera", {})
    depth_cfg = cfg.get("depth", {})
    runtime_cfg = cfg.get("runtime", {})
    disp_cfg = cfg.get("display", {})
    qr_cfg = cfg.get("qr", {})
    light_cfg = cfg.get("assembly_light", {})
    models_cfg = cfg.get("models", [])
    scanned = _scan_models_dir(base_dir)
    models: List[ModelEntry] = []

    if models_cfg:
        for m in models_cfg:
            models.append(_model_entry_from_config(base_dir, scanned, m))

    config_folders = {m.folder for m in models}
    for folder, paths in scanned.items():
        if folder not in config_folders:
            # A folder on disk that is NOT in config.json is auto-DISABLED. config.json
            # is the single source of truth for what runs, so a stray/leftover model
            # folder (e.g. an old gesture net) can never be silently loaded with a
            # default class_id_offset=0 that collides with KFS class ids 0-5.
            models.append(ModelEntry(
                name=folder.replace("_", " ").title(),
                folder=folder,
                pt_path=paths.get("pt", ""),
                onnx_path=paths.get("onnx", ""),
                engine_path=paths.get("engine", ""),
                enabled=False,
            ))

    if not models:
        models = _get_hardcoded_models(base_dir)

    return Settings(
        udp=UdpConfig(
            local_ip=str(udp_cfg.get("local_ip", "192.168.10.101")),
            target_ip=str(udp_cfg.get("target_ip", "192.168.10.100")),
            target_port=int(udp_cfg.get("target_port", 12345)),
        ),
        models=models,
        inference=InferenceConfig(
            backend=inf_cfg.get("backend", "auto"),
            conf_threshold=float(inf_cfg.get("conf_threshold", 0.45)),
            iou_threshold=float(inf_cfg.get("iou_threshold", 0.45)),
            img_size=int(inf_cfg.get("img_size", 640)),
            min_box_area=int(inf_cfg.get("min_box_area", 500)),
            max_box_area=int(inf_cfg.get("max_box_area", 250000)),
            max_aspect_ratio=float(inf_cfg.get("max_aspect_ratio", 10.0)),
            edge_low_confidence=float(inf_cfg.get("edge_low_confidence", 0.30)),
            nms_iou_threshold=float(inf_cfg.get("nms_iou_threshold", 0.35)),
            containment_threshold=float(inf_cfg.get("containment_threshold", 0.65)),
            cross_class_nms_iou_threshold=float(inf_cfg.get("cross_class_nms_iou_threshold", 0.35)),
            cross_class_near_iou_threshold=float(inf_cfg.get("cross_class_near_iou_threshold", 0.20)),
            cross_class_center_px=float(inf_cfg.get("cross_class_center_px", 24.0)),
            fake_kfs_prefer_margin=float(inf_cfg.get("fake_kfs_prefer_margin", 0.12)),
            max_detections_per_frame=int(inf_cfg.get("max_detections_per_frame", 10)),
        ),
        output=OutputConfig(
            min_confidence=float(out_cfg.get("min_confidence", inf_cfg.get("output_min_confidence", 0.55))),
            require_valid_depth=_bool(out_cfg.get("require_valid_depth", inf_cfg.get("output_require_valid_depth", True)), True),
            max_targets_per_frame=int(out_cfg.get("max_targets_per_frame", inf_cfg.get("output_max_targets_per_frame", 6))),
            sort_by_confidence=_bool(out_cfg.get("sort_by_confidence", inf_cfg.get("output_sort_by_confidence", True)), True),
            log_interval_sec=float(out_cfg.get("log_interval_sec", inf_cfg.get("output_log_interval_sec", 2.0))),
            quality_gate_enabled=_bool(out_cfg.get("quality_gate_enabled", True), True),
            quality_edge_margin_px=max(0, int(out_cfg.get("quality_edge_margin_px", 30))),
            quality_center_error_norm=max(0.0, min(1.0, float(out_cfg.get("quality_center_error_norm", 0.18)))),
            quality_min_depth_valid_ratio=max(0.0, min(1.0, float(out_cfg.get("quality_min_depth_valid_ratio", 0.50)))),
            quality_max_depth_spread_m=max(0.0, float(out_cfg.get("quality_max_depth_spread_m", 0.015))),
            quality_require_no_fallback=_bool(out_cfg.get("quality_require_no_fallback", True), True),
            quality_bad_confidence=max(0.0, min(1.0, float(out_cfg.get("quality_bad_confidence", 0.01)))),
        ),
        camera=CameraConfig(
            width=int(cam_cfg.get("width", 640)),
            height=int(cam_cfg.get("height", 480)),
            fx=float(cam_cfg.get("fx", 367.93)),
            fy=float(cam_cfg.get("fy", 368.05)),
            cx=float(cam_cfg.get("cx", 318.75)),
            cy=float(cam_cfg.get("cy", 236.20)),
        ),
        depth=DepthConfig(
            offset_mm=float(depth_cfg.get("offset_mm", -50.0)),
            scale_corr=float(depth_cfg.get("scale_corr", 1.0)),
            min_mm=float(depth_cfg.get("min_mm", 200.0)),
            max_mm=float(depth_cfg.get("max_mm", 8000.0)),
        ),
        runtime=RuntimeConfig(require_orbbec=_bool(runtime_cfg.get("require_orbbec", True), True), strict_detector_load=_bool(runtime_cfg.get("strict_detector_load", True), True)),
        display=DisplayConfig(confirm_frames=int(disp_cfg.get("confirm_frames", 5)), display_every_n=int(disp_cfg.get("display_every_n", 8))),
        qr=QrConfig(
            enabled=_bool(qr_cfg.get("enabled", True), True),
            class_id=int(qr_cfg.get("class_id", 200)),
            class_name=str(qr_cfg.get("class_name", "qr_code")),
            detect_every_n=max(1, int(qr_cfg.get("detect_every_n", 1))),
            min_size_px=max(4, int(qr_cfg.get("min_size_px", 24))),
            decoded_confidence=float(qr_cfg.get("decoded_confidence", 1.0)),
            detected_confidence=float(qr_cfg.get("detected_confidence", 0.75)),
        ),
        assembly_light=AssemblyLightConfig(
            enabled=_bool(light_cfg.get("enabled", True), True),
            detect_every_n=max(1, int(light_cfg.get("detect_every_n", 1))),
            roi_xywh=_int_list(light_cfg.get("roi_xywh", [0, 0, 0, 0]), [0, 0, 0, 0]),
            min_area_px=max(1, int(light_cfg.get("min_area_px", 80))),
            max_area_ratio=max(0.0, float(light_cfg.get("max_area_ratio", 0.08))),
            min_width_px=max(1, int(light_cfg.get("min_width_px", 8))),
            min_height_px=max(1, int(light_cfg.get("min_height_px", 3))),
            min_aspect_ratio=max(1.0, float(light_cfg.get("min_aspect_ratio", 2.0))),
            max_aspect_ratio=max(1.0, float(light_cfg.get("max_aspect_ratio", 25.0))),
            min_fill_ratio=max(0.01, min(1.0, float(light_cfg.get("min_fill_ratio", 0.30)))),
            min_saturation=max(0, min(255, int(light_cfg.get("min_saturation", 80)))),
            min_brightness=max(0, min(255, int(light_cfg.get("min_brightness", 140)))),
            confidence=max(0.0, min(1.0, float(light_cfg.get("confidence", 0.85)))),
            event_confidence=max(0.0, min(1.0, float(light_cfg.get("event_confidence", 0.90)))),
            max_components_per_color=max(1, int(light_cfg.get("max_components_per_color", 2))),
            emit_color_event=_bool(light_cfg.get("emit_color_event", True), True),
            emit_success_event=_bool(light_cfg.get("emit_success_event", True), True),
            colors=[c for c in light_cfg.get("colors", []) if isinstance(c, dict)],
        ),
    )
