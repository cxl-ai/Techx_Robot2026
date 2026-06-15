#!/usr/bin/env python3
"""Static checker for TECHX Jetson vision runtime config.

This does not need camera hardware. It checks that the configured global class_id
allocation matches the GMK bridge contract, that enabled model folders contain at
least one supported model artifact, and that competition runtime/output safety
flags are visible in config.json.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, Set

EXPECTED_IDS: Dict[int, str] = {
    0: "fake_kfs",
    1: "r1_kfs_red",
    2: "r1_kfs_blue",
    3: "r2_kfs_red",
    4: "r2_kfs_blue",
    100: "weapon_head_fist",
    101: "weapon_head_palm",
    102: "weapon_head_spear",
    200: "qr_code",
}


def artifact_exists(base_dir: str, folder: str) -> bool:
    model_dir = os.path.join(base_dir, "models", folder)
    return any(os.path.exists(os.path.join(model_dir, name)) for name in ("best.engine", "best.onnx", "best.pt"))


def discovered_model_folders(base_dir: str) -> list:
    """Folders under models/ that actually contain a weight file."""
    models_dir = os.path.join(base_dir, "models")
    out = []
    if not os.path.isdir(models_dir):
        return out
    for folder in sorted(os.listdir(models_dir)):
        fp = os.path.join(models_dir, folder)
        try:
            has_weight = os.path.isdir(fp) and any(f.endswith((".engine", ".onnx", ".pt")) for f in os.listdir(fp))
        except OSError:
            has_weight = False
        if has_weight:
            out.append(folder)
    return out


def mapped_ids(model: dict) -> Set[int]:
    out: Set[int] = set()
    raw = model.get("class_id_map", {}) or {}
    for value in raw.values():
        try:
            out.add(int(value))
        except (TypeError, ValueError):
            pass
    return out


def as_bool(raw, default: bool) -> bool:
    if isinstance(raw, bool):
        return raw
    if raw is None:
        return default
    if isinstance(raw, (int, float)):
        return bool(raw)
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def warn(msg: str) -> None:
    print(f"[WARN] {msg}")


def error(msg: str) -> None:
    print(f"[ERROR] {msg}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check TECHX Jetson vision config without hardware")
    parser.add_argument("--config", default="config.json")
    args = parser.parse_args()

    config_path = os.path.abspath(args.config)
    base_dir = os.path.dirname(config_path)
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    errors = 0
    runtime = cfg.get("runtime", {}) or {}
    require_orbbec = as_bool(runtime.get("require_orbbec", True), True)
    strict_detector_load = as_bool(runtime.get("strict_detector_load", True), True)
    if not require_orbbec:
        warn("runtime.require_orbbec=false: Orbbec/Gemini may fall back to UVC; use this only for desktop/debug")
    if not strict_detector_load:
        warn("runtime.strict_detector_load=false: partial detector load is allowed; not recommended for match runtime")

    models = cfg.get("models", []) or []
    seen: Set[int] = set()
    enabled_models = 0
    for model in models:
        name = model.get("name", model.get("folder", "<unnamed>"))
        folder = model.get("folder", "")
        enabled = as_bool(model.get("enabled", True), True)
        mids = mapped_ids(model)
        if not enabled:
            continue
        if enabled:
            enabled_models += 1
            if not artifact_exists(base_dir, folder):
                error(f"enabled model '{name}' folder '{folder}' has no best.engine/best.onnx/best.pt")
                errors += 1
        if mids & seen:
            error(f"duplicate global class_id(s) {sorted(mids & seen)} in model '{name}'")
            errors += 1
        seen |= mids
        if enabled and not mids and folder:
            warn(f"enabled model '{name}' has no class_id_map; it will use local_id + class_id_offset")

    qr = cfg.get("qr", {}) or {}
    if as_bool(qr.get("enabled", True), True):
        try:
            seen.add(int(qr.get("class_id", 200)))
        except (TypeError, ValueError):
            error("qr.class_id is invalid")
            errors += 1

    for cid, name in EXPECTED_IDS.items():
        if cid not in seen:
            warn(f"global class_id {cid} ({name}) is not produced by current enabled config")

    # Folders on disk that are not in config.json are auto-disabled at runtime; flag
    # them so a leftover model (e.g. an old gesture net) is noticed before the match.
    config_folders = {model.get("folder", "") for model in models}
    for folder in discovered_model_folders(base_dir):
        if folder not in config_folders:
            warn(f"models/{folder} has weights but is not in config.json; it is auto-DISABLED and will not run. Add it (with class_id_map) or remove the folder.")

    # Assembly lightbar without a tight ROI can be false-triggered by other field
    # lights even though it still passes continuous-frame confirmation.
    light = cfg.get("assembly_light", {}) or {}
    if as_bool(light.get("enabled", True), True):
        roi = list(light.get("roi_xywh", [0, 0, 0, 0]) or [0, 0, 0, 0])[:4]
        full_frame = roi == [0, 0, 0, 0] or (len(roi) >= 4 and (int(roi[2]) <= 0 or int(roi[3]) <= 0))
        if full_frame:
            warn("assembly_light.roi_xywh is full-frame [0,0,0,0]; set a tight ROI around the R1 lightbar before competition or other field lights may false-trigger it")

    if enabled_models == 0 and not as_bool(qr.get("enabled", True), True):
        error("no enabled model and QR disabled")
        errors += 1

    inf = cfg.get("inference", {}) or {}
    output = cfg.get("output", {}) or {}
    min_conf = float(output.get("min_confidence", inf.get("output_min_confidence", 0.55)))
    require_depth = as_bool(output.get("require_valid_depth", inf.get("output_require_valid_depth", True)), True)
    max_targets = int(output.get("max_targets_per_frame", inf.get("output_max_targets_per_frame", 6)))
    if min_conf < 0.45:
        warn(f"output.min_confidence={min_conf:.2f}: too low for baseline match testing")
    if not require_depth:
        warn("output.require_valid_depth=false: targets without depth may be sent to GMK")
    if max_targets <= 0:
        warn("output.max_targets_per_frame<=0: final UDP output is unlimited")

    udp = cfg.get("udp", {}) or {}
    print(f"UDP target: {udp.get('target_ip', '192.168.10.100')}:{udp.get('target_port', 12345)}")
    print(f"Runtime: require_orbbec={require_orbbec} strict_detector_load={strict_detector_load}")
    print(
        "Output: min_confidence=%.2f require_valid_depth=%s max_targets_per_frame=%d" %
        (min_conf, require_depth, max_targets)
    )
    print(f"Global IDs seen: {sorted(seen)}")
    if errors:
        print(f"FAILED: {errors} error(s)")
        return 1
    print("OK: config is internally consistent; hardware/model accuracy still requires real testing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
