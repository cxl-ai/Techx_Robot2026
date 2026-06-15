#!/usr/bin/env python3
"""Static checker for the GMK techx_vision_bridge YAML config.

This does not require ROS 2 runtime or hardware. It checks the single-node
competition contract:

    vision_bridge_node receives Jetson UDP V2, publishes /frame,
    subscribes /request, publishes /selected, and shuts down after a
    configurable long no-data timeout.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Tuple

REQUIRED_CLASS_IDS = {
    0: "kfs_red_r1",
    1: "kfs_red_r2_fake",
    2: "kfs_red_r2_true",
    3: "kfs_blue_r1",
    4: "kfs_blue_r2_fake",
    5: "kfs_blue_r2_true",
    100: "weapon_head_fist",
    101: "weapon_head_palm",
    102: "weapon_head_spear",
    200: "qr_code",
}

RULE_RE = re.compile(r'"([^":]+):(\d+):(\d+):(\d+):([-+0-9.]+)"')


def read_rules(text: str) -> List[Tuple[int, int, int, int, int, float]]:
    rules = []
    for match in RULE_RE.finditer(text):
        range_s, zone_s, type_s, frame_s, bias_s = match.groups()
        if "-" in range_s:
            lo_s, hi_s = range_s.split("-", 1)
            lo, hi = int(lo_s), int(hi_s)
        else:
            lo = hi = int(range_s)
        if hi < lo:
            lo, hi = hi, lo
        rules.append((lo, hi, int(zone_s), int(type_s), int(frame_s), float(bias_s)))
    return rules


def covered(rules: List[Tuple[int, int, int, int, int, float]], cid: int) -> bool:
    return any(lo <= cid <= hi for lo, hi, *_ in rules)


def scalar_value(text: str, key: str) -> str | None:
    m = re.search(rf"^\s*{re.escape(key)}:\s*([^#\n]+)", text, re.M)
    return m.group(1).strip() if m else None


def require_key(text: str, key: str) -> bool:
    if scalar_value(text, key) is None:
        print(f"[ERROR] missing {key}")
        return False
    return True


def parse_float(text: str, key: str) -> float | None:
    value = scalar_value(text, key)
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        print(f"[ERROR] {key} is not a float: {value}")
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Check GMK single-node vision bridge config without ROS/hardware")
    parser.add_argument("--config", default="src/techx_vision_bridge/config/vision_bridge.yaml")
    args = parser.parse_args()

    text = Path(args.config).read_text(encoding="utf-8")
    errors = 0

    if "vision_bridge_node:" not in text:
        print("[ERROR] missing top-level vision_bridge_node")
        errors += 1
    if "vision_selector_node:" in text:
        print("[ERROR] vision_selector_node should not be a separate top-level node in the simplified single-node design")
        errors += 1

    for key in (
        "udp_bind_addr",
        "udp_port",
        "frame_topic_name",
        "request_topic_name",
        "selected_topic_name",
        "enable_request_selector",
        "watchdog_timeout_sec",
        "fatal_no_udp_timeout_sec",
    ):
        if not require_key(text, key):
            errors += 1

    rules = read_rules(text)
    if not rules:
        print("[ERROR] no valid class_rules found")
        errors += 1
    else:
        print("class_rules:")
        for lo, hi, zone, typ, frame, bias in rules:
            print(f"  {lo}-{hi}: zone={zone} type={typ} frame={frame} bias={bias}")

    for cid, name in REQUIRED_CLASS_IDS.items():
        if not covered(rules, cid):
            print(f"[ERROR] class_id {cid} ({name}) is not covered by class_rules")
            errors += 1

    for key in ("publish_detail_topic", "publish_legacy_topic", "accept_legacy"):
        value = scalar_value(text, key)
        if value is None:
            print(f"[ERROR] missing {key}")
            errors += 1
        elif value.lower() != "false":
            print(f"[WARN] {key} is {value}; canonical competition path should keep legacy/detail topics off by default")

    if scalar_value(text, "enable_request_selector") not in ("true", "True"):
        print("[WARN] enable_request_selector is not true; /request -> /selected will be disabled")

    fatal_timeout = parse_float(text, "fatal_no_udp_timeout_sec")
    if fatal_timeout is None:
        errors += 1
    elif fatal_timeout == 0.0:
        print("[WARN] fatal_no_udp_timeout_sec is disabled; long no-data runs will not auto-terminate")
    elif fatal_timeout < 60.0:
        print("[WARN] fatal_no_udp_timeout_sec is very short; this may kill the bridge during normal startup")
    else:
        print(f"fatal_no_udp_timeout_sec: {fatal_timeout:.1f}s")

    for key in ("T_robot_camera_xyz_rpy", "T_arm1_robot_xyz_rpy", "T_arm2_robot_xyz_rpy"):
        if scalar_value(text, key) is None:
            print(f"[ERROR] missing {key}")
            errors += 1

    if errors:
        print(f"FAILED: {errors} error(s)")
        return 1
    print("OK: GMK single-node vision bridge config matches the competition communication contract")
    return 0


if __name__ == "__main__":
    sys.exit(main())
