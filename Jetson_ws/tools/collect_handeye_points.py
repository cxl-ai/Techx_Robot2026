#!/usr/bin/env python3
"""Interactive helper to collect paired points for GMK hand-eye calibration.

This script is intentionally simple and ROS-free. It reads the latest Jetson
recorder targets.csv row, lets the operator enter the matching destination point,
and appends a pair to a calibration CSV accepted by export_handeye_yaml.py.

Typical flow for T_robot_camera:
  1. Run Jetson vision with recorder enabled:
       TECHX_STAGE=head TECHX_DEBUG_RECORDER=1 TECHX_DEBUG_DIR=runs/calib_robot_camera bash scripts/field_start_jetson.sh
  2. Put a calibration target at a known robot_base point.
  3. Wait until targets.csv shows stable camera_x/y/z and output_valid_xyz=1.
  4. Run this helper and enter the known robot_base x/y/z.
  5. Repeat 6-12 non-coplanar points.
  6. Export YAML with tools/export_handeye_yaml.py.

For T_robot_camera:
  from = Jetson camera_link x/y/z
  to   = robot_base x/y/z measured on the robot

For T_arm1_robot:
  from = robot_base x/y/z
  to   = arm1_base x/y/z from the arm-1 controller/mechanical measurement

For T_arm2_robot:
  from = robot_base x/y/z
  to   = arm2_base x/y/z from the arm-2 controller/mechanical measurement

The helper can auto-read only the Jetson side from targets.csv. For pure
mechanical robot->arm calibration, use --manual-from and enter both sides.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REQUIRED_OUT_COLUMNS = ["from_x", "from_y", "from_z", "to_x", "to_y", "to_z"]
TARGET_COLUMNS = [
    "time", "frame_id", "track_id", "class_id", "class_name", "conf", "raw_conf",
    "u", "v", "camera_x", "camera_y", "camera_z", "depth_m", "color",
    "can_grab", "reject_reason", "quality_score", "edge_margin_px", "center_error_norm",
    "depth_valid_ratio", "depth_spread_m", "depth_fallback_used",
]


def _float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def _int(row: Dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, default)))
    except (TypeError, ValueError):
        return default


def read_recent_targets(path: Path, max_age_sec: float) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    now = time.time()
    out: List[Dict[str, str]] = []
    for row in rows:
        t = _float(row, "time", 0.0)
        if t <= 0 or now - t <= max_age_sec:
            out.append(row)
    return out


def select_best_target(rows: List[Dict[str, str]], class_id: Optional[int], require_valid_xyz: bool) -> Optional[Dict[str, str]]:
    candidates: List[Dict[str, str]] = []
    for row in rows:
        if class_id is not None and _int(row, "class_id", -1) != class_id:
            continue
        camera_z = _float(row, "camera_z", 0.0)
        depth_m = _float(row, "depth_m", 0.0)
        valid_xyz = camera_z > 0.0 and depth_m > 0.0
        if require_valid_xyz and not valid_xyz:
            continue
        candidates.append(row)
    if not candidates:
        return None
    candidates.sort(
        key=lambda r: (
            1 if _int(r, "can_grab", 0) else 0,
            1 if _float(r, "camera_z", 0.0) > 0.0 else 0,
            _float(r, "quality_score", 0.0),
            _float(r, "conf", 0.0),
            _float(r, "time", 0.0),
        ),
        reverse=True,
    )
    return candidates[0]


def prompt_vec3(prompt: str) -> Tuple[float, float, float]:
    while True:
        raw = input(prompt).strip()
        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            print("请输入 3 个数，例如: 0.120 -0.035 0.450")
            continue
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("数字格式错误，请重新输入")


def ensure_output_header(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(REQUIRED_OUT_COLUMNS + [
            "note", "source", "time", "frame_id", "track_id", "class_id", "class_name",
            "conf", "u", "v", "quality_score", "reject_reason",
        ])


def append_pair(path: Path, p_from: Tuple[float, float, float], p_to: Tuple[float, float, float], meta: Dict[str, str]) -> None:
    ensure_output_header(path)
    with path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            *[f"{v:.9f}" for v in p_from],
            *[f"{v:.9f}" for v in p_to],
            meta.get("note", ""),
            meta.get("source", ""),
            meta.get("time", ""),
            meta.get("frame_id", ""),
            meta.get("track_id", ""),
            meta.get("class_id", ""),
            meta.get("class_name", ""),
            meta.get("conf", ""),
            meta.get("u", ""),
            meta.get("v", ""),
            meta.get("quality_score", ""),
            meta.get("reject_reason", ""),
        ])


def format_row(row: Dict[str, str]) -> str:
    return (
        f"frame={row.get('frame_id','?')} track={row.get('track_id','?')} "
        f"cls={row.get('class_id','?')}:{row.get('class_name','')} conf={_float(row,'conf',0):.3f} "
        f"uv=({_float(row,'u',0):.1f},{_float(row,'v',0):.1f}) "
        f"cam=({_float(row,'camera_x',0):.4f},{_float(row,'camera_y',0):.4f},{_float(row,'camera_z',0):.4f}) "
        f"grab={_int(row,'can_grab',0)} q={_float(row,'quality_score',0):.2f} reason={row.get('reject_reason','')}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Collect paired calibration points for export_handeye_yaml.py")
    parser.add_argument("--targets-csv", default="runs/calib/targets.csv", help="Jetson recorder targets.csv")
    parser.add_argument("--output-csv", required=True, help="output calibration CSV with from_x/from_y/from_z/to_x/to_y/to_z")
    parser.add_argument("--class-id", type=int, help="optional class_id to pick from targets.csv")
    parser.add_argument("--max-age-sec", type=float, default=5.0, help="only use recent target rows")
    parser.add_argument("--manual-from", action="store_true", help="enter from_x/y/z manually instead of reading Jetson target camera_x/y/z")
    parser.add_argument("--allow-invalid-target", action="store_true", help="allow target rows with camera_z/depth_m <= 0")
    parser.add_argument("--note", default="", help="note saved with this point")
    args = parser.parse_args()

    out_path = Path(args.output_csv)
    if args.manual_from:
        p_from = prompt_vec3("输入 from_x from_y from_z（米）: ")
        meta = {"source": "manual", "note": args.note}
    else:
        try:
            rows = read_recent_targets(Path(args.targets_csv), args.max_age_sec)
        except Exception as exc:
            print(f"[ERROR] cannot read targets csv: {exc}", file=sys.stderr)
            return 1
        row = select_best_target(rows, args.class_id, require_valid_xyz=not args.allow_invalid_target)
        if row is None:
            print("[ERROR] no suitable recent target found", file=sys.stderr)
            print("检查 TECHX_DEBUG_RECORDER=1、targets.csv 路径、class_id、目标是否居中且 output_valid_xyz=1", file=sys.stderr)
            return 2
        print("选中的 Jetson camera_link 点:")
        print("  " + format_row(row))
        ok = input("确认使用这个 from 点吗？[y/N] ").strip().lower()
        if ok not in {"y", "yes"}:
            print("cancelled")
            return 3
        p_from = (_float(row, "camera_x"), _float(row, "camera_y"), _float(row, "camera_z"))
        meta = {
            "source": str(args.targets_csv),
            "note": args.note,
            "time": row.get("time", ""),
            "frame_id": row.get("frame_id", ""),
            "track_id": row.get("track_id", ""),
            "class_id": row.get("class_id", ""),
            "class_name": row.get("class_name", ""),
            "conf": row.get("conf", ""),
            "u": row.get("u", ""),
            "v": row.get("v", ""),
            "quality_score": row.get("quality_score", ""),
            "reject_reason": row.get("reject_reason", ""),
        }

    p_to = prompt_vec3("输入对应的 to_x to_y to_z（米）: ")
    append_pair(out_path, p_from, p_to, meta)
    print(f"已追加到 {out_path}")
    print(f"from = ({p_from[0]:.6f}, {p_from[1]:.6f}, {p_from[2]:.6f})")
    print(f"to   = ({p_to[0]:.6f}, {p_to[1]:.6f}, {p_to[2]:.6f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
