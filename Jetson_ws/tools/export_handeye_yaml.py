#!/usr/bin/env python3
"""Estimate field extrinsics and export GMK-ready calibration files.

This tool is intended to run on Jetson or a laptop after collecting paired 3D
points during field hand-eye calibration. It does not need ROS 2.

CSV columns:
  from_x,from_y,from_z,to_x,to_y,to_z

Meaning:
  point_to = R * point_from + t

Coordinate directions must not be guessed. For each transform:
  T_robot_camera:
    from = camera_link point measured by Jetson vision
    to   = robot_base point measured in the robot body frame
  T_arm1_robot:
    from = robot_base point
    to   = arm1_base point measured in arm 1 controller/base frame
  T_arm2_robot:
    from = robot_base point
    to   = arm2_base point measured in arm 2 controller/base frame

Examples:
  python3 tools/export_handeye_yaml.py \
    --csv runs/calib/robot_camera_points.csv \
    --name T_robot_camera \
    --output-yaml runs/calib/gmk_robot_camera.yaml \
    --report-json runs/calib/gmk_robot_camera_report.json \
    --residual-csv runs/calib/gmk_robot_camera_residuals.csv \
    --max-rmse 0.015 \
    --max-point-error 0.030

Copy the printed YAML lines or the output YAML file into GMK:
  src/techx_vision_bridge/config/vision_bridge.yaml
For field use, the same T_* value must be copied into both:
  vision_bridge_node.ros__parameters
  calibration_guard_node.ros__parameters
and the matching *_calibrated flag must be true in calibration_guard_node.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path
from typing import Iterable, List, Tuple

# Field default thresholds are tight enough for ~±1cm grasping. They can be
# relaxed for early bring-up via environment variables without editing code:
#   TECHX_HANDEYE_MAX_RMSE        (meters, default 0.010)
#   TECHX_HANDEYE_MAX_POINT_ERROR (meters, default 0.020)
_DEFAULT_MAX_RMSE = 0.010
_DEFAULT_MAX_POINT_ERROR = 0.020


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return default
    try:
        return float(raw)
    except ValueError:
        return default

try:
    import numpy as np
except ImportError as exc:
    print("[ERROR] numpy is required: python3 -m pip install numpy", file=sys.stderr)
    raise SystemExit(2) from exc

PointPair = Tuple[List[float], List[float]]


class CalibrationError(RuntimeError):
    pass


def load_pairs(path: Path) -> List[PointPair]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = ["from_x", "from_y", "from_z", "to_x", "to_y", "to_z"]
        missing = [c for c in required if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("CSV missing columns: " + ", ".join(missing))
        pairs: List[PointPair] = []
        for line_no, row in enumerate(reader, start=2):
            try:
                p_from = [float(row["from_x"]), float(row["from_y"]), float(row["from_z"])]
                p_to = [float(row["to_x"]), float(row["to_y"]), float(row["to_z"])]
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid number at line {line_no}: {row}") from exc
            pairs.append((p_from, p_to))
    if len(pairs) < 4:
        raise ValueError("at least 4 non-coplanar point pairs are required; 6-12 are recommended")
    return pairs


def estimate_transform(p_from: np.ndarray, p_to: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rigid least-squares transform from p_from to p_to using Kabsch/SVD."""
    if p_from.shape != p_to.shape or p_from.ndim != 2 or p_from.shape[1] != 3:
        raise ValueError("point arrays must be Nx3")
    if p_from.shape[0] < 4:
        raise ValueError("at least 4 point pairs are required")
    c_from = p_from.mean(axis=0)
    c_to = p_to.mean(axis=0)
    x = p_from - c_from
    y = p_to - c_to
    # Rank 3 means the points span 3D. Rank 2 can still fit a plane, but is less
    # safe for hand-eye calibration because one rotation axis is weakly observed.
    rank = int(np.linalg.matrix_rank(x, tol=1e-9))
    if rank < 2:
        raise ValueError("point geometry is degenerate; use spread-out non-collinear points")
    h = x.T @ y
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = c_to - r @ c_from
    return r, t


def matrix_to_rpy_zyx(r: np.ndarray) -> Tuple[float, float, float]:
    sy = max(-1.0, min(1.0, -float(r[2, 0])))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(r[0, 1]), float(r[1, 1]))
    return roll, pitch, yaw


def format_list(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(v):.9f}" for v in values) + "]"


def guard_flag_name(name: str) -> str | None:
    if name == "T_robot_camera":
        return "robot_camera_calibrated"
    if name == "T_arm1_robot":
        return "arm1_robot_calibrated"
    if name == "T_arm2_robot":
        return "arm2_robot_calibrated"
    return None


def compute_residuals(p_from: np.ndarray, p_to: np.ndarray, r: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    pred = (r @ p_from.T).T + t
    err = pred - p_to
    norms = np.linalg.norm(err, axis=1)
    return pred, err, norms


def write_residual_csv(path: Path, p_from: np.ndarray, p_to: np.ndarray, pred: np.ndarray, err: np.ndarray, norms: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "index",
            "from_x", "from_y", "from_z",
            "to_x", "to_y", "to_z",
            "pred_x", "pred_y", "pred_z",
            "err_x", "err_y", "err_z", "err_norm",
        ])
        for i in range(len(norms)):
            writer.writerow([
                i,
                *[f"{v:.9f}" for v in p_from[i]],
                *[f"{v:.9f}" for v in p_to[i]],
                *[f"{v:.9f}" for v in pred[i]],
                *[f"{v:.9f}" for v in err[i]],
                f"{float(norms[i]):.9f}",
            ])


def yaml_snippet(name: str, values: List[float], calibrated: bool) -> str:
    flag = guard_flag_name(name)
    lines = []
    lines.append("# Copy the transform value into BOTH vision_bridge_node and calibration_guard_node.")
    lines.append("vision_bridge_node:")
    lines.append("  ros__parameters:")
    lines.append(f"    {name}_xyz_rpy: {format_list(values)}")
    lines.append("calibration_guard_node:")
    lines.append("  ros__parameters:")
    lines.append(f"    {name}_xyz_rpy: {format_list(values)}")
    if flag:
        lines.append(f"    {flag}: {'true' if calibrated else 'false'}")
    return "\n".join(lines) + "\n"


def write_report_json(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Export GMK hand-eye calibration YAML from paired 3D points.")
    parser.add_argument("--csv", required=True, help="CSV with from_x,from_y,from_z,to_x,to_y,to_z")
    parser.add_argument("--name", required=True, choices=["T_robot_camera", "T_arm1_robot", "T_arm2_robot"])
    parser.add_argument("--output-yaml", help="write GMK-ready YAML snippet to this file")
    parser.add_argument("--report-json", help="write calibration report JSON to this file")
    parser.add_argument("--residual-csv", help="write per-point residuals CSV to this file")
    parser.add_argument("--max-rmse", type=float, default=_env_float("TECHX_HANDEYE_MAX_RMSE", _DEFAULT_MAX_RMSE),
                        help="fail if RMSE is above this value in meters (default 0.010m; env TECHX_HANDEYE_MAX_RMSE)")
    parser.add_argument("--max-point-error", type=float, default=_env_float("TECHX_HANDEYE_MAX_POINT_ERROR", _DEFAULT_MAX_POINT_ERROR),
                        help="fail if any point residual is above this value in meters (default 0.020m; env TECHX_HANDEYE_MAX_POINT_ERROR)")
    parser.add_argument("--allow-poor-fit", action="store_true", help="export even when thresholds fail; report will mark accepted=false")
    args = parser.parse_args()

    try:
        pairs = load_pairs(Path(args.csv))
        p_from = np.asarray([p[0] for p in pairs], dtype=float)
        p_to = np.asarray([p[1] for p in pairs], dtype=float)
        r, t = estimate_transform(p_from, p_to)
        pred, err, norms = compute_residuals(p_from, p_to, r, t)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    rmse = math.sqrt(float(np.mean(norms**2)))
    max_err = float(np.max(norms)) if len(norms) else 0.0
    mean_err = float(np.mean(norms)) if len(norms) else 0.0
    roll, pitch, yaw = matrix_to_rpy_zyx(r)
    values = [float(t[0]), float(t[1]), float(t[2]), roll, pitch, yaw]
    accepted = bool(rmse <= args.max_rmse and max_err <= args.max_point_error)

    report = {
        "name": args.name,
        "equation": "point_to = R * point_from + t",
        "point_pairs": len(pairs),
        "accepted": accepted,
        "thresholds": {
            "max_rmse_m": args.max_rmse,
            "max_point_error_m": args.max_point_error,
        },
        "rmse_m": rmse,
        "mean_error_m": mean_err,
        "max_error_m": max_err,
        "translation_xyz_m": [float(v) for v in t],
        "rpy_rad": [roll, pitch, yaw],
        "rotation_matrix": [[float(v) for v in row] for row in r.tolist()],
        "gmk_xyz_rpy": values,
    }

    print(f"# Estimated {args.name}: point_to = R * point_from + t")
    print(f"# point_pairs: {len(pairs)}")
    print(f"# rmse_m: {rmse:.6f}")
    print(f"# mean_error_m: {mean_err:.6f}")
    print(f"# max_error_m: {max_err:.6f}")
    if not accepted:
        print(
            f"# ERROR: calibration rejected: rmse <= {args.max_rmse:.3f}m and max_error <= {args.max_point_error:.3f}m required",
            file=sys.stderr,
        )
        if args.allow_poor_fit:
            print(
                "# NOTE: --allow-poor-fit still EXPORTS the transform, but the *_calibrated flag stays FALSE "
                "so GMK keeps blocking grasp coordinates until a field operator verifies and flips it by hand.",
                file=sys.stderr,
            )

    # The calibrated flag is strictly tied to the fit passing its thresholds.
    # --allow-poor-fit only controls whether the file is written and the exit
    # code, never whether a failed fit is advertised as calibrated.
    calibrated_flag = accepted
    print(yaml_snippet(args.name, values, calibrated=calibrated_flag), end="")

    if args.output_yaml:
        Path(args.output_yaml).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output_yaml).write_text(yaml_snippet(args.name, values, calibrated=calibrated_flag), encoding="utf-8")
        print(f"# wrote_yaml: {args.output_yaml}")
    if args.report_json:
        write_report_json(Path(args.report_json), report)
        print(f"# wrote_report: {args.report_json}")
    if args.residual_csv:
        write_residual_csv(Path(args.residual_csv), p_from, p_to, pred, err, norms)
        print(f"# wrote_residuals: {args.residual_csv}")

    if not accepted and not args.allow_poor_fit:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
