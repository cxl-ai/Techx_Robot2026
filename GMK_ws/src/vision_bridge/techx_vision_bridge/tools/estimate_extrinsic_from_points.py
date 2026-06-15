#!/usr/bin/env python3
"""Estimate a rigid transform from paired 3D points.

This helper is intentionally independent from ROS 2 and hardware. It is used to
convert measured point pairs into the [x, y, z, roll, pitch, yaw] values required
by techx_vision_bridge/config/vision_bridge.yaml.

CSV input format:

    from_x,from_y,from_z,to_x,to_y,to_z
    0.10,0.00,0.80,0.80,-0.10,0.25
    ...

The script estimates:

    point_to = R * point_from + t

Examples:

    # Estimate T_robot_camera: point_robot = T_robot_camera * point_camera
    python3 estimate_extrinsic_from_points.py --csv robot_camera_points.csv --name T_robot_camera

    # Estimate T_arm1_robot: point_arm1 = T_arm1_robot * point_robot
    python3 estimate_extrinsic_from_points.py --csv arm1_robot_points.csv --name T_arm1_robot
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Iterable, List, Tuple

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - depends on user machine
    print("[ERROR] numpy is required for SVD-based transform estimation.", file=sys.stderr)
    print("Install it with: python3 -m pip install numpy", file=sys.stderr)
    raise SystemExit(2) from exc


PointPair = Tuple[List[float], List[float]]


def load_pairs(path: Path) -> List[PointPair]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = ["from_x", "from_y", "from_z", "to_x", "to_y", "to_z"]
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(missing)}")

        pairs: List[PointPair] = []
        for line_no, row in enumerate(reader, start=2):
            try:
                p_from = [float(row["from_x"]), float(row["from_y"]), float(row["from_z"])]
                p_to = [float(row["to_x"]), float(row["to_y"]), float(row["to_z"])]
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid numeric value at CSV line {line_no}: {row}") from exc
            pairs.append((p_from, p_to))
    return pairs


def estimate_transform(p_from: np.ndarray, p_to: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return R,t such that p_to ~= R @ p_from + t."""
    if p_from.shape != p_to.shape or p_from.ndim != 2 or p_from.shape[1] != 3:
        raise ValueError("point arrays must both be Nx3")
    if p_from.shape[0] < 4:
        raise ValueError("at least 4 point pairs are recommended; more is better")

    c_from = p_from.mean(axis=0)
    c_to = p_to.mean(axis=0)
    x = p_from - c_from
    y = p_to - c_to

    h = x.T @ y
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T

    # Avoid a reflected solution when the point set is noisy.
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T

    t = c_to - r @ c_from
    return r, t


def matrix_to_rpy_zyx(r: np.ndarray) -> Tuple[float, float, float]:
    """Return roll,pitch,yaw for R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    sy = -float(r[2, 0])
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    cp = math.cos(pitch)

    if abs(cp) > 1e-8:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        # Gimbal-lock fallback. This is rare for normal fixed camera extrinsics.
        roll = 0.0
        yaw = math.atan2(-float(r[0, 1]), float(r[1, 1]))
    return roll, pitch, yaw


def format_list(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{v:.9f}" for v in values) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate [x,y,z,roll,pitch,yaw] from paired 3D points."
    )
    parser.add_argument("--csv", required=True, help="CSV with from_x,from_y,from_z,to_x,to_y,to_z columns")
    parser.add_argument(
        "--name",
        default="T_to_from",
        help="Name printed in the YAML example, e.g. T_robot_camera",
    )
    parser.add_argument(
        "--warn-rmse",
        type=float,
        default=0.03,
        help="Warn when RMSE is above this value in meters; default 0.03",
    )
    args = parser.parse_args()

    try:
        pairs = load_pairs(Path(args.csv))
        p_from = np.asarray([p[0] for p in pairs], dtype=float)
        p_to = np.asarray([p[1] for p in pairs], dtype=float)
        r, t = estimate_transform(p_from, p_to)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    pred = (r @ p_from.T).T + t
    err = pred - p_to
    norms = np.linalg.norm(err, axis=1)
    rmse = math.sqrt(float(np.mean(norms**2)))
    max_err = float(np.max(norms)) if len(norms) else 0.0
    roll, pitch, yaw = matrix_to_rpy_zyx(r)

    yaml_values = [float(t[0]), float(t[1]), float(t[2]), roll, pitch, yaw]

    print(f"Estimated {args.name}: point_to = R * point_from + t")
    print(f"point pairs: {len(pairs)}")
    print(f"rmse_m: {rmse:.6f}")
    print(f"max_error_m: {max_err:.6f}")
    print()
    print("YAML value:")
    print(f"{args.name}_xyz_rpy: {format_list(yaml_values)}")
    print()
    print("Rotation matrix R:")
    for row in r:
        print("  " + " ".join(f"{float(v): .9f}" for v in row))

    if rmse > args.warn_rmse:
        print()
        print(
            f"[WARN] RMSE {rmse:.3f} m is above {args.warn_rmse:.3f} m. "
            "Check point measurement, axis direction, units, and outliers."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
