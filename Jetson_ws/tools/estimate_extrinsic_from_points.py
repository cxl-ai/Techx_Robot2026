#!/usr/bin/env python3
"""Estimate a rigid extrinsic transform from paired 3D calibration points.

This tool belongs on the Jetson side because Jetson is the easiest place to
collect camera_link points from RGB-D observations. The result should normally
be copied into the GMK bridge config, for example:

    T_robot_camera_xyz_rpy: [x, y, z, roll, pitch, yaw]

CSV format:

    from_x,from_y,from_z,to_x,to_y,to_z
    0.10,0.02,0.80,0.80,-0.10,0.25

For T_robot_camera:
    from = camera_link point measured by Jetson
    to   = robot_base point measured by ruler, marker rig, chassis frame, or arm TCP
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np


def _load_points(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    src = []
    dst = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"from_x", "from_y", "from_z", "to_x", "to_y", "to_z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}")
        for row_no, row in enumerate(reader, start=2):
            try:
                a = [float(row["from_x"]), float(row["from_y"]), float(row["from_z"])]
                b = [float(row["to_x"]), float(row["to_y"]), float(row["to_z"])]
            except Exception as exc:
                raise ValueError(f"Invalid numeric value at row {row_no}: {exc}") from exc
            if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
                raise ValueError(f"Non-finite value at row {row_no}")
            src.append(a)
            dst.append(b)
    if len(src) < 4:
        raise ValueError("Need at least 4 paired 3D points; 10~20 points is recommended")
    return np.asarray(src, dtype=np.float64), np.asarray(dst, dtype=np.float64)


def _estimate_rigid(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return R,t that minimizes dst ~= R @ src + t using Kabsch/SVD."""
    src_centroid = src.mean(axis=0)
    dst_centroid = dst.mean(axis=0)
    src_c = src - src_centroid
    dst_c = dst - dst_centroid
    h = src_c.T @ dst_c
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = dst_centroid - r @ src_centroid
    return r, t


def _rpy_from_matrix(r: np.ndarray) -> Tuple[float, float, float]:
    """Return roll,pitch,yaw for R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    sy = math.sqrt(float(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0]))
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = math.atan2(float(-r[1, 2]), float(r[1, 1]))
        pitch = math.atan2(float(-r[2, 0]), sy)
        yaw = 0.0
    return roll, pitch, yaw


def _format_list(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{v:.9g}" for v in values) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(description="Estimate TECHX vision extrinsic transform from paired 3D points")
    parser.add_argument("--csv", required=True, help="CSV with from_x,from_y,from_z,to_x,to_y,to_z")
    parser.add_argument("--name", default="T_robot_camera", help="YAML parameter name prefix to print")
    parser.add_argument("--max-rmse", type=float, default=0.03, help="Warn if RMSE is above this many metres")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    src, dst = _load_points(csv_path)
    r, t = _estimate_rigid(src, dst)
    roll, pitch, yaw = _rpy_from_matrix(r)

    pred = (r @ src.T).T + t
    err = np.linalg.norm(pred - dst, axis=1)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    max_err = float(np.max(err))

    print("# Transform convention: point_to = R * point_from + t")
    print(f"# points: {len(src)}")
    print(f"# rmse_m: {rmse:.6f}")
    print(f"# max_error_m: {max_err:.6f}")
    print(f"{args.name}_xyz_rpy: {_format_list([t[0], t[1], t[2], roll, pitch, yaw])}")
    print("# R:")
    for row in r:
        print("#   " + _format_list(row))

    if rmse > args.max_rmse:
        print(
            f"[WARN] RMSE {rmse:.3f} m is above {args.max_rmse:.3f} m. "
            "Check point pairs, coordinate axes, depth quality, and units.",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
