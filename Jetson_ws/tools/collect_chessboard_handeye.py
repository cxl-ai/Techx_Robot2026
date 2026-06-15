#!/usr/bin/env python3
"""Collect chessboard-based hand-eye calibration point pairs on Jetson.

This is the preferred field calibration path when an A4 chessboard is available.
It detects a full chessboard in RGB, refines corners to subpixel precision,
estimates the board pose with solvePnP, checks depth consistency, and appends a
3D point pair accepted by tools/export_handeye_yaml.py.

What this script can do automatically:
  - Open the Orbbec RGB-D camera through the project camera driver.
  - Detect an A4 chessboard and estimate T_camera_board.
  - Reject blurry/edge/unstable/poor-depth samples.
  - Save overlay images for audit.
  - Append camera_link -> destination point pairs to a CSV.

What it cannot magically know:
  - The board position in robot_base/arm_base. You must either type it when
    prompted, or provide it through a future robot/arm pose bridge.

Transform directions:
  T_robot_camera:
    from = board reference point in camera_link, measured by this script
    to   = same board reference point in robot_base
  T_arm1_robot / T_arm2_robot are normally mechanical calibrations and should be
  collected as robot_base -> arm*_base pairs, not through the camera.

Recommended field use:
  python3 tools/collect_chessboard_handeye.py \
    --output-csv runs/calib_robot_camera/robot_camera_points.csv \
    --pattern-cols 9 --pattern-rows 6 --square-size 0.025 \
    --reference center \
    --overlay-dir runs/calib_robot_camera/overlays

Then export:
  python3 tools/export_handeye_yaml.py \
    --csv runs/calib_robot_camera/robot_camera_points.csv \
    --name T_robot_camera \
    --output-yaml runs/calib_robot_camera/gmk_robot_camera.yaml \
    --report-json runs/calib_robot_camera/gmk_robot_camera_report.json \
    --residual-csv runs/calib_robot_camera/gmk_robot_camera_residuals.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np

# Allow running from repository root without installing the package.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from camera.orbbec import OrbbecCamera
except Exception as exc:  # pragma: no cover - runtime hardware dependency
    print(f"[ERROR] cannot import OrbbecCamera: {exc}", file=sys.stderr)
    raise


CSV_COLUMNS = [
    "from_x", "from_y", "from_z", "to_x", "to_y", "to_z",
    "note", "timestamp", "reproj_err_px", "depth_median_m", "pnp_z_m",
    "depth_pnp_delta_m", "depth_valid_ratio", "depth_spread_m", "u", "v",
]


def parse_vec3(raw: str) -> Tuple[float, float, float]:
    parts = raw.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("expected 3 values")
    return float(parts[0]), float(parts[1]), float(parts[2])


def prompt_vec3(prompt: str) -> Tuple[float, float, float]:
    while True:
        raw = input(prompt).strip()
        try:
            return parse_vec3(raw)
        except Exception:
            print("请输入 3 个以米为单位的数，例如: 0.350 0.000 0.220")


def build_object_points(cols: int, rows: int, square: float) -> np.ndarray:
    obj = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj[:, :2] = grid * float(square)
    return obj


def board_reference_point(cols: int, rows: int, square: float, reference: str) -> np.ndarray:
    if reference == "origin":
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)
    # Center of the inner-corner grid, not necessarily the physical board edge.
    return np.array([(cols - 1) * square * 0.5, (rows - 1) * square * 0.5, 0.0], dtype=np.float32)


def project_reprojection_error(obj_pts: np.ndarray, corners: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, k: np.ndarray) -> float:
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, k, None)
    err = proj.reshape(-1, 2) - corners.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))


def valid_depth_stats(depth_mm: np.ndarray, u: float, v: float, radius: int, min_mm: float, max_mm: float) -> Tuple[float, float, float, float]:
    h, w = depth_mm.shape[:2]
    x0 = max(0, int(round(u)) - radius)
    x1 = min(w, int(round(u)) + radius + 1)
    y0 = max(0, int(round(v)) - radius)
    y1 = min(h, int(round(v)) + radius + 1)
    roi = depth_mm[y0:y1, x0:x1].astype(np.float32)
    if roi.size == 0:
        return 0.0, 0.0, 0.0, 0.0
    valid = roi[(roi >= min_mm) & (roi <= max_mm)]
    ratio = float(valid.size) / float(roi.size)
    if valid.size == 0:
        return 0.0, 0.0, 0.0, ratio
    p10 = float(np.percentile(valid, 10)) / 1000.0
    med = float(np.median(valid)) / 1000.0
    p90 = float(np.percentile(valid, 90)) / 1000.0
    return med, p90 - p10, p10, ratio


def transform_point(rvec: np.ndarray, tvec: np.ndarray, p_board: np.ndarray) -> np.ndarray:
    rmat, _ = cv2.Rodrigues(rvec)
    return (rmat @ p_board.reshape(3, 1) + tvec.reshape(3, 1)).reshape(3)


def ensure_csv(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        csv.writer(f).writerow(CSV_COLUMNS)


def append_pair(path: Path, p_from: Iterable[float], p_to: Iterable[float], meta: dict) -> None:
    ensure_csv(path)
    row = [
        *[f"{float(v):.9f}" for v in p_from],
        *[f"{float(v):.9f}" for v in p_to],
        meta.get("note", ""),
        f"{float(meta.get('timestamp', 0.0)):.6f}",
        f"{float(meta.get('reproj_err_px', 0.0)):.6f}",
        f"{float(meta.get('depth_median_m', 0.0)):.6f}",
        f"{float(meta.get('pnp_z_m', 0.0)):.6f}",
        f"{float(meta.get('depth_pnp_delta_m', 0.0)):.6f}",
        f"{float(meta.get('depth_valid_ratio', 0.0)):.6f}",
        f"{float(meta.get('depth_spread_m', 0.0)):.6f}",
        f"{float(meta.get('u', 0.0)):.3f}",
        f"{float(meta.get('v', 0.0)):.3f}",
    ]
    with path.open("a", encoding="utf-8", newline="") as f:
        csv.writer(f).writerow(row)


def draw_overlay(bgr: np.ndarray, pattern_size: Tuple[int, int], corners: np.ndarray, accepted: bool, text: str) -> np.ndarray:
    out = bgr.copy()
    cv2.drawChessboardCorners(out, pattern_size, corners, True)
    color = (0, 220, 0) if accepted else (0, 0, 255)
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Collect chessboard hand-eye calibration pairs on Jetson.")
    parser.add_argument("--output-csv", required=True, help="Calibration CSV for export_handeye_yaml.py")
    parser.add_argument("--pattern-cols", type=int, required=True, help="Number of inner corners per row, e.g. 9")
    parser.add_argument("--pattern-rows", type=int, required=True, help="Number of inner corners per column, e.g. 6")
    parser.add_argument("--square-size", type=float, required=True, help="Chessboard square size in meters")
    parser.add_argument("--reference", choices=["center", "origin"], default="center", help="Board point to pair with robot coordinates")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--samples", type=int, default=9, help="Number of accepted point pairs to collect")
    parser.add_argument("--overlay-dir", default="runs/chessboard_calib/overlays")
    parser.add_argument("--min-depth-mm", type=float, default=150.0)
    parser.add_argument("--max-depth-mm", type=float, default=2500.0)
    parser.add_argument("--depth-radius", type=int, default=5)
    parser.add_argument("--min-depth-valid-ratio", type=float, default=0.60)
    parser.add_argument("--max-depth-spread", type=float, default=0.020, help="Reject if p90-p10 depth spread exceeds this many meters")
    parser.add_argument("--max-depth-pnp-delta", type=float, default=0.040, help="Reject if median depth and solvePnP Z differ too much")
    parser.add_argument("--max-reproj-err", type=float, default=0.80, help="Reject if reprojection RMSE exceeds this many pixels")
    parser.add_argument("--min-margin-px", type=int, default=20, help="Reject boards too close to image edge")
    parser.add_argument("--auto-accept", action="store_true", help="Do not ask for confirmation before saving accepted samples")
    parser.add_argument("--note-prefix", default="P")
    args = parser.parse_args()

    pattern = (args.pattern_cols, args.pattern_rows)
    obj_pts = build_object_points(args.pattern_cols, args.pattern_rows, args.square_size)
    ref_board = board_reference_point(args.pattern_cols, args.pattern_rows, args.square_size, args.reference)
    overlay_dir = Path(args.overlay_dir)
    overlay_dir.mkdir(parents=True, exist_ok=True)
    out_csv = Path(args.output_csv)

    cam = OrbbecCamera(width=args.width, height=args.height)
    if not cam.open():
        print("[ERROR] failed to open Orbbec camera", file=sys.stderr)
        return 2
    fx, fy, cx, cy = cam.intrinsics
    if fx <= 0 or fy <= 0:
        print("[ERROR] invalid camera intrinsics; cannot run chessboard calibration", file=sys.stderr)
        cam.close()
        return 3
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    print("棋盘格采集启动。按 Ctrl+C 退出。")
    print(f"pattern={pattern} square={args.square_size:.6f}m reference={args.reference} fx={fx:.1f} fy={fy:.1f}")
    accepted_count = 0
    frame_count = 0
    try:
        while accepted_count < args.samples:
            frame = cam.grab()
            if frame is None:
                time.sleep(0.01)
                continue
            frame_count += 1
            if frame.depth_mm is None:
                continue
            gray = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if not found or corners is None:
                if frame_count % 30 == 0:
                    print("未检测到完整棋盘格，请调整角度/距离/光照")
                continue

            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
            pts2 = corners.reshape(-1, 2)
            margin = min(float(pts2[:, 0].min()), float(pts2[:, 1].min()), float(args.width - pts2[:, 0].max()), float(args.height - pts2[:, 1].max()))

            ok, rvec, tvec = cv2.solvePnP(obj_pts, corners, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            reproj = project_reprojection_error(obj_pts, corners, rvec, tvec, k)
            p_cam = transform_point(rvec, tvec, ref_board)
            ref_img, _ = cv2.projectPoints(ref_board.reshape(1, 3), rvec, tvec, k, None)
            u, v = ref_img.reshape(2)
            depth_med, depth_spread, _depth_p10, depth_ratio = valid_depth_stats(
                frame.depth_mm,
                float(u),
                float(v),
                int(args.depth_radius),
                args.min_depth_mm,
                args.max_depth_mm,
            )
            pnp_z = float(p_cam[2])
            depth_delta = abs(depth_med - pnp_z) if depth_med > 0 else 999.0
            reasons = []
            if margin < args.min_margin_px:
                reasons.append("EDGE")
            if reproj > args.max_reproj_err:
                reasons.append("REPROJ")
            if depth_ratio < args.min_depth_valid_ratio:
                reasons.append("DEPTH_RATIO")
            if depth_spread > args.max_depth_spread:
                reasons.append("DEPTH_SPREAD")
            if depth_delta > args.max_depth_pnp_delta:
                reasons.append("PNP_DEPTH_DELTA")
            accepted = len(reasons) == 0
            text = (
                f"{accepted_count}/{args.samples} err={reproj:.2f}px z={pnp_z:.3f}m "
                f"depth={depth_med:.3f}m spread={depth_spread:.3f} {'OK' if accepted else ','.join(reasons)}"
            )
            overlay = draw_overlay(frame.bgr, pattern, corners, accepted, text)
            cv2.circle(overlay, (int(round(u)), int(round(v))), 5, (255, 0, 0), -1)

            if not accepted:
                if frame_count % 15 == 0:
                    print("拒绝:", text)
                continue

            print("候选点:")
            print(f"  camera_link from = ({p_cam[0]:.6f}, {p_cam[1]:.6f}, {p_cam[2]:.6f}) m")
            print(f"  reproj={reproj:.3f}px depth_med={depth_med:.4f}m spread={depth_spread:.4f}m ratio={depth_ratio:.2f} delta={depth_delta:.4f}m")
            if args.auto_accept:
                save = True
            else:
                raw = input("输入该棋盘格参考点在 robot_base 下的 to_x to_y to_z，或回车跳过: ").strip()
                if not raw:
                    save = False
                    p_to = (0.0, 0.0, 0.0)
                else:
                    try:
                        p_to = parse_vec3(raw)
                        save = True
                    except Exception:
                        print("输入无效，本帧跳过")
                        save = False
                        p_to = (0.0, 0.0, 0.0)
            if not save:
                continue
            if args.auto_accept:
                p_to = prompt_vec3("输入该棋盘格参考点在 robot_base 下的 to_x to_y to_z（米）: ")
            note = f"{args.note_prefix}{accepted_count + 1}"
            meta = {
                "note": note,
                "timestamp": frame.timestamp,
                "reproj_err_px": reproj,
                "depth_median_m": depth_med,
                "pnp_z_m": pnp_z,
                "depth_pnp_delta_m": depth_delta,
                "depth_valid_ratio": depth_ratio,
                "depth_spread_m": depth_spread,
                "u": float(u),
                "v": float(v),
            }
            append_pair(out_csv, p_cam, p_to, meta)
            overlay_path = overlay_dir / f"{note}_{int(time.time())}.jpg"
            cv2.imwrite(str(overlay_path), overlay)
            accepted_count += 1
            print(f"已保存 {accepted_count}/{args.samples}: {out_csv} overlay={overlay_path}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        cam.close()
    print(f"采集完成: accepted={accepted_count}, output={out_csv}")
    return 0 if accepted_count >= 4 else 4


if __name__ == "__main__":
    raise SystemExit(main())
