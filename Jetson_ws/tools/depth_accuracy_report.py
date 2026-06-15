#!/usr/bin/env python3
"""Estimate depth bias and recommend depth_offset_mm / depth_scale_corr.

Run this on Jetson (or any machine) against field_recorder CSVs collected while a
target sat at a KNOWN distance from the camera. It compares the camera's reported
depth to the ground truth and prints a recommended correction for config.json
"depth": { "offset_mm": ..., "scale_corr": ... }.

Correction model (same as pipeline/solver.py):
    z_corrected_mm = z_raw_mm * scale_corr + offset_mm

Single distance  -> recommends an offset_mm only (scale_corr unchanged).
Two+ distances   -> least-squares fit recommends BOTH scale_corr and offset_mm.

Examples:
  # one capture at 0.50 m, depth.csv has z_raw_m (preferred)
  python3 tools/depth_accuracy_report.py --depth-csv runs/field_001/depth.csv \
      --truth-m 0.50 --class-id 2

  # multi-distance fit (recommends scale + offset)
  python3 tools/depth_accuracy_report.py \
      --sample 0.40=runs/d040/depth.csv --sample 0.80=runs/d080/depth.csv \
      --sample 1.20=runs/d120/depth.csv

  # only corrected depth available (e.g. targets.csv); invert with current values
  python3 tools/depth_accuracy_report.py --targets-csv runs/field_001/targets.csv \
      --truth-m 0.60 --current-offset-mm -50 --current-scale 1.0
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import List, Optional, Tuple


def _median(values: List[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def _std(values: List[float]) -> float:
    return float(statistics.pstdev(values)) if len(values) > 1 else 0.0


def _f(row: dict, *keys: str) -> Optional[float]:
    for k in keys:
        if k in row and str(row[k]).strip() not in ("", "nan", "None"):
            try:
                return float(row[k])
            except ValueError:
                continue
    return None


def load_raw_mm(
    path: Path,
    class_id: Optional[int],
    track_id: Optional[int],
    min_valid_ratio: float,
    current_offset_mm: float,
    current_scale: float,
) -> Tuple[List[float], List[float], List[float], int]:
    """Return (raw_mm, corrected_mm, valid_ratio, skipped) lists from a CSV.

    Prefers a z_raw_m column (depth.csv). Falls back to inverting a corrected
    column (z / depth_m / camera_z) using the supplied current offset/scale.
    """
    raw_mm: List[float] = []
    corr_mm: List[float] = []
    ratios: List[float] = []
    skipped = 0
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if class_id is not None and int(_f(row, "class_id") or -1) != class_id:
                continue
            if track_id is not None and int(_f(row, "track_id") or -1) != track_id:
                continue
            vr = _f(row, "valid_ratio", "depth_valid_ratio")
            if vr is not None and vr < min_valid_ratio:
                skipped += 1
                continue
            corrected = _f(row, "z", "depth_m", "camera_z")
            if corrected is None or corrected <= 0.0:
                skipped += 1
                continue
            corrected_mm = corrected * 1000.0
            raw = _f(row, "z_raw_m")
            if raw is not None and raw > 0.0:
                raw_value_mm = raw * 1000.0
            elif abs(current_scale) > 1e-9:
                raw_value_mm = (corrected_mm - current_offset_mm) / current_scale
            else:
                skipped += 1
                continue
            raw_mm.append(raw_value_mm)
            corr_mm.append(corrected_mm)
            ratios.append(vr if vr is not None else 1.0)
    return raw_mm, corr_mm, ratios, skipped


def linear_fit(xs: List[float], ys: List[float]) -> Tuple[float, float, float]:
    """Least-squares y = a*x + b. Returns (a, b, r2)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx < 1e-12:
        return 1.0, my - mx, 0.0
    a = sxy / sxx
    b = my - a * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    return a, b, r2


def describe_sample(truth_m: float, raw_mm: List[float], corr_mm: List[float], ratios: List[float]) -> dict:
    truth_mm = truth_m * 1000.0
    raw_med = _median(raw_mm)
    corr_med = _median(corr_mm)
    return {
        "truth_m": truth_m,
        "n": len(raw_mm),
        "raw_median_mm": raw_med,
        "corr_median_mm": corr_med,
        "corr_bias_mm": corr_med - truth_mm,
        "raw_std_mm": _std(raw_mm),
        "corr_std_mm": _std(corr_mm),
        "valid_ratio_mean": (sum(ratios) / len(ratios)) if ratios else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--depth-csv", help="field_recorder depth.csv (preferred; has z_raw_m)")
    parser.add_argument("--targets-csv", help="field_recorder targets.csv (corrected camera_z only)")
    parser.add_argument("--truth-m", type=float, help="ground-truth camera->target distance in meters for a single capture")
    parser.add_argument("--sample", action="append", default=[], metavar="TRUTH_M=CSV",
                        help="repeatable multi-distance sample, e.g. --sample 0.40=runs/d040/depth.csv")
    parser.add_argument("--class-id", type=int, help="only use rows with this class_id")
    parser.add_argument("--track-id", type=int, help="only use rows with this track_id")
    parser.add_argument("--min-valid-ratio", type=float, default=0.30, help="ignore rows with depth valid_ratio below this")
    parser.add_argument("--current-offset-mm", type=float, default=-50.0, help="current config.json depth.offset_mm (for inversion + comparison)")
    parser.add_argument("--current-scale", type=float, default=1.0, help="current config.json depth.scale_corr")
    args = parser.parse_args()

    samples: List[Tuple[float, Path]] = []
    for s in args.sample:
        if "=" not in s:
            print(f"[ERROR] --sample must be TRUTH_M=CSV, got {s!r}", file=sys.stderr)
            return 2
        truth_s, path_s = s.split("=", 1)
        samples.append((float(truth_s), Path(path_s)))
    if args.truth_m is not None:
        csv_path = args.depth_csv or args.targets_csv
        if not csv_path:
            print("[ERROR] --truth-m needs --depth-csv or --targets-csv", file=sys.stderr)
            return 2
        samples.append((args.truth_m, Path(csv_path)))
    if not samples:
        print("[ERROR] provide --truth-m with a CSV, or one or more --sample TRUTH=CSV", file=sys.stderr)
        return 2

    fit_x: List[float] = []   # raw median mm
    fit_y: List[float] = []   # truth mm
    reports = []
    for truth_m, path in samples:
        if not path.exists():
            print(f"[ERROR] CSV not found: {path}", file=sys.stderr)
            return 2
        raw_mm, corr_mm, ratios, skipped = load_raw_mm(
            path, args.class_id, args.track_id, args.min_valid_ratio, args.current_offset_mm, args.current_scale
        )
        if not raw_mm:
            print(f"[ERROR] no usable depth rows in {path} (skipped={skipped}); check filters/columns", file=sys.stderr)
            return 2
        rep = describe_sample(truth_m, raw_mm, corr_mm, ratios)
        rep["csv"] = str(path)
        rep["skipped"] = skipped
        reports.append(rep)
        fit_x.append(rep["raw_median_mm"])
        fit_y.append(truth_m * 1000.0)

    print("# Depth accuracy report")
    print(f"# correction model: z_corrected_mm = z_raw_mm * scale_corr + offset_mm")
    print(f"# current config: scale_corr={args.current_scale:.6f} offset_mm={args.current_offset_mm:.3f}")
    print(f"{'truth_m':>8} {'n':>5} {'raw_med_mm':>11} {'corr_med_mm':>12} {'corr_bias_mm':>13} {'raw_std_mm':>11} {'vr_mean':>8}")
    for r in reports:
        print(f"{r['truth_m']:>8.3f} {r['n']:>5d} {r['raw_median_mm']:>11.1f} {r['corr_median_mm']:>12.1f} "
              f"{r['corr_bias_mm']:>+13.1f} {r['raw_std_mm']:>11.1f} {r['valid_ratio_mean']:>8.2f}")
        if r["raw_std_mm"] > 15.0:
            print(f"  [WARN] truth={r['truth_m']:.2f}m raw spread {r['raw_std_mm']:.1f}mm is high; surface may be shiny/textureless or ROI too large")
        if r["valid_ratio_mean"] < 0.5:
            print(f"  [WARN] truth={r['truth_m']:.2f}m mean valid_ratio {r['valid_ratio_mean']:.2f} is low; depth is sparse here")

    print()
    if len(reports) == 1:
        truth_mm = reports[0]["truth_m"] * 1000.0
        raw_med = reports[0]["raw_median_mm"]
        rec_offset = truth_mm - args.current_scale * raw_med
        cur_bias = reports[0]["corr_bias_mm"]
        print("# Single distance -> recommend offset only (scale_corr unchanged).")
        print(f"# current corrected bias at this distance: {cur_bias:+.1f} mm")
        print('"depth": {')
        print(f'    "offset_mm": {rec_offset:.1f},')
        print(f'    "scale_corr": {args.current_scale:.6f}')
        print("}")
        print(f"# (move >=2 distances apart, e.g. 0.4/0.8/1.2 m, to also fit scale_corr)")
    else:
        a, b, r2 = linear_fit(fit_x, fit_y)
        print(f"# Multi-distance least-squares fit (R^2={r2:.4f}) over {len(reports)} distances.")
        # residuals after applying the recommended correction
        res = [abs((a * x + b) - y) for x, y in zip(fit_x, fit_y)]
        print(f"# max residual after correction: {max(res):.1f} mm")
        if r2 < 0.99:
            print("# [WARN] R^2 < 0.99: depth is non-linear or samples are noisy; collect cleaner captures")
        print('"depth": {')
        print(f'    "offset_mm": {b:.1f},')
        print(f'    "scale_corr": {a:.6f}')
        print("}")
    print("\n# Apply by editing config.json depth.* then restart; verify with a fresh capture at a new distance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
