#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

RUN_DIR="${TECHX_CALIB_DIR:-runs/calib_robot_camera}"
PATTERN_COLS="${TECHX_CHESSBOARD_COLS:-9}"
PATTERN_ROWS="${TECHX_CHESSBOARD_ROWS:-6}"
SQUARE_SIZE="${TECHX_CHESSBOARD_SQUARE_M:-0.025}"
SAMPLES="${TECHX_CALIB_SAMPLES:-9}"
WIDTH="${TECHX_CAMERA_WIDTH:-640}"
HEIGHT="${TECHX_CAMERA_HEIGHT:-480}"
# Tight field defaults aimed at ~±1cm grasping. Relax for early bring-up via env.
MAX_RMSE="${TECHX_CALIB_MAX_RMSE:-0.010}"
MAX_POINT_ERROR="${TECHX_CALIB_MAX_POINT_ERROR:-0.020}"

POINTS_CSV="${RUN_DIR}/robot_camera_points.csv"
YAML_OUT="${RUN_DIR}/gmk_robot_camera.yaml"
REPORT_JSON="${RUN_DIR}/gmk_robot_camera_report.json"
RESIDUAL_CSV="${RUN_DIR}/gmk_robot_camera_residuals.csv"
OVERLAY_DIR="${RUN_DIR}/overlays"

mkdir -p "${RUN_DIR}" "${OVERLAY_DIR}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[TECHX] ERROR: python3 not found" >&2
  exit 2
fi

python3 - <<'PY'
import sys
print('[TECHX] Python version:', sys.version.split()[0])
if sys.version_info < (3, 10):
    raise SystemExit('[TECHX] ERROR: Python >= 3.10 is required by current field branch')
try:
    import cv2
    import numpy
    import pyorbbecsdk  # noqa: F401
except Exception as exc:
    raise SystemExit('[TECHX] ERROR: calibration dependency check failed: %s' % exc)
print('[TECHX] OK deps: cv2', cv2.__version__, 'numpy', numpy.__version__, 'pyorbbecsdk')
PY

echo "[TECHX] Chessboard hand-eye calibration"
echo "[TECHX] Run dir       : ${RUN_DIR}"
echo "[TECHX] Pattern       : ${PATTERN_COLS}x${PATTERN_ROWS} inner corners"
echo "[TECHX] Square size   : ${SQUARE_SIZE} m"
echo "[TECHX] Samples       : ${SAMPLES}"
echo "[TECHX] Camera        : ${WIDTH}x${HEIGHT}"
echo "[TECHX] Points CSV    : ${POINTS_CSV}"
echo "[TECHX] YAML output   : ${YAML_OUT}"
echo "[TECHX] Thresholds    : rmse<=${MAX_RMSE}m max_point<=${MAX_POINT_ERROR}m"
echo ""
echo "[TECHX] Important: each accepted board sample still needs the board reference point in robot_base coordinates."
echo "[TECHX] Use meters. Example input: 0.350 0.000 0.220"
echo ""

python3 tools/collect_chessboard_handeye.py \
  --output-csv "${POINTS_CSV}" \
  --pattern-cols "${PATTERN_COLS}" \
  --pattern-rows "${PATTERN_ROWS}" \
  --square-size "${SQUARE_SIZE}" \
  --reference center \
  --width "${WIDTH}" \
  --height "${HEIGHT}" \
  --samples "${SAMPLES}" \
  --overlay-dir "${OVERLAY_DIR}"

python3 tools/export_handeye_yaml.py \
  --csv "${POINTS_CSV}" \
  --name T_robot_camera \
  --output-yaml "${YAML_OUT}" \
  --report-json "${REPORT_JSON}" \
  --residual-csv "${RESIDUAL_CSV}" \
  --max-rmse "${MAX_RMSE}" \
  --max-point-error "${MAX_POINT_ERROR}"

echo "[TECHX] Calibration complete"
echo "[TECHX] Copy YAML into GMK vision_bridge.yaml: ${YAML_OUT}"
echo "[TECHX] Audit report: ${REPORT_JSON}"
echo "[TECHX] Residuals   : ${RESIDUAL_CSV}"
