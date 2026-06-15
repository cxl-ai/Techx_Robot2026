#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

CONFIG="${TECHX_CONFIG:-config.json}"
NET_IFACE="${TECHX_NET_IFACE:-}"
JETSON_IP="192.168.10.101"
GMK_IP="192.168.10.100"
UDP_PORT="12345"

export TECHX_STAGE="${TECHX_STAGE:-all}"
export TECHX_DEBUG_RECORDER="${TECHX_DEBUG_RECORDER:-1}"
export TECHX_DEBUG_DIR="${TECHX_DEBUG_DIR:-runs/field_001}"
export TECHX_DEBUG_FRAME_EVERY="${TECHX_DEBUG_FRAME_EVERY:-10}"
export TECHX_NO_TORCH_BACKEND="${TECHX_NO_TORCH_BACKEND:-1}"

BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
CONFIG_ABS="$(python3 - <<PY 2>/dev/null || realpath "${CONFIG}" 2>/dev/null || echo "${CONFIG}"
import os
print(os.path.abspath('${CONFIG}'))
PY
)"

echo "[TECHX] Jetson field startup"
echo "[TECHX] Repo      : ${REPO_DIR}"
echo "[TECHX] Branch    : ${BRANCH}"
echo "[TECHX] Commit    : ${COMMIT}"
echo "[TECHX] Config    : ${CONFIG_ABS}"
echo "[TECHX] Stage     : ${TECHX_STAGE}"
echo "[TECHX] Recorder  : ${TECHX_DEBUG_RECORDER} dir=${TECHX_DEBUG_DIR} frame_every=${TECHX_DEBUG_FRAME_EVERY}"
echo "[TECHX] Network   : Jetson=${JETSON_IP} GMK=${GMK_IP} UDP=${UDP_PORT} iface=${NET_IFACE:-auto}"

run_net_setup() {
  local args=(jetson)
  if [[ -n "${NET_IFACE}" ]]; then
    args+=("${NET_IFACE}")
  fi
  if [[ "${EUID}" -eq 0 ]]; then
    bash scripts/setup_field_network.sh "${args[@]}"
  else
    sudo bash scripts/setup_field_network.sh "${args[@]}"
  fi
}

if [[ "${TECHX_SKIP_NET_SETUP:-0}" != "1" ]]; then
  run_net_setup
else
  echo "[TECHX] Skip network setup because TECHX_SKIP_NET_SETUP=1"
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[TECHX] ERROR: python3 not found" >&2
  exit 2
fi

python3 - <<'PY'
import sys
print('[TECHX] Python version:', sys.version.split()[0])
if sys.version_info < (3, 10):
    raise SystemExit('[TECHX] ERROR: this branch uses Python 3.10+ typing syntax; install/use Python >= 3.10 or backport type hints to Python 3.8.')
PY

check_model_dir() {
  local dir="$1"
  if [[ -f "${dir}/best.engine" || -f "${dir}/best.onnx" || -f "${dir}/best.pt" ]]; then
    echo "[TECHX] OK model: ${dir}"
    return 0
  fi
  echo "[TECHX] ERROR missing model in ${dir}; expected best.engine / best.onnx / best.pt" >&2
  return 1
}

python3 - <<PY
import json, os, sys
cfg = json.load(open("${CONFIG}", encoding="utf-8"))
stage = os.environ.get("TECHX_STAGE", "all").lower()
models = cfg.get("models", []) or []
selected = []
for model in models:
    if not model.get("enabled", True):
        continue
    folder = str(model.get("folder", "")).strip()
    name = str(model.get("name", "")).lower()
    if stage == "all" or (stage == "kfs" and "kfs" in folder.lower()) or (stage == "head" and ("head" in folder.lower() or "weapon" in name)):
        selected.append(folder)
if not selected and stage not in {"assembly", "qr"}:
    raise SystemExit("[TECHX] ERROR: no enabled model selected for TECHX_STAGE=%s" % stage)
for folder in selected:
    path = os.path.join("models", folder)
    if not any(os.path.exists(os.path.join(path, name)) for name in ("best.engine", "best.onnx", "best.pt")):
        raise SystemExit("[TECHX] ERROR missing model in %s; expected best.engine / best.onnx / best.pt" % path)
    print("[TECHX] OK model:", path)
PY

python3 - <<'PY'
try:
    import cv2
except Exception as exc:
    raise SystemExit('[TECHX] ERROR cv2 import failed: %s' % exc)
try:
    import numpy
except Exception as exc:
    raise SystemExit('[TECHX] ERROR numpy import failed: %s' % exc)
print('[TECHX] OK python deps: cv2', cv2.__version__, 'numpy', numpy.__version__)
try:
    import pyorbbecsdk  # noqa: F401
    print('[TECHX] OK pyorbbecsdk')
except Exception as exc:
    raise SystemExit('[TECHX] ERROR pyorbbecsdk import failed: %s' % exc)
PY

# ── Power / performance pre-flight (informational; never blocks startup) ──
echo "[TECHX] Power / performance pre-flight:"
if command -v nvpmodel >/dev/null 2>&1; then
  NVP="$(nvpmodel -q 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g')"
  echo "[TECHX]   nvpmodel: ${NVP:-unknown}"
  if echo "${NVP}" | grep -qiE 'MAXN|Mode: *0'; then
    echo "[TECHX]   OK: Jetson in max power mode"
  else
    echo "[TECHX]   WARN: Jetson NOT in max power mode -> YOLO/TensorRT will be slow." >&2
    echo "[TECHX]          Run once: sudo nvpmodel -m 0 && sudo jetson_clocks" >&2
  fi
else
  echo "[TECHX]   nvpmodel not found (JetPack tools missing or not a Jetson)"
fi
for Z in /sys/devices/virtual/thermal/thermal_zone*/temp; do
  [[ -r "${Z}" ]] || continue
  T="$(cat "${Z}" 2>/dev/null || echo 0)"
  if [[ "${T}" =~ ^[0-9]+$ ]] && [[ "${T}" -ge 85000 ]]; then
    echo "[TECHX]   WARN: ${Z%/temp} at $((T/1000))C -> thermal throttle risk; check cooling/airflow." >&2
  fi
done
echo "[TECHX]   POWER NOTE: power the Orin from a dedicated, regulated DC-DC source, NOT directly off the raw"
echo "[TECHX]              motor/battery bus. Motor current spikes can brown out the Jetson and reboot it mid-match."

if ping -c 1 -W 1 "${GMK_IP}" >/dev/null 2>&1; then
  echo "[TECHX] OK: GMK ${GMK_IP} reachable"
else
  echo "[TECHX] ERROR: GMK ${GMK_IP} is not reachable. Check GMK power/cable/IP/firewall before starting Jetson." >&2
  exit 4
fi

ip -br addr || true

echo "[TECHX] Starting Jetson vision: python3 main.py --config ${CONFIG} --headless"
exec python3 main.py --config "${CONFIG}" --headless --log-level "${TECHX_LOG_LEVEL:-INFO}"
