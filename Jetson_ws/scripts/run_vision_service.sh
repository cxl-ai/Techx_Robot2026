#!/usr/bin/env bash
# Boot wrapper used by techx-vision.service to run the Jetson vision headless.
# Network IP is already persistent (NetworkManager), so this does NOT touch the
# network or need root. It activates the conda env (if any) and launches headless.
#
# Override via the service Environment= or env file:
#   TECHX_CONDA_ENV   conda env name (default: techx)
#   TECHX_CONDA_SH    explicit path to conda.sh
#   PYTHON            explicit python to use (skips conda search)
#   TECHX_STAGE       all | head | kfs | assembly | qr (default: all)
#   TECHX_DEBUG_DIR   recorder dir (default: runs/boot_<timestamp>)
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

# Boot autostart toggle. systemd also loads this via EnvironmentFile; we source it
# too so a manual run behaves the same. TECHX_AUTOSTART must be 'true' to run.
ENV_FILE="${REPO_DIR}/scripts/autostart.env"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi
if [[ "${TECHX_AUTOSTART:-false}" != "true" ]]; then
  echo "[TECHX] boot autostart disabled (TECHX_AUTOSTART=${TECHX_AUTOSTART:-false}); start manually with start_jetson.sh"
  exit 0
fi

ENV_NAME="${TECHX_CONDA_ENV:-techx}"
if [[ -z "${PYTHON:-}" ]]; then
  for CSH in "${TECHX_CONDA_SH:-}" \
             "${HOME}/miniconda3/etc/profile.d/conda.sh" \
             "${HOME}/archiconda3/etc/profile.d/conda.sh" \
             "${HOME}/mambaforge/etc/profile.d/conda.sh" \
             "${HOME}/miniforge3/etc/profile.d/conda.sh" \
             "/opt/conda/etc/profile.d/conda.sh"; do
    if [[ -n "${CSH}" && -f "${CSH}" ]]; then
      # shellcheck disable=SC1090
      source "${CSH}" && conda activate "${ENV_NAME}" 2>/dev/null && break
    fi
  done
fi

PY="${PYTHON:-$(command -v python3 || command -v python)}"
if [[ -z "${PY}" ]]; then
  echo "[TECHX] ERROR: no python found (set PYTHON or install conda env ${ENV_NAME})" >&2
  exit 1
fi

export TECHX_STAGE="${TECHX_STAGE:-all}"
export TECHX_DEBUG_RECORDER="${TECHX_DEBUG_RECORDER:-1}"
export TECHX_DEBUG_DIR="${TECHX_DEBUG_DIR:-runs/boot_$(date +%Y%m%d_%H%M%S)}"
export TECHX_SKIP_TIME_SYNC="${TECHX_SKIP_TIME_SYNC:-1}"
export TECHX_SKIP_JETSON_OPT="${TECHX_SKIP_JETSON_OPT:-1}"
export TECHX_LOG_FILE="${TECHX_LOG_FILE:-logs/techx_current.log}"

echo "[TECHX] boot service: python=${PY} stage=${TECHX_STAGE} dir=${TECHX_DEBUG_DIR}"
exec "${PY}" launch.py --headless --fast-start \
  --camera-wait-timeout "${CAMERA_WAIT_TIMEOUT:-600}" \
  --no-camera-frame-timeout "${NO_CAMERA_FRAME_TIMEOUT:-600}" \
  --no-infer-result-timeout "${NO_INFER_RESULT_TIMEOUT:-600}" \
  --log-level "${LOG_LEVEL:-INFO}"
