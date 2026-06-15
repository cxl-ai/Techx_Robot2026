#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "========================================"
echo "  TECHX_vision Jetson fast startup"
echo "========================================"

ENV_NAME="${TECHX_CONDA_ENV-techx}"
if [ -n "$ENV_NAME" ] && command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)" || true
    conda activate "$ENV_NAME" >/dev/null 2>&1 || echo "warn: conda env $ENV_NAME not activated"
fi

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
    PY="$(command -v python3 || command -v python)"
fi
if [ -z "$PY" ]; then
    echo "error: python not found"
    exit 1
fi

CAMERA_WAIT_TIMEOUT="${CAMERA_WAIT_TIMEOUT:-600}"
NO_CAMERA_FRAME_TIMEOUT="${NO_CAMERA_FRAME_TIMEOUT:-600}"
NO_INFER_RESULT_TIMEOUT="${NO_INFER_RESULT_TIMEOUT:-600}"
LOG_LEVEL="${LOG_LEVEL:-INFO}"
TECHX_SKIP_TIME_SYNC="${TECHX_SKIP_TIME_SYNC:-1}"
TECHX_SKIP_JETSON_OPT="${TECHX_SKIP_JETSON_OPT:-1}"
TECHX_SYSLOG="${TECHX_SYSLOG:-0}"
TECHX_LOG_FILE="${TECHX_LOG_FILE:-logs/techx_current.log}"
TECHX_NO_TORCH_BACKEND="${TECHX_NO_TORCH_BACKEND:-1}"
export TECHX_SKIP_TIME_SYNC TECHX_SKIP_JETSON_OPT TECHX_SYSLOG TECHX_LOG_FILE TECHX_NO_TORCH_BACKEND

ARGS=""
if [ "${TECHX_FULL_CHECK:-0}" != "1" ]; then
    ARGS="$ARGS --fast-start"
fi
if [ "${TECHX_GUI:-0}" != "1" ]; then
    ARGS="$ARGS --headless"
fi
if [ "${ALLOW_UVC_FALLBACK:-0}" = "1" ]; then
    ARGS="$ARGS --allow-uvc-fallback"
fi

MAIN_LOG="$DIR/$TECHX_LOG_FILE"

if [ "$TECHX_SYSLOG" = "1" ]; then
    echo "syslog: enabled, view with: journalctl -t techx_vision -f"
else
    echo "syslog: disabled; main log is enough for normal debugging"
fi

echo "main log: $MAIN_LOG"
echo "watch log: tail -f $TECHX_LOG_FILE"
echo "summary : $PY tools/log_summary.py $TECHX_LOG_FILE"
echo "python: $($PY --version 2>&1)"
echo "mode:$ARGS camera_wait=${CAMERA_WAIT_TIMEOUT}s no_frame=${NO_CAMERA_FRAME_TIMEOUT}s no_infer=${NO_INFER_RESULT_TIMEOUT}s log=${LOG_LEVEL}"
echo "========================================"

exec "$PY" launch.py $ARGS \
    --camera-wait-timeout "$CAMERA_WAIT_TIMEOUT" \
    --no-camera-frame-timeout "$NO_CAMERA_FRAME_TIMEOUT" \
    --no-infer-result-timeout "$NO_INFER_RESULT_TIMEOUT" \
    --log-level "$LOG_LEVEL" "$@"
