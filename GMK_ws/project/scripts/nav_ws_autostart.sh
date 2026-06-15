#!/usr/bin/env bash
set -euo pipefail

# nav_ws autostart entrypoint
# - source ROS2 Humble + workspace overlay
# - start local_planner nav (rviz disabled) + waypoint_search
# - handle SIGTERM/SIGINT and stop the whole process group

WS_DIR=~/nav_ws

set +u
source "/opt/ros/humble/setup.bash"

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  # overlay for this workspace (merged or isolated install both provide setup.bash at top-level)
  source "${WS_DIR}/install/setup.bash"
else
  echo "[nav_ws_autostart] ERROR: ${WS_DIR}/install/setup.bash not found" >&2
  exit 1
fi
set -u

cd "${WS_DIR}"

echo "[nav_ws_autostart] Starting local_planner nav.launch.py (rviz_enable:=false)"
ros2 launch local_planner nav.launch.py rviz_enable:=false &
PID_NAV=$!

# Give core nodes a moment to come up (optional but helps reduce race conditions)
sleep 2

echo "[nav_ws_autostart] Starting waypoint_manager waypoint_search.launch.py"
ros2 launch waypoint_manager waypoint_search.launch.py &
PID_WAYPOINT=$!

stop_once=0
stop_children() {
  # 防止信号重入导致重复打印/重复 kill
  if [[ "${stop_once}" -ne 0 ]]; then
    return 0
  fi
  stop_once=1

  # 先取消 trap，避免 kill 子进程时再触发自身 trap 重入
  trap - SIGINT SIGTERM

  echo "[nav_ws_autostart] Caught termination signal, stopping..."

  # 只杀两个 ros2 launch 入口进程（它们会自行清理/转发给子进程）
  kill -TERM "${PID_WAYPOINT}" 2>/dev/null || true
  kill -TERM "${PID_NAV}" 2>/dev/null || true
}

trap stop_children SIGINT SIGTERM

# 等待任意一个退出；如果其中一个异常退出，也停止另一个
wait -n "${PID_NAV}" "${PID_WAYPOINT}" || true
echo "[nav_ws_autostart] One launch exited; stopping remaining processes..."
stop_children

# 等待两个都退出（避免脚本提前退出导致 systemd 误判）
wait "${PID_NAV}" 2>/dev/null || true
wait "${PID_WAYPOINT}" 2>/dev/null || true