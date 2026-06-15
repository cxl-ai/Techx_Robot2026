#!/usr/bin/env bash
# Install + enable the headless boot auto-start service on the Jetson.
#
#   sudo bash scripts/install_vision_service.sh
#
# After this, the vision runs automatically on every boot with NO monitor.
# Prereq (once): run `sudo bash scripts/setup_field_network.sh jetson` so the
# field IP is persistent, otherwise the service restart-loops on UDP bind.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="${REPO_DIR}/scripts/techx-vision.service"
UNIT="/etc/systemd/system/techx-vision.service"
WRAPPER="${REPO_DIR}/scripts/run_vision_service.sh"

if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: needs root to install a systemd unit. Run: sudo bash scripts/install_vision_service.sh" >&2
  exit 1
fi
[[ -f "${TEMPLATE}" ]] || { echo "ERROR: missing ${TEMPLATE}" >&2; exit 1; }

# The service should run as the login user (conda env + camera access), not root.
SERVICE_USER="${TECHX_SERVICE_USER:-${SUDO_USER:-root}}"
if [[ "${SERVICE_USER}" == "root" ]]; then
  echo "WARN: installing to run as root; conda env / camera perms may differ. Set TECHX_SERVICE_USER=<user> to override." >&2
fi

chmod +x "${WRAPPER}" || true
sed -e "s|@REPO@|${REPO_DIR}|g" -e "s|@USER@|${SERVICE_USER}|g" "${TEMPLATE}" > "${UNIT}"
systemctl daemon-reload
systemctl enable techx-vision.service

echo "[TECHX] Installed: ${UNIT} (user=${SERVICE_USER}, repo=${REPO_DIR})"
echo "[TECHX] AUTOSTART TOGGLE: edit ${REPO_DIR}/scripts/autostart.env"
echo "         TECHX_AUTOSTART=false -> boots normally, vision does NOT auto-run (default)"
echo "         TECHX_AUTOSTART=true  -> Orin auto-runs the vision on every boot"
echo "         (the service stays installed either way; just flip this one line + reboot)"
echo "[TECHX] Manage the service with:"
echo "  sudo systemctl start techx-vision      # start now"
echo "  sudo systemctl status techx-vision     # check state"
echo "  journalctl -u techx-vision -f          # live logs"
echo "  sudo systemctl restart techx-vision    # restart"
echo "  sudo systemctl disable techx-vision    # stop auto-start on boot"
echo "[TECHX] Reminder: run setup_field_network.sh once so the field IP is persistent."
