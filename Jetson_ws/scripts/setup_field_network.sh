#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-jetson}"
IFACE="${2:-${TECHX_NET_IFACE:-}}"

JETSON_IP="192.168.10.101"
GMK_IP="192.168.10.100"
CIDR="24"
PORT="12345"

usage() {
  cat <<'USAGE'
Usage:
  sudo bash scripts/setup_field_network.sh jetson [interface]

Defaults:
  jetson IP = 192.168.10.101/24
  peer GMK  = 192.168.10.100
  UDP port  = 12345

If [interface] is omitted, the script auto-selects a wired interface with carrier.
You can also set TECHX_NET_IFACE=eth0.
USAGE
}

if [[ "${ROLE}" == "-h" || "${ROLE}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "${ROLE}" != "jetson" ]]; then
  echo "ERROR: this script is for the Jetson vision repo; role must be 'jetson'." >&2
  usage
  exit 2
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: network setup needs sudo/root." >&2
  echo "Run: sudo bash scripts/setup_field_network.sh jetson [interface]" >&2
  exit 1
fi

is_candidate_iface() {
  local name="$1"
  [[ "${name}" == "lo" ]] && return 1
  [[ "${name}" == wl* ]] && return 1
  [[ "${name}" == docker* ]] && return 1
  [[ "${name}" == br-* ]] && return 1
  [[ "${name}" == veth* ]] && return 1
  [[ "${name}" == virbr* ]] && return 1
  [[ "${name}" == tailscale* ]] && return 1
  [[ "${name}" == tun* ]] && return 1
  return 0
}

pick_iface() {
  local found=()
  for p in /sys/class/net/*; do
    local n
    n="$(basename "$p")"
    is_candidate_iface "$n" || continue
    if [[ -r "$p/carrier" ]] && [[ "$(cat "$p/carrier" 2>/dev/null || echo 0)" == "1" ]]; then
      found+=("$n")
    fi
  done
  if [[ "${#found[@]}" -eq 1 ]]; then
    echo "${found[0]}"
    return 0
  fi
  if [[ "${#found[@]}" -gt 1 ]]; then
    echo "ERROR: multiple wired interfaces with carrier: ${found[*]}" >&2
    echo "Pass one explicitly, e.g. sudo bash scripts/setup_field_network.sh jetson eth0" >&2
    exit 3
  fi
  for p in /sys/class/net/*; do
    local n
    n="$(basename "$p")"
    is_candidate_iface "$n" || continue
    found+=("$n")
  done
  if [[ "${#found[@]}" -eq 1 ]]; then
    echo "${found[0]}"
    return 0
  fi
  echo "ERROR: could not auto-select a wired interface." >&2
  echo "Available interfaces:" >&2
  ip -br link >&2
  echo "Pass one explicitly, e.g. sudo bash scripts/setup_field_network.sh jetson eth0" >&2
  exit 3
}

if [[ -z "${IFACE}" ]]; then
  IFACE="$(pick_iface)"
fi

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "ERROR: interface '${IFACE}' does not exist." >&2
  ip -br link >&2
  exit 4
fi

echo "[TECHX] Configuring Jetson field network"
echo "[TECHX] Interface : ${IFACE}"
echo "[TECHX] Jetson IP : ${JETSON_IP}/${CIDR}"
echo "[TECHX] GMK IP    : ${GMK_IP}"
echo "[TECHX] UDP port  : ${PORT}"

CON_NAME="techx-field"
# Prefer a PERSISTENT NetworkManager profile (JetPack/Ubuntu use NM by default).
# This survives reboots and NM will not steal the IP back to DHCP. Set it once and
# you can just power on and run. Falls back to a transient ip-addr config only when
# NetworkManager is unavailable (lost on reboot).
if command -v nmcli >/dev/null 2>&1 && nmcli general status >/dev/null 2>&1; then
  echo "[TECHX] Using NetworkManager profile '${CON_NAME}' (persistent across reboots)."
  if nmcli -t -f NAME con show 2>/dev/null | grep -qx "${CON_NAME}"; then
    nmcli con delete "${CON_NAME}" >/dev/null 2>&1 || true
  fi
  nmcli con add type ethernet con-name "${CON_NAME}" ifname "${IFACE}" \
        ipv4.method manual ipv4.addresses "${JETSON_IP}/${CIDR}" ipv6.method ignore \
        connection.autoconnect yes connection.autoconnect-priority 999 >/dev/null
  nmcli con up "${CON_NAME}" >/dev/null
  echo "[TECHX] Persistent IP set. It will auto-apply on every boot; re-running this is safe."
else
  echo "[TECHX] NetworkManager not available; using TRANSIENT ip-addr config (lost on reboot)." >&2
  ip link set dev "${IFACE}" up
  ip addr flush dev "${IFACE}"
  ip addr add "${JETSON_IP}/${CIDR}" dev "${IFACE}"
  # Route is explicit for the direct Jetson<->GMK link; no gateway is required.
  ip route replace "192.168.10.0/24" dev "${IFACE}" src "${JETSON_IP}"
fi

ip -br addr show "${IFACE}"

echo "[TECHX] Network configured. Testing peer ping..."
if ping -c 1 -W 1 "${GMK_IP}" >/dev/null 2>&1; then
  echo "[TECHX] OK: GMK ${GMK_IP} is reachable."
else
  echo "[TECHX] WARN: GMK ${GMK_IP} is not reachable yet. Check GMK power/cable/IP/firewall." >&2
fi

echo "[TECHX] Done. Start Jetson vision after GMK bridge is running."