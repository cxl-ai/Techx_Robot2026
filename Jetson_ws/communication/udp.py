"""UDP sender for TECHx vision telemetry.

send_many() emits one V2 telemetry datagram for every new inference result.
V2 may carry all fresh detections in one frame, or count=0 when no fresh target
exists. A target with z=0 carries recognition data but no valid 3D coordinate.

Field network is fixed for competition use:
    Jetson vision sender: 192.168.10.101
    GMK receiver:         192.168.10.100:12345

The sender binds its source socket to the Jetson IP by default so multi-NIC
Jetson setups do not accidentally route UDP through Wi-Fi or another adapter.
A constructor local_ip from config.json has highest priority. TECHX_UDP_LOCAL_IP
can still override the built-in field default when no constructor local_ip is
provided.

Legacy 29-byte XYZ output is disabled by default for competition use. Set
TECHX_SEND_LEGACY_UDP=1 only when debugging an old GMK receiver.
"""

from __future__ import annotations

import os
import socket
import struct
import time
from typing import List, Optional

from interface.interfaces import ISender
from interface.types import Target3D
from utils.crc16 import crc16_ccitt
from utils.logger import get_logger

log = get_logger(__name__)

_MAGIC_LEGACY = 0x55AA
_MAGIC_V2 = 0x55AB
_VERSION_V2 = 2
_UINT32_MASK = 0xFFFFFFFF
_MAX_TARGETS_PER_V2_PACKET = 16

_FIELD_JETSON_IP = "192.168.10.101"
_FIELD_GMK_IP = "192.168.10.100"
_FIELD_UDP_PORT = 12345

_LEGACY_PAYLOAD_FMT = "<H I d B 3f"
_LEGACY_FRAME_FMT = "<H I d B 3f H"
_LEGACY_PAYLOAD_SIZE = struct.calcsize(_LEGACY_PAYLOAD_FMT)
_LEGACY_FRAME_SIZE = struct.calcsize(_LEGACY_FRAME_FMT)

_V2_HEADER_FMT = "<H B B I d B"
_V2_TARGET_FMT = "<B B B f 2f 3f"
_V2_HEADER_SIZE = struct.calcsize(_V2_HEADER_FMT)
_V2_TARGET_SIZE = struct.calcsize(_V2_TARGET_FMT)


def _legacy_enabled_from_env() -> bool:
    return os.getenv("TECHX_SEND_LEGACY_UDP", "0").strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return default


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    if value is None:
        return default
    value = value.strip()
    return value if value else default


def _resolve_local_ip(config_local_ip: Optional[str]) -> str:
    if config_local_ip is not None and str(config_local_ip).strip():
        return str(config_local_ip).strip()
    return _env_str("TECHX_UDP_LOCAL_IP", _FIELD_JETSON_IP)


def _color_code(target: Target3D) -> int:
    color = (target.color or "").lower()
    name = (target.class_name or "").lower()
    if color == "red" or name.endswith("_red") or "_red" in name:
        return 1
    if color == "blue" or name.endswith("_blue") or "_blue" in name:
        return 2
    return 0


class UdpSender(ISender):
    def __init__(self, target_ip: str, target_port: int, local_ip: Optional[str] = None, local_port: int = 0):
        self._addr = (target_ip, target_port)
        self._seq = 0
        self._sock: Optional[socket.socket] = None
        self._ready = False
        self._send_legacy_enabled = _legacy_enabled_from_env()
        self._local_ip = _resolve_local_ip(local_ip)
        self._local_port = int(local_port or _env_int("TECHX_UDP_LOCAL_PORT", 0))
        self._v2_log_every = max(0, _env_int("TECHX_UDP_LOG_EVERY", 30))
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setblocking(False)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            if self._local_ip and self._local_ip not in {"0.0.0.0", "*"}:
                self._sock.bind((self._local_ip, self._local_port))
            self._ready = True
            log.info(
                "UDP sender ready %s:%d → %s:%d (V2%s, log_every=%d)",
                self._local_ip or "auto",
                self._local_port,
                target_ip,
                target_port,
                "+legacy" if self._send_legacy_enabled else " only",
                self._v2_log_every,
            )
            if target_ip != _FIELD_GMK_IP or target_port != _FIELD_UDP_PORT:
                log.warning(
                    "Field UDP target differs from competition default: configured=%s:%d expected=%s:%d",
                    target_ip,
                    target_port,
                    _FIELD_GMK_IP,
                    _FIELD_UDP_PORT,
                )
        except OSError as e:
            log.error(
                "UDP socket bind/init failed: %s. Expected field network Jetson=%s GMK=%s:%d. "
                "Configure Jetson NIC to %s/24, fix config.json udp.local_ip, or use TECHX_UDP_LOCAL_IP only when config local_ip is empty.",
                e,
                _FIELD_JETSON_IP,
                _FIELD_GMK_IP,
                _FIELD_UDP_PORT,
                _FIELD_JETSON_IP,
            )
            self._sock = None
            self._ready = False
            # Standalone/Windows model validation: same config.json, no GMK/NIC.
            # Set TECHX_UDP_OPTIONAL=1 to keep running without UDP instead of exiting.
            if os.getenv("TECHX_UDP_OPTIONAL", "0").strip().lower() in {"1", "true", "yes", "on"}:
                log.warning(
                    "TECHX_UDP_OPTIONAL=1: continuing WITHOUT UDP (sender not ready). "
                    "Use only for standalone validation, never on the competition Jetson."
                )
                return
            raise SystemExit(4)

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & _UINT32_MASK
        return self._seq

    def _should_log_v2(self, seq: int, count: int) -> bool:
        if self._v2_log_every <= 0:
            return False
        if seq % self._v2_log_every == 0:
            return True
        return count == 0 and seq % max(self._v2_log_every, 30) == 0

    def send(self, target: Target3D) -> bool:
        if self._send_legacy_enabled:
            return self._send_legacy(target)
        return self._send_v2([target], timestamp=target.timestamp)

    def send_many(self, targets: List[Target3D], timestamp: Optional[float] = None) -> int:
        if not self._ready or self._sock is None:
            return 0
        limited = targets[:_MAX_TARGETS_PER_V2_PACKET]
        sent = 0
        if self._send_v2(limited, timestamp=timestamp):
            sent += 1
        if self._send_legacy_enabled:
            valid_xyz = [t for t in limited if t.z > 0]
            if valid_xyz and self._send_legacy(valid_xyz[0]):
                sent += 1
        return sent

    def _send_legacy(self, target: Target3D) -> bool:
        xc, yc, zc = target.camera_xyz
        if zc <= 0:
            return False
        if not self._ready or self._sock is None:
            return False
        seq = self._next_seq()
        tid_byte = target.track_id & 0xFF
        try:
            payload = struct.pack(
                _LEGACY_PAYLOAD_FMT,
                _MAGIC_LEGACY,
                seq,
                float(target.timestamp),
                tid_byte,
                float(xc), float(yc), float(zc),
            )
            crc = crc16_ccitt(payload)
            frame = struct.pack(
                _LEGACY_FRAME_FMT,
                _MAGIC_LEGACY, seq, float(target.timestamp),
                tid_byte, float(xc), float(yc), float(zc), crc,
            )
            if len(payload) != _LEGACY_PAYLOAD_SIZE or len(frame) != _LEGACY_FRAME_SIZE:
                log.debug("UDP legacy frame size mismatch")
                return False
            self._sock.sendto(frame, self._addr)
            if seq % 10 == 0:
                log.info(
                    "UDP legacy→GMK #%d T%d %s | XYZ=(%+.4f,%+.4f,%+.4f)m | CRC=0x%04X",
                    seq, tid_byte, target.class_name, xc, yc, zc, crc,
                )
            return True
        except (BlockingIOError, OSError, struct.error, TypeError, ValueError) as e:
            log.debug("UDP legacy send failed: %s", e)
            return False

    def _send_v2(self, targets: List[Target3D], timestamp: Optional[float] = None) -> bool:
        if not self._ready or self._sock is None:
            return False
        seq = self._next_seq()
        count = min(len(targets), _MAX_TARGETS_PER_V2_PACKET)
        if timestamp is not None:
            packet_ts = float(timestamp)
        elif count > 0:
            packet_ts = float(targets[0].timestamp)
        else:
            packet_ts = time.time()
        try:
            payload = struct.pack(_V2_HEADER_FMT, _MAGIC_V2, _VERSION_V2, 0, seq, packet_ts, count)
            for target in targets[:count]:
                xc, yc, zc = target.camera_xyz
                u, v = target.pixel_uv
                payload += struct.pack(
                    _V2_TARGET_FMT,
                    target.track_id & 0xFF,
                    max(0, int(target.class_id)) & 0xFF,
                    _color_code(target),
                    max(0.0, min(1.0, float(target.confidence))),
                    float(u), float(v),
                    float(xc), float(yc), float(zc),
                )
            crc = crc16_ccitt(payload)
            frame = payload + struct.pack("<H", crc)
            expected = _V2_HEADER_SIZE + count * _V2_TARGET_SIZE + 2
            if len(frame) != expected:
                log.debug("UDP V2 frame size mismatch: %d != %d", len(frame), expected)
                return False
            self._sock.sendto(frame, self._addr)
            if self._should_log_v2(seq, count):
                if count > 0:
                    first = targets[0]
                    log.info(
                        "UDP V2→GMK #%d count=%d first=%s cls=%d conf=%.2f z=%.3fm CRC=0x%04X",
                        seq, count, first.class_name, int(first.class_id), float(first.confidence), float(first.z), crc,
                    )
                else:
                    log.info("UDP V2→GMK #%d count=0 no fresh target | CRC=0x%04X", seq, crc)
            return True
        except (BlockingIOError, OSError, struct.error, TypeError, ValueError) as e:
            log.debug("UDP V2 send failed: %s", e)
            return False

    def close(self) -> None:
        try:
            if self._sock is not None:
                self._sock.close()
                self._sock = None
        except OSError:
            pass
        self._ready = False
        log.info("UDP sender closed")

    def __del__(self) -> None:
        self.close()

    @property
    def is_ready(self) -> bool:
        return self._ready

    @property
    def sequence(self) -> int:
        return self._seq

    @property
    def target(self) -> str:
        return f"{self._addr[0]}:{self._addr[1]}"
