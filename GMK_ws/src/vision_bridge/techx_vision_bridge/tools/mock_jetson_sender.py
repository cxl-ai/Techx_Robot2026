#!/usr/bin/env python3
"""Mock Jetson UDP sender for techx_vision_bridge.

It can run without Jetson/camera/model and is intended for GMK-side data-flow tests.
"""

import argparse
import math
import socket
import struct
import time
from typing import Iterable, List, Tuple

MAGIC_LEGACY = 0x55AA
MAGIC_V2 = 0x55AB
VERSION_V2 = 2
HEADER_V2 = "<H B B I d B"
TARGET_V2 = "<B B B f 2f 3f"
LEGACY = "<H I d B 3f"

COLOR_UNKNOWN = 0
COLOR_RED = 1
COLOR_BLUE = 2

# Class convention used by the GMK bridge:
#   0..4   KFS
#   100..149 head-area targets
#   200    QR target


def _build_crc16_table() -> List[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


CRC16_TABLE = _build_crc16_table()


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[idx]
    return crc


def pack_v2(seq: int, targets: Iterable[Tuple[int, int, int, float, float, float, float, float, float]]) -> bytes:
    items = list(targets)[:16]
    header = struct.pack(HEADER_V2, MAGIC_V2, VERSION_V2, 0, seq & 0xFFFFFFFF, time.time(), len(items))
    body = b"".join(struct.pack(TARGET_V2, *item) for item in items)
    payload = header + body
    return payload + struct.pack("<H", crc16_ccitt(payload))


def pack_legacy(seq: int, track_id: int, x: float, y: float, z: float) -> bytes:
    payload = struct.pack(LEGACY, MAGIC_LEGACY, seq & 0xFFFFFFFF, time.time(), track_id & 0xFF, x, y, z)
    return payload + struct.pack("<H", crc16_ccitt(payload))


def make_targets(mode: str, tick: int):
    phase = tick * 0.1
    center_u = 320.0 + 50.0 * math.sin(phase)
    center_v = 240.0 + 20.0 * math.cos(phase)

    if mode == "empty":
        return []
    if mode == "kfs":
        return [
            (7, 1, COLOR_RED, 0.91, center_u, center_v, 0.12, -0.03, 1.20),
            (8, 2, COLOR_BLUE, 0.84, 250.0, 230.0, -0.18, 0.02, 1.65),
        ]
    if mode == "head":
        return [(20, 100, COLOR_UNKNOWN, 0.88, center_u, center_v, 0.05, 0.01, 1.10)]
    if mode == "qr":
        return [(200, 200, COLOR_UNKNOWN, 1.00, center_u, center_v, 0.02, 0.00, 0.60)]
    if mode == "invalid-depth":
        return [(200, 200, COLOR_UNKNOWN, 1.00, center_u, center_v, 0.0, 0.0, 0.0)]
    if mode == "mixed":
        return [
            (20, 100, COLOR_UNKNOWN, 0.88, 180.0, 220.0, -0.30, 0.00, 1.40),
            (7, 1, COLOR_RED, 0.92, 320.0, 240.0, 0.10, -0.02, 1.15),
            (8, 2, COLOR_BLUE, 0.84, 260.0, 245.0, -0.16, 0.01, 1.55),
            (200, 200, COLOR_UNKNOWN, 1.00, center_u, center_v, 0.02, 0.00, 0.62),
        ]
    raise ValueError(f"unknown mode: {mode}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock Jetson UDP sender for GMK vision bridge")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=12345)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--mode", choices=["empty", "kfs", "head", "qr", "invalid-depth", "mixed", "legacy"], default="mixed")
    parser.add_argument("--legacy-extra", action="store_true", help="also send a legacy packet after V2 when a valid XYZ target exists")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / max(args.rate, 0.1)
    seq = 0
    tick = 0
    print(f"mock sender -> {args.ip}:{args.port}, mode={args.mode}, rate={args.rate}Hz")

    try:
        while True:
            if args.mode == "legacy":
                pkt = pack_legacy(seq, 7, 0.10, -0.02, 1.20)
                sock.sendto(pkt, (args.ip, args.port))
                sent_desc = "legacy"
                seq += 1
            else:
                targets = make_targets(args.mode, tick)
                pkt = pack_v2(seq, targets)
                sock.sendto(pkt, (args.ip, args.port))
                sent_desc = f"v2 count={len(targets)}"
                seq += 1
                if args.legacy_extra:
                    valid = [t for t in targets if t[8] > 0.0]
                    if valid:
                        t = valid[0]
                        sock.sendto(pack_legacy(seq, t[0], t[6], t[7], t[8]), (args.ip, args.port))
                        sent_desc += " + legacy"
                        seq += 1

            if tick % max(1, int(args.rate)) == 0:
                print(f"sent seq={seq - 1} {sent_desc}")
            tick += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
