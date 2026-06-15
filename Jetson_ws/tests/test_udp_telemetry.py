import os
import socket
import struct
import unittest
from unittest import mock

from communication.udp import (
    UdpSender,
    _LEGACY_FRAME_FMT,
    _MAGIC_LEGACY,
    _MAGIC_V2,
    _V2_HEADER_FMT,
    _V2_TARGET_FMT,
)
from interface.types import Target3D
from utils.crc16 import crc16_ccitt


def _two_targets():
    return [
        Target3D(
            track_id=7,
            class_id=1,
            class_name="r1_kfs_red",
            confidence=0.91,
            color="red",
            timestamp=123.456,
            pixel_uv=(320.0, 240.0),
            camera_xyz=(0.10, -0.02, 1.25),
            depth_m=1.25,
        ),
        Target3D(
            track_id=8,
            class_id=2,
            class_name="r1_kfs_blue",
            confidence=0.82,
            color="blue",
            timestamp=123.456,
            pixel_uv=(120.0, 180.0),
            camera_xyz=(-0.20, 0.04, 1.80),
            depth_m=1.80,
        ),
    ]


class UdpTelemetryTest(unittest.TestCase):
    def test_send_many_default_emits_v2_only(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(0.2)
        host, port = rx.getsockname()

        with mock.patch.dict(os.environ, {}, clear=True):
            sender = UdpSender(host, port, local_ip=host)
            self.assertEqual(sender.send_many(_two_targets()), 1)
            packet = rx.recvfrom(2048)[0]
            sender.close()
        rx.close()

        self.assertEqual(struct.unpack_from("<H", packet, 0)[0], _MAGIC_V2)
        self.assertEqual(crc16_ccitt(packet[:-2]), struct.unpack_from("<H", packet, len(packet) - 2)[0])

        header_size = struct.calcsize(_V2_HEADER_FMT)
        target_size = struct.calcsize(_V2_TARGET_FMT)
        _, version, flags, seq, ts, count = struct.unpack_from(_V2_HEADER_FMT, packet, 0)
        self.assertEqual(version, 2)
        self.assertEqual(flags, 0)
        self.assertEqual(seq, 1)
        self.assertAlmostEqual(ts, 123.456, places=3)
        self.assertEqual(count, 2)
        self.assertEqual(len(packet), header_size + count * target_size + 2)

        t0 = struct.unpack_from(_V2_TARGET_FMT, packet, header_size)
        self.assertEqual(t0[0], 7)      # track_id
        self.assertEqual(t0[1], 1)      # class_id
        self.assertEqual(t0[2], 1)      # red
        self.assertAlmostEqual(t0[3], 0.91, places=2)
        self.assertAlmostEqual(t0[4], 320.0, places=1)
        self.assertAlmostEqual(t0[5], 240.0, places=1)
        self.assertAlmostEqual(t0[8], 1.25, places=2)

    def test_legacy_output_requires_env_flag(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1.0)
        host, port = rx.getsockname()

        with mock.patch.dict(os.environ, {"TECHX_SEND_LEGACY_UDP": "1"}, clear=True):
            sender = UdpSender(host, port, local_ip=host)
            self.assertEqual(sender.send_many(_two_targets()), 2)
            packets = [rx.recvfrom(2048)[0], rx.recvfrom(2048)[0]]
            sender.close()
        rx.close()

        by_magic = {struct.unpack_from("<H", p, 0)[0]: p for p in packets}
        self.assertIn(_MAGIC_V2, by_magic)
        self.assertIn(_MAGIC_LEGACY, by_magic)
        legacy = struct.unpack(_LEGACY_FRAME_FMT, by_magic[_MAGIC_LEGACY])
        self.assertEqual(legacy[0], _MAGIC_LEGACY)
        self.assertEqual(legacy[3], 7)
        self.assertAlmostEqual(legacy[6], 1.25, places=2)
        self.assertEqual(crc16_ccitt(by_magic[_MAGIC_LEGACY][:-2]), legacy[-1])

    def test_send_many_empty_emits_v2_count_zero_only(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1.0)
        host, port = rx.getsockname()

        sender = UdpSender(host, port, local_ip=host)
        self.assertEqual(sender.send_many([], timestamp=456.789), 1)
        packet = rx.recvfrom(2048)[0]
        sender.close()
        rx.close()

        header_size = struct.calcsize(_V2_HEADER_FMT)
        self.assertEqual(len(packet), header_size + 2)
        self.assertEqual(crc16_ccitt(packet[:-2]), struct.unpack_from("<H", packet, len(packet) - 2)[0])
        magic, version, flags, seq, ts, count = struct.unpack_from(_V2_HEADER_FMT, packet, 0)
        self.assertEqual(magic, _MAGIC_V2)
        self.assertEqual(version, 2)
        self.assertEqual(flags, 0)
        self.assertEqual(seq, 1)
        self.assertAlmostEqual(ts, 456.789, places=3)
        self.assertEqual(count, 0)

    def test_send_many_invalid_depth_keeps_v2_detection_without_legacy(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1.0)
        host, port = rx.getsockname()

        sender = UdpSender(host, port, local_ip=host)
        target = Target3D(
            track_id=9,
            class_id=3,
            class_name="r2_kfs_red",
            confidence=0.77,
            color="red",
            timestamp=789.123,
            pixel_uv=(250.0, 160.0),
            camera_xyz=(0.0, 0.0, 0.0),
            depth_m=0.0,
        )
        self.assertEqual(sender.send_many([target]), 1)
        packet = rx.recvfrom(2048)[0]
        sender.close()
        rx.close()

        header_size = struct.calcsize(_V2_HEADER_FMT)
        target_size = struct.calcsize(_V2_TARGET_FMT)
        self.assertEqual(len(packet), header_size + target_size + 2)
        magic, version, flags, seq, ts, count = struct.unpack_from(_V2_HEADER_FMT, packet, 0)
        self.assertEqual(magic, _MAGIC_V2)
        self.assertEqual(version, 2)
        self.assertEqual(count, 1)
        entry = struct.unpack_from(_V2_TARGET_FMT, packet, header_size)
        self.assertEqual(entry[0], 9)
        self.assertEqual(entry[1], 3)
        self.assertAlmostEqual(entry[3], 0.77, places=2)
        self.assertAlmostEqual(entry[8], 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
