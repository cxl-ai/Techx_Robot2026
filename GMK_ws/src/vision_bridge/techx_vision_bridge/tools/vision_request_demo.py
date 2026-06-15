#!/usr/bin/env python3
"""Publish a VisionRequest and print VisionSelection results.

Examples:
  ros2 run techx_vision_bridge vision_request_demo.py --name qr
  ros2 run techx_vision_bridge vision_request_demo.py --name head_fist
  ros2 run techx_vision_bridge vision_request_demo.py --class-id 2 --type kfs --require-xyz
"""

import argparse
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from techx_vision_bridge.msg import VisionRequest, VisionSelection


PRESETS: Dict[str, Tuple[int, int, int, bool]] = {
    # name: (target_type, zone_id, class_id, require_control_xyz)
    "head_fist": (VisionRequest.TYPE_WEAPON_HEAD, VisionRequest.ZONE_WEAPON_HEAD, 100, True),
    "head_palm": (VisionRequest.TYPE_WEAPON_HEAD, VisionRequest.ZONE_WEAPON_HEAD, 101, True),
    "head_spear": (VisionRequest.TYPE_WEAPON_HEAD, VisionRequest.ZONE_WEAPON_HEAD, 102, True),
    "kfs_red_r2_true": (VisionRequest.TYPE_KFS, VisionRequest.ZONE_KFS, 2, True),
    "kfs_blue_r2_true": (VisionRequest.TYPE_KFS, VisionRequest.ZONE_KFS, 5, True),
    "qr": (VisionRequest.TYPE_QR, VisionRequest.ZONE_QR, 200, False),
}

TYPE_MAP = {
    "any": VisionRequest.TYPE_ANY,
    "head": VisionRequest.TYPE_WEAPON_HEAD,
    "kfs": VisionRequest.TYPE_KFS,
    "qr": VisionRequest.TYPE_QR,
    "custom": VisionRequest.TYPE_CUSTOM,
}

ZONE_MAP = {
    "any": VisionRequest.ZONE_ANY,
    "head": VisionRequest.ZONE_WEAPON_HEAD,
    "kfs": VisionRequest.ZONE_KFS,
    "qr": VisionRequest.ZONE_QR,
    "custom": VisionRequest.ZONE_CUSTOM,
}

STATUS_NAMES = {
    VisionSelection.STATUS_OK: "OK",
    VisionSelection.STATUS_NO_REQUEST: "NO_REQUEST",
    VisionSelection.STATUS_NO_FRAME: "NO_FRAME",
    VisionSelection.STATUS_NO_MATCH: "NO_MATCH",
    VisionSelection.STATUS_FRAME_STALE: "FRAME_STALE",
    VisionSelection.STATUS_REQUEST_STALE: "REQUEST_STALE",
}

FRAME_NAMES = {
    0: "unknown",
    1: "camera_link",
    2: "robot_base",
    3: "arm1_base",
    4: "arm2_base",
}


class VisionRequestDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vision_request_demo")
        self.args = args
        self.request_seq = args.request_seq
        self.publisher = self.create_publisher(VisionRequest, args.request_topic, 10)
        self.subscription = self.create_subscription(
            VisionSelection, args.selected_topic, self.on_selection, 10
        )
        self.timer = self.create_timer(args.repeat_sec, self.publish_request)
        self.publish_request()

    def publish_request(self) -> None:
        msg = VisionRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.request_seq = self.request_seq
        msg.target_type = self.args.target_type
        msg.zone_id = self.args.zone_id
        msg.use_class_id = self.args.class_id is not None
        msg.class_id = int(self.args.class_id or 0)
        msg.use_color = self.args.color is not None
        msg.color = int(self.args.color or 0)
        msg.require_control_xyz = bool(self.args.require_control_xyz)
        msg.min_confidence = float(self.args.min_confidence)
        msg.max_frame_age_sec = float(self.args.max_frame_age_sec)
        self.publisher.publish(msg)
        self.get_logger().info(
            "request seq=%d type=%d zone=%d class_id=%s require_xyz=%s" % (
                msg.request_seq,
                msg.target_type,
                msg.zone_id,
                str(self.args.class_id),
                msg.require_control_xyz,
            )
        )
        self.request_seq += 1

    def on_selection(self, msg: VisionSelection) -> None:
        status_name = STATUS_NAMES.get(msg.status, str(msg.status))
        if not msg.has_match:
            self.get_logger().info(
                "selected status=%s has_match=false frame_age=%.3fs" % (status_name, msg.frame_age_sec)
            )
            return

        target = msg.target
        frame_name = FRAME_NAMES.get(target.control_frame, str(target.control_frame))
        self.get_logger().info(
            "selected status=%s class_id=%d conf=%.3f frame=%s ctrl=(%.3f, %.3f, %.3f) "
            "robot=(%.3f, %.3f, %.3f) arm1=(%.3f, %.3f, %.3f) arm2=(%.3f, %.3f, %.3f)"
            % (
                status_name,
                target.class_id,
                target.confidence,
                frame_name,
                target.control_x,
                target.control_y,
                target.control_z,
                target.robot_x,
                target.robot_y,
                target.robot_z,
                target.arm1_x,
                target.arm1_y,
                target.arm1_z,
                target.arm2_x,
                target.arm2_y,
                target.arm2_z,
            )
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", choices=sorted(PRESETS.keys()), help="common target preset")
    parser.add_argument("--class-id", type=int, help="exact global class_id to request")
    parser.add_argument("--type", choices=sorted(TYPE_MAP.keys()), default="any", help="target type filter")
    parser.add_argument("--zone", choices=sorted(ZONE_MAP.keys()), default="any", help="zone filter")
    parser.add_argument("--color", type=int, choices=[0, 1, 2], help="optional color filter: 0 unknown, 1 red, 2 blue")
    parser.add_argument("--require-xyz", dest="require_control_xyz", action="store_true")
    parser.add_argument("--allow-no-xyz", dest="require_control_xyz", action="store_false")
    parser.set_defaults(require_control_xyz=False)
    parser.add_argument("--min-confidence", type=float, default=0.3)
    parser.add_argument("--max-frame-age-sec", type=float, default=0.2)
    parser.add_argument("--request-seq", type=int, default=1)
    parser.add_argument("--repeat-sec", type=float, default=1.0, help="repeat request period")
    parser.add_argument("--request-topic", default="/techx/vision/request")
    parser.add_argument("--selected-topic", default="/techx/vision/selected")
    args = parser.parse_args()

    if args.name:
        target_type, zone_id, class_id, require_xyz = PRESETS[args.name]
        args.target_type = target_type
        args.zone_id = zone_id
        args.class_id = class_id
        args.require_control_xyz = require_xyz
    else:
        args.target_type = TYPE_MAP[args.type]
        args.zone_id = ZONE_MAP[args.zone]

    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = VisionRequestDemo(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
