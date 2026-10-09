"""Report nearest valid lidar returns in robot-relative clearance sectors."""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


SECTORS_DEG = {
    "front": (-30.0, 30.0),
    "left": (60.0, 120.0),
    "rear": (150.0, -150.0),
    "right": (-120.0, -60.0),
}


def sector_minimum(message: LaserScan, lo_deg: float, hi_deg: float) -> float:
    ranges = np.asarray(message.ranges, dtype=float)
    angles = message.angle_min + np.arange(ranges.size) * message.angle_increment
    angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    selected = ((angles >= lo) & (angles <= hi)) if lo <= hi else ((angles >= lo) | (angles <= hi))
    valid = selected & np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
    return float(np.min(ranges[valid])) if np.any(valid) else math.inf


class Clearance(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("hwr_scan_clearance")
        self.message: LaserScan | None = None
        self.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, message: LaserScan) -> None:
        self.message = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = Clearance(args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.message is None:
            raise RuntimeError(f"no scan received from {args.topic}")
        for name, (lo, hi) in SECTORS_DEG.items():
            distance = sector_minimum(node.message, lo, hi)
            print(f"{name}: {'clear' if math.isinf(distance) else f'{distance:.3f} m'}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
