#!/usr/bin/env python3
"""Capture one usable ROS lidar scan as base-frame NumPy points."""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from hwr_core.matching import scan_to_points
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("hwr_capture_scan")
        self.points = None
        self.create_subscription(LaserScan, "scan", self._scan,
                                 qos_profile_sensor_data)

    def _scan(self, message: LaserScan) -> None:
        ranges = np.asarray(message.ranges, dtype=float)
        angles = message.angle_min + np.arange(len(ranges)) * message.angle_increment
        points = scan_to_points(angles, ranges, message.range_max,
                                sensor_yaw=math.pi,
                                sensor_xy=(0.048156, 0.0))
        if len(points) >= 10:
            self.points = points


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.points is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.points is None:
            raise RuntimeError("no usable scan received")
        np.save(args.output, node.points)
        print(f"saved {len(node.points)} base-frame points to {args.output}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
