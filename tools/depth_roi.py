"""Report raw metric-depth statistics for a rectangular image region."""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hwr_sensors.depth_to_ranges_node import _depth_metres


class Capture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("hwr_depth_roi")
        self.depth = None
        self.create_subscription(Image, topic, self._on_depth,
                                 qos_profile_sensor_data)

    def _on_depth(self, message: Image) -> None:
        if self.depth is None:
            self.depth = _depth_metres(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("x1", type=int)
    parser.add_argument("y1", type=int)
    parser.add_argument("x2", type=int)
    parser.add_argument("y2", type=int)
    parser.add_argument("--topic", default="/camera/depth/image_raw")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.depth is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.depth is None:
            raise RuntimeError("no depth frame received")
        image = node.depth
        if not (0 <= args.x1 < args.x2 <= image.shape[1]
                and 0 <= args.y1 < args.y2 <= image.shape[0]):
            raise ValueError(f"ROI outside {image.shape[1]}x{image.shape[0]} frame")
        roi = image[args.y1:args.y2, args.x1:args.x2]
        valid = np.isfinite(roi) & (roi > 0.0)
        values = roi[valid]
        print(f"roi=({args.x1},{args.y1})-({args.x2},{args.y2}) "
              f"valid={len(values)}/{roi.size} ({100.0 * len(values) / roi.size:.1f}%)")
        if len(values):
            print("depth metres p05/p25/p50/p75/p95: "
                  + " ".join(f"{value:.3f}" for value in
                             np.percentile(values, (5, 25, 50, 75, 95))))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
