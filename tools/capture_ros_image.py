"""Save one ROS color image to disk for live perception acceptance."""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Capture(Node):
    def __init__(self, topic: str, output: str) -> None:
        super().__init__("hwr_capture_image")
        self.output = output
        self.bridge = CvBridge()
        self.saved = False
        self.create_subscription(Image, topic, self._image,
                                 qos_profile_sensor_data)

    def _image(self, message: Image) -> None:
        if self.saved:
            return
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if not cv2.imwrite(self.output, frame):
            raise RuntimeError(f"failed to write {self.output}")
        self.saved = True
        print(f"saved {frame.shape[1]}x{frame.shape[0]} frame to {self.output}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic, args.output)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.saved:
        print(f"no image received from {args.topic}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
