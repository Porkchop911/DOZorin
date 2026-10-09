"""Publish one short attended teleop command and always finish with zeros."""
from __future__ import annotations

import argparse
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("vx", type=float)
    parser.add_argument("vy", type=float)
    parser.add_argument("wz", type=float)
    parser.add_argument("duration", type=float)
    args = parser.parse_args()
    values = (args.vx, args.vy, args.wz, args.duration)
    if not all(math.isfinite(value) for value in values):
        parser.error("all values must be finite")
    if not (0.0 <= args.vx <= 0.15 and abs(args.vy) <= 0.15
            and abs(args.wz) <= 0.35 and 0.05 <= args.duration <= 8.0):
        parser.error("command is outside the attended no-reverse envelope")

    rclpy.init()
    node = Node("hwr_bounded_drive")
    publisher = node.create_publisher(Twist, "cmd_vel_teleop", 10)
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    command = Twist()
    command.linear.x, command.linear.y, command.angular.z = args.vx, args.vy, args.wz
    deadline = time.monotonic() + args.duration
    try:
        while not interrupted and time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(5):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.03)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("bounded teleop command complete; zero command published")
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
