"""Publish a bounded synthetic forward depth obstacle for safety testing."""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/depth_ranges")
    parser.add_argument("--frame", default="base_footprint")
    parser.add_argument("--range-m", type=float, default=0.25)
    parser.add_argument("--beam-count", type=int, default=64)
    parser.add_argument("--beam-half-width", type=int, default=6)
    parser.add_argument("--fov-degrees", type=float, default=74.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--delay", type=float, default=4.0)
    parser.add_argument("--trigger-on-applied", action="store_true")
    parser.add_argument("--trigger-timeout", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()
    if not 0.15 <= args.range_m <= 1.0:
        parser.error("--range-m must be 0.15..1.0")
    if (args.beam_count < 3 or args.beam_half_width < 0
            or args.beam_half_width >= args.beam_count // 2):
        parser.error("invalid beam geometry")
    if (min(args.warmup, args.delay, args.duration) < 0.0
            or args.rate <= 0.0 or args.trigger_timeout <= 0.0):
        parser.error("timings must be non-negative and rate positive")

    rclpy.init()
    node = Node("hwr_synthetic_depth_obstacle")
    publisher = node.create_publisher(LaserScan, args.topic, 10)
    applied_seen = [False]

    def on_applied(message: TwistStamped) -> None:
        values = (message.twist.linear.x, message.twist.linear.y,
                  message.twist.angular.z)
        if any(abs(float(value)) > 1e-6 for value in values):
            applied_seen[0] = True

    subscription = node.create_subscription(
        TwistStamped, "/cmd_vel_applied", on_applied, 10)
    try:
        warmup_deadline = time.monotonic() + args.warmup
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        trigger = " after applied motion" if args.trigger_on_applied else ""
        print(f"synthetic obstacle publisher ready{trigger}; "
              f"inject after {args.delay:.2f}s", flush=True)
        if args.trigger_on_applied:
            trigger_deadline = time.monotonic() + args.trigger_timeout
            while (rclpy.ok() and not applied_seen[0]
                   and time.monotonic() < trigger_deadline):
                rclpy.spin_once(node, timeout_sec=0.02)
            if not applied_seen[0]:
                raise RuntimeError("timed out waiting for applied motion")
            print("applied motion detected", flush=True)
        delay_deadline = time.monotonic() + args.delay
        while rclpy.ok() and time.monotonic() < delay_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        half_fov = math.radians(args.fov_degrees) / 2.0
        increment = 2.0 * half_fov / max(args.beam_count - 1, 1)
        ranges = [math.inf] * args.beam_count
        center = args.beam_count // 2
        for index in range(
                center - args.beam_half_width,
                center + args.beam_half_width + 1):
            ranges[index] = args.range_m
        period = 1.0 / args.rate
        deadline = time.monotonic() + args.duration
        published = 0
        while rclpy.ok() and time.monotonic() < deadline:
            message = LaserScan()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = args.frame
            message.angle_min = -half_fov
            message.angle_max = half_fov
            message.angle_increment = increment
            message.scan_time = period
            message.range_min = 0.15
            message.range_max = 3.0
            message.ranges = ranges
            publisher.publish(message)
            published += 1
            rclpy.spin_once(node, timeout_sec=period)
        print(f"published {published} synthetic obstacle scans", flush=True)
        return 0
    finally:
        del subscription
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
