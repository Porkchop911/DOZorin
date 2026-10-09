"""Capture maxima and source selection across the HWR velocity pipeline."""
from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from hwr_msgs.msg import WheelSpeeds
from rclpy.node import Node
from std_msgs.msg import String


class Trace(Node):
    def __init__(self) -> None:
        super().__init__("hwr_trace_velocity")
        self.nav = [0.0, 0.0, 0.0]
        self.applied = [0.0, 0.0, 0.0]
        self.wheels = [0.0] * 4
        self.nav_nonzero = self.applied_nonzero = self.wheel_nonzero = 0
        self.nav_samples = self.applied_samples = self.wheel_samples = 0
        self.sources: set[str] = set()
        self.capture_started = time.monotonic()
        self.applied_bursts = 0
        self._applied_was_nonzero = False
        self.first_applied_nonzero = None
        self.last_applied_nonzero = None
        self.create_subscription(Twist, "cmd_vel_nav", self._nav, 10)
        self.create_subscription(TwistStamped, "cmd_vel_applied", self._applied, 10)
        self.create_subscription(WheelSpeeds, "wheel_speeds", self._wheels, 10)
        self.create_subscription(String, "cmd_vel_source", self._source, 10)

    @staticmethod
    def _update(maxima, values) -> None:
        for index, value in enumerate(values):
            maxima[index] = max(maxima[index], abs(float(value)))

    def _nav(self, message: Twist) -> None:
        self.nav_samples += 1
        values = (message.linear.x, message.linear.y, message.angular.z)
        self._update(self.nav, values)
        self.nav_nonzero += int(any(abs(value) > 1e-6 for value in values))

    def _applied(self, message: TwistStamped) -> None:
        self.applied_samples += 1
        values = (message.twist.linear.x, message.twist.linear.y,
                  message.twist.angular.z)
        self._update(self.applied, values)
        nonzero = any(abs(value) > 1e-6 for value in values)
        self.applied_nonzero += int(nonzero)
        if nonzero and not self._applied_was_nonzero:
            self.applied_bursts += 1
        if nonzero:
            elapsed = time.monotonic() - self.capture_started
            if self.first_applied_nonzero is None:
                self.first_applied_nonzero = elapsed
            self.last_applied_nonzero = elapsed
        self._applied_was_nonzero = nonzero

    def _wheels(self, message: WheelSpeeds) -> None:
        self.wheel_samples += 1
        self._update(self.wheels, message.rps)
        self.wheel_nonzero += int(any(abs(value) > 1e-6 for value in message.rps))

    def _source(self, message: String) -> None:
        self.sources.add(message.data)

    def reset_capture(self) -> None:
        self.nav = [0.0, 0.0, 0.0]
        self.applied = [0.0, 0.0, 0.0]
        self.wheels = [0.0] * 4
        self.nav_nonzero = self.applied_nonzero = self.wheel_nonzero = 0
        self.nav_samples = self.applied_samples = self.wheel_samples = 0
        self.sources.clear()
        self.capture_started = time.monotonic()
        self.applied_bursts = 0
        self._applied_was_nonzero = False
        self.first_applied_nonzero = None
        self.last_applied_nonzero = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--warmup", type=float, default=3.0,
                        help="DDS discovery time excluded from capture")
    args = parser.parse_args()
    if args.duration <= 0.0 or args.warmup < 0.0:
        parser.error("duration must be positive and warmup non-negative")
    rclpy.init()
    node = Trace()
    try:
        warmup_deadline = time.monotonic() + args.warmup
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.reset_capture()
        print("velocity trace ready", flush=True)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"nav max vx/vy/wz: {node.nav}; samples: {node.nav_samples}; "
              f"nonzero samples: {node.nav_nonzero}")
        print(f"applied max vx/vy/wz: {node.applied}; "
              f"samples: {node.applied_samples}; "
              f"nonzero samples: {node.applied_nonzero}")
        print("applied motion bursts: "
              f"{node.applied_bursts}; first/last nonzero s: "
              f"{node.first_applied_nonzero}/{node.last_applied_nonzero}")
        print(f"wheel max rps: {node.wheels}; samples: {node.wheel_samples}; "
              f"nonzero samples: {node.wheel_nonzero}")
        print(f"mux sources observed: {sorted(node.sources)}")
        if node.applied_samples == 0 or node.wheel_samples == 0:
            print("INVALID TRACE: applied or wheel stream had no samples")
            return 2
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
