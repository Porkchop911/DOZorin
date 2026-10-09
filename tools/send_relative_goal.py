"""Send one bounded relative or explicit-map HWR goal and report its result."""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hwr_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class RelativeGoal(Node):
    def __init__(self) -> None:
        super().__init__("hwr_relative_goal")
        self.pose: tuple[float, float, float] | None = None
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._pose, 10)

    def _pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))


def _wait(node: Node, future, deadline: float) -> bool:
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.0)
    parser.add_argument("--left", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--map-x", type=float)
    parser.add_argument("--map-y", type=float)
    parser.add_argument("--map-yaw-deg", type=float)
    parser.add_argument("--max-translation", type=float, default=0.75)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    map_mode = args.map_x is not None or args.map_y is not None
    if (args.map_x is None) != (args.map_y is None):
        parser.error("--map-x and --map-y must be provided together")
    if map_mode and (args.forward != 0.0 or args.left != 0.0
                     or args.yaw_deg != 0.0):
        parser.error("map goals cannot be combined with relative offsets")
    values = [args.forward, args.left, args.yaw_deg]
    values.extend(v for v in (args.map_x, args.map_y, args.map_yaw_deg)
                  if v is not None)
    if not all(math.isfinite(v) for v in values):
        parser.error("goal offsets must be finite")

    rclpy.init()
    node = RelativeGoal()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            raise RuntimeError("localized pose was not received")
        discovery_timeout = min(10.0, max(0.1, deadline - time.monotonic()))
        if not node.client.wait_for_server(timeout_sec=discovery_timeout):
            raise RuntimeError("navigation action server unavailable")

        x, y, yaw = node.pose
        if map_mode:
            goal_x, goal_y = args.map_x, args.map_y
            goal_yaw = (yaw if args.map_yaw_deg is None
                        else math.radians(args.map_yaw_deg))
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            goal_x = x + c * args.forward - s * args.left
            goal_y = y + s * args.forward + c * args.left
            goal_yaw = yaw + math.radians(args.yaw_deg)
        translation = math.hypot(goal_x - x, goal_y - y)
        if translation > args.max_translation:
            raise ValueError(f"translation {translation:.3f} m exceeds bounded "
                             f"limit {args.max_translation:.3f} m")
        request = NavigateToPose.Goal()
        request.target.header.frame_id = "map"
        request.target.header.stamp = node.get_clock().now().to_msg()
        request.target.pose.position.x = goal_x
        request.target.pose.position.y = goal_y
        request.target.pose.orientation.z = math.sin(goal_yaw / 2.0)
        request.target.pose.orientation.w = math.cos(goal_yaw / 2.0)
        print(f"start=({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg) "
              f"target=({goal_x:.3f}, {goal_y:.3f}, "
              f"{math.degrees(goal_yaw):.1f} deg)")

        sent = node.client.send_goal_async(
            request,
            feedback_callback=lambda message: print(
                f"feedback state={message.feedback.state} "
                f"remaining={message.feedback.distance_remaining:.3f} m"))
        if not _wait(node, sent, deadline):
            raise RuntimeError("timed out sending navigation goal")
        handle = sent.result()
        if not handle.accepted:
            print("goal rejected")
            return 2
        result_future = handle.get_result_async()
        if not _wait(node, result_future, deadline):
            canceled = handle.cancel_goal_async()
            _wait(node, canceled, time.monotonic() + 2.0)
            raise RuntimeError("navigation goal timed out and was canceled")
        wrapped = result_future.result()
        result = wrapped.result
        print(f"status={wrapped.status} success={result.success} "
              f"message={result.message}")
        return 0 if result.success else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
