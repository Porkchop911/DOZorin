#!/usr/bin/env python3
"""Print live frontier ranking and bounded local approach waypoints.

This tool is read-only: it subscribes to map, costmap, and pose, then runs the
same ranking and staged-waypoint functions used by ``hwr_explore``.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hwr_core.exploration import find_frontiers
from hwr_core.navigation import staged_path_waypoint


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * orientation.w * orientation.z,
        1.0 - 2.0 * orientation.z * orientation.z,
    )


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__("hwr_frontier_diagnostics")
        self.map_message = None
        self.costmap_message = None
        self.pose = None
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "map", self._map, map_qos)
        self.create_subscription(OccupancyGrid, "costmap", self._costmap, 1)
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._pose, 10)

    def _map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _costmap(self, message: OccupancyGrid) -> None:
        self.costmap_message = message

    def _pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--limit", type=int, default=8)
    parser.add_argument("--min-cells", type=int, default=8)
    parser.add_argument("--setback", type=float, default=0.25)
    parser.add_argument("--max-leg", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if args.limit <= 0 or args.timeout <= 0.0:
        parser.error("limit and timeout must be positive")

    rclpy.init()
    node = Snapshot()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (node.map_message is not None
                    and node.costmap_message is not None
                    and node.pose is not None):
                break
        if (node.map_message is None
                or node.costmap_message is None
                or node.pose is None):
            raise RuntimeError("map, costmap, or pose was not received")
        map_message = node.map_message
        costmap_message = node.costmap_message
        occupancy = np.asarray(map_message.data, dtype=np.int16).reshape(
            map_message.info.height, map_message.info.width).astype(np.int8)
        cost_values = np.asarray(costmap_message.data, dtype=float).reshape(
            costmap_message.info.height, costmap_message.info.width)
        costmap = np.clip(np.rint(cost_values * 2.55), 0, 255).astype(np.uint8)
        map_origin = (
            map_message.info.origin.position.x,
            map_message.info.origin.position.y,
        )
        costmap_origin = (
            costmap_message.info.origin.position.x,
            costmap_message.info.origin.position.y,
        )
        if occupancy.shape != costmap.shape:
            raise RuntimeError("map and costmap shapes differ")
        frontiers = find_frontiers(
            occupancy,
            map_message.info.resolution,
            map_origin,
            node.pose[:2],
            min_cells=args.min_cells,
            costmap=costmap,
            robot_yaw=node.pose[2],
            setback_m=args.setback,
        )
        rows = []
        for frontier in frontiers[:args.limit]:
            try:
                waypoint = staged_path_waypoint(
                    costmap,
                    costmap_message.info.resolution,
                    costmap_origin,
                    node.pose[:2],
                    frontier.goal,
                    max_leg=args.max_leg,
                )
                dx = waypoint[0] - node.pose[0]
                dy = waypoint[1] - node.pose[1]
                relative = math.atan2(dy, dx) - node.pose[2]
                relative = math.atan2(math.sin(relative), math.cos(relative))
                local = {
                    "waypoint": list(waypoint),
                    "distance_m": math.hypot(dx, dy),
                    "relative_bearing_deg": math.degrees(relative),
                }
            except ValueError as exc:
                local = {"error": str(exc)}
            rows.append({
                "goal": list(frontier.goal),
                "score": frontier.score,
                "path_length_m": frontier.path_length_m,
                "clearance_m": frontier.clearance_m,
                "information_gain": frontier.information_gain,
                "local": local,
            })
        print(json.dumps({"pose": list(node.pose), "frontiers": rows}, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
