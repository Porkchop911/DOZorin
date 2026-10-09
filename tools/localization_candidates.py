"""List coarse map-localization hypotheses, optionally ranked by a landmark."""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from hwr_core.localization import coarse_global_search, distance_field
from hwr_core.mapping import load_map
from hwr_core.matching import scan_to_points
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("hwr_localization_candidates")
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
    parser.add_argument("map_yaml")
    parser.add_argument("--hypotheses", type=int, default=12)
    parser.add_argument("--landmark-x", type=float)
    parser.add_argument("--landmark-y", type=float)
    parser.add_argument("--bearing-deg", type=float, default=0.0,
                        help="observed base-relative landmark bearing")
    parser.add_argument("--range", type=float,
                        help="optional observed planar landmark range")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if (args.landmark_x is None) != (args.landmark_y is None):
        parser.error("--landmark-x and --landmark-y are required together")
    occupancy, resolution, origin = load_map(args.map_yaml)
    field = distance_field(occupancy, resolution)
    rclpy.init()
    node = Capture()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.points is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.points is None:
            raise RuntimeError("no usable scan received")
        poses, scores = coarse_global_search(
            occupancy, field, resolution, origin, node.points,
            hypotheses=args.hypotheses)
        output = []
        for pose, score in zip(poses, scores):
            item = {"x": float(pose[0]), "y": float(pose[1]),
                    "yaw": float(pose[2]),
                    "yaw_deg": math.degrees(float(pose[2])),
                    "scan_score": float(score)}
            if args.landmark_x is not None:
                dx, dy = args.landmark_x - pose[0], args.landmark_y - pose[1]
                predicted_bearing = _wrap(math.atan2(dy, dx) - pose[2])
                bearing_error = abs(_wrap(
                    predicted_bearing - math.radians(args.bearing_deg)))
                item.update({
                    "landmark_range_m": float(math.hypot(dx, dy)),
                    "landmark_bearing_deg": math.degrees(predicted_bearing),
                    "bearing_error_deg": math.degrees(bearing_error),
                })
                range_error = (0.0 if args.range is None else
                               abs(math.hypot(dx, dy) - args.range))
                item["semantic_rank_score"] = (
                    float(score) - 0.4 * bearing_error - 0.3 * range_error)
            output.append(item)
        if args.landmark_x is not None:
            output.sort(key=lambda item: item["semantic_rank_score"], reverse=True)
        print(json.dumps({"candidates": output}, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
