"""Compare the live navigation costmap with its saved static-map baseline."""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hwr_core.mapping import load_map
from hwr_core.navigation import Costmap, astar, simplify_path
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def ray_clearance(grid: np.ndarray, origin: tuple[float, float],
                  resolution: float, pose: tuple[float, float, float],
                  relative_angle: float, max_distance: float = 1.5) -> float:
    """Return the first lethal distance along a robot-relative costmap ray."""
    distances = np.arange(0.0, max_distance + resolution / 2.0, resolution)
    heading = pose[2] + relative_angle
    wx = pose[0] + distances * math.cos(heading)
    wy = pose[1] + distances * math.sin(heading)
    ix = np.floor((wx - origin[0]) / resolution).astype(int)
    iy = np.floor((wy - origin[1]) / resolution).astype(int)
    inside = ((ix >= 0) & (iy >= 0) & (ix < grid.shape[1])
              & (iy < grid.shape[0]))
    clipped_x = ix.clip(0, grid.shape[1] - 1)
    clipped_y = iy.clip(0, grid.shape[0] - 1)
    blocked = (~inside) | (grid[clipped_y, clipped_x] >= 200.0)
    first = np.flatnonzero(blocked)
    return float(distances[first[0]]) if len(first) else math.inf


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__("hwr_costmap_diagnostics")
        self.grid: OccupancyGrid | None = None
        self.pose: tuple[float, float, float] | None = None
        self.depth: LaserScan | None = None
        self.create_subscription(OccupancyGrid, "costmap", self._on_grid, 1)
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._on_pose, 10)
        self.create_subscription(LaserScan, "depth_ranges", self._on_depth,
                                 qos_profile_sensor_data)

    def _on_grid(self, message: OccupancyGrid) -> None:
        self.grid = message

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        p = message.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_depth(self, message: LaserScan) -> None:
        self.depth = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml")
    parser.add_argument("--inflation-radius", type=float, default=0.30)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=1.0,
                        help="keep latest messages for this long after all arrive")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-search-radius", type=float, default=0.0,
                        help="find the nearest reachable free cell around a blocked goal")
    parser.add_argument("--require-start-cost-at-least", type=float,
                        help="exit nonzero unless the current pose cell reaches this cost")
    args = parser.parse_args()
    occupancy, resolution, origin = load_map(args.map_yaml)
    baseline = Costmap(occupancy, resolution, origin,
                       args.inflation_radius).grid(0.0).astype(float)

    rclpy.init()
    node = Snapshot()
    deadline = time.monotonic() + args.timeout
    try:
        complete_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            complete = node.grid is not None and node.pose is not None and node.depth is not None
            if complete and complete_since is None:
                complete_since = time.monotonic()
            if complete_since is not None and time.monotonic() - complete_since >= args.settle:
                break
        if node.grid is None or node.pose is None or node.depth is None:
            raise RuntimeError("costmap, pose, or depth fan was not received")
        message = node.grid
        live = np.asarray(message.data, dtype=float).reshape(message.info.height,
                                                             message.info.width) * 2.55
        if live.shape != baseline.shape:
            raise RuntimeError(f"costmap shape {live.shape} != baseline {baseline.shape}")
        changed = live >= baseline + 20.0
        ys, xs = np.where(changed)
        print(f"live-raised cells: {len(xs)}")
        if len(xs):
            wx = origin[0] + (xs + 0.5) * resolution
            wy = origin[1] + (ys + 0.5) * resolution
            px, py, yaw = node.pose
            dx, dy = wx - px, wy - py
            distance = np.hypot(dx, dy)
            nearest = int(np.argmin(distance))
            bearing = math.atan2(dy[nearest], dx[nearest]) - yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            print(f"nearest live mark: {distance[nearest]:.3f} m at "
                  f"{math.degrees(bearing):.1f} deg robot-relative")
        px, py, yaw = node.pose
        start_x = math.floor((px - origin[0]) / resolution)
        start_y = math.floor((py - origin[1]) / resolution)
        start_inside = (0 <= start_x < live.shape[1]
                        and 0 <= start_y < live.shape[0])
        start_cost = (float(live[start_y, start_x])
                      if start_inside else math.inf)
        print("current pose cell cost: "
              + (f"{start_cost:.0f}" if start_inside else "outside"))
        if (args.require_start_cost_at_least is not None
                and start_cost < args.require_start_cost_at_least):
            print("required current-pose cost was not observed")
            return 3
        directions = {
            "forward": 0.0,
            "front-left": 45.0,
            "left": 90.0,
            "front-right": -45.0,
            "right": -90.0,
        }
        for name, degrees in directions.items():
            clearance = ray_clearance(live, origin, resolution, node.pose,
                                      math.radians(degrees))
            print(f"{name} centerline first lethal: "
                  + (f"{clearance:.2f} m" if math.isfinite(clearance)
                     else ">1.50 m"))
        if (args.goal_x is None) != (args.goal_y is None):
            raise ValueError("--goal-x and --goal-y must be provided together")
        if args.goal_x is not None:
            start = (math.floor((px - origin[0]) / resolution),
                     math.floor((py - origin[1]) / resolution))
            target = (math.floor((args.goal_x - origin[0]) / resolution),
                      math.floor((args.goal_y - origin[1]) / resolution))
            inside = (0 <= target[0] < live.shape[1]
                      and 0 <= target[1] < live.shape[0])
            target_cost = live[target[1], target[0]] if inside else math.inf
            cells = astar(live, start, target) if inside else []
            path = simplify_path(live, cells) if cells else []
            print(f"goal ({args.goal_x:.3f}, {args.goal_y:.3f}) cell={target} "
                  f"cost={'outside' if not inside else f'{target_cost:.0f}'}")
            if not path:
                print("goal path: unavailable")
                radius_cells = max(0, math.floor(args.goal_search_radius / resolution))
                if radius_cells:
                    # Match the gateway geofence: alternatives must be genuinely
                    # free, not merely below the hard-footprint threshold.
                    ys, xs = np.where(live < 65.0)
                    delta = np.hypot(xs - target[0], ys - target[1])
                    order = np.argsort(delta)
                    alternative = None
                    for index in order:
                        if delta[index] > radius_cells:
                            break
                        candidate = (int(xs[index]), int(ys[index]))
                        candidate_cells = astar(live, start, candidate)
                        if candidate_cells:
                            alternative = candidate, candidate_cells
                            break
                    if alternative is None:
                        print(f"reachable alternative: none within "
                              f"{args.goal_search_radius:.2f} m")
                    else:
                        candidate, candidate_cells = alternative
                        wx = origin[0] + (candidate[0] + 0.5) * resolution
                        wy = origin[1] + (candidate[1] + 0.5) * resolution
                        offset = math.hypot(wx - args.goal_x, wy - args.goal_y)
                        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                     for a, b in zip(candidate_cells,
                                                     candidate_cells[1:])) * resolution
                        print(f"reachable alternative: ({wx:.3f}, {wy:.3f}), "
                              f"goal offset {offset:.2f} m, path {length:.2f} m, "
                              f"cost {live[candidate[1], candidate[0]]:.0f}")
            else:
                length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                             for a, b in zip(cells, cells[1:])) * resolution
                print(f"goal path: {len(cells)} cells, {len(path)} simplified, "
                      f"{length:.2f} m")
                if len(path) > 1:
                    wx = origin[0] + (path[1][0] + 0.5) * resolution
                    wy = origin[1] + (path[1][1] + 0.5) * resolution
                    bearing = _wrap(math.atan2(wy - py, wx - px) - yaw)
                    distance = math.hypot(wx - px, wy - py)
                    print(f"first waypoint: ({wx:.3f}, {wy:.3f}), "
                          f"{distance:.2f} m at {math.degrees(bearing):.1f} deg")
        scan = node.depth
        grid_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        depth_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        print(f"costmap/depth stamp skew: {abs(grid_stamp - depth_stamp):.3f} s")
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= scan.range_min)
                 & (ranges <= scan.range_max))
        print(f"valid depth beams: {int(np.count_nonzero(valid))}/{len(ranges)}")
        if np.any(valid):
            px, py, yaw = node.pose
            wx = px + ranges[valid] * np.cos(yaw + angles[valid])
            wy = py + ranges[valid] * np.sin(yaw + angles[valid])
            ix = np.floor((wx - origin[0]) / resolution).astype(int)
            iy = np.floor((wy - origin[1]) / resolution).astype(int)
            inside = ((ix >= 0) & (iy >= 0) & (ix < live.shape[1])
                      & (iy < live.shape[0]))
            live_cost = live[iy[inside], ix[inside]]
            static_cost = baseline[iy[inside], ix[inside]]
            nearest = int(np.argmin(ranges[valid]))
            print(f"nearest depth hit: {ranges[valid][nearest]:.3f} m at "
                  f"{math.degrees(angles[valid][nearest]):.1f} deg; "
                  f"world=({wx[nearest]:.3f}, {wy[nearest]:.3f})")
            print(f"depth endpoints lethal in live costmap: "
                  f"{int(np.count_nonzero(live_cost >= 200.0))}/{len(live_cost)}; "
                  f"raised above static: {int(np.count_nonzero(live_cost >= static_cost + 20.0))}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
