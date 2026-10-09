"""Generate reachable close approaches to point and extended objects."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from collections.abc import Sequence

import numpy as np

from .planner import astar


@dataclass(frozen=True)
class ObjectApproach:
    """A collision-free robot pose facing a closest point on an object."""

    x: float
    y: float
    yaw: float
    object_x: float
    object_y: float
    object_distance_m: float
    path_length_m: float
    clearance_m: float
    target_cost: int

    def as_dict(self) -> dict:
        return asdict(self)


def _finite_point(point: Sequence[float], name: str) -> tuple[float, float]:
    if len(point) != 2:
        raise ValueError(f"{name} must contain exactly two coordinates")
    result = float(point[0]), float(point[1])
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite coordinates")
    return result


def closest_point_on_segment(
        point: Sequence[float], start: Sequence[float], end: Sequence[float],
) -> tuple[float, float]:
    """Return the closest point on a finite 2-D segment."""
    px, py = _finite_point(point, "point")
    ax, ay = _finite_point(start, "segment start")
    bx, by = _finite_point(end, "segment end")
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return ax, ay
    scale = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy)
                              / length_squared))
    return ax + scale * dx, ay + scale * dy


def point_to_segment_distance(
        point: Sequence[float], start: Sequence[float], end: Sequence[float],
) -> float:
    nearest = closest_point_on_segment(point, start, end)
    px, py = _finite_point(point, "point")
    return math.hypot(px - nearest[0], py - nearest[1])


def project_depth_pixel(
        pixel: Sequence[float], depth_m: float,
        intrinsics: Sequence[float], target_from_camera: np.ndarray,
) -> tuple[float, float, float]:
    """Project one optical-frame depth sample through a 4x4 transform.

    Intrinsics are ``(fx, fy, cx, cy)`` and follow the ROS optical convention:
    +X right, +Y down, +Z forward. The returned point is in ``target`` frame.
    """
    u, v = _finite_point(pixel, "pixel")
    if len(intrinsics) != 4:
        raise ValueError("intrinsics must be (fx, fy, cx, cy)")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    depth_m = float(depth_m)
    if (not all(math.isfinite(value) for value in
                (fx, fy, cx, cy, depth_m)) or fx <= 0.0 or fy <= 0.0
            or depth_m <= 0.0):
        raise ValueError("intrinsics and depth must be finite and positive")
    transform = np.asarray(target_from_camera, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("target_from_camera must be a finite 4x4 matrix")
    optical = np.array([(u - cx) * depth_m / fx,
                        (v - cy) * depth_m / fy, depth_m, 1.0])
    target = transform @ optical
    if abs(target[3]) <= 1e-12:
        raise ValueError("projection produced an invalid homogeneous point")
    return tuple(float(value / target[3]) for value in target[:3])


def _world_to_cell(point, resolution, origin):
    return (math.floor((point[0] - origin[0]) / resolution),
            math.floor((point[1] - origin[1]) / resolution))


def staged_path_waypoint(
        costmap: np.ndarray, resolution: float, origin: Sequence[float],
        start_xy: Sequence[float], goal_xy: Sequence[float], *,
        max_leg: float = 0.35, path_block_cost: int = 180,
        allow_trapped_start: bool = True,
) -> tuple[float, float]:
    """Return a point at most ``max_leg`` along the collision-free A* path.

    Distance is measured from the robot's actual pose, not from the center of
    its containing grid cell. This prevents a nominal 0.10 m stage from
    collapsing into an independently unconfirmable 2--3 cm micro-goal when the
    robot starts near a cell edge.
    """
    grid = np.asarray(costmap)
    if grid.ndim != 2 or not grid.size:
        raise ValueError("costmap must be a non-empty 2D array")
    resolution = float(resolution)
    ox, oy = _finite_point(origin, "origin")
    start = _finite_point(start_xy, "start_xy")
    goal = _finite_point(goal_xy, "goal_xy")
    max_leg = float(max_leg)
    if (not math.isfinite(resolution) or resolution <= 0.0
            or not math.isfinite(max_leg) or max_leg < resolution):
        raise ValueError("max_leg must be finite and at least one map cell")
    start_cell = _world_to_cell(start, resolution, (ox, oy))
    goal_cell = _world_to_cell(goal, resolution, (ox, oy))
    cells = astar(grid, start_cell, goal_cell, lethal=path_block_cost,
                  allow_trapped_start=allow_trapped_start)
    if not cells:
        raise ValueError("goal has no collision-free path")
    if len(cells) == 1:
        return goal
    path_points = [
        (ox + (cell[0] + 0.5) * resolution,
         oy + (cell[1] + 0.5) * resolution)
        for cell in cells[1:-1]
    ]
    path_points.append(goal)
    previous = start
    remaining = max_leg
    for point in path_points:
        step = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if step <= remaining + 1e-9:
            remaining -= step
            previous = point
            continue
        scale = remaining / step
        return (previous[0] + scale * (point[0] - previous[0]),
                previous[1] + scale * (point[1] - previous[1]))
    return goal


def staged_yaw_waypoints(
        start_yaw: float, goal_yaw: float, *,
        max_step: float = math.radians(30.0)) -> list[float]:
    """Split an in-place alignment into bounded wrapped-yaw waypoints."""
    values = float(start_yaw), float(goal_yaw), float(max_step)
    if not all(math.isfinite(value) for value in values) or max_step <= 0.0:
        raise ValueError("yaw values must be finite and max_step positive")
    error = math.atan2(math.sin(goal_yaw - start_yaw),
                       math.cos(goal_yaw - start_yaw))
    if abs(error) <= 1e-9:
        return []
    count = max(1, math.ceil(abs(error) / max_step - 1e-12))
    return [math.atan2(math.sin(start_yaw + error * index / count),
                       math.cos(start_yaw + error * index / count))
            for index in range(1, count + 1)]


def _clearance(grid: np.ndarray, cell: tuple[int, int], resolution: float,
               block_cost: int, limit: float) -> float:
    radius = max(1, math.ceil(limit / resolution))
    cx, cy = cell
    x0, x1 = max(0, cx - radius), min(grid.shape[1], cx + radius + 1)
    y0, y1 = max(0, cy - radius), min(grid.shape[0], cy + radius + 1)
    ys, xs = np.where(grid[y0:y1, x0:x1] >= block_cost)
    if not len(xs):
        return limit
    return min(limit, float(np.hypot(xs + x0 - cx, ys + y0 - cy).min()
                            * resolution))


def _support_samples(start: tuple[float, float], end: tuple[float, float],
                     spacing: float) -> list[tuple[float, float]]:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, math.ceil(length / spacing))
    return [(start[0] + (end[0] - start[0]) * index / count,
             start[1] + (end[1] - start[1]) * index / count)
            for index in range(count + 1)]


def object_approach_candidates(
        costmap: np.ndarray, resolution: float, origin: Sequence[float],
        start_xy: Sequence[float], object_start: Sequence[float],
        object_end: Sequence[float] | None = None, *,
        min_distance: float = 0.30, max_distance: float = 0.48,
        distance_samples: int = 3, angle_samples: int = 24,
        support_spacing: float = 0.20, max_target_cost: int = 64,
        path_block_cost: int = 180, max_clearance: float = 0.75,
        allow_trapped_start: bool = True, limit: int | None = None,
) -> list[ObjectApproach]:
    """Rank reachable poses within the strict object-arrival envelope.

    ``object_start == object_end`` describes a point target. Distinct endpoints
    describe an extended physical surface, such as a window or door. Candidate
    yaw always faces the closest point on that surface. Unknown and inflated
    cells are excluded using the same thresholds as the navigation stack.
    """
    grid = np.asarray(costmap)
    if grid.ndim != 2 or not grid.size:
        raise ValueError("costmap must be a non-empty 2D array")
    resolution = float(resolution)
    ox, oy = _finite_point(origin, "origin")
    sx, sy = _finite_point(start_xy, "start_xy")
    target_start = _finite_point(object_start, "object_start")
    target_end = target_start if object_end is None else _finite_point(
        object_end, "object_end")
    numeric = (resolution, min_distance, max_distance, support_spacing,
               max_clearance)
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError("approach parameters must be finite")
    if resolution <= 0.0 or support_spacing <= 0.0 or max_clearance <= 0.0:
        raise ValueError("resolution, support spacing, and clearance must be positive")
    if not 0.0 < min_distance <= max_distance <= 0.50:
        raise ValueError("require 0 < min_distance <= max_distance <= 0.50")
    if distance_samples < 1 or angle_samples < 4:
        raise ValueError("distance_samples must be positive and angle_samples >= 4")
    if limit is not None and limit < 1:
        raise ValueError("limit must be positive")

    start_cell = _world_to_cell((sx, sy), resolution, (ox, oy))
    if not (0 <= start_cell[0] < grid.shape[1]
            and 0 <= start_cell[1] < grid.shape[0]):
        return []
    distances = np.linspace(min_distance, max_distance, distance_samples)
    supports = _support_samples(target_start, target_end, support_spacing)
    by_cell: dict[tuple[int, int], ObjectApproach] = {}
    for tx, ty in supports:
        for angle_index in range(angle_samples):
            angle = 2.0 * math.pi * angle_index / angle_samples
            for distance in distances:
                candidate_xy = (tx + distance * math.cos(angle),
                                ty + distance * math.sin(angle))
                cell = _world_to_cell(candidate_xy, resolution, (ox, oy))
                if not (0 <= cell[0] < grid.shape[1]
                        and 0 <= cell[1] < grid.shape[0]):
                    continue
                target_cost = int(grid[cell[1], cell[0]])
                if target_cost > max_target_cost:
                    continue
                nearest = closest_point_on_segment(
                    candidate_xy, target_start, target_end)
                object_distance = math.hypot(candidate_xy[0] - nearest[0],
                                             candidate_xy[1] - nearest[1])
                if not min_distance - 1e-9 <= object_distance <= max_distance + 1e-9:
                    continue
                cells = astar(grid, start_cell, cell, lethal=path_block_cost,
                              allow_trapped_start=allow_trapped_start)
                if not cells:
                    continue
                path_length = sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(cells, cells[1:])) * resolution
                approach = ObjectApproach(
                    x=float(candidate_xy[0]), y=float(candidate_xy[1]),
                    yaw=math.atan2(nearest[1] - candidate_xy[1],
                                   nearest[0] - candidate_xy[0]),
                    object_x=float(nearest[0]), object_y=float(nearest[1]),
                    object_distance_m=float(object_distance),
                    path_length_m=float(path_length),
                    clearance_m=_clearance(grid, cell, resolution,
                                           path_block_cost, max_clearance),
                    target_cost=target_cost)
                previous = by_cell.get(cell)
                if previous is None or _rank_key(approach) < _rank_key(previous):
                    by_cell[cell] = approach

    ranked = sorted(by_cell.values(), key=_rank_key)
    return ranked if limit is None else ranked[:limit]


def _rank_key(candidate: ObjectApproach) -> tuple[float, float, int, float, float]:
    # Clearance is primary: a slightly longer route is preferable to squeezing
    # beside a thin pole or furniture base. Remaining fields make ties stable.
    return (-candidate.clearance_m, candidate.path_length_m,
            candidate.target_cost, candidate.x, candidate.y)
