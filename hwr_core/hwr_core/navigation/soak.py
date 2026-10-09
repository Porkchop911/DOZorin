"""Pure goal sampling helpers for repeatable navigation soak tests."""
from __future__ import annotations

import math

import numpy as np

from .planner import astar


def reachable_goal_candidates(
        costmap: np.ndarray, resolution: float, origin: tuple[float, float],
        start_xy: tuple[float, float], *, count: int = 20,
        min_distance: float = 0.35, max_distance: float = 0.75,
        min_separation: float = 0.25, max_target_cost: int = 64,
        path_block_cost: int = 180, seed: int = 0,
        allow_trapped_start: bool = False,
        excluded_xy: tuple[tuple[float, float], ...] = (),
) -> list[tuple[float, float, float]]:
    """Choose deterministic, reachable free goals from an internal 0..255 grid.

    Unknown cells use cost 180 in the HWR costmap and are deliberately blocked
    here. Targets must be genuinely free rather than merely below the planner's
    hard-footprint threshold.
    """
    grid = np.asarray(costmap)
    if grid.ndim != 2 or not grid.size:
        raise ValueError("costmap must be a non-empty 2D array")
    values = (resolution, *origin, *start_xy, min_distance, max_distance,
              min_separation)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("goal-sampling inputs must be finite")
    if resolution <= 0.0 or count < 1:
        raise ValueError("resolution and count must be positive")
    if not 0.0 <= min_distance <= max_distance:
        raise ValueError("distance bounds are invalid")

    ox, oy = map(float, origin)
    sx, sy = map(float, start_xy)
    start = (math.floor((sx - ox) / resolution),
             math.floor((sy - oy) / resolution))
    height, width = grid.shape
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        return []

    ys, xs = np.where(grid <= max_target_cost)
    wx = ox + (xs + 0.5) * resolution
    wy = oy + (ys + 0.5) * resolution
    distance = np.hypot(wx - sx, wy - sy)
    eligible = ((distance >= min_distance) & (distance <= max_distance))
    indices = np.flatnonzero(eligible)
    rng = np.random.default_rng(seed)
    rng.shuffle(indices)
    selected: list[tuple[float, float, float]] = []
    excluded = [tuple(map(float, point)) for point in excluded_xy]
    for index in indices:
        point = (float(wx[index]), float(wy[index]))
        if any(math.hypot(point[0] - px, point[1] - py) < min_separation
               for px, py in (*excluded, *((x, y) for x, y, _ in selected))):
            continue
        target = (int(xs[index]), int(ys[index]))
        if not astar(grid, start, target, lethal=path_block_cost,
                     allow_trapped_start=allow_trapped_start):
            continue
        selected.append((point[0], point[1], float(rng.uniform(-math.pi,
                                                                math.pi))))
        if len(selected) >= count:
            break
    return selected


def rank_goal_candidates_by_clearance(
        costmap: np.ndarray, resolution: float, origin: tuple[float, float],
        candidates: list[tuple[float, float, float]], *,
        block_cost: int = 180, max_clearance: float = 0.75,
) -> list[tuple[float, float, float]]:
    """Prefer candidate targets with more surrounding known free space."""
    grid = np.asarray(costmap)
    if grid.ndim != 2 or not grid.size:
        raise ValueError("costmap must be a non-empty 2D array")
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError("resolution must be finite and positive")
    if max_clearance <= 0.0 or not math.isfinite(max_clearance):
        raise ValueError("max_clearance must be finite and positive")
    ox, oy = map(float, origin)
    radius = max(1, math.ceil(max_clearance / resolution))
    height, width = grid.shape

    def score(candidate: tuple[float, float, float]):
        x, y, _ = candidate
        cx = math.floor((x - ox) / resolution)
        cy = math.floor((y - oy) / resolution)
        if not (0 <= cx < width and 0 <= cy < height):
            return (-1.0, -255.0)
        x0, x1 = max(0, cx - radius), min(width, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(height, cy + radius + 1)
        blocked_y, blocked_x = np.where(grid[y0:y1, x0:x1] >= block_cost)
        if blocked_x.size:
            dx = blocked_x + x0 - cx
            dy = blocked_y + y0 - cy
            clearance = min(max_clearance, float(
                np.hypot(dx, dy).min() * resolution))
        else:
            clearance = max_clearance
        return clearance, -float(grid[cy, cx])

    return sorted(candidates, key=score, reverse=True)
