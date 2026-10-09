"""Detect frontiers and rank reachable free-space observation poses."""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable

import numpy as np

@dataclass(frozen=True)
class Frontier:
    cells: tuple[tuple[int, int], ...]
    centroid: tuple[float, float]
    score: float
    goal: tuple[float, float]
    goal_cell: tuple[int, int]
    path_length_m: float
    clearance_m: float
    information_gain: int
    heading_change_rad: float


def _world_to_cell(point: tuple[float, float], resolution: float,
                   origin: tuple[float, float]) -> tuple[int, int]:
    return (int(math.floor((point[0] - origin[0]) / resolution)),
            int(math.floor((point[1] - origin[1]) / resolution)))


def _cell_to_world(cell: tuple[int, int], resolution: float,
                   origin: tuple[float, float]) -> tuple[float, float]:
    return (origin[0] + (cell[0] + 0.5) * resolution,
            origin[1] + (cell[1] + 0.5) * resolution)


def _clusters(mask: np.ndarray) -> list[tuple[tuple[int, int], ...]]:
    seen = np.zeros(mask.shape, dtype=bool)
    output = []
    for sy, sx in np.argwhere(mask):
        if seen[sy, sx]:
            continue
        queue, cells = [(int(sx), int(sy))], []
        seen[sy, sx] = True
        while queue:
            x, y = queue.pop()
            cells.append((x, y))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < mask.shape[1] and 0 <= ny < mask.shape[0]
                            and mask[ny, nx] and not seen[ny, nx]):
                        seen[ny, nx] = True
                        queue.append((nx, ny))
        output.append(tuple(sorted(cells)))
    return output


def _observation_candidates(
        cells: tuple[tuple[int, int], ...], unknown: np.ndarray,
        free: np.ndarray, setback_cells: int) -> list[tuple[int, int]]:
    """Return deterministic cells behind the frontier, away from unknown."""
    height, width = free.shape
    output = set()
    for x, y in cells:
        unknown_neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if (0 <= nx < width and 0 <= ny < height
                        and unknown[ny, nx]):
                    unknown_neighbors.append((nx, ny))
        if not unknown_neighbors:
            continue
        mean_x = sum(item[0] for item in unknown_neighbors) / len(unknown_neighbors)
        mean_y = sum(item[1] for item in unknown_neighbors) / len(unknown_neighbors)
        away_x, away_y = x - mean_x, y - mean_y
        norm = math.hypot(away_x, away_y)
        if norm <= 1e-9:
            continue
        for distance in range(setback_cells, 0, -1):
            candidate = (int(round(x + away_x * distance / norm)),
                         int(round(y + away_y * distance / norm)))
            cx, cy = candidate
            if 0 <= cx < width and 0 <= cy < height and free[cy, cx]:
                output.add(candidate)
    return sorted(output)


def _clearance(costmap: np.ndarray, cell: tuple[int, int],
               resolution: float, block_cost: int, limit: float) -> float:
    radius = max(1, int(math.ceil(limit / resolution)))
    x, y = cell
    y0, y1 = max(0, y - radius), min(costmap.shape[0], y + radius + 1)
    x0, x1 = max(0, x - radius), min(costmap.shape[1], x + radius + 1)
    blocked_y, blocked_x = np.where(costmap[y0:y1, x0:x1] >= block_cost)
    if not len(blocked_x):
        return limit
    distances = np.hypot(blocked_x + x0 - x, blocked_y + y0 - y)
    return min(limit, float(np.min(distances)) * resolution)


def _travel_costs(costmap: np.ndarray, start: tuple[int, int],
                  resolution: float, lethal: int,
                  cost_weight: float = 3.0,
                  allow_trapped_start: bool = False) -> np.ndarray:
    """One Dijkstra field equivalent to repeated cost-weighted A* queries."""
    height, width = costmap.shape
    distances = np.full((height, width), np.inf, dtype=float)
    sx, sy = start
    if not (0 <= sx < width and 0 <= sy < height):
        return distances
    if int(costmap[sy, sx]) >= lethal and not allow_trapped_start:
        return distances
    distances[sy, sx] = 0.0
    queue = [(0.0, sx, sy)]
    while queue:
        distance, x, y = heapq.heappop(queue)
        if distance != distances[y, x]:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                cell_cost = int(costmap[ny, nx])
                current_cost = int(costmap[y, x])
                if current_cost >= lethal and allow_trapped_start:
                    # Match navigation's bounded trapped-start contract: the
                    # base may occupy an unknown/inflated center cell, but it
                    # can only leave through an immediately descending cost
                    # gradient. A true 255 cell must descend on the first step.
                    if (cell_cost > current_cost
                            or (current_cost >= 255
                                and cell_cost >= current_cost)):
                        continue
                elif cell_cost >= lethal:
                    continue
                step = (math.hypot(dx, dy) * resolution
                        * (1.0 + cost_weight * cell_cost / 255.0))
                candidate = distance + step
                if candidate < distances[ny, nx]:
                    distances[ny, nx] = candidate
                    heapq.heappush(queue, (candidate, nx, ny))
    return distances


def _excluded(goal: tuple[float, float], excluded: Iterable[tuple[float, float]],
              radius: float) -> bool:
    return any(math.hypot(goal[0] - item[0], goal[1] - item[1]) <= radius
               for item in excluded)


def find_frontiers(
        occupancy: np.ndarray, resolution: float,
        origin: tuple[float, float], robot_xy: tuple[float, float],
        min_cells: int = 5, *, costmap: np.ndarray | None = None,
        robot_yaw: float = 0.0, setback_m: float = 0.25,
        max_target_cost: int = 64, path_block_cost: int = 180,
        excluded_goals: Iterable[tuple[float, float]] = (),
        exclusion_radius_m: float = 0.20) -> list[Frontier]:
    """Find frontiers with reachable observation goals in known free space.

    Unknown cells are never traversed. If an inflated live costmap is supplied,
    its costs control route eligibility and clearance; map occupancy still
    controls which cells represent exploration frontiers.
    """
    grid = np.asarray(occupancy)
    if grid.ndim != 2 or not grid.size:
        raise ValueError("occupancy must be a non-empty 2D array")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if min_cells < 1:
        raise ValueError("min_cells must be positive")

    unknown = grid < 0
    free = grid == 0
    adjacent_unknown = np.zeros(grid.shape, dtype=bool)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    mask = free & adjacent_unknown

    if costmap is None:
        traversal = np.where(free, 0, 255).astype(np.uint8)
    else:
        traversal = np.asarray(costmap)
        if traversal.shape != grid.shape:
            raise ValueError("costmap shape must match occupancy")
        traversal = np.clip(traversal, 0, 255).astype(np.uint8).copy()
        traversal[~free] = 255

    start = _world_to_cell(robot_xy, resolution, origin)
    if not (0 <= start[0] < grid.shape[1] and 0 <= start[1] < grid.shape[0]):
        return []
    setback_cells = max(1, int(math.ceil(setback_m / resolution)))
    excluded_goals = tuple(excluded_goals)
    travel_costs = _travel_costs(
        traversal, start, resolution, path_block_cost,
        allow_trapped_start=True)
    output = []

    for cells in _clusters(mask):
        if len(cells) < min_cells:
            continue
        mean = np.mean(cells, axis=0)
        centroid = _cell_to_world((int(round(mean[0])), int(round(mean[1]))),
                                  resolution, origin)
        best = None
        for candidate in _observation_candidates(
                cells, unknown, free, setback_cells):
            if int(traversal[candidate[1], candidate[0]]) > max_target_cost:
                continue
            goal = _cell_to_world(candidate, resolution, origin)
            if _excluded(goal, excluded_goals, exclusion_radius_m):
                continue
            path_length = float(travel_costs[candidate[1], candidate[0]])
            if not math.isfinite(path_length):
                continue
            clearance = _clearance(
                traversal, candidate, resolution, path_block_cost, 0.75)
            heading = math.atan2(goal[1] - robot_xy[1],
                                 goal[0] - robot_xy[0])
            heading_change = abs(math.atan2(
                math.sin(heading - robot_yaw), math.cos(heading - robot_yaw)))
            information_gain = len(cells)
            score = (information_gain * (0.20 + clearance)
                     / ((0.50 + path_length)
                        * (1.0 + 0.25 * heading_change / math.pi)))
            ranked = (score, clearance, -path_length,
                      -heading_change, -candidate[1], -candidate[0])
            if best is None or ranked > best[0]:
                best = (ranked, candidate, goal, path_length,
                        clearance, heading_change, score)
        if best is None:
            continue
        _, candidate, goal, path_length, clearance, heading_change, score = best
        output.append(Frontier(
            cells=cells, centroid=centroid, score=float(score), goal=goal,
            goal_cell=candidate, path_length_m=path_length,
            clearance_m=clearance, information_gain=len(cells),
            heading_change_rad=heading_change))
    return sorted(output, key=lambda item: (
        -item.score, -item.clearance_m, item.path_length_m,
        item.goal[1], item.goal[0]))
