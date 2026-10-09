"""Static occupancy plus decaying live obstacles and inflation."""
from __future__ import annotations

import math
import threading

import numpy as np


class Costmap:
    def __init__(self, occupancy: np.ndarray, resolution: float,
                 origin: tuple[float, float], inflation_radius: float = 0.25,
                 robot_radius: float = 0.20):
        self.occupancy = np.asarray(occupancy).copy()
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.inflation_radius = float(inflation_radius)
        self.robot_radius = min(max(0.0, float(robot_radius)),
                                self.inflation_radius)
        self.live: dict[tuple[int, int], float] = {}
        self._live_lock = threading.Lock()

    def world_to_cell(self, point) -> tuple[int, int]:
        cell = np.floor((np.asarray(point) - self.origin) / self.resolution).astype(int)
        return int(cell[0]), int(cell[1])

    def mark(self, points, now: float, ttl: float = 1.0) -> None:
        h, w = self.occupancy.shape
        with self._live_lock:
            for point in np.asarray(points, dtype=float).reshape(-1, 2):
                x, y = self.world_to_cell(point)
                if 0 <= x < w and 0 <= y < h:
                    self.live[x, y] = max(self.live.get((x, y), 0.0), now + ttl)

    def clear_live(self) -> None:
        with self._live_lock:
            self.live.clear()

    def segment_is_clear(self, start, end, now: float,
                         lethal_cost: int = 200,
                         allow_trapped_start: bool = False,
                         allow_partial_escape: bool = False) -> bool:
        """Check the entire world-frame segment, including both endpoints."""
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        if start.shape != (2,) or end.shape != (2,) or not (
                np.isfinite(start).all() and np.isfinite(end).all()):
            return False
        distance = float(np.linalg.norm(end - start))
        count = max(2, int(math.ceil(distance / (0.5 * self.resolution))) + 1)
        points = start + np.linspace(0.0, 1.0, count)[:, None] * (end - start)
        cells = np.floor((points - self.origin) / self.resolution).astype(int)
        grid = self.grid(now)
        inside = ((cells[:, 0] >= 0) & (cells[:, 1] >= 0)
                  & (cells[:, 0] < grid.shape[1]) & (cells[:, 1] < grid.shape[0]))
        if not np.all(inside):
            return False
        if len(cells) > 1:
            changed = np.r_[True, np.any(cells[1:] != cells[:-1], axis=1)]
            cells = cells[changed]
        costs = grid[cells[:, 1], cells[:, 0]].astype(int)
        if costs[0] < lethal_cost:
            return bool(np.all(costs < lethal_cost))
        if not allow_trapped_start:
            return False
        still_trapped = True
        previous = int(costs[0])
        for cost in costs[1:]:
            cost = int(cost)
            if still_trapped:
                # Permit flat non-obstacle inflation plateaus, but never move
                # to higher cost and never traverse a flat 255 obstacle cell.
                if (cost > previous
                        or (previous >= 255 and cost >= previous)):
                    return False
                if cost < lethal_cost:
                    still_trapped = False
            elif cost >= lethal_cost:
                return False
            previous = cost
        # A short controller projection can end within a wider inflation
        # plateau even though its globally planned route exits that plateau.
        # Callers that already hold such a route may accept this safe prefix.
        return not still_trapped or allow_partial_escape

    def grid(self, now: float) -> np.ndarray:
        obstacles = self.occupancy >= 65
        with self._live_lock:
            self.live = {cell: expiry for cell, expiry in self.live.items()
                         if expiry > now}
            live_cells = tuple(self.live)
        for x, y in live_cells:
            obstacles[y, x] = True
        costs = np.zeros(self.occupancy.shape, dtype=np.uint8)
        costs[self.occupancy < 0] = 180
        h, w = obstacles.shape
        radius = max(0, int(math.ceil(self.inflation_radius / self.resolution)))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                distance = math.hypot(dx, dy) * self.resolution
                if distance >= self.inflation_radius:
                    continue
                if distance == 0.0:
                    cost = 255
                elif distance <= self.robot_radius:
                    # The robot center may never enter the inscribed footprint.
                    cost = 253
                else:
                    margin = max(self.inflation_radius - self.robot_radius, 1e-9)
                    cost = max(1, min(199, int(199 *
                        (1.0 - (distance - self.robot_radius) / margin))))
                src_y = slice(max(0, -dy), min(h, h - dy))
                src_x = slice(max(0, -dx), min(w, w - dx))
                dst_y = slice(max(0, dy), min(h, h + dy))
                dst_x = slice(max(0, dx), min(w, w + dx))
                region = costs[dst_y, dst_x]
                contribution = np.where(obstacles[src_y, src_x], cost, 0).astype(
                    np.uint8)
                np.maximum(region, contribution, out=region)
        costs[obstacles] = 255
        return costs
