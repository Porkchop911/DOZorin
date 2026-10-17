"""Map coverage numbers and the half-leg retry for exploration (plan/18.3)."""
from __future__ import annotations

import math

import numpy as np


def map_coverage(grid: np.ndarray, resolution: float) -> dict:
    """Known / free / occupied area (m^2) of an occupancy grid (-1 unknown, 0..100)."""
    grid = np.asarray(grid)
    cell = float(resolution) ** 2
    free = int(np.count_nonzero((grid >= 0) & (grid < 50)))
    occupied = int(np.count_nonzero(grid >= 50))
    return {"known_m2": round((free + occupied) * cell, 2), "free_m2": round(free * cell, 2),
            "occupied_m2": round(occupied * cell, 2)}


def shorter_leg(start, waypoint, minimum: float = 0.30, longer_than: float = 0.40):
    """Half of a rejected translation leg, or None when the leg is already short.

    Live 2026-10-16 the VLM rejected all three frontier legs for clutter far ahead and mapping
    ended after 0.1 m; a shorter look at the same direction is often still fine."""
    dx, dy = waypoint[0] - start[0], waypoint[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= longer_than or distance / 2.0 < minimum:
        return None
    return (start[0] + dx / 2.0, start[1] + dy / 2.0)
