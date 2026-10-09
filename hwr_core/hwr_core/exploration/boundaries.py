"""Persistence helpers for semantically rejected exploration boundaries."""
from __future__ import annotations

import math
from typing import Iterable


def restore_semantic_boundary_cooldowns(
        records: Iterable[dict], duration_seconds: float, *,
        now_wall: float, now_monotonic: float,
) -> tuple[list[tuple[tuple[float, float], float]],
           list[tuple[float, float]]]:
    """Rebuild unexpired glass-frontier and bearing cooldowns from a journal."""
    values = (duration_seconds, now_wall, now_monotonic)
    if (not all(math.isfinite(float(value)) for value in values)
            or duration_seconds <= 0.0):
        raise ValueError("cooldown clocks and duration must be finite and positive")
    goals: list[tuple[tuple[float, float], float]] = []
    bearings: list[tuple[float, float]] = []
    for record in records:
        if not isinstance(record, dict):
            continue
        if record.get("event") != "semantic_preflight_rejected":
            continue
        assessment = ((record.get("report") or {}).get("assessment") or {})
        if assessment.get("glass_boundary_in_corridor") is not True:
            continue
        try:
            remaining = duration_seconds - (
                float(now_wall) - float(record["time"]))
            pose = (record["report"]["navigation_context"]["pose"])
            waypoint = record["waypoint"]
            frontier_goal = record["frontier_goal"]
            numbers = (*pose[:2], *waypoint[:2], *frontier_goal[:2], remaining)
            if (len(pose) < 2 or len(waypoint) < 2 or len(frontier_goal) < 2
                    or not all(math.isfinite(float(value)) for value in numbers)
                    or remaining <= 0.0):
                continue
        except (KeyError, TypeError, ValueError):
            continue
        expiry = float(now_monotonic) + remaining
        goal = float(frontier_goal[0]), float(frontier_goal[1])
        goals.append((goal, expiry))
        dx = float(waypoint[0]) - float(pose[0])
        dy = float(waypoint[1]) - float(pose[1])
        if math.hypot(dx, dy) > 0.03:
            bearings.append((math.atan2(dy, dx), expiry))
    return goals, bearings
