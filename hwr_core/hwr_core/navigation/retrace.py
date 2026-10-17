"""Reverse policy (plan/18.4): back up only along the path just driven, after a rear camera look."""
from __future__ import annotations

import math
from collections.abc import Sequence

MAX_REVERSE_M = 0.15
MAX_REVERSE_SPEED = 0.06


def retrace_budget(trail: Sequence[Sequence[float]], pose: Sequence[float],
                   max_m: float = MAX_REVERSE_M, lateral_tol: float = 0.03,
                   yaw_tol: float = math.radians(10.0)) -> float:
    """How far (m) the robot may back straight up: the odometry trail (oldest first, x/y/yaw),
    walked back from the newest point while it stays on the line behind the robot (within
    lateral_tol) at the current heading (within yaw_tol). A turn or strafe since ends it."""
    x, y, yaw = (float(v) for v in pose[:3])
    c, s = math.cos(yaw), math.sin(yaw)
    budget = 0.0
    for px, py, pyaw in reversed([tuple(p[:3]) for p in trail]):
        bx = c * (px - x) + s * (py - y)
        by = -s * (px - x) + c * (py - y)
        if abs(by) > lateral_tol or bx > 0.01 or \
                abs(math.atan2(math.sin(pyaw - yaw), math.cos(pyaw - yaw))) > yaw_tol:
            break
        budget = max(budget, -bx)
    return min(budget, max_m)


def rear_blocked(angles: Sequence[float], ranges: Sequence[float], meters: float,
                 half_width: float = 0.18, near: float = 0.12, margin: float = 0.30):
    """Nearest depth return (x, y in the base frame) inside the corridor the robot backs into,
    or None. The corridor runs from `near` behind the centre to meters + margin behind it."""
    hits = []
    for angle, distance in zip(angles, ranges):
        if not math.isfinite(distance):
            continue
        bx, by = distance * math.cos(angle), distance * math.sin(angle)
        if -(near + meters + margin) <= bx <= -near and abs(by) <= half_width:
            hits.append((round(bx, 3), round(by, 3)))
    return max(hits, default=None)  # largest x = nearest behind


def clamp_reverse(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    """The only reverse the base accepts: pure -x, slow. Anything else becomes a stop."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)) or vy or wz or vx > 0.0:
        return (0.0, 0.0, 0.0)
    return (max(-MAX_REVERSE_SPEED, vx), 0.0, 0.0)
