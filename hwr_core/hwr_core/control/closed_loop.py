"""Closed-loop bounded motions (turn by angle, move by distance) as pure step functions."""
from __future__ import annotations

import math


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def approach_speed(remaining: float, max_speed: float, min_speed: float,
                   gain: float) -> float:
    """Proportional speed toward zero remaining, floored so friction cannot stall it."""
    magnitude = min(max_speed, max(min_speed, gain * abs(remaining)))
    return math.copysign(magnitude, remaining)


class TurnTracker:
    """Accumulates yaw change across the +-pi wrap, so 180 and 360 degree turns work."""

    def __init__(self, target_rad: float, start_yaw: float, tolerance_rad: float):
        self.target, self.last, self.tolerance = target_rad, start_yaw, tolerance_rad
        self.turned = 0.0

    def update(self, yaw: float) -> float:
        self.turned += wrap(yaw - self.last)
        self.last = yaw
        return self.target - self.turned

    @property
    def done(self) -> bool:
        return abs(self.target - self.turned) <= self.tolerance


def progress_along(start: tuple[float, float, float], now: tuple[float, float],
                   direction_body: float) -> float:
    """Distance travelled from start along a body-frame heading (0 forward, +pi/2 left)."""
    heading = start[2] + direction_body
    return ((now[0] - start[0]) * math.cos(heading)
            + (now[1] - start[1]) * math.sin(heading))
