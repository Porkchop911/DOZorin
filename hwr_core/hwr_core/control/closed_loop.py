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


def pick_target(objects: list[dict], target: str, track_id: int | None) -> dict | None:
    """The locked track if still visible, else the nearest (then most confident) object of the class."""
    if track_id is not None:
        for item in objects:
            if item.get("id") == track_id:
                return item
    matches = [o for o in objects if o.get("class") == target and o.get("bearing_deg") is not None]
    if not matches:
        return None
    return min(matches, key=lambda o: (o.get("range_m") is None, o.get("range_m") or 0.0,
                                       -o.get("conf", 0.0)))


def approach_command(bearing_deg: float, range_m: float | None, stop_at: float,
                     max_wz: float, min_wz: float, max_v: float, min_v: float,
                     align_deg: float = 10.0):
    """(vx, wz) toward a target seen at bearing/range; None when within stop_at.

    Turns in place until aligned within align_deg, then drives while steering.
    Without a range it only aligns (the caller decides how long to wait for depth)."""
    if range_m is not None and range_m <= stop_at and abs(bearing_deg) <= align_deg:
        return None
    wz = 0.0
    if abs(bearing_deg) > 2.0:
        wz = approach_speed(math.radians(bearing_deg), max_wz, min_wz, 1.5)
    vx = 0.0
    if range_m is not None and range_m > stop_at and abs(bearing_deg) <= align_deg:
        vx = abs(approach_speed(range_m - stop_at, max_v, min_v, 1.0))
    return vx, wz
