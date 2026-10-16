"""Pure helpers for floor-cube pick/place and camera following (plan/16)."""
from __future__ import annotations

import math

from .kinematics import ArmKinematics, JointSolution, joints_to_pulses


def wrap_quarter(angle: float) -> float:
    """Cube yaw is symmetric every 90 deg: map to [-45, 45) deg."""
    return (angle + math.pi / 4) % (math.pi / 2) - math.pi / 4


def locate_cube(item: dict, k, joints: JointSolution, arm: ArmKinematics,
                plane_z: float) -> dict | None:
    """Base-frame centre (x, y) and yaw of a blob seen by the arm camera, on plane z = plane_z."""
    centre = arm.pixel_to_plane(*item["center"], k, joints, plane_z)
    if centre is None:
        return None
    # Dominant edge direction mod 90 deg: length-weighted circular mean of 4*theta over the
    # outline projected onto the plane.
    points = [arm.pixel_to_plane(u, v, k, joints, plane_z) for u, v in item.get("outline", [])]
    points = [p for p in points if p is not None]
    sx = sy = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        length = math.hypot(x1 - x0, y1 - y0)
        theta = math.atan2(y1 - y0, x1 - x0)
        sx, sy = sx + length * math.cos(4 * theta), sy + length * math.sin(4 * theta)
    yaw = wrap_quarter(math.atan2(sy, sx) / 4) if len(points) >= 3 else 0.0
    return {"x": round(centre[0], 4), "y": round(centre[1], 4), "yaw": round(yaw, 3)}


def plan_grasp(arm: ArmKinematics, x: float, y: float, z: float, yaw: float,
               hover: float = 0.05, roll_sign: float = 1.0,
               flattest: float = -math.radians(30)):
    """(hover pulses, grasp pulses, pitch) with the jaws square to the cube, or None."""
    pan = math.atan2(y, x - arm.pan_x)
    roll = roll_sign * wrap_quarter(yaw - pan)
    # Steepest pitch that works for both points: the straight vertical descent needs one pitch.
    for step in range(13):
        pitch = -math.pi / 2 + math.radians(5 * step)
        if pitch > flattest + 1e-9:
            break
        grasp, above = arm.ik(x, y, z, pitch, roll), arm.ik(x, y, z + hover, pitch, roll)
        if grasp is not None and above is not None:
            return joints_to_pulses(above), joints_to_pulses(grasp), pitch
    return None


def follow_step(center, k, bearing: float, elevation: float, range_m: float | None,
                distance: float, gain: float = 0.5, base_gain: float = 1.2,
                max_wz: float = 0.5, max_v: float = 0.15, align: float = math.radians(15)):
    """One tick: new camera (bearing, elevation) toward the target pixel, and base (vx, wz).

    The arm centres the target; the base turns to unwind the arm pan and drives
    (forward only) to hold `distance` once the arm points roughly ahead."""
    fx, fy, cx, cy = k
    target = bearing - math.atan2(center[0] - cx, fx)  # target bearing from the base; pixel right = right
    bearing += gain * (target - bearing)
    elevation -= gain * math.atan2(center[1] - cy, fy)    # pixel down -> look down
    wz = max(-max_wz, min(max_wz, base_gain * target))
    vx = 0.0
    if range_m is not None and abs(target) < align and range_m > distance:
        vx = min(max_v, 0.5 * (range_m - distance))
    return bearing, elevation, vx, wz
