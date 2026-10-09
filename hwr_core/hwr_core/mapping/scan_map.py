"""Keyframed scan-to-map correction for occupancy mapping.

The mapper keeps odometry continuous and represents global corrections as an
SE(2) ``map->odom`` transform.  Each accepted keyframe is matched directly
against occupied map cells before it is integrated, which prevents odometry
drift from being painted as a second wall.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from hwr_core.matching import CorrelativeMatcher, scan_to_points

from .grid import LogOddsGrid


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b) -> tuple[float, float, float]:
    """Compose planar poses/transforms: ``T_result = T_a * T_b``."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            _wrap(ath + bth))


def inverse(pose) -> tuple[float, float, float]:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, _wrap(-yaw))


def mapping_scan_to_points(
        angles, ranges, sensor_max_range: float, mapping_max_range: float, *,
        sensor_yaw: float = 0.0,
        sensor_xy: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Drop long indoor mapping rays that may pass through glazing.

    Long hits are omitted rather than converted to a synthetic obstacle. This
    keeps space beyond glass unknown until it is observed from a closer,
    non-glass viewpoint.
    """
    values = float(sensor_max_range), float(mapping_max_range)
    if (not all(math.isfinite(value) for value in values)
            or sensor_max_range <= 0.0 or mapping_max_range <= 0.0):
        raise ValueError("scan range limits must be finite and positive")
    return scan_to_points(
        np.asarray(angles, dtype=float), np.asarray(ranges, dtype=float),
        min(sensor_max_range, mapping_max_range),
        sensor_yaw=sensor_yaw, sensor_xy=sensor_xy)


@dataclass(frozen=True)
class MapUpdate:
    integrated: bool
    corrected: bool
    pose: tuple[float, float, float]
    map_to_odom: tuple[float, float, float]
    score: float = 0.0


class ScanMapMapper:
    """Own keyframe selection, map matching, correction, and integration."""

    def __init__(self, grid: LogOddsGrid | None = None,
                 matcher: CorrelativeMatcher | None = None,
                 keyframe_dist: float = 0.10,
                 keyframe_angle: float = math.radians(8.0),
                 minimum_score: float = 0.45,
                 reference_radius: float = 6.0):
        self.grid = grid or LogOddsGrid()
        self.matcher = matcher or CorrelativeMatcher(
            resolution=self.grid.resolution, window_xy=0.20,
            window_theta=math.radians(12.0), coarse_xy=0.04)
        self.keyframe_dist = float(keyframe_dist)
        self.keyframe_angle = float(keyframe_angle)
        self.minimum_score = float(minimum_score)
        self.reference_radius = float(reference_radius)
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.last_odom_pose: tuple[float, float, float] | None = None
        self.keyframes = 0

    def process(self, odom_pose, points) -> MapUpdate:
        odom_pose = tuple(float(v) for v in odom_pose)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        predicted = compose(self.map_to_odom, odom_pose)
        if not len(points):
            return MapUpdate(False, False, predicted, self.map_to_odom)

        if self.last_odom_pose is not None:
            dx = odom_pose[0] - self.last_odom_pose[0]
            dy = odom_pose[1] - self.last_odom_pose[1]
            da = _wrap(odom_pose[2] - self.last_odom_pose[2])
            if math.hypot(dx, dy) < self.keyframe_dist and abs(da) < self.keyframe_angle:
                return MapUpdate(False, False, predicted, self.map_to_odom)

        corrected_pose = predicted
        score = 0.0
        corrected = False
        reference = self.grid.occupied_points(predicted[:2], self.reference_radius)
        if len(reference) >= self.matcher.min_points:
            self.matcher.set_reference(reference)
            match = self.matcher.match(points, prior=predicted)
            score = match.score
            if match.valid and match.score >= self.minimum_score:
                corrected_pose = match.delta
                self.map_to_odom = compose(corrected_pose, inverse(odom_pose))
                corrected = True

        self.grid.integrate(corrected_pose, points)
        self.last_odom_pose = odom_pose
        self.keyframes += 1
        return MapUpdate(True, corrected, corrected_pose, self.map_to_odom, score)
