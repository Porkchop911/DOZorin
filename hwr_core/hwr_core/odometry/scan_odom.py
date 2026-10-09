"""Scan odometry: keyframe-managed matcher + fusion (core of plan task 5.2).

Owns the keyframe policy: match every scan against the current keyframe
(prior-seeded from the fused pose — a small window is both faster and
more robust), re-key when the robot has moved far enough that the scans
decorrelate or the match quality decays. The hwr_odom ROS node wraps
this with topics and TF; everything testable lives here.
"""
from __future__ import annotations

import math

from hwr_core.matching import CorrelativeMatcher, MatchResult
from hwr_core.odometry.fusion import OdomFusion


class ScanOdometry:
    def __init__(self,
                 matcher: CorrelativeMatcher | None = None,
                 fusion: OdomFusion | None = None,
                 rekey_dist: float = 0.3,
                 rekey_angle: float = math.radians(20.0),
                 rekey_score: float = 0.6):
        self.matcher = matcher or CorrelativeMatcher()
        self.fusion = fusion or OdomFusion()
        self.rekey_dist = rekey_dist
        self.rekey_angle = rekey_angle
        self.rekey_score = rekey_score
        self.keyframes = 0
        self._key_pose: tuple[float, float, float] | None = None

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.fusion.pose

    def predict(self, vx: float, vy: float, wz: float, dt: float) -> None:
        """Model velocity (post-slew command) + IMU yaw rate, every tick."""
        self.fusion.predict(vx, vy, wz, dt)

    def process_scan(self, points, *,
                     observe_translation: bool = True) -> MatchResult | None:
        """Feed one scan's base-frame points (~5 Hz). Returns the match,
        or None for the scan that (re)seeds a keyframe."""
        if self._key_pose is None:
            self._rekey(points)
            return None

        kx, ky, kth = self._key_pose
        x, y, th = self.fusion.pose
        c, s = math.cos(kth), math.sin(kth)
        prior = (c * (x - kx) + s * (y - ky),
                 -s * (x - kx) + c * (y - ky),
                 math.atan2(math.sin(th - kth), math.cos(th - kth)))

        result = self.matcher.match(points, prior=prior)
        if result.valid:
            self.fusion.update_scan(self._key_pose, result.delta,
                                    result.cov, result.score,
                                    observe_translation=observe_translation)

        moved = math.hypot(prior[0], prior[1])
        if (not result.valid or moved > self.rekey_dist
                or abs(prior[2]) > self.rekey_angle
                or result.score < self.rekey_score):
            self._rekey(points)
        return result

    def _rekey(self, points) -> None:
        self.matcher.set_reference(points)
        self._key_pose = self.fusion.pose
        self.keyframes += 1
