"""Odometry fusion filter (plan task 5.3, ADR-004).

Small 2D EKF over the pose (x, y, theta) in the odom frame. No encoders
exist on this robot (ADR-004), so:

  - predict: commanded body velocity from the model (vx AND vy — the
    vendor drops vy) rotated into odom, heading from the IMU yaw rate.
    Process noise grows with commanded motion: standing still is certain,
    driving is not (that's where slip lives).
  - update: a scan-match delta against a keyframe whose fused pose was
    stored when the keyframe was taken. Measurement covariance comes from
    the matcher's score surface, so a corridor's flat axis is naturally
    down-weighted instead of being trusted.

ROS-free; the hwr_odom node wraps this and owns TF. Same conventions as
hwr_core.matching: the delta maps new-scan points into the keyframe base
frame (robot motion since the keyframe, in the keyframe's frame).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class FusionConfig:
    # process noise VARIANCE per unit of commanded motion. Linear in distance
    # (not per-tick in quadrature): slip is correlated — a stalled wheel is
    # wrong for the whole stall — and with no encoders a full stall means the
    # model is off by 100% of the commanded distance. sqrt(0.09) = 0.3 m
    # 1-sigma per commanded meter keeps the filter honest about that.
    var_xy_per_m: float = 0.09      # m^2 per commanded meter
    var_theta_per_rad: float = 2.5e-3   # rad^2 per integrated radian (gyro is good)
    var_xy_per_s: float = 1e-6      # standstill floor, keeps P alive
    var_theta_per_s: float = 4e-6   # gyro bias wander (~0.002 rad/s over 1 s)
    min_score: float = 0.5          # scan matches below this are rejected
    gate_mahalanobis: float = 25.0  # generous: slip makes honest innovations big
    meas_floor_xy: float = 0.003    # m, floor under the matcher covariance
    meas_floor_theta: float = math.radians(0.25)


class OdomFusion:
    """EKF pose filter: model-velocity + IMU-yaw predict, scan-delta update."""

    def __init__(self, config: FusionConfig | None = None):
        self.cfg = config or FusionConfig()
        self.x = np.zeros(3)                    # x, y, theta (odom frame)
        self.P = np.diag([1e-6, 1e-6, 1e-6])
        self.rejected = 0
        self.accepted = 0

    @property
    def pose(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def predict(self, vx: float, vy: float, wz: float, dt: float) -> None:
        """Advance by commanded body velocity (model) + IMU yaw rate."""
        if dt <= 0.0:
            return
        th = self.x[2] + 0.5 * wz * dt          # midpoint heading
        c, s = math.cos(th), math.sin(th)
        dx_w = (vx * c - vy * s) * dt
        dy_w = (vx * s + vy * c) * dt
        self.x[0] += dx_w
        self.x[1] += dy_w
        self.x[2] = _wrap(self.x[2] + wz * dt)

        F = np.eye(3)
        F[0, 2] = -dy_w
        F[1, 2] = dx_w
        step = math.hypot(vx, vy) * dt
        q_xy = self.cfg.var_xy_per_m * step + self.cfg.var_xy_per_s * dt
        q_th = (self.cfg.var_theta_per_rad * abs(wz) * dt
                + self.cfg.var_theta_per_s * dt)
        self.P = F @ self.P @ F.T + np.diag([q_xy, q_xy, q_th])

    def update_scan(self, keyframe_pose: tuple[float, float, float],
                    delta: tuple[float, float, float],
                    delta_cov: np.ndarray, score: float, *,
                    observe_translation: bool = True) -> bool:
        """Fuse a scan-match delta measured against a keyframe.

        `keyframe_pose` is the fused pose stored when the keyframe scan
        was taken; `delta`/`delta_cov`/`score` come straight from a
        MatchResult. Returns True when the measurement was accepted.
        """
        if score < self.cfg.min_score:
            self.rejected += 1
            return False

        kx, ky, kth = keyframe_pose
        c, s = math.cos(kth), math.sin(kth)
        z = np.array([kx + c * delta[0] - s * delta[1],
                      ky + s * delta[0] + c * delta[1],
                      _wrap(kth + delta[2])])

        # matcher covariance is in the keyframe base frame -> rotate to odom
        R2 = np.array([[c, -s], [s, c]])
        R = np.asarray(delta_cov, dtype=float).copy()
        R[:2, :2] = R2 @ R[:2, :2] @ R2.T
        R[:2, 2] = R2 @ R[:2, 2]
        R[2, :2] = R[:2, 2]
        R += np.diag([self.cfg.meas_floor_xy ** 2,
                      self.cfg.meas_floor_xy ** 2,
                      self.cfg.meas_floor_theta ** 2])

        if not observe_translation:
            # During commanded pure yaw, scan correlation may trade rotation
            # for a large fictitious x/y shift in repetitive scenes. Preserve
            # the gyro/model translation baseline and fuse only scan yaw.
            innovation = _wrap(float(z[2]) - float(self.x[2]))
            variance = float(self.P[2, 2] + R[2, 2])
            d2 = innovation * innovation / variance
            if d2 > self.cfg.gate_mahalanobis:
                self.rejected += 1
                return False
            gain = float(self.P[2, 2] / variance)
            self.x[2] = _wrap(float(self.x[2]) + gain * innovation)
            self.P[2, 2] = ((1.0 - gain) ** 2 * self.P[2, 2]
                            + gain * gain * R[2, 2])
            self.P[0, 2] = self.P[2, 0] = 0.0
            self.P[1, 2] = self.P[2, 1] = 0.0
            self.accepted += 1
            return True

        nu = z - self.x
        nu[2] = _wrap(nu[2])
        S = self.P + R                          # H = I
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 > self.cfg.gate_mahalanobis:
            self.rejected += 1
            return False

        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.x[2] = _wrap(self.x[2])
        I_KH = np.eye(3) - K
        # Joseph form keeps P symmetric PSD under rounding
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.accepted += 1
        return True
