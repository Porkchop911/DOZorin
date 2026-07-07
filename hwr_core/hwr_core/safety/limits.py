"""Velocity and acceleration clamps applied to every drive command."""
from __future__ import annotations

from dataclasses import dataclass


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class Limits:
    """Defaults follow the vendor's proven safe envelope."""
    max_vx: float = 0.2     # m/s (forward only is enforced elsewhere; this is magnitude)
    max_vy: float = 0.2     # m/s
    max_wz: float = 0.5     # rad/s
    max_accel: float = 0.5  # m/s^2, per linear axis
    max_alpha: float = 2.0  # rad/s^2

    def clamp_velocity(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        return (_clamp(vx, -self.max_vx, self.max_vx),
                _clamp(vy, -self.max_vy, self.max_vy),
                _clamp(wz, -self.max_wz, self.max_wz))

    def clamp_accel(self, cmd: tuple[float, float, float],
                    prev: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """Limit the change from the previous command over dt seconds."""
        if dt <= 0:
            return prev
        dv = self.max_accel * dt
        dw = self.max_alpha * dt
        return (_clamp(cmd[0], prev[0] - dv, prev[0] + dv),
                _clamp(cmd[1], prev[1] - dv, prev[1] + dv),
                _clamp(cmd[2], prev[2] - dw, prev[2] + dw))
