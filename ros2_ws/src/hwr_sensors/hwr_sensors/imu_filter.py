"""Small gyro-bias and gravity complementary filter, independent of ROS."""
from __future__ import annotations

import math


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_euler(roll: float, pitch: float, yaw: float
                          ) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class ImuComplementaryFilter:
    def __init__(self, bias_samples: int = 100, alpha: float = 0.98):
        if bias_samples < 1 or not 0.0 <= alpha < 1.0:
            raise ValueError("invalid filter configuration")
        self.bias_samples = bias_samples
        self.alpha = alpha
        self._bias_data: list[tuple[float, float, float]] = []
        self.bias = (0.0, 0.0, 0.0)
        self.roll = self.pitch = self.yaw = 0.0
        self._last_stamp: float | None = None
        self.ready = False

    def update(self, accel: tuple[float, float, float],
               gyro: tuple[float, float, float], stamp: float
               ) -> tuple[float, float, float, float] | None:
        if not self.ready:
            self._bias_data.append(tuple(float(v) for v in gyro))
            if len(self._bias_data) < self.bias_samples:
                return None
            self.bias = tuple(sum(row[i] for row in self._bias_data) / len(self._bias_data)
                              for i in range(3))
            self.roll, self.pitch = self._gravity_angles(accel)
            self._last_stamp = float(stamp)
            self.ready = True
            return quaternion_from_euler(self.roll, self.pitch, self.yaw)

        dt = float(stamp) - self._last_stamp
        self._last_stamp = float(stamp)
        if dt <= 0.0 or dt > 1.0:
            return quaternion_from_euler(self.roll, self.pitch, self.yaw)
        gx, gy, gz = (float(gyro[i]) - self.bias[i] for i in range(3))
        acc_roll, acc_pitch = self._gravity_angles(accel)
        self.roll = self.alpha * (self.roll + gx * dt) + (1.0 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1.0 - self.alpha) * acc_pitch
        self.yaw = _wrap(self.yaw + gz * dt)
        return quaternion_from_euler(self.roll, self.pitch, self.yaw)

    @staticmethod
    def _gravity_angles(accel: tuple[float, float, float]) -> tuple[float, float]:
        ax, ay, az = (float(v) for v in accel)
        return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))
