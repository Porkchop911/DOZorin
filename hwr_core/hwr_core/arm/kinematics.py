"""Geometric FK/IK for the practical pan + planar arm subset."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class JointSolution:
    pan: float
    shoulder: float
    elbow: float
    wrist: float


class ArmKinematics:
    def __init__(self, shoulder_height: float = 0.190785,
                 upper: float = 0.08513, forearm: float = 0.085147,
                 tool: float = 0.08, joint_limit: float = 2.09):
        self.shoulder_height = shoulder_height
        self.upper, self.forearm, self.tool = upper, forearm, tool
        self.joint_limit = joint_limit

    def fk(self, joints: JointSolution) -> tuple[float, float, float]:
        a, b, c = joints.shoulder, joints.elbow, joints.wrist
        radial = (self.upper * math.cos(a)
                  + self.forearm * math.cos(a + b)
                  + self.tool * math.cos(a + b + c))
        z = (self.shoulder_height + self.upper * math.sin(a)
             + self.forearm * math.sin(a + b)
             + self.tool * math.sin(a + b + c))
        return radial * math.cos(joints.pan), radial * math.sin(joints.pan), z

    def ik(self, x: float, y: float, z: float,
           tool_pitch: float = 0.0) -> JointSolution | None:
        if not all(math.isfinite(v) for v in (x, y, z, tool_pitch)):
            return None
        # Keep the tool out of the low central chassis/camera-cable volume.
        if math.hypot(x, y) < 0.10 and z < 0.20:
            return None
        pan = math.atan2(y, x)
        radial = math.hypot(x, y) - self.tool * math.cos(tool_pitch)
        vertical = z - self.shoulder_height - self.tool * math.sin(tool_pitch)
        cos_elbow = ((radial * radial + vertical * vertical
                      - self.upper ** 2 - self.forearm ** 2)
                     / (2 * self.upper * self.forearm))
        if not -1.0 <= cos_elbow <= 1.0:
            return None
        elbow = -math.acos(cos_elbow)
        shoulder = (math.atan2(vertical, radial)
                    - math.atan2(self.forearm * math.sin(elbow),
                                 self.upper + self.forearm * math.cos(elbow)))
        wrist = tool_pitch - shoulder - elbow
        solution = JointSolution(pan, shoulder, elbow, wrist)
        if any(abs(value) > self.joint_limit for value in solution.__dict__.values()):
            return None
        return solution

    @staticmethod
    def radians_to_pulse(angle: float, center: int = 500) -> int:
        pulse = round(center + math.degrees(angle) / 0.24)
        if not 0 <= pulse <= 1000:
            raise ValueError("joint angle is outside servo pulse range")
        return pulse


class CameraAim:
    """Conservative camera bearing/elevation to bus-servo pulse mapping."""

    def __init__(self, pan_center: int = 500, tilt_forward: int = 300,
                 max_bearing: float = math.radians(60.0),
                 min_elevation: float = math.radians(-50.0),
                 max_elevation: float = math.radians(48.0)):
        self.pan_center = pan_center
        self.tilt_forward = tilt_forward
        self.max_bearing = max_bearing
        self.min_elevation = min_elevation
        self.max_elevation = max_elevation

    def pulses(self, bearing: float, elevation: float) -> tuple[int, int]:
        if not all(math.isfinite(v) for v in (bearing, elevation)):
            raise ValueError("camera aim must be finite")
        if abs(bearing) > self.max_bearing:
            raise ValueError("bearing outside conservative camera envelope")
        if not self.min_elevation <= elevation <= self.max_elevation:
            raise ValueError("elevation outside conservative camera envelope")
        scale = 1.0 / math.radians(0.24)
        pan = round(self.pan_center + bearing * scale)
        tilt = round(self.tilt_forward + elevation * scale)
        if not (0 <= pan <= 1000 and 0 <= tilt <= 1000):
            raise ValueError("camera aim outside servo pulse range")
        return pan, tilt

    def sweep(self, views: int, elevation: float = 0.0) -> list[tuple[int, int]]:
        if views < 1:
            raise ValueError("views must be positive")
        bearings = ([0.0] if views == 1 else
                    np.linspace(-self.max_bearing, self.max_bearing, views))
        return [self.pulses(float(bearing), elevation) for bearing in bearings]
