"""Mecanum wheel kinematics for the ROSOrin Pro.

Constants and mixing verified against the vendor instantiation values
(`plan/90-clean-room-reference.md` §Mecanum) and the Phase 1 bench session:
wheel map m1=front-left, m2=rear-left, m3=front-right, m4=rear-right, with
the right-side motors mounted mirrored -> wire signs [+, +, -, -].

Body frame: x forward, y left, z up (REP-103). Positive wz = CCW.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

WHEELBASE = 0.17706   # m, front-rear axle distance
TRACK = 0.17165       # m, left-right wheel distance
WHEEL_DIAMETER = 0.08  # m
K = (WHEELBASE + TRACK) / 2.0
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
WIRE_SIGNS = (1.0, 1.0, -1.0, -1.0)  # m1..m4; right side mirrored


@dataclass
class Calibration:
    """Scale factors derived on the floor (task 2.5). 1.0 = uncalibrated."""
    linear: float = 1.0
    strafe: float = 1.0
    angular: float = 1.0

    @classmethod
    def load(cls, path) -> "Calibration":
        import yaml  # only needed when loading from file
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        return cls(**{k: float(raw[k]) for k in ("linear", "strafe", "angular") if k in raw})


def wheel_speeds(vx: float, vy: float, wz: float,
                 cal: Calibration | None = None) -> tuple[float, float, float, float]:
    """Body velocity (m/s, m/s, rad/s) -> wire wheel speeds in rev/s (m1..m4)."""
    if cal is not None:
        vx, vy, wz = vx * cal.linear, vy * cal.strafe, wz * cal.angular
    m1 = vx - vy - wz * K
    m2 = vx + vy - wz * K
    m3 = vx + vy + wz * K
    m4 = vx - vy + wz * K
    return tuple(s * m / WHEEL_CIRCUMFERENCE
                 for s, m in zip(WIRE_SIGNS, (m1, m2, m3, m4)))


def body_velocity(rps: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Wire wheel speeds (rev/s, m1..m4) -> body velocity. Exact inverse of
    ``wheel_speeds`` (the mecanum mixing matrix is square-invertible here)."""
    m1, m2, m3, m4 = (s * r * WHEEL_CIRCUMFERENCE for s, r in zip(WIRE_SIGNS, rps))
    vx = (m1 + m2 + m3 + m4) / 4.0
    vy = (-m1 + m2 + m3 - m4) / 4.0
    wz = (-m1 - m2 + m3 + m4) / (4.0 * K)
    return vx, vy, wz


URDF_WHEELS = ("wheel_fl_joint", "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint")  # m1..m4


def rps_to_urdf(rps: tuple[float, float, float, float]) -> dict:
    """Wire rev/s (m1..m4) -> sim wheel joint rad/s; every URDF wheel spins +y = forward."""
    return {name: 2 * math.pi * s * r for name, s, r in zip(URDF_WHEELS, WIRE_SIGNS, rps)}


def urdf_to_rps(q_dot: dict) -> tuple[float, float, float, float]:
    return tuple(s * q_dot[name] / (2 * math.pi) for name, s in zip(URDF_WHEELS, WIRE_SIGNS))


def scale_to_limit(rps: tuple[float, float, float, float],
                   max_rps: float) -> tuple[float, float, float, float]:
    """Uniformly scale wheel speeds so none exceeds max_rps (direction preserved)."""
    peak = max(abs(r) for r in rps)
    if peak <= max_rps or peak == 0.0:
        return rps
    f = max_rps / peak
    return tuple(r * f for r in rps)
