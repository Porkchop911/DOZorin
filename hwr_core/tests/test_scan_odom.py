"""End-to-end scan odometry vs sim2d ground truth (plan 5.2 core logic).

The whole point of Phase 5: with slip, the open-loop model diverges from
truth; matcher + fusion must stay locked on. sim2d applies slip to
translation only (the gyro sees true rotation, like the real IMU), so the
model velocity fed to predict() is the post-slew command — exactly what
the robot will know at runtime.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from sim2d import LIDAR_MAX_RANGE, LIDAR_YAW_OFFSET, OccupancyMap, Sim2D  # noqa: E402

from hwr_core.matching import scan_to_points
from hwr_core.odometry import ScanOdometry

DT = 0.02
SCAN_EVERY = 10                     # 5 Hz matching


def sim_points(sim):
    angles, ranges = sim.scan()
    return scan_to_points(angles, ranges, LIDAR_MAX_RANGE,
                          sensor_yaw=LIDAR_YAW_OFFSET)


def run_leg(sim, odo, model, cmd, seconds, tick_holder):
    for _ in range(int(round(seconds / DT))):
        sim.step(cmd, DT)
        vx, vy, wz = sim.velocity          # post-slew command = model input
        odo.predict(vx, vy, wz, DT)
        # open-loop model for comparison (same midpoint integration)
        mx, my, mth = model
        mid = mth + 0.5 * wz * DT
        model[0] = mx + (vx * math.cos(mid) - vy * math.sin(mid)) * DT
        model[1] = my + (vx * math.sin(mid) + vy * math.cos(mid)) * DT
        model[2] = mth + wz * DT
        tick_holder[0] += 1
        if tick_holder[0] % SCAN_EVERY == 0:
            odo.process_scan(sim_points(sim))


def test_fused_beats_model_under_slip():
    sim = Sim2D(OccupancyMap.empty_room(4.0, 4.0), seed=3,
                range_noise_std=0.005, slip=0.15)
    odo = ScanOdometry()
    model = [0.0, 0.0, 0.0]
    tick = [0]
    odo.process_scan(sim_points(sim))                  # seed keyframe

    run_leg(sim, odo, model, (0.15, 0.0, 0.0), 6.0, tick)   # ~0.77 m fwd
    run_leg(sim, odo, model, (0.0, 0.12, 0.0), 5.0, tick)   # strafe left
    run_leg(sim, odo, model, (0.0, 0.0, 0.4), 3.0, tick)    # rotate ~65 deg
    run_leg(sim, odo, model, (0.12, 0.0, 0.0), 4.0, tick)   # fwd on new heading

    tx, ty, tth = sim.pose
    fx, fy, fth = odo.pose
    model_err = math.hypot(model[0] - tx, model[1] - ty)
    fused_err = math.hypot(fx - tx, fy - ty)

    # slip=0.15 over ~1.7 m commanded -> model is off by ~0.25 m
    assert model_err > 0.15
    assert fused_err < 0.05
    assert abs(math.atan2(math.sin(fth - tth), math.cos(fth - tth))) \
        < math.radians(3.0)
    assert fused_err < model_err / 3.0
    assert odo.keyframes >= 3                          # re-keyed while moving
    assert odo.fusion.accepted > odo.fusion.rejected


def test_stationary_stays_put():
    sim = Sim2D(OccupancyMap.empty_room(4.0, 4.0), seed=5,
                range_noise_std=0.005)
    odo = ScanOdometry()
    odo.process_scan(sim_points(sim))
    for k in range(200):                               # 4 s at standstill
        odo.predict(0.0, 0.0, 0.0, DT)
        if (k + 1) % SCAN_EVERY == 0:
            odo.process_scan(sim_points(sim))
    fx, fy, fth = odo.pose
    assert math.hypot(fx, fy) < 0.01
    assert abs(fth) < math.radians(0.5)
    assert odo.keyframes == 1                          # nothing forced a re-key


def test_rekey_on_rotation():
    sim = Sim2D(OccupancyMap.empty_room(4.0, 4.0), seed=7)
    odo = ScanOdometry()
    model = [0.0, 0.0, 0.0]
    tick = [0]
    odo.process_scan(sim_points(sim))
    run_leg(sim, odo, model, (0.0, 0.0, 0.5), 4.0, tick)    # ~110 deg in place
    assert odo.keyframes >= 2                          # 20 deg threshold hit
    tth = sim.pose[2]
    fth = odo.pose[2]
    assert abs(math.atan2(math.sin(fth - tth), math.cos(fth - tth))) \
        < math.radians(2.0)
