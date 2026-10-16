import math

import pytest

from hwr_core.arm import ArmKinematics, pulses_to_joints
from hwr_core.arm.manipulation import follow_step, locate_cube, plan_grasp, wrap_quarter

LOOK_DOWN = {1: 500, 2: 701, 3: 120, 4: 50, 5: 500}
K = (500.0, 500.0, 320.0, 200.0)


def test_wrap_quarter():
    assert wrap_quarter(math.radians(100)) == pytest.approx(math.radians(10))
    assert wrap_quarter(math.radians(-50)) == pytest.approx(math.radians(40))


def test_locate_then_grasp_lands_on_the_cube():
    arm = ArmKinematics()
    joints = pulses_to_joints(LOOK_DOWN)
    item = {"center": [320, 260], "corners": [[300, 240], [340, 240], [340, 280], [300, 280]]}
    cube = locate_cube(item, K, joints, arm, 0.015)
    assert 0.13 < cube["x"] < 0.30 and abs(cube["y"]) < 0.01
    assert abs(cube["yaw"]) < math.radians(5)
    hover, grasp, pitch = plan_grasp(arm, cube["x"], cube["y"], 0.015, cube["yaw"])
    tip = arm.fk(pulses_to_joints(grasp))
    assert tip[:3] == pytest.approx((cube["x"], cube["y"], 0.015), abs=0.005)
    assert arm.fk(pulses_to_joints(hover))[2] == pytest.approx(0.065, abs=0.005)


def test_grasp_out_of_reach_is_none():
    assert plan_grasp(ArmKinematics(), 0.45, 0.0, 0.015, 0.0) is None


def test_follow_turns_toward_target_and_drives_only_when_aligned():
    b, e, vx, wz = follow_step((520, 200), K, 0.0, 0.0, 2.0, 1.0)
    assert b < 0 and wz < 0 and vx == 0.0  # target right, 22 deg off: turn, no drive
    b, e, vx, wz = follow_step((330, 150), K, 0.0, 0.0, 2.0, 1.0)
    assert e > 0 and vx > 0
    assert follow_step((320, 200), K, 0.0, 0.0, 0.8, 1.0)[2] == 0.0  # close enough: no reverse


def test_place_on_one_cube_uses_a_pitch_both_points_reach():
    # Live 2026-10-16: grasp reachable at -90 but hover only at -85 -> was "out of reach".
    plan = plan_grasp(ArmKinematics(), 0.2091, -0.0368, 0.05, -0.2)
    assert plan is not None and plan[2] > -math.pi / 2
