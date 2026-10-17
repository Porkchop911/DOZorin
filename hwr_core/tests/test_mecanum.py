import math
import random

import pytest

from hwr_core.kinematics import (Calibration, K, WHEEL_DIAMETER, WIRE_SIGNS,
                                 body_velocity, rps_to_urdf, scale_to_limit, urdf_to_rps,
                                 wheel_speeds)

CIRC = math.pi * WHEEL_DIAMETER


def test_pure_forward_all_wheels_equal_magnitude():
    rps = wheel_speeds(0.2, 0.0, 0.0)
    expected = 0.2 / CIRC
    assert rps[0] == pytest.approx(expected)
    assert rps[1] == pytest.approx(expected)
    # right side wire-inverted (bench-verified)
    assert rps[2] == pytest.approx(-expected)
    assert rps[3] == pytest.approx(-expected)


def test_pure_strafe_left_diagonal_pattern():
    m1, m2, m3, m4 = wheel_speeds(0.0, 0.2, 0.0)
    v = 0.2 / CIRC
    assert m1 == pytest.approx(-v)   # FL backward
    assert m2 == pytest.approx(v)    # RL forward
    assert m3 == pytest.approx(-v)   # FR: +m, wire-inverted
    assert m4 == pytest.approx(v)    # RR: -m, wire-inverted


def test_pure_rotation_ccw_left_back_right_fwd():
    m1, m2, m3, m4 = wheel_speeds(0.0, 0.0, 0.5)
    v = 0.5 * K / CIRC
    assert m1 == pytest.approx(-v) and m2 == pytest.approx(-v)   # left backward
    assert m3 == pytest.approx(-v) and m4 == pytest.approx(-v)   # right forward on wire


def test_roundtrip_identity():
    rng = random.Random(1)
    for _ in range(500):
        v = (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-1, 1))
        out = body_velocity(wheel_speeds(*v))
        assert out == pytest.approx(v, abs=1e-12)


def test_zero_is_zero():
    assert wheel_speeds(0, 0, 0) == (0, 0, 0, 0)
    assert body_velocity((0, 0, 0, 0)) == (0, 0, 0)


def test_calibration_scales_axes_independently():
    cal = Calibration(linear=1.1, strafe=0.9, angular=1.05)
    assert wheel_speeds(0.1, 0, 0, cal) == pytest.approx(wheel_speeds(0.11, 0, 0))
    assert wheel_speeds(0, 0.1, 0, cal) == pytest.approx(wheel_speeds(0, 0.09, 0))
    assert wheel_speeds(0, 0, 0.4, cal) == pytest.approx(wheel_speeds(0, 0, 0.42))


def test_scale_to_limit_preserves_direction():
    rps = wheel_speeds(0.2, 0.2, 0.5)
    limited = scale_to_limit(rps, 1.0)
    assert max(abs(r) for r in limited) == pytest.approx(1.0)
    # same body-motion direction, smaller magnitude
    v_full = body_velocity(rps)
    v_lim = body_velocity(limited)
    ratios = [a / b for a, b in zip(v_lim, v_full)]
    assert all(r == pytest.approx(ratios[0]) for r in ratios)
    assert 0 < ratios[0] < 1


def test_scale_to_limit_noop_within_limit():
    rps = wheel_speeds(0.05, 0, 0)
    assert scale_to_limit(rps, 4.0) == rps


def test_calibration_yaml_roundtrip(tmp_path):
    p = tmp_path / "cal.yaml"
    p.write_text("linear: 1.02\nstrafe: 0.97\nangular: 1.1\n")
    cal = Calibration.load(p)
    assert (cal.linear, cal.strafe, cal.angular) == (1.02, 0.97, 1.1)


def test_urdf_wheels_all_spin_forward_for_forward_motion():
    q_dot = rps_to_urdf(wheel_speeds(0.2, 0.0, 0.0))
    assert all(v == pytest.approx(0.2 / (WHEEL_DIAMETER / 2)) for v in q_dot.values())
    rps = wheel_speeds(0.1, -0.05, 0.3)
    assert urdf_to_rps(rps_to_urdf(rps)) == pytest.approx(rps)
