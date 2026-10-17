import math

import pytest

from hwr_core.navigation.retrace import clamp_reverse, rear_blocked, retrace_budget


def straight(n, yaw=0.0, step=0.02):
    return [(i * step * math.cos(yaw), i * step * math.sin(yaw), yaw) for i in range(n)]


def test_retrace_budget_is_the_straight_path_just_driven_capped():
    trail = straight(5)                                  # 0.08 m driven forward
    assert retrace_budget(trail, trail[-1]) == pytest.approx(0.08)
    trail = straight(20, yaw=1.0)                        # 0.38 m driven: capped
    assert retrace_budget(trail, trail[-1]) == pytest.approx(0.15)


def test_retrace_budget_ends_at_a_turn_or_strafe():
    trail = straight(10)
    turned = (trail[-1][0], trail[-1][1], math.radians(30))
    assert retrace_budget(trail, turned) == 0.0          # turned in place since
    strafed = trail + [(0.18, 0.05, 0.0)]
    assert retrace_budget(strafed, strafed[-1]) == 0.0   # the line behind is not the path
    bent = [(0.0, 0.08, 0.0), (0.04, 0.04, 0.0)] + [(0.06 + i * 0.02, 0.0, 0.0) for i in range(4)]
    assert retrace_budget(bent, bent[-1]) == pytest.approx(0.06)  # only the straight part


def test_rear_blocked_reports_returns_in_the_corridor_only():
    angles = [math.pi, math.radians(150), math.radians(90)]
    assert rear_blocked(angles, [0.30, math.inf, 0.20], 0.15) == (-0.3, 0.0)
    assert rear_blocked(angles, [0.70, 1.0, 0.20], 0.15) is None      # beyond 0.57, beside
    assert rear_blocked([math.radians(150)], [0.30], 0.15) == (-0.26, 0.15)  # rear corner
    assert rear_blocked([math.radians(140)], [0.35], 0.15) is None     # y 0.225: beside the corridor


def test_clamp_reverse_allows_only_slow_straight_back():
    assert clamp_reverse(-0.2, 0.0, 0.0) == (-0.06, 0.0, 0.0)
    assert clamp_reverse(-0.03, 0.0, 0.1) == (0.0, 0.0, 0.0)
    assert clamp_reverse(0.1, 0.0, 0.0) == (0.0, 0.0, 0.0)
    assert clamp_reverse(float("nan"), 0.0, 0.0) == (0.0, 0.0, 0.0)
