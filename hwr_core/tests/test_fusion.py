"""Odometry fusion EKF — scripted scenarios (plan 5.3).

The plan's named scenario: slip — model says moving, scan says stuck ->
fused follows the scan. Plus bias correction, degenerate-axis weighting,
gating, wrap handling, covariance hygiene.
"""
import math

import numpy as np
import pytest

from hwr_core.odometry import FusionConfig, OdomFusion

DT = 0.02                            # 50 Hz predict
SCAN_COV = np.diag([0.005 ** 2, 0.005 ** 2, math.radians(0.25) ** 2])


def predict_for(f, vx, vy, wz, seconds):
    for _ in range(int(round(seconds / DT))):
        f.predict(vx, vy, wz, DT)


def test_dead_reckoning_straight():
    f = OdomFusion()
    predict_for(f, 0.1, 0.0, 0.0, 10.0)
    assert f.pose[0] == pytest.approx(1.0, abs=1e-9)
    assert f.pose[1] == 0.0
    # no absolute reference yet -> uncertainty must be honest (~0.3 m sigma)
    assert 0.05 < f.P[0, 0] < 0.2


def test_dead_reckoning_integrates_vy():
    # the vendor odometry drops vy; ours must not
    f = OdomFusion()
    predict_for(f, 0.0, 0.1, 0.0, 5.0)
    assert f.pose[1] == pytest.approx(0.5, abs=1e-9)
    assert f.pose[0] == 0.0


def test_dead_reckoning_heading():
    f = OdomFusion()
    predict_for(f, 0.0, 0.0, math.pi / 2, 1.0)    # rotate 90 deg on gyro
    predict_for(f, 0.1, 0.0, 0.0, 1.0)            # then drive "forward"
    assert f.pose[2] == pytest.approx(math.pi / 2, abs=1e-6)
    assert f.pose[1] == pytest.approx(0.1, abs=1e-6)
    assert abs(f.pose[0]) < 1e-6


def test_scan_corrects_model_bias():
    # model claims 0.1 m/s, truth is 0.08 -> scan updates pull to truth
    f = OdomFusion()
    truth_x = 0.0
    keyframe = (0.0, 0.0, 0.0)
    for k in range(250):                          # 5 s @ 50 Hz
        f.predict(0.1, 0.0, 0.0, DT)
        truth_x += 0.08 * DT
        if (k + 1) % 10 == 0:                     # 5 Hz scan matching
            assert f.update_scan(keyframe, (truth_x, 0.0, 0.0), SCAN_COV, 0.9)
    assert f.pose[0] == pytest.approx(0.4, abs=0.01)


def test_slip_scan_says_stuck():
    # the plan's scenario: commanded forward, robot against a wall
    f = OdomFusion()
    keyframe = (0.0, 0.0, 0.0)
    for k in range(250):
        f.predict(0.1, 0.0, 0.0, DT)
        if (k + 1) % 10 == 0:
            f.update_scan(keyframe, (0.0, 0.0, 0.0), SCAN_COV, 0.9)
    assert f.accepted == 25
    assert abs(f.pose[0]) < 0.03                  # model alone would say 0.5


def test_degenerate_axis_follows_model():
    # corridor: matcher x-variance huge -> fused x keeps the model,
    # y snaps to the measurement
    f = OdomFusion()
    predict_for(f, 0.1, 0.0, 0.0, 2.0)            # x = 0.2, P_x ~ 0.018
    corridor_cov = np.diag([0.5 ** 2, 0.01 ** 2, math.radians(0.5) ** 2])
    assert f.update_scan((0.0, 0.0, 0.0), (0.0, 0.05, 0.0), corridor_cov, 0.9)
    assert f.pose[0] > 0.15                       # x barely moved toward 0
    assert f.pose[1] == pytest.approx(0.05, abs=0.005)


def test_low_score_rejected():
    f = OdomFusion()
    predict_for(f, 0.1, 0.0, 0.0, 1.0)
    before = f.pose
    assert not f.update_scan((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), SCAN_COV, 0.3)
    assert f.pose == before
    assert f.rejected == 1 and f.accepted == 0


def test_mahalanobis_gate_rejects_wild_jump():
    f = OdomFusion()
    predict_for(f, 0.0, 0.0, 0.0, 1.0)            # standstill: P tiny
    before = f.pose
    assert not f.update_scan((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), SCAN_COV, 0.9)
    assert f.pose == before
    assert f.rejected == 1


def test_theta_wraps_across_pi():
    f = OdomFusion()
    f.x = np.array([0.0, 0.0, math.pi - 0.05])
    f.P = np.diag([0.01, 0.01, 0.01])
    keyframe = (0.0, 0.0, math.pi - 0.05)
    assert f.update_scan(keyframe, (0.0, 0.0, 0.1), SCAN_COV, 0.9)
    assert f.pose[2] == pytest.approx(-math.pi + 0.05, abs=0.01)
    assert abs(f.pose[2]) <= math.pi


def test_covariance_hygiene():
    f = OdomFusion()
    predict_for(f, 0.1, 0.05, 0.2, 2.0)
    grown = f.P.copy()
    assert np.trace(grown) > np.trace(np.diag([1e-6] * 3))
    assert f.update_scan((0.0, 0.0, 0.0), f.pose, SCAN_COV, 0.9)
    assert np.trace(f.P) < np.trace(grown)        # update shrinks
    assert np.allclose(f.P, f.P.T)
    assert (np.linalg.eigvalsh(f.P) > 0).all()


def test_matcher_result_plugs_in():
    # integration seam: a MatchResult's fields feed update_scan directly
    from hwr_core.matching import MatchResult
    r = MatchResult(0.02, -0.01, 0.005, score=0.92,
                    cov=SCAN_COV.copy(), degenerate=False)
    f = OdomFusion()
    predict_for(f, 0.1, 0.0, 0.0, 0.5)
    assert f.update_scan((0.0, 0.0, 0.0), r.delta, r.cov, r.score)
