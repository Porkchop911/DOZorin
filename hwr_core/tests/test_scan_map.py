import math

import numpy as np
import pytest

from hwr_core.mapping import (LogOddsGrid, ScanMapMapper, compose, inverse,
                              mapping_scan_to_points)


def _landmarks():
    # Asymmetric geometry avoids the ambiguity of a bare square room.
    a = np.column_stack((np.linspace(-1.5, 1.5, 80), np.full(80, 1.5)))
    b = np.column_stack((np.full(60, -1.2), np.linspace(-1.2, 1.5, 60)))
    c = np.column_stack((np.linspace(-0.4, 1.3, 50), np.full(50, -0.9)))
    return np.vstack((a, b, c))


def _scan(world_points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (world_points - (x, y)) @ np.array(((c, -s), (s, c)))


def test_se2_compose_inverse_roundtrip():
    pose = (1.2, -0.7, 0.8)
    assert np.allclose(compose(pose, inverse(pose)), (0.0, 0.0, 0.0), atol=1e-12)


def test_stationary_scans_do_not_overintegrate():
    mapper = ScanMapMapper(LogOddsGrid(resolution=0.05, size=100))
    points = _scan(_landmarks(), (0.0, 0.0, 0.0))
    first = mapper.process((0.0, 0.0, 0.0), points)
    before = mapper.grid.log_odds.copy()
    second = mapper.process((0.0, 0.0, 0.0), points)
    assert first.integrated
    assert not second.integrated
    assert np.array_equal(before, mapper.grid.log_odds)


def test_mapping_range_cap_omits_long_glass_pass_through_hits():
    points = mapping_scan_to_points(
        np.array([0.0, math.pi / 2.0, math.pi]),
        np.array([3.5, 4.5, 9.0]), sensor_max_range=10.0,
        mapping_max_range=4.0)
    assert points.shape == (1, 2)
    assert np.allclose(points[0], (3.5, 0.0))
    with pytest.raises(ValueError):
        mapping_scan_to_points([0.0], [1.0], 10.0, 0.0)


def test_scan_to_map_corrects_biased_odometry():
    world = _landmarks()
    mapper = ScanMapMapper(LogOddsGrid(resolution=0.03, size=160),
                           keyframe_dist=0.05, minimum_score=0.35)
    mapper.process((0.0, 0.0, 0.0), _scan(world, (0.0, 0.0, 0.0)))
    truth = (0.12, -0.04, math.radians(3.0))
    biased_odom = (0.18, -0.08, math.radians(5.0))
    update = mapper.process(biased_odom, _scan(world, truth))
    assert update.integrated and update.corrected
    assert update.score > 0.6
    assert np.allclose(update.pose[:2], truth[:2], atol=0.025)
    assert abs(update.pose[2] - truth[2]) < math.radians(1.0)
    assert np.allclose(compose(update.map_to_odom, biased_odom), update.pose)
