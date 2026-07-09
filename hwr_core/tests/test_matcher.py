"""Correlative scan matcher vs deterministic sim2d scans (plan 5.1).

Ground truth: sim poses are exact, so the expected matcher output is the
new pose expressed in the reference scan's base frame:
    dtheta = yaw2 - yaw1
    (dx, dy) = R(-yaw1) @ (xy2 - xy1)
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from sim2d import (  # noqa: E402
    LIDAR_MAX_RANGE, LIDAR_YAW_OFFSET, OccupancyMap, Sim2D,
)

from hwr_core.matching import CorrelativeMatcher, scan_to_points


def room(w=4.0, h=4.0):
    return OccupancyMap.empty_room(w, h)


def corridor(length=20.0, width=0.8, res=0.05):
    """Two parallel walls, ends beyond lidar range -> x unconstrained."""
    cols, rows = int(length / res), int(width / res)
    grid = np.zeros((rows, cols), dtype=bool)
    grid[0, :] = grid[-1, :] = True
    return OccupancyMap(grid=grid, resolution=res,
                        origin=(-length / 2.0, -width / 2.0))


def points_at(world, x, y, yaw, seed=0, noise=0.0):
    sim = Sim2D(world, x=x, y=y, yaw=yaw, seed=seed, range_noise_std=noise)
    angles, ranges = sim.scan()
    return scan_to_points(angles, ranges, LIDAR_MAX_RANGE,
                          sensor_yaw=LIDAR_YAW_OFFSET)


def expected_delta(p1, p2):
    dxw, dyw = p2[0] - p1[0], p2[1] - p1[1]
    c, s = math.cos(p1[2]), math.sin(p1[2])
    return (c * dxw + s * dyw,
            -s * dxw + c * dyw,
            math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2])))


def match_poses(world, p1, p2, prior=(0.0, 0.0, 0.0), noise=0.0):
    m = CorrelativeMatcher()
    m.set_reference(points_at(world, *p1, seed=1, noise=noise))
    return m.match(points_at(world, *p2, seed=2, noise=noise), prior=prior)


def assert_delta(result, expect, tol_xy=0.015, tol_th=math.radians(0.75)):
    assert result.valid
    assert abs(result.dx - expect[0]) <= tol_xy
    assert abs(result.dy - expect[1]) <= tol_xy
    assert abs(result.dtheta - expect[2]) <= tol_th


def test_zero_motion():
    p = (0.3, -0.2, 0.4)
    r = match_poses(room(), p, p)
    assert_delta(r, (0.0, 0.0, 0.0), tol_xy=0.008, tol_th=math.radians(0.3))
    assert r.score > 0.9
    assert not r.degenerate


def test_pure_translation():
    p1 = (0.0, 0.0, 0.0)
    p2 = (0.10, -0.04, 0.0)
    r = match_poses(room(), p1, p2)
    assert_delta(r, expected_delta(p1, p2))
    assert r.score > 0.8


def test_pure_rotation():
    p1 = (0.5, 0.2, 0.3)
    p2 = (0.5, 0.2, 0.3 + math.radians(8.0))
    r = match_poses(room(), p1, p2)
    assert_delta(r, expected_delta(p1, p2))


def test_combined_motion_with_noise():
    p1 = (-0.3, 0.4, 1.1)
    p2 = (-0.22, 0.45, 1.1 + math.radians(5.0))
    r = match_poses(room(), p1, p2, noise=0.01)
    assert_delta(r, expected_delta(p1, p2), tol_xy=0.02, tol_th=math.radians(1.0))


def test_heading_changes_delta_frame():
    # same world displacement, rotated start pose -> delta is in base frame
    p1 = (0.0, 0.0, math.pi / 2)
    p2 = (0.0, 0.10, math.pi / 2)   # world +y == base +x at yaw 90 deg
    r = match_poses(room(), p1, p2)
    assert_delta(r, (0.10, 0.0, 0.0))


def test_prior_extends_search_range():
    # 0.25 m jump is outside the +-0.15 m window; a rough prior recovers it
    p1 = (0.0, 0.0, 0.0)
    p2 = (0.25, 0.0, 0.0)
    blind = match_poses(room(), p1, p2)
    seeded = match_poses(room(), p1, p2, prior=(0.22, 0.0, 0.0))
    assert abs(blind.dx - 0.25) > 0.05          # honest failure without prior
    assert_delta(seeded, (0.25, 0.0, 0.0))
    assert seeded.score > blind.score


def test_corridor_is_degenerate_along_axis():
    world = corridor()
    p1 = (0.0, 0.0, 0.0)
    p2 = (0.05, 0.02, 0.0)
    r = match_poses(world, p1, p2)
    assert r.valid
    assert r.degenerate
    # cross-track (y) and heading stay constrained; along-track (x) does not
    assert abs(r.dy - 0.02) <= 0.015
    assert abs(r.dtheta) <= math.radians(1.0)
    assert math.sqrt(r.cov[0, 0]) > 3.0 * math.sqrt(r.cov[1, 1])


def test_square_room_is_not_degenerate():
    r = match_poses(room(), (0.2, 0.1, 0.0), (0.25, 0.12, 0.05))
    assert not r.degenerate
    assert math.sqrt(r.cov[0, 0]) < 0.03
    assert math.sqrt(r.cov[1, 1]) < 0.03


def test_mismatched_scene_scores_low():
    m = CorrelativeMatcher()
    m.set_reference(points_at(room(4.0, 4.0), 0.0, 0.0, 0.0))
    good = m.match(points_at(room(4.0, 4.0), 0.02, 0.0, 0.0))
    bad = m.match(points_at(corridor(), 0.0, 0.0, 0.0))
    assert good.score > 0.85
    assert bad.score < 0.6
    assert bad.score < good.score


def test_covariance_is_symmetric_psd():
    r = match_poses(room(), (0.0, 0.0, 0.0), (0.05, 0.05, 0.1))
    assert np.allclose(r.cov, r.cov.T)
    assert (np.linalg.eigvalsh(r.cov) > 0).all()


def test_too_few_points_is_invalid():
    m = CorrelativeMatcher()
    m.set_reference(np.zeros((3, 2)))
    r = m.match(np.zeros((3, 2)))
    assert not r.valid
    assert r.score == 0.0
    assert r.degenerate
