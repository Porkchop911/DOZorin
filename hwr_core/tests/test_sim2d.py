import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from sim2d import LIDAR_MAX_RANGE, OccupancyMap, Sim2D  # noqa: E402


def room(w=4.0, h=4.0):
    return OccupancyMap.empty_room(w, h)


def drive(sim, cmd, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        sim.step(cmd, dt)


def test_forward_motion():
    sim = Sim2D(room())
    drive(sim, (0.1, 0.0, 0.0), 10.0)
    # accel ramp eats ~0.01 m at the start; expect just under 1 m
    assert 0.95 <= sim.x <= 1.001
    assert abs(sim.y) < 1e-9
    assert not sim.collided


def test_strafe_and_rotation():
    sim = Sim2D(room())
    drive(sim, (0.0, 0.1, 0.0), 5.0)
    assert 0.45 <= sim.y <= 0.501 and abs(sim.x) < 1e-9
    sim2 = Sim2D(room())
    drive(sim2, (0.0, 0.0, 0.5), 2.0)
    assert 0.85 <= sim2.yaw <= 1.001  # ~1 rad minus ramp


def test_heading_affects_world_motion():
    sim = Sim2D(room())
    sim.yaw = math.pi / 2
    drive(sim, (0.1, 0.0, 0.0), 5.0)
    assert sim.y > 0.4 and abs(sim.x) < 0.02


def test_wall_collision_stops_robot():
    sim = Sim2D(room(2.0, 2.0))
    drive(sim, (0.2, 0.0, 0.0), 10.0)
    assert sim.collided
    assert sim.x < 1.0  # never passes the wall
    assert sim.velocity == (0.0, 0.0, 0.0)


def test_scan_sees_walls_at_expected_distance():
    sim = Sim2D(room(4.0, 4.0))  # robot at center, walls at +-2 m
    angles, ranges = sim.scan()
    # lidar frame angle a hits world angle yaw+pi+a; every wall within
    # [sqrt(2)*2] distance; all beams must terminate on a wall (closed room)
    assert (ranges < LIDAR_MAX_RANGE).all()
    assert ranges.min() >= 1.9  # nearest wall ~2 m
    assert ranges.max() <= 2.0 * math.sqrt(2) + 0.1
    # beam pointing at world 0: lidar angle a = -pi (i.e. pi) -> in band?
    idx = np.argmin(np.abs((sim.yaw + math.pi + angles) % (2 * math.pi) - 0))
    assert ranges[idx] == pytest.approx(2.0, abs=0.1)


def test_scan_points_world_land_on_walls():
    sim = Sim2D(room(4.0, 4.0), x=0.5, y=-0.3, yaw=0.7)
    pts = sim.scan_points_world()
    assert len(pts) > 300
    on_wall = (np.isclose(np.abs(pts[:, 0]), 2.0, atol=0.06) |
               np.isclose(np.abs(pts[:, 1]), 2.0, atol=0.06))
    assert on_wall.mean() > 0.95


def test_determinism_with_seed():
    a = Sim2D(room(), seed=7, range_noise_std=0.01)
    b = Sim2D(room(), seed=7, range_noise_std=0.01)
    drive(a, (0.1, 0.05, 0.2), 3.0)
    drive(b, (0.1, 0.05, 0.2), 3.0)
    assert a.pose == b.pose
    assert np.array_equal(a.scan()[1], b.scan()[1])


def test_slip_reduces_distance():
    clean = Sim2D(room())
    slippy = Sim2D(room(), slip=0.1)
    drive(clean, (0.1, 0.0, 0.0), 8.0)
    drive(slippy, (0.1, 0.0, 0.0), 8.0)
    assert slippy.x == pytest.approx(clean.x * 0.9, rel=0.02)
