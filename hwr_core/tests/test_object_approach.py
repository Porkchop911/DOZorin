import math

import numpy as np
import pytest

from hwr_core.navigation import (closest_point_on_segment,
                                 object_approach_candidates,
                                 point_to_segment_distance,
                                 project_depth_pixel, staged_path_waypoint,
                                 staged_yaw_waypoints)


def test_closest_point_and_distance_handle_extended_target():
    assert closest_point_on_segment((0.5, 1.0), (0.0, 0.0), (1.0, 0.0)) == (
        0.5, 0.0)
    assert point_to_segment_distance(
        (2.0, 0.3), (0.0, 0.0), (1.0, 0.0)) == pytest.approx(
            math.hypot(1.0, 0.3))


def test_point_target_candidates_are_reachable_close_and_face_object():
    grid = np.zeros((80, 80), dtype=np.uint8)
    candidates = object_approach_candidates(
        grid, 0.05, (-2.0, -2.0), (-1.0, 0.0), (1.0, 0.0),
        min_distance=0.35, max_distance=0.48, limit=10)
    assert len(candidates) == 10
    for candidate in candidates:
        assert 0.35 - 1e-9 <= candidate.object_distance_m <= 0.48 + 1e-9
        expected = math.atan2(-candidate.y, 1.0 - candidate.x)
        assert candidate.yaw == pytest.approx(expected)
        assert candidate.path_length_m > 0.0


def test_segment_target_can_approach_an_accessible_end():
    grid = np.zeros((100, 100), dtype=np.uint8)
    # Occupied wall on the lower side of the segment and around its middle;
    # only the right end has useful free approach cells.
    origin = (-2.0, -2.0)
    resolution = 0.05
    grid[35:50, 35:61] = 255
    candidates = object_approach_candidates(
        grid, resolution, origin, (-1.0, 1.0), (-0.25, 0.0), (1.0, 0.0),
        min_distance=0.30, max_distance=0.45, limit=20)
    assert candidates
    assert any(candidate.object_x > 0.65 for candidate in candidates)
    assert all(point_to_segment_distance(
        (candidate.x, candidate.y), (-0.25, 0.0), (1.0, 0.0)) <= 0.45 + 1e-9
               for candidate in candidates)


def test_blocked_or_unknown_cells_are_not_candidates():
    grid = np.full((40, 40), 180, dtype=np.uint8)
    assert object_approach_candidates(
        grid, 0.05, (-1.0, -1.0), (0.0, 0.8), (0.0, 0.0)) == []


@pytest.mark.parametrize("minimum,maximum", [
    (0.0, 0.4), (0.4, 0.3), (0.3, 0.5001),
])
def test_arrival_envelope_never_exceeds_fifty_centimetres(minimum, maximum):
    with pytest.raises(ValueError, match="0.50"):
        object_approach_candidates(
            np.zeros((10, 10)), 0.1, (0.0, 0.0), (0.1, 0.1),
            (0.5, 0.5), min_distance=minimum, max_distance=maximum)


def test_candidates_are_deterministic_and_clearance_ranked():
    grid = np.zeros((80, 80), dtype=np.uint8)
    grid[20:60, 20] = 255
    kwargs = dict(min_distance=0.30, max_distance=0.45, limit=20)
    first = object_approach_candidates(
        grid, 0.05, (-2.0, -2.0), (1.0, 1.0), (0.0, 0.0), **kwargs)
    second = object_approach_candidates(
        grid, 0.05, (-2.0, -2.0), (1.0, 1.0), (0.0, 0.0), **kwargs)
    assert first == second
    assert [item.clearance_m for item in first] == sorted(
        (item.clearance_m for item in first), reverse=True)


def test_depth_pixel_projection_uses_ros_optical_convention():
    transform = np.array([
        [0.0, 0.0, 1.0, 1.0],
        [-1.0, 0.0, 0.0, 2.0],
        [0.0, -1.0, 0.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ])
    # Pixel is 10 px right and 20 px down from principal point at 2 m.
    point = project_depth_pixel(
        (60, 70), 2.0, (100, 100, 50, 50), transform)
    assert point == pytest.approx((3.0, 1.8, 0.1))


def test_depth_projection_rejects_bad_calibration():
    with pytest.raises(ValueError, match="positive"):
        project_depth_pixel((1, 1), 1.0, (0, 1, 0, 0), np.eye(4))


def test_staged_waypoint_follows_astar_around_obstacle():
    grid = np.zeros((30, 30), dtype=np.uint8)
    grid[8:23, 14] = 255
    origin = (-1.5, -1.5)
    start, goal = (-1.0, 0.0), (1.0, 0.0)
    waypoint = staged_path_waypoint(
        grid, 0.1, origin, start, goal, max_leg=0.50)
    assert math.hypot(waypoint[0] - start[0], waypoint[1] - start[1]) <= 0.51
    # The selected cell is on the detour; a straight interpolation helper would
    # remain at y=0 and eventually drive into the wall.
    assert waypoint[1] != pytest.approx(0.0)


def test_staged_waypoint_returns_exact_goal_on_final_leg():
    grid = np.zeros((20, 20), dtype=np.uint8)
    goal = (0.37, 0.22)
    assert staged_path_waypoint(
        grid, 0.1, (-1.0, -1.0), (0.0, 0.0), goal,
        max_leg=0.50) == goal


def test_staged_waypoint_measures_leg_from_actual_pose_near_cell_edge():
    grid = np.zeros((40, 40), dtype=np.uint8)
    start = (0.3999, 0.031)
    waypoint = staged_path_waypoint(
        grid, 0.05, (0.0, 0.0), start, (1.5, 1.5), max_leg=0.10)
    displacement = math.hypot(
        waypoint[0] - start[0], waypoint[1] - start[1])
    # The path arclength is exactly 0.10 m; a bend can make endpoint
    # displacement slightly shorter, but it must not collapse to a micro-goal.
    assert 0.09 <= displacement <= 0.10 + 1e-9


def test_staged_waypoint_fails_closed_without_path():
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[:, 10] = 255
    with pytest.raises(ValueError, match="no collision-free path"):
        staged_path_waypoint(
            grid, 0.1, (-1.0, -1.0), (-0.5, 0.0), (0.5, 0.0))


def test_staged_yaw_waypoints_are_wrapped_and_bounded():
    points = staged_yaw_waypoints(
        math.radians(170.0), math.radians(-100.0),
        max_step=math.radians(30.0))
    assert len(points) == 3
    previous = math.radians(170.0)
    for point in points:
        step = math.atan2(math.sin(point - previous), math.cos(point - previous))
        assert 0.0 < step <= math.radians(30.0) + 1e-9
        previous = point
    assert points[-1] == pytest.approx(math.radians(-100.0))
