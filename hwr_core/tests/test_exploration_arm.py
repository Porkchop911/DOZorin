import math

import numpy as np
import pytest

from hwr_core.arm import (ArmKinematics, CameraAim, JointSolution, joints_to_pulses,
                          pulses_to_joints)
from hwr_core.exploration import (find_frontiers,
                                  restore_semantic_boundary_cooldowns)
from hwr_core.navigation import astar


def test_frontiers_cluster_and_rank():
    grid = np.full((20, 20), -1, dtype=np.int8)
    grid[5:15, 5:15] = 0
    frontiers = find_frontiers(grid, 0.1, (0, 0), (1, 1), min_cells=3)
    assert frontiers
    assert sum(len(frontier.cells) for frontier in frontiers) >= 30
    assert frontiers == sorted(frontiers, key=lambda item: item.score, reverse=True)


def test_no_frontier_in_known_map():
    assert find_frontiers(np.zeros((10, 10), dtype=np.int8), 0.1,
                          (0, 0), (0, 0)) == []


def test_glass_boundary_cooldowns_restore_with_remaining_duration():
    record = {
        "time": 90.0,
        "event": "semantic_preflight_rejected",
        "frontier_goal": [4.0, 5.0],
        "waypoint": [2.0, 1.0],
        "report": {
            "assessment": {"glass_boundary_in_corridor": True},
            "navigation_context": {"pose": [1.0, 1.0, 0.0]},
        },
    }
    expired = {**record, "time": -1000.0}
    goals, bearings = restore_semantic_boundary_cooldowns(
        [expired, record], 30.0, now_wall=100.0, now_monotonic=50.0)
    assert goals == [((4.0, 5.0), 70.0)]
    assert bearings == [(0.0, 70.0)]


def test_frontier_goals_are_set_back_free_and_reachable():
    grid = np.full((30, 30), -1, dtype=np.int8)
    grid[5:25, 5:25] = 0
    robot = (1.5, 1.5)
    frontiers = find_frontiers(
        grid, 0.1, (0.0, 0.0), robot, min_cells=3, setback_m=0.30)
    assert frontiers
    traversal = np.where(grid == 0, 0, 255).astype(np.uint8)
    for frontier in frontiers:
        x, y = frontier.goal_cell
        assert grid[y, x] == 0
        assert not np.any(grid[y-1:y+2, x-1:x+2] < 0)
        assert astar(traversal, (15, 15), frontier.goal_cell)
        assert frontier.path_length_m >= 0.0
        assert frontier.clearance_m > 0.0
        assert frontier.information_gain == len(frontier.cells)


def test_frontier_ranking_obeys_inflated_costmap_and_exclusions():
    grid = np.full((40, 40), -1, dtype=np.int8)
    grid[5:35, 5:35] = 0
    costs = np.where(grid == 0, 0, 255).astype(np.uint8)
    costs[5:35, 18:22] = 200
    costs[19:22, 18:22] = 0
    frontiers = find_frontiers(
        grid, 0.1, (0.0, 0.0), (1.0, 2.0), min_cells=3,
        costmap=costs, setback_m=0.20)
    assert frontiers
    selected = frontiers[0]
    assert costs[selected.goal_cell[1], selected.goal_cell[0]] <= 64
    assert selected.path_length_m > 0.0

    reranked = find_frontiers(
        grid, 0.1, (0.0, 0.0), (1.0, 2.0), min_cells=3,
        costmap=costs, setback_m=0.20,
        excluded_goals=(selected.goal,), exclusion_radius_m=0.60)
    assert all(math.hypot(item.goal[0] - selected.goal[0],
                          item.goal[1] - selected.goal[1]) > 0.60
               for item in reranked)


def test_frontier_rejects_mismatched_costmap_shape():
    grid = np.full((10, 10), -1, dtype=np.int8)
    grid[2:8, 2:8] = 0
    with pytest.raises(ValueError, match="shape"):
        find_frontiers(grid, 0.1, (0.0, 0.0), (0.5, 0.5),
                       costmap=np.zeros((9, 10), dtype=np.uint8))


def test_frontier_can_descend_from_unknown_robot_center_to_known_free_space():
    grid = np.full((20, 20), -1, dtype=np.int8)
    grid[4:16, 4:16] = 0
    costmap = np.where(grid == 0, 0, 255).astype(np.uint8)

    # Cell (3, 10) represents the lidar-center blind spot occupied by the
    # robot. The immediately adjacent cell is known free, so the same bounded
    # trapped-start escape accepted by navigation should reach frontiers.
    frontiers = find_frontiers(
        grid, 0.1, (0.0, 0.0), (0.35, 1.05), min_cells=3,
        costmap=costmap, setback_m=0.2)

    assert frontiers
    assert all(np.isfinite(item.path_length_m) for item in frontiers)


FORWARD = {1: 500, 2: 701, 3: 120, 4: 300, 5: 500}
FLOOR = {1: 500, 2: 701, 3: 120, 4: 88, 5: 500}


def test_arm_fk_ik_roundtrip():
    arm = ArmKinematics()
    original = JointSolution(0.3, -0.4, 1.2, 0.9, 0.2)
    *point, pitch = arm.fk(original)
    solved = arm.ik(*point, pitch=pitch, roll=0.2)
    assert solved is not None
    assert arm.fk(solved) == pytest.approx((*point, pitch), abs=1e-6)
    assert joints_to_pulses(pulses_to_joints(joints_to_pulses(solved))) == joints_to_pulses(solved)


def test_arm_rejects_unreachable_target():
    arm = ArmKinematics()
    assert arm.ik(1.0, 0.0, 0.2) is None
    assert arm.ik(0.2, 0.0, -0.01) is None
    assert arm.ik(0.10, 0.0, 0.05) is None  # chassis keep-out


def test_floor_pick_envelope():
    arm = ArmKinematics()
    solution, pitch = arm.ik_any_pitch(0.18, 0.0, 0.015)
    assert solution is not None and pitch == pytest.approx(-math.pi / 2)
    far, far_pitch = arm.ik_any_pitch(0.26, 0.0, 0.015)
    assert far is not None and far_pitch > -math.pi / 2
    assert arm.ik_any_pitch(0.40, 0.0, 0.015)[0] is None


def test_camera_matches_observed_poses():
    arm = ArmKinematics()
    rotation, origin = arm.camera_pose(pulses_to_joints(FORWARD))
    assert rotation[:, 2] == pytest.approx([1, 0, 0], abs=0.03)  # looks forward
    assert rotation[:, 0] == pytest.approx([0, -1, 0], abs=0.03)  # image right = robot right
    assert 0.3 < origin[2] < 0.4
    down, _ = arm.camera_pose(pulses_to_joints(FLOOR))
    assert math.degrees(math.asin(-down[2, 2])) == pytest.approx(52, abs=2)
    left, _ = arm.camera_pose(pulses_to_joints({**FORWARD, 1: 750}))
    assert left[1, 2] == pytest.approx(math.sin(math.radians(60)), abs=0.02)  # 750 = 60 deg left


def test_pixel_to_plane_hits_floor_ahead():
    arm = ArmKinematics()
    k = (500.0, 500.0, 320.0, 200.0)
    joints = pulses_to_joints(FLOOR)
    x, y = arm.pixel_to_plane(320, 200, k, joints)
    _, origin = arm.camera_pose(joints)
    assert x == pytest.approx(origin[0] + origin[2] / math.tan(math.radians(52)), abs=0.02)
    assert abs(y) < 0.01
    assert arm.pixel_to_plane(500, 200, k, joints)[1] < 0  # right of centre = robot right
    assert arm.pixel_to_plane(320, 100, k, pulses_to_joints(FORWARD)) is None  # above horizon


def test_camera_aim_and_sweep_are_bounded():
    aim = CameraAim()
    assert aim.pulses(0.0, 0.0) == (500, 300)
    sweep = aim.sweep(3)
    assert len(sweep) == 3 and sweep[1] == (500, 300)
    assert sweep[0][0] < 500 < sweep[-1][0]
    with pytest.raises(ValueError):
        aim.pulses(math.pi, 0.0)
