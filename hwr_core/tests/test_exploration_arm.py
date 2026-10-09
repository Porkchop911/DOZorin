import math

import numpy as np
import pytest

from hwr_core.arm import ArmKinematics, CameraAim, JointSolution
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


def test_arm_fk_ik_roundtrip():
    arm = ArmKinematics()
    original = JointSolution(0.3, 0.5, -0.8, 0.3)
    point = arm.fk(original)
    solved = arm.ik(*point, tool_pitch=0.0)
    assert solved is not None
    assert arm.fk(solved) == pytest.approx(point, abs=1e-6)


def test_arm_rejects_unreachable_target():
    assert ArmKinematics().ik(1.0, 0.0, 0.2) is None
    assert ArmKinematics().ik(0.02, 0.0, 0.10) is None


def test_arm_pulse_conversion():
    assert ArmKinematics.radians_to_pulse(0.0) == 500
    assert ArmKinematics.radians_to_pulse(math.radians(24)) == 600


def test_camera_aim_and_sweep_are_bounded():
    aim = CameraAim()
    assert aim.pulses(0.0, 0.0) == (500, 300)
    sweep = aim.sweep(3)
    assert len(sweep) == 3 and sweep[1] == (500, 300)
    assert sweep[0][0] < 500 < sweep[-1][0]
    with pytest.raises(ValueError):
        aim.pulses(math.pi, 0.0)
