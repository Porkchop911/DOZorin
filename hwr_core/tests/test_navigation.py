import math
import pytest

import numpy as np

from hwr_core.navigation import (Costmap, ForwardRecovery, GoalProgress,
                                 HolonomicController, ReplanBudget, astar,
                                 gyro_yaw_motion_confirmed,
                                 limit_translation_target,
                                 odometry_translation_within_limit,
                                 odometry_motion_confirmed,
                                 sensor_sample_fresh, simplify_path,
                                 translation_deadline_seconds,
                                 translation_goal_reached_or_passed)


def test_costmap_inflation_and_live_decay():
    occ = np.zeros((20, 20), dtype=np.int8)
    occ[10, 10] = 100
    cm = Costmap(occ, 0.1, (0, 0), inflation_radius=0.3)
    first = cm.grid(0.0)
    assert first[10, 10] == 255 and first[10, 11] > 0
    assert first[10, 11] >= 200
    assert first[12, 12] > 0
    assert first[12, 12] < 200
    assert first[10, 13] == 0
    cm.mark([[0.5, 0.5]], now=1.0, ttl=1.0)
    assert cm.grid(1.5)[5, 5] == 255
    assert cm.grid(2.1)[5, 5] != 255
    cm.mark([[0.6, 0.6]], now=3.0, ttl=10.0)
    cm.clear_live()
    assert not cm.live


def test_costmap_checks_the_whole_motion_segment():
    occ = np.zeros((10, 10), dtype=np.int8)
    occ[0, 4] = 100
    cm = Costmap(occ, 0.1, (0, 0), inflation_radius=0.1)
    assert not cm.segment_is_clear((0.05, 0.05), (0.85, 0.05), now=0.0)
    assert cm.segment_is_clear((0.05, 0.25), (0.85, 0.25), now=0.0)
    assert not cm.segment_is_clear((0.05, 0.25), (1.05, 0.25), now=0.0)
    assert not cm.segment_is_clear((math.nan, 0.0), (0.1, 0.1), now=0.0)


def test_inscribed_robot_radius_is_non_traversable_but_outer_margin_is_soft():
    occ = np.zeros((20, 20), dtype=np.int8)
    occ[10, 10] = 100
    cm = Costmap(occ, 0.1, (0, 0), inflation_radius=0.4, robot_radius=0.2)
    costs = cm.grid(0.0)
    assert costs[10, 11] == 253
    assert costs[10, 12] == 253
    assert 0 < costs[10, 13] < 200
    assert costs[10, 14] == 0
    assert not cm.segment_is_clear((0.05, 1.05), (1.85, 1.05), now=0.0)


def test_close_live_obstacle_blocks_rotation_at_current_pose():
    cm = Costmap(np.zeros((100, 100), dtype=np.int8), 0.05, (-2.5, -2.5),
                 inflation_radius=0.30, robot_radius=0.20)
    cm.mark([[0.15, 0.0]], now=1.0, ttl=10.0)
    costs = cm.grid(2.0)
    x, y = cm.world_to_cell((0.0, 0.0))
    assert costs[y, x] >= 200
    assert not cm.segment_is_clear((0.0, 0.0), (0.0, 0.0), now=2.0)


def test_astar_routes_through_gap():
    costs = np.zeros((10, 10), dtype=np.uint8)
    costs[:, 5] = 255
    costs[4, 5] = 0
    path = astar(costs, (1, 1), (8, 8))
    assert path[0] == (1, 1) and path[-1] == (8, 8)
    assert (5, 4) in path
    assert len(simplify_path(costs, path)) < len(path)


def test_astar_no_path_and_bad_goal():
    costs = np.zeros((5, 5), dtype=np.uint8)
    costs[:, 2] = 255
    assert astar(costs, (0, 0), (4, 4)) == []
    assert astar(costs, (0, 0), (2, 2)) == []


def test_astar_can_descend_out_of_trapped_start_but_not_cross_obstacle():
    costs = np.array([[0, 100, 199, 253, 255]], dtype=np.uint8)
    assert astar(costs, (3, 0), (0, 0)) == []
    assert astar(costs, (3, 0), (0, 0),
                 allow_trapped_start=True) == [
                     (3, 0), (2, 0), (1, 0), (0, 0)]
    assert astar(costs, (3, 0), (4, 0),
                 allow_trapped_start=True) == []
    assert astar(costs, (3, 0), (3, 0),
                 allow_trapped_start=True) == [(3, 0)]


def test_astar_can_cross_flat_inflation_plateau_but_not_flat_obstacle():
    inflated = np.array([[0, 199, 252, 252, 252]], dtype=np.uint8)
    assert astar(inflated, (4, 0), (0, 0),
                 allow_trapped_start=True) == [
                     (4, 0), (3, 0), (2, 0), (1, 0), (0, 0)]
    obstacle = np.array([[0, 199, 255, 255]], dtype=np.uint8)
    assert astar(obstacle, (3, 0), (0, 0),
                 allow_trapped_start=True) == []


def test_segment_can_only_escape_trapped_start_down_cost_gradient():
    cm = Costmap(np.zeros((1, 6), dtype=np.int16), 0.1, (0.0, 0.0),
                 inflation_radius=0.0)
    cm.grid = lambda now: np.array([[0, 0, 100, 199, 253, 255]],
                                   dtype=np.uint8)
    assert cm.segment_is_clear((0.45, 0.05), (0.05, 0.05), 0.0,
                               allow_trapped_start=True)
    assert not cm.segment_is_clear((0.45, 0.05), (0.55, 0.05), 0.0,
                                   allow_trapped_start=True)


def test_segment_can_cross_flat_inflation_plateau_but_not_flat_obstacle():
    cm = Costmap(np.zeros((1, 5), dtype=np.int16), 0.1, (0.0, 0.0),
                 inflation_radius=0.0)
    cm.grid = lambda now: np.array([[0, 199, 252, 252, 252]],
                                   dtype=np.uint8)
    assert cm.segment_is_clear((0.45, 0.05), (0.05, 0.05), 0.0,
                               allow_trapped_start=True)
    cm.grid = lambda now: np.array([[0, 199, 255, 255, 255]],
                                   dtype=np.uint8)
    assert not cm.segment_is_clear((0.45, 0.05), (0.05, 0.05), 0.0,
                                   allow_trapped_start=True)


def test_segment_can_accept_safe_partial_prefix_of_planned_escape():
    cm = Costmap(np.zeros((1, 5), dtype=np.int16), 0.1, (0.0, 0.0),
                 inflation_radius=0.0)
    cm.grid = lambda now: np.array([[0, 199, 252, 252, 252]],
                                   dtype=np.uint8)
    assert not cm.segment_is_clear((0.45, 0.05), (0.25, 0.05), 0.0,
                                   allow_trapped_start=True)
    assert cm.segment_is_clear((0.45, 0.05), (0.25, 0.05), 0.0,
                               allow_trapped_start=True,
                               allow_partial_escape=True)
    cm.grid = lambda now: np.array([[0, 199, 252, 253, 252]],
                                   dtype=np.uint8)
    assert not cm.segment_is_clear((0.45, 0.05), (0.25, 0.05), 0.0,
                                   allow_trapped_start=True,
                                   allow_partial_escape=True)


def test_controller_never_commands_reverse():
    controller = HolonomicController()
    command = controller.command((0, 0, 0), [(-1, 0)])
    assert command[0] == 0.0
    assert abs(command[2]) > 0.0


def test_controller_rotates_before_translating_outside_perception_cone():
    controller = HolonomicController(translation_heading_limit=math.radians(30))
    vx, vy, wz, done = controller.command((0, 0, 0), [(1.0, 1.0)])
    assert vx == 0.0 and vy == 0.0 and wz > 0.0 and not done


def test_controller_strafes_and_finishes_yaw():
    controller = HolonomicController()
    vx, vy, _, done = controller.command((0, 0, 0), [(0.2, 0.1)])
    assert vx >= 0 and vy > 0 and not done
    _, _, wz, done = controller.command((0, 0, 0), [(0.01, 0.01)], goal_yaw=0.0)
    assert wz == 0 and done


def test_five_centimetre_canary_is_not_swallowed_by_goal_tolerance():
    controller = HolonomicController()
    vx, vy, wz, done = controller.command((0.0, 0.0, 0.0), [(0.05, 0.0)])
    assert vx > 0.0
    assert vy == 0.0 and wz == 0.0 and not done


def test_short_leg_holds_approach_heading_as_goal_vector_shrinks():
    controller = HolonomicController(lookahead=0.3)
    # Two centimetres of lateral map noise near the end of a ten-centimetre leg
    # makes the instantaneous point bearing about -39 degrees. The preflighted
    # approach was straight ahead, so the holonomic correction must translate
    # without starting a terminal heading oscillation.
    vx, vy, wz, done = controller.command(
        (0.075, 0.02, 0.0), [(0.10, 0.0)], goal_yaw=0.0)
    assert vx > 0.0 and vy < 0.0
    assert wz == 0.0 and not done


def test_long_route_faces_path_before_final_goal_yaw():
    controller = HolonomicController(lookahead=0.3)
    vx, vy, wz, done = controller.command(
        (0.0, 0.0, 0.0), [(1.0, 0.0)], goal_yaw=math.pi / 2.0)
    assert vx > 0.0 and vy == 0.0 and wz == 0.0 and not done


def test_yaw_command_never_introduces_translation_semantics():
    controller = HolonomicController(max_wz=0.5)
    wz, done = controller.yaw_command(0.0, math.pi)
    assert wz == 0.5 and not done
    wz, done = controller.yaw_command(math.radians(178), math.pi)
    assert 0.0 < wz < 0.1 and done


def test_goal_progress_counts_yaw_near_goal_and_detects_stall():
    progress = GoalProgress(distance=0.02, yaw_error=math.pi, now=0.0)
    assert progress.update(0.04, math.radians(170), True, 1.0)
    assert not progress.stalled(8.5, 8.0)
    assert progress.stalled(9.0, 8.0)
    progress = GoalProgress(distance=1.0, yaw_error=math.pi, now=0.0)
    assert not progress.update(1.0, math.radians(150), False, 1.0)
    assert progress.stalled(8.0, 8.0)


def test_live_obstacle_replan_budget_is_bounded_and_reset_by_progress():
    budget = ReplanBudget(limit=2)
    assert budget.request()
    assert budget.request()
    assert not budget.request()
    assert budget.attempts == 2
    budget.reset()
    assert budget.request()
    assert budget.attempts == 1


def test_motion_sensor_freshness_fails_closed():
    assert sensor_sample_fresh(10.0, 9.5, 0.75)
    assert not sensor_sample_fresh(10.0, 0.0, 0.75)
    assert not sensor_sample_fresh(10.0, 9.0, 0.75)
    assert not sensor_sample_fresh(10.0, 10.1, 0.75)


def test_short_translation_has_a_hard_runtime_bound():
    assert translation_deadline_seconds(0.10) == 6.0
    assert translation_deadline_seconds(0.25) == 9.0
    assert translation_deadline_seconds(10.0) == 10.0
    # Turning to face the path and then the goal yaw is budgeted separately:
    # a 150 deg turn at 0.3 rad/s alone takes ~9 s.
    assert translation_deadline_seconds(1.0, rotation=math.pi,
                                        yaw_rate=0.3) == pytest.approx(
        10.0 + 1.5 * math.pi / 0.3)


def test_committed_translation_target_retains_bearing_but_never_exceeds_cap():
    target = limit_translation_target((0.0, 0.0), (0.08, 0.08), 0.10)
    assert math.isclose(math.hypot(*target), 0.10)
    assert math.isclose(target[0], target[1])
    assert limit_translation_target((0.0, 0.0), (0.05, 0.0), 0.10) == (
        0.05, 0.0)


def test_no_reverse_terminal_accepts_only_bounded_on_track_overshoot():
    start, goal = (0.0, 0.0), (0.10, 0.0)
    assert translation_goal_reached_or_passed(start, (0.09, 0.0), goal)
    assert translation_goal_reached_or_passed(start, (0.13, 0.01), goal)
    assert not translation_goal_reached_or_passed(start, (0.17, 0.0), goal)
    assert not translation_goal_reached_or_passed(start, (0.13, 0.06), goal)
    assert not translation_goal_reached_or_passed(start, (0.06, 0.0), goal)


def test_physical_translation_is_leashed_independently_of_map_pose():
    start = (1.0, 2.0, 0.4)
    assert odometry_translation_within_limit(
        0.10, start, (1.149, 2.0, 1.2))
    assert not odometry_translation_within_limit(
        0.10, start, (1.151, 2.0, 0.4))
    assert not odometry_translation_within_limit(
        0.10, start, (math.nan, 2.0, 0.4))


def test_goal_success_requires_independent_odometry_translation():
    start = (1.0, 2.0, 0.4)
    assert not odometry_motion_confirmed(
        0.05, 0.0, start, (1.019, 2.0, 0.4))
    assert odometry_motion_confirmed(
        0.05, 0.0, start, (1.021, 2.0, 0.4))
    # 10 % of distance allowance: 0.50 m map leg needs >= 0.425 m of odometry.
    assert not odometry_motion_confirmed(
        0.50, 0.0, start, (1.424, 2.0, 0.4))
    assert odometry_motion_confirmed(
        0.50, 0.0, start, (1.426, 2.0, 0.4))
    # The earlier false-success sample advanced only 1.7 cm of a 5 cm goal.
    assert not odometry_motion_confirmed(
        0.05, 0.0, start, (1.017, 2.0, 0.4))


def test_goal_success_requires_independent_odometry_yaw():
    start = (0.0, 0.0, 0.0)
    assert not odometry_motion_confirmed(
        0.0, math.radians(30.0), start,
        (0.0, 0.0, math.radians(24.0)))
    assert odometry_motion_confirmed(
        0.0, math.radians(30.0), start,
        (0.0, 0.0, math.radians(26.0)))
    assert not odometry_motion_confirmed(
        0.05, 0.0, start, (math.nan, 0.0, 0.0))


def test_physical_gyro_yaw_confirmation_is_independent_of_scan_pose():
    needed = math.radians(33.0)
    start = 1.2
    # The live repetitive-window failure: scan-fused yaw reported only about
    # 18 degrees, while integrated physical gyro reported about 41 degrees.
    assert not gyro_yaw_motion_confirmed(
        needed, start, start + math.radians(18.0))
    assert gyro_yaw_motion_confirmed(
        needed, start, start + math.radians(41.0))
    assert gyro_yaw_motion_confirmed(math.radians(4.0), start, start)
    assert not gyro_yaw_motion_confirmed(needed, start, math.nan)
    assert not gyro_yaw_motion_confirmed(
        needed, start, start + math.radians(41.0), yaw_noise_allowance=-0.1)


def test_recovery_sequence_is_bounded_and_never_reverses():
    recovery = ForwardRecovery(rotate_seconds=4.0, arc_seconds=2.0)
    commands = [recovery.command(t) for t in (0.0, 3.9, 4.0, 5.9, 6.0)]
    assert [command[3] for command in commands] == [
        "rotate-scan", "rotate-scan", "forward-arc", "forward-arc", "replan"]
    assert all(command[0] >= 0.0 for command in commands)
