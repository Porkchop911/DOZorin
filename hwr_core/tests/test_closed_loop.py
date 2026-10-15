import math

from hwr_core.control.closed_loop import (TurnTracker, approach_speed,
                                          progress_along)


def test_turn_tracker_crosses_wrap_for_half_and_full_turns():
    for degrees in (180.0, -180.0, 360.0, 90.0):
        target = math.radians(degrees)
        yaw = 3.0  # start near +pi so the turn crosses the wrap
        tracker = TurnTracker(target, yaw, math.radians(4))
        step = math.copysign(0.05, target)
        for _ in range(1000):
            if tracker.done:
                break
            yaw = math.atan2(math.sin(yaw + step), math.cos(yaw + step))
            tracker.update(yaw)
        assert tracker.done, degrees
        assert abs(tracker.turned - target) <= math.radians(4)


def test_approach_speed_is_bounded_signed_and_floored():
    assert approach_speed(3.0, 0.5, 0.15, 1.5) == 0.5
    assert approach_speed(-0.01, 0.5, 0.15, 1.5) == -0.15
    assert approach_speed(0.2, 0.5, 0.15, 1.5) == 0.2 * 1.5


def test_progress_along_projects_on_body_direction():
    start = (1.0, 1.0, math.pi / 2)  # facing +y
    assert math.isclose(progress_along(start, (1.0, 1.5), 0.0), 0.5)
    assert math.isclose(progress_along(start, (0.6, 1.0), math.pi / 2), 0.4)  # left = -x
    assert math.isclose(progress_along(start, (1.3, 1.0), 0.0), 0.0, abs_tol=1e-12)


def test_pick_target_prefers_locked_track_then_nearest():
    from hwr_core.control.closed_loop import pick_target
    objs = [{"class": "person", "id": 1, "bearing_deg": 5, "range_m": 2.0, "conf": 0.9},
            {"class": "person", "id": 2, "bearing_deg": -5, "range_m": 1.0, "conf": 0.5},
            {"class": "chair", "id": 3, "bearing_deg": 0, "range_m": 0.5, "conf": 0.9}]
    assert pick_target(objs, "person", None)["id"] == 2
    assert pick_target(objs, "person", 1)["id"] == 1
    assert pick_target(objs, "dog", None) is None


def test_approach_command_aligns_then_drives_then_stops():
    from hwr_core.control.closed_loop import approach_command
    args = dict(stop_at=0.8, max_wz=0.5, min_wz=0.15, max_v=0.15, min_v=0.05)
    vx, wz = approach_command(40, 2.0, **args)
    assert vx == 0.0 and wz > 0  # target to the left: turn left in place first
    vx, wz = approach_command(-3, 2.0, **args)
    assert vx > 0 and wz < 0
    assert approach_command(1, 0.7, **args) is None
    assert approach_command(0, None, **args) == (0.0, 0.0)
