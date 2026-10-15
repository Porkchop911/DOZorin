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
