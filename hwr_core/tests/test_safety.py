import time

import pytest

from hwr_core.safety import EStop, Limits, Watchdog, validate_wheel_rps


# ---------------------------------------------------------------- Limits

def test_velocity_clamp():
    lim = Limits()
    assert lim.clamp_velocity(1.0, -1.0, 3.0) == (0.2, -0.2, 0.5)
    assert lim.clamp_velocity(0.1, 0.05, -0.3) == (0.1, 0.05, -0.3)


def test_accel_clamp():
    lim = Limits(max_accel=0.5, max_alpha=2.0)
    # from rest, 0.1 s -> at most 0.05 m/s, 0.2 rad/s
    out = lim.clamp_accel((0.2, -0.2, 0.5), (0.0, 0.0, 0.0), 0.1)
    assert out == (0.05, -0.05, 0.2)
    # small change passes through
    out = lim.clamp_accel((0.02, 0.0, 0.1), (0.0, 0.0, 0.0), 0.1)
    assert out == (0.02, 0.0, 0.1)
    # dt<=0 keeps previous
    assert lim.clamp_accel((1, 1, 1), (0.1, 0.0, 0.0), 0.0) == (0.1, 0.0, 0.0)


def test_wheel_command_hard_limit():
    assert validate_wheel_rps([1, -2, 3, -4]) == (1.0, -2.0, 3.0, -4.0)
    for bad in ([0, 0, 0], [0, 0, 0, float("nan")], [4.01, 0, 0, 0]):
        with pytest.raises(ValueError):
            validate_wheel_rps(bad)


# ---------------------------------------------------------------- Watchdog

def test_watchdog_fires_after_timeout():
    stops = []
    dog = Watchdog(lambda: stops.append(time.monotonic()), timeout=0.15, tick=0.01)
    try:
        t0 = time.monotonic()
        dog.feed()
        time.sleep(0.4)
        assert len(stops) == 1          # one shot, not repeated
        assert 0.1 <= stops[0] - t0 <= 0.35
    finally:
        dog.close()


def test_watchdog_kept_alive_by_feeding():
    stops = []
    dog = Watchdog(lambda: stops.append(1), timeout=0.15, tick=0.01)
    try:
        for _ in range(10):
            dog.feed()
            time.sleep(0.05)
        assert stops == []
        assert dog.armed
    finally:
        dog.close()


def test_watchdog_disarm():
    stops = []
    dog = Watchdog(lambda: stops.append(1), timeout=0.1, tick=0.01)
    try:
        dog.feed()
        dog.disarm()
        time.sleep(0.3)
        assert stops == []
        assert not dog.armed
    finally:
        dog.close()


def test_watchdog_action_exception_does_not_kill_thread():
    calls = []

    def bad():
        calls.append(1)
        raise RuntimeError("boom")

    dog = Watchdog(bad, timeout=0.05, tick=0.01)
    try:
        dog.feed()
        time.sleep(0.15)
        dog.feed()
        time.sleep(0.15)
        assert len(calls) == 2
    finally:
        dog.close()


# ---------------------------------------------------------------- EStop

def test_estop_latch():
    e = EStop()
    assert not e.tripped
    e.trip("test")
    e.trip("second")                    # first reason wins
    assert e.tripped and e.reason == "test"
    e.reset()
    assert not e.tripped
