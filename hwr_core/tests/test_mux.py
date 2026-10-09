import pytest

from hwr_core.control import CommandMux


def make_mux():
    return CommandMux(("teleop", "agent", "nav"),
                      {"teleop": 0.35, "agent": 0.5, "nav": 0.5})


def test_priority_and_fallback():
    mux = make_mux()
    mux.update("nav", (0.1, 0, 0), 10.0)
    mux.update("agent", (0, 0.1, 0), 10.0)
    mux.update("teleop", (0, 0, 0.2), 10.0)
    assert mux.select(10.1) == ("teleop", (0.0, 0.0, 0.2))
    assert mux.select(10.4) == ("agent", (0.0, 0.1, 0.0))
    assert mux.select(10.6) == (None, (0.0, 0.0, 0.0))


def test_zero_command_still_owns_priority_until_stale():
    mux = make_mux()
    mux.update("nav", (0.1, 0, 0), 10.0)
    mux.update("teleop", (0, 0, 0), 10.0)
    assert mux.select(10.2)[0] == "teleop"


def test_clock_rewind_fails_closed():
    mux = make_mux()
    mux.update("nav", (0.1, 0, 0), 10.0)
    assert mux.select(9.0) == (None, (0.0, 0.0, 0.0))


def test_configuration_validation():
    with pytest.raises(ValueError):
        CommandMux(("teleop",), {"nav": 1.0})
    with pytest.raises(ValueError):
        CommandMux(("teleop",), {"teleop": 0.0})
