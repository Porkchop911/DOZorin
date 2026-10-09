import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "ros2_ws" / "src" / "hwr_sensors"))
from hwr_sensors.imu_filter import ImuComplementaryFilter  # noqa: E402


def test_bias_calibration_and_yaw_integration():
    filt = ImuComplementaryFilter(bias_samples=3)
    gravity = (0.0, 0.0, 9.81)
    assert filt.update(gravity, (0.0, 0.0, 0.01), 0.0) is None
    assert filt.update(gravity, (0.0, 0.0, 0.01), 0.01) is None
    assert filt.update(gravity, (0.0, 0.0, 0.01), 0.02) is not None
    q = filt.update(gravity, (0.0, 0.0, 1.01), 0.12)
    assert filt.bias[2] == pytest.approx(0.01)
    assert filt.yaw == pytest.approx(0.1)
    assert q[2] == pytest.approx(math.sin(0.05), abs=1e-6)


def test_stationary_bias_has_negligible_yaw_drift():
    filt = ImuComplementaryFilter(bias_samples=2)
    gravity = (0.0, 0.0, 9.81)
    filt.update(gravity, (0.0, 0.0, 0.02), 0.0)
    filt.update(gravity, (0.0, 0.0, 0.02), 0.1)
    for i in range(1, 601):
        filt.update(gravity, (0.0, 0.0, 0.02), 0.1 + i * 0.1)
    assert abs(filt.yaw) < math.radians(0.01)


def test_gravity_initializes_roll_and_pitch():
    filt = ImuComplementaryFilter(bias_samples=1)
    filt.update((0.0, 9.81, 9.81), (0.0, 0.0, 0.0), 0.0)
    assert filt.roll == pytest.approx(math.pi / 4)
    assert filt.pitch == pytest.approx(0.0)


def test_bad_timestamps_do_not_integrate():
    filt = ImuComplementaryFilter(bias_samples=1)
    filt.update((0, 0, 9.81), (0, 0, 0), 10.0)
    filt.update((0, 0, 9.81), (0, 0, 1), 9.0)
    assert filt.yaw == 0.0
