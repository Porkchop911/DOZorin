import struct
import time

import pytest

from hwr_core.robot import ARM_POSES, Robot
from hwr_core.rrc import Board, Function
from hwr_core.rrc.transport import Transport

from test_transport import FakeSerial


def motor_cmds(fake: "FakeSerial") -> list[tuple[float, float, float, float]]:
    """Decode all motor set-speed frames captured on the fake wire."""
    from hwr_core.rrc.frames import Decoder
    out = []
    for fr in Decoder().feed(bytes(fake.tx)):
        if fr.function == Function.MOTOR and fr.data[0] == 0x01:
            speeds = {}
            for i in range(fr.data[1]):
                wire_id, rps = struct.unpack_from("<Bf", fr.data, 2 + 5 * i)
                speeds[wire_id + 1] = rps
            out.append(tuple(speeds.get(m, 0.0) for m in (1, 2, 3, 4)))
    return out


def make_robot(watchdog_timeout=0.4):
    fake = FakeSerial()
    robot = Robot(Board(Transport(fake)), watchdog_timeout=watchdog_timeout)
    return fake, robot


def test_drive_does_not_require_power_telemetry():
    fake = FakeSerial()
    robot = Robot(Board(Transport(fake)))
    try:
        assert robot.drive(0.1, 0, 0) == (0.1, 0.0, 0.0)
        assert len(motor_cmds(fake)) == 1
    finally:
        robot.close()


def test_drive_sends_mixed_wheel_speeds():
    fake, robot = make_robot()
    try:
        applied = robot.drive(0.1, 0.0, 0.0)
        assert applied == (0.1, 0.0, 0.0)
        cmds = motor_cmds(fake)
        assert len(cmds) == 1
        m1, m2, m3, m4 = cmds[0]
        assert m1 > 0 and m2 > 0 and m3 < 0 and m4 < 0     # bench-verified signs
        assert m1 == pytest.approx(-m3)
    finally:
        robot.close()


def test_drive_clamps_to_limits():
    fake, robot = make_robot()
    try:
        # jump straight to max: velocity clamp to 0.2, accel clamp inactive on first cmd
        applied = robot.drive(5.0, 0.0, 0.0)
        assert applied[0] == 0.2
        # immediate second call: accel clamp bites (dt tiny -> barely any change)
        applied2 = robot.drive(-5.0, 0.0, 0.0)
        assert applied2[0] > 0.15
    finally:
        robot.close()


def test_stop_zeroes_and_disarms():
    fake, robot = make_robot()
    try:
        robot.drive(0.1, 0, 0)
        assert robot.watchdog.armed
        robot.stop()
        assert not robot.watchdog.armed
        assert motor_cmds(fake)[-1] == (0.0, 0.0, 0.0, 0.0)
    finally:
        robot.close()


def test_watchdog_stops_dead_driver():
    fake, robot = make_robot(watchdog_timeout=0.15)
    try:
        robot.drive(0.1, 0, 0)
        time.sleep(0.5)                                    # driver "dies"
        assert robot.watchdog.fired_count == 1
        assert motor_cmds(fake)[-1] == (0.0, 0.0, 0.0, 0.0)
    finally:
        robot.close()


def test_arm_pose_and_context_manager():
    fake = FakeSerial()
    robot = Robot(Board(Transport(fake)))
    with robot:
        robot.arm.set_pose("forward", 1500)
    # context exit must have sent a final all-stop
    assert motor_cmds(fake)[-1] == (0.0, 0.0, 0.0, 0.0)
    # servo frame present with the forward pose
    from hwr_core.rrc.frames import Decoder
    servo_frames = [fr for fr in Decoder().feed(bytes(fake.tx))
                    if fr.function == Function.BUS_SERVO and fr.data[0] == 0x01]
    assert len(servo_frames) == 1
    assert servo_frames[0].data[3] == len(ARM_POSES["forward"])
