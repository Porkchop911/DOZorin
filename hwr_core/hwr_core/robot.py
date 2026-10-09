"""The hwr_core.Robot facade — the Python API everything above builds on.

Safety pipeline on every drive() call:
    estop latch -> velocity clamp -> accel clamp ->
    mecanum mixing -> wheel-speed scale -> wire; watchdog auto-stops
    0.4 s after the last command if the caller dies mid-drive.

Usage:
    with Robot.connect() as r:
        r.drive(0.1, 0, 0); time.sleep(2); r.stop()
"""
from __future__ import annotations

import time
from typing import Mapping, Sequence

from .kinematics import Calibration, scale_to_limit, wheel_speeds
from .rrc import Board, codecs
from .safety import EStop, Limits, Watchdog

MAX_WHEEL_RPS = 4.0  # conservative wire-level cap; envelope really set by Limits

ARM_POSES: dict[str, dict[int, int]] = {
    # known-good pulse sets (Phase 1 verified; camera tilt 88 floor / 300 fwd / 500 up)
    "forward": {1: 500, 2: 701, 3: 120, 4: 300, 5: 500},
}
ARM_SERVO_IDS = (1, 2, 3, 4, 5)
GRIPPER_ID = 10


class EStopError(RuntimeError):
    pass


class Arm:
    def __init__(self, board: Board, require_actuation) -> None:
        self._board = board
        self._require_actuation = require_actuation

    def set_pose(self, pose: str | Mapping[int, int], duration_ms: int = 1500) -> None:
        self._require_actuation()
        if isinstance(pose, str):
            pose = ARM_POSES[pose]
        self._board.set_servo_positions(duration_ms, sorted(pose.items()))

    def read(self, servo_ids: Sequence[int] = (*ARM_SERVO_IDS, GRIPPER_ID)) -> dict[int, int | None]:
        return {sid: self._board.servo_position(sid) for sid in servo_ids}

    def stop(self) -> None:
        self._board.stop_servos([*ARM_SERVO_IDS, GRIPPER_ID])

    def torque(self, enable: bool, servo_ids: Sequence[int] = (*ARM_SERVO_IDS, GRIPPER_ID)) -> None:
        if enable:
            self._require_actuation()
        for sid in servo_ids:
            self._board.set_servo_torque(sid, enable)

    def gripper(self, pulse: int, duration_ms: int = 800) -> None:
        self._require_actuation()
        self._board.set_servo_positions(duration_ms, [(GRIPPER_ID, pulse)])


class Robot:
    def __init__(self, board: Board, limits: Limits | None = None,
                 calibration: Calibration | None = None,
                 watchdog_timeout: float = 0.4) -> None:
        self._board = board
        self.limits = limits or Limits()
        self.calibration = calibration or Calibration()
        self.estop_latch = EStop()
        self.watchdog = Watchdog(self._watchdog_stop, timeout=watchdog_timeout)
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time: float | None = None
        self.arm = Arm(board, self._require_motion)

    # ------------------------------------------------------------- lifecycle

    @classmethod
    def connect(cls, device: str = "/dev/rrc", **kwargs) -> "Robot":
        robot = cls(Board.open(device), **kwargs)
        robot._board.set_motor_type(codecs.MOTOR_TYPE_JGB37)  # idempotent, persistence unknown
        return robot

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.watchdog.close()
            self._board.close()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------- reports

    def _watchdog_stop(self) -> None:
        self._board.stop_all_motors()
        self._last_cmd = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------- motion

    def _require_motion(self) -> None:
        if self.estop_latch.tripped:
            raise EStopError(f"estop: {self.estop_latch.reason}")

    def drive(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        """Command a body velocity. Returns what was actually applied after
        clamping. Raises EStopError instead of silently moving less safely."""
        self._require_motion()

        cmd = self.limits.clamp_velocity(vx, vy, wz)
        now = time.monotonic()
        if self._last_cmd_time is not None:
            cmd = self.limits.clamp_accel(cmd, self._last_cmd, now - self._last_cmd_time)
        rps = scale_to_limit(wheel_speeds(*cmd, cal=self.calibration), MAX_WHEEL_RPS)
        self._board.set_motor_rps(list(zip((1, 2, 3, 4), rps)))
        self._last_cmd, self._last_cmd_time = cmd, now
        self.watchdog.feed()
        return cmd

    def stop(self) -> None:
        """Zero all motors and disarm the watchdog. Always allowed."""
        self.watchdog.disarm()
        self._board.stop_all_motors()
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = None

    def estop(self, reason: str = "manual") -> None:
        """Latched emergency stop: motors zero, servos halt, motion refused
        until reset_estop()."""
        self.estop_latch.trip(reason)
        self.watchdog.disarm()
        self._board.stop_all_motors()
        self.arm.stop()

    def reset_estop(self) -> None:
        self.estop_latch.reset()

    # ------------------------------------------------------------- sensors / misc

    def battery_mv(self, timeout: float = 2.0) -> int | None:
        return self._board.battery_mv(timeout)

    def imu(self, timeout: float = 1.0) -> codecs.ImuSample | None:
        return self._board.imu(timeout)

    def subscribe_imu(self, callback) -> None:
        self._board.subscribe_imu(callback)

    def beep(self, freq_hz: int = 1500, on_ms: int = 100, off_ms: int = 100,
             repeat: int = 1) -> None:
        self._board.beep(freq_hz, on_ms, off_ms, repeat)

    def oled_text(self, line: int, text: str) -> None:
        self._board.oled_text(line, text)
