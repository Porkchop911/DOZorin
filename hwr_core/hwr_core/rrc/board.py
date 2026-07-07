"""High-level Board API over the RRC transport.

The one hardware-facing object everything else builds on. Reads are
mailbox-based (battery/IMU stream in from the board unprompted); bus-servo
reads are request/response under a lock because replies share one channel.
"""
from __future__ import annotations

import threading
from typing import Callable, Sequence

from . import codecs
from .frames import Function
from .transport import Transport, open_rrc


class Board:
    def __init__(self, transport: Transport) -> None:
        self._t = transport
        self._servo_lock = threading.Lock()

    @classmethod
    def open(cls, device: str = "/dev/rrc") -> "Board":
        return cls(open_rrc(device))

    def close(self) -> None:
        self._t.close()

    # ------------------------------------------------------------- reads

    def battery_mv(self, timeout: float = 2.0) -> int | None:
        """Latest battery report in millivolts (board sends ~1 Hz)."""
        latest = self._t.latest(Function.SYS)
        if latest is not None:
            mv = codecs.parse_battery_mv(latest[0])
            if mv is not None:
                return mv
        data = self._t.wait_report(
            Function.SYS, timeout, lambda d: codecs.parse_battery_mv(d) is not None)
        return codecs.parse_battery_mv(data) if data else None

    def imu(self, timeout: float = 1.0) -> codecs.ImuSample | None:
        """Next IMU sample in SI units."""
        data = self._t.wait_report(Function.IMU, timeout)
        return codecs.parse_imu(data) if data else None

    def subscribe_imu(self, callback: Callable[[codecs.ImuSample, float], None]) -> None:
        """Stream IMU samples from the RX thread (full wire rate)."""
        self._t.subscribe(
            Function.IMU, lambda data, stamp: callback(codecs.parse_imu(data), stamp))

    def subscribe_battery(self, callback: Callable[[int, float], None]) -> None:
        """Battery reports (millivolts) from the RX thread, ~1 Hz."""
        def _relay(data: bytes, stamp: float) -> None:
            mv = codecs.parse_battery_mv(data)
            if mv is not None:
                callback(mv, stamp)
        self._t.subscribe(Function.SYS, _relay)

    def subscribe_key(self, callback: Callable[[int, str, float], None]) -> None:
        """Button events (key_id, event name) from the RX thread."""
        def _relay(data: bytes, stamp: float) -> None:
            evt = codecs.parse_key(data)
            if evt is not None:
                callback(evt[0], evt[1], stamp)
        self._t.subscribe(Function.KEY, _relay)

    def key_event(self, timeout: float = 0.0) -> tuple[int, str] | None:
        data = self._t.wait_report(
            Function.KEY, timeout, lambda d: codecs.parse_key(d) is not None)
        return codecs.parse_key(data) if data else None

    # ------------------------------------------------------------- simple actuators

    def set_led(self, on_ms: int, off_ms: int, repeat: int = 1, led_id: int = 1) -> None:
        self._t.send(Function.LED, codecs.encode_led(led_id, on_ms, off_ms, repeat))

    def beep(self, freq_hz: int = 1500, on_ms: int = 100, off_ms: int = 100,
             repeat: int = 1) -> None:
        # off_ms=0 makes the firmware latch the buzzer ON continuously
        # (2026-07-07 incident: required a power-off to silence) - never send it
        self._t.send(Function.BUZZER,
                     codecs.encode_buzzer(freq_hz, on_ms, max(off_ms, 10), repeat))

    def buzzer_off(self) -> None:
        """Best-effort silence: overwrite any running pattern with a minimal one."""
        self._t.send(Function.BUZZER, codecs.encode_buzzer(0, 1, 10, 1))

    def oled_text(self, line: int, text: str) -> None:
        self._t.send(Function.OLED, codecs.encode_oled_text(line, text))

    # ------------------------------------------------------------- motors

    def set_motor_type(self, motor_type: int = codecs.MOTOR_TYPE_JGB37) -> None:
        self._t.send(Function.MOTOR, codecs.encode_motor_type(motor_type))

    def set_motor_rps(self, speeds: Sequence[tuple[int, float]]) -> None:
        """Set wheel speeds, ``[(motor_id 1..4, rev/s)]``. Sign handling is
        the kinematics layer's job — this sends exactly what it is given."""
        self._t.send(Function.MOTOR, codecs.encode_motor_speeds(speeds))

    def stop_all_motors(self) -> None:
        self.set_motor_rps([(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)])

    # ------------------------------------------------------------- bus servos

    def set_servo_positions(self, duration_ms: int,
                            positions: Sequence[tuple[int, int]]) -> None:
        """Move servos to pulse targets (0..1000) over duration_ms."""
        self._t.send(Function.BUS_SERVO,
                     codecs.encode_servo_positions(duration_ms, positions))

    def stop_servos(self, servo_ids: Sequence[int]) -> None:
        self._t.send(Function.BUS_SERVO, codecs.encode_servo_stop(servo_ids))

    def set_servo_torque(self, servo_id: int, enable: bool) -> None:
        self._t.send(Function.BUS_SERVO, codecs.encode_servo_torque(servo_id, enable))

    def _servo_read(self, sub: int, servo_id: int, parser, timeout: float):
        with self._servo_lock:
            self._t.send(Function.BUS_SERVO, codecs.encode_servo_read(sub, servo_id))
            data = self._t.wait_report(
                Function.BUS_SERVO, timeout, lambda d: parser(d) is not None)
            return parser(data) if data else None

    def servo_position(self, servo_id: int, timeout: float = 1.0) -> int | None:
        """Current pulse of one servo, or None if it does not answer."""
        reply = self._servo_read(codecs.SERVO_READ_POSITION, servo_id,
                                 codecs.parse_servo_position, timeout)
        return reply[1] if reply else None

    def servo_vin(self, servo_id: int, timeout: float = 1.0) -> int | None:
        reply = self._servo_read(codecs.SERVO_READ_VIN, servo_id,
                                 codecs.parse_servo_vin, timeout)
        return reply[1] if reply else None

    def servo_temp(self, servo_id: int, timeout: float = 1.0) -> int | None:
        reply = self._servo_read(codecs.SERVO_READ_TEMP, servo_id,
                                 codecs.parse_servo_temp, timeout)
        return reply[1] if reply else None
