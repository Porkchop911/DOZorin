"""Pure payload encoders/parsers for RRC functions. No I/O here.

Formats per ``plan/90-clean-room-reference.md`` (verified against the wire
in Phase 1 golden traces). All multi-byte integers are little-endian.
"""
from __future__ import annotations

import math
import struct
from typing import Iterable, NamedTuple, Sequence

GRAVITY = 9.80665

# ---------------------------------------------------------------- SYS (0)

SYS_BATTERY_REPORT = 0x04


def parse_battery_mv(data: bytes) -> int | None:
    """SYS report ``[0x04, mV u16]`` -> millivolts, or None if not a battery report."""
    if len(data) == 3 and data[0] == SYS_BATTERY_REPORT:
        return struct.unpack("<H", data[1:3])[0]
    return None


# ---------------------------------------------------------------- LED (1) / BUZZER (2) / OLED (10)

def encode_led(led_id: int, on_ms: int, off_ms: int, repeat: int = 1) -> bytes:
    return struct.pack("<BHHH", led_id, on_ms, off_ms, repeat)


def encode_buzzer(freq_hz: int, on_ms: int, off_ms: int, repeat: int = 1) -> bytes:
    return struct.pack("<HHHH", freq_hz, on_ms, off_ms, repeat)


def encode_oled_text(line: int, text: str) -> bytes:
    raw = text.encode("utf-8")
    return bytes([line, len(raw)]) + raw


# ---------------------------------------------------------------- MOTOR (3)

MOTOR_SET_SPEED = 0x01
MOTOR_SET_TYPE = 0x05
MOTOR_TYPE_JGB520, MOTOR_TYPE_JGB37, MOTOR_TYPE_JGB27, MOTOR_TYPE_JGB528 = range(4)


def encode_motor_speeds(speeds: Sequence[tuple[int, float]]) -> bytes:
    """``[(motor_id 1..4, rps float)]`` -> set-speed payload (wire ids are 0-based)."""
    out = bytearray([MOTOR_SET_SPEED, len(speeds)])
    for motor_id, rps in speeds:
        if not 1 <= motor_id <= 4:
            raise ValueError(f"motor id {motor_id} out of range 1..4")
        if not math.isfinite(float(rps)):
            raise ValueError(f"motor {motor_id} speed must be finite")
        out += struct.pack("<Bf", motor_id - 1, float(rps))
    return bytes(out)


def encode_motor_type(motor_type: int) -> bytes:
    return struct.pack("<BB", MOTOR_SET_TYPE, motor_type)


# ---------------------------------------------------------------- IMU (7)

class ImuSample(NamedTuple):
    ax: float  # m/s^2
    ay: float
    az: float
    gx: float  # rad/s
    gy: float
    gz: float


def scale_imu_gyro(sample: ImuSample, *, z_scale: float = 1.0) -> ImuSample:
    """Apply the hardware-calibrated yaw-rate scale to an SI IMU sample."""
    z_scale = float(z_scale)
    if not math.isfinite(z_scale) or z_scale <= 0.0:
        raise ValueError("gyro z scale must be finite and positive")
    return sample._replace(gz=sample.gz * z_scale)


def parse_imu(data: bytes) -> ImuSample | None:
    """IMU report in SI units, or ``None`` for a malformed payload."""
    if len(data) != struct.calcsize("<6f"):
        return None
    ax, ay, az, gx, gy, gz = struct.unpack("<6f", data)
    return ImuSample(
        ax * GRAVITY, ay * GRAVITY, az * GRAVITY,
        math.radians(gx), math.radians(gy), math.radians(gz),
    )


# ---------------------------------------------------------------- KEY (6)

KEY_EVENTS = {
    0x01: "pressed",
    0x02: "longpress",
    0x04: "longpress_repeat",
    0x08: "release_from_longpress",
    0x10: "release_from_shortpress",
    0x20: "click",
    0x40: "double_click",
    0x80: "triple_click",
}


def parse_key(data: bytes) -> tuple[int, str] | None:
    """KEY report ``[key_id, event]`` -> (key_id, event name)."""
    if len(data) >= 2 and data[1] in KEY_EVENTS:
        return data[0], KEY_EVENTS[data[1]]
    return None


# ---------------------------------------------------------------- BUS_SERVO (5)

SERVO_SET_POSITION = 0x01
SERVO_STOP = 0x03
SERVO_READ_POSITION = 0x05
SERVO_READ_VIN = 0x07
SERVO_READ_TEMP = 0x09
SERVO_TORQUE_ON = 0x0B
SERVO_TORQUE_OFF = 0x0C
SERVO_READ_TORQUE = 0x0D


def encode_servo_positions(duration_ms: int, positions: Sequence[tuple[int, int]]) -> bytes:
    """``[0x01, dur u16, count, {id, pulse u16}...]``; pulse 0..1000 = 0..240 deg."""
    if not 0 <= duration_ms <= 0xFFFF:
        raise ValueError(f"duration {duration_ms} ms out of range")
    out = bytearray([SERVO_SET_POSITION, duration_ms & 0xFF, duration_ms >> 8, len(positions)])
    for servo_id, pulse in positions:
        if not 0 <= pulse <= 1000:
            raise ValueError(f"servo {servo_id} pulse {pulse} out of range 0..1000")
        out += struct.pack("<BH", servo_id, pulse)
    return bytes(out)


def encode_servo_stop(servo_ids: Iterable[int]) -> bytes:
    ids = list(servo_ids)
    return bytes([SERVO_STOP, len(ids), *ids])


def encode_servo_torque(servo_id: int, enable: bool) -> bytes:
    return bytes([SERVO_TORQUE_ON if enable else SERVO_TORQUE_OFF, servo_id])


def encode_servo_read(sub: int, servo_id: int) -> bytes:
    """Request payload for any bus-servo read sub-command."""
    return bytes([sub, servo_id])


def parse_servo_reply(data: bytes, sub: int, value_format: str):
    """Read reply ``[id, sub, success i8, value...]`` -> (servo_id, *values).

    Returns None if the reply is for a different sub-command or reports
    failure (success != 0, e.g. servo offline).
    """
    head = struct.calcsize("<BBb")
    value_size = struct.calcsize(value_format)
    if len(data) < head + value_size or data[1] != sub:
        return None
    servo_id, _, success = struct.unpack("<BBb", data[:head])
    if success != 0:
        return None
    return (servo_id, *struct.unpack(value_format, data[head:head + value_size]))


def parse_servo_position(data: bytes) -> tuple[int, int] | None:
    """-> (servo_id, pulse) — pulse is signed i16 on the wire."""
    return parse_servo_reply(data, SERVO_READ_POSITION, "<h")


def parse_servo_vin(data: bytes) -> tuple[int, int] | None:
    """-> (servo_id, millivolts)."""
    return parse_servo_reply(data, SERVO_READ_VIN, "<H")


def parse_servo_temp(data: bytes) -> tuple[int, int] | None:
    """-> (servo_id, deg C)."""
    return parse_servo_reply(data, SERVO_READ_TEMP, "<B")


def parse_servo_torque(data: bytes) -> tuple[int, int] | None:
    """-> (servo_id, torque state)."""
    return parse_servo_reply(data, SERVO_READ_TORQUE, "<b")
