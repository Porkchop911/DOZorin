import math
import struct

import pytest

from hwr_core.rrc import codecs


# ---------------------------------------------------------------- SYS / battery

def test_parse_battery():
    assert codecs.parse_battery_mv(b"\x04" + struct.pack("<H", 12082)) == 12082
    assert codecs.parse_battery_mv(b"\x01\x00\x00") is None      # wrong sub
    assert codecs.parse_battery_mv(b"\x04\x00") is None          # short


# ---------------------------------------------------------------- LED / buzzer / OLED

def test_encode_led():
    assert codecs.encode_led(1, 100, 200, 3) == struct.pack("<BHHH", 1, 100, 200, 3)


def test_encode_buzzer():
    assert codecs.encode_buzzer(1500, 100, 100, 1) == struct.pack("<HHHH", 1500, 100, 100, 1)


def test_encode_oled():
    payload = codecs.encode_oled_text(1, "HWR")
    assert payload == bytes([1, 3]) + b"HWR"


# ---------------------------------------------------------------- motors

def test_encode_motor_speeds_wire_ids_zero_based():
    payload = codecs.encode_motor_speeds([(1, 0.5), (4, -1.25)])
    assert payload[0] == 0x01
    assert payload[1] == 2
    assert payload[2] == 0                                  # id 1 -> 0
    assert struct.unpack("<f", payload[3:7])[0] == 0.5
    assert payload[7] == 3                                  # id 4 -> 3
    assert struct.unpack("<f", payload[8:12])[0] == -1.25


def test_encode_motor_speeds_rejects_bad_id():
    with pytest.raises(ValueError):
        codecs.encode_motor_speeds([(0, 1.0)])
    with pytest.raises(ValueError):
        codecs.encode_motor_speeds([(5, 1.0)])


def test_encode_motor_type():
    assert codecs.encode_motor_type(codecs.MOTOR_TYPE_JGB37) == b"\x05\x01"


# ---------------------------------------------------------------- IMU

def test_parse_imu_si_conversion():
    raw = struct.pack("<6f", 0.0, 0.0, 1.0, 90.0, -90.0, 45.0)
    s = codecs.parse_imu(raw)
    assert s.az == pytest.approx(9.80665)
    assert s.gx == pytest.approx(math.pi / 2)
    assert s.gy == pytest.approx(-math.pi / 2)
    assert s.gz == pytest.approx(math.pi / 4)


# ---------------------------------------------------------------- key

def test_parse_key():
    assert codecs.parse_key(b"\x01\x20") == (1, "click")
    assert codecs.parse_key(b"\x02\x40") == (2, "double_click")
    assert codecs.parse_key(b"\x01\x33") is None


# ---------------------------------------------------------------- bus servos

def test_encode_servo_positions():
    payload = codecs.encode_servo_positions(1500, [(1, 500), (10, 200)])
    assert payload[:4] == bytes([0x01, 1500 & 0xFF, 1500 >> 8, 2])
    assert payload[4:7] == struct.pack("<BH", 1, 500)
    assert payload[7:10] == struct.pack("<BH", 10, 200)


def test_encode_servo_positions_validates():
    with pytest.raises(ValueError):
        codecs.encode_servo_positions(100, [(1, 1001)])
    with pytest.raises(ValueError):
        codecs.encode_servo_positions(-1, [(1, 500)])


def test_encode_servo_stop_and_torque():
    assert codecs.encode_servo_stop([1, 2, 10]) == bytes([0x03, 3, 1, 2, 10])
    assert codecs.encode_servo_torque(5, True) == bytes([0x0B, 5])
    assert codecs.encode_servo_torque(5, False) == bytes([0x0C, 5])


def test_servo_read_roundtrip():
    assert codecs.encode_servo_read(codecs.SERVO_READ_POSITION, 4) == b"\x05\x04"
    reply = struct.pack("<BBbh", 4, 0x05, 0, 300)
    assert codecs.parse_servo_position(reply) == (4, 300)


def test_servo_reply_failure_and_mismatch():
    offline = struct.pack("<BBbh", 4, 0x05, -1, 0)
    assert codecs.parse_servo_position(offline) is None
    wrong_sub = struct.pack("<BBbH", 4, 0x07, 0, 7400)
    assert codecs.parse_servo_position(wrong_sub) is None
    assert codecs.parse_servo_vin(wrong_sub) == (4, 7400)


def test_servo_temp_parse():
    reply = struct.pack("<BBbB", 10, 0x09, 0, 38)
    assert codecs.parse_servo_temp(reply) == (10, 38)
