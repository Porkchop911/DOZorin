"""Golden-trace tests: our codecs must emit byte-identical frames to the
vendor SDK (captured as fixtures by tools/gen_golden_traces.py)."""
import json
import pathlib

from hwr_core.rrc import Function, codecs, encode

FIXTURES = json.loads(
    (pathlib.Path(__file__).parent / "fixtures" / "golden_traces.json").read_text())


def ours(function, payload) -> list[str]:
    return [encode(function, payload).hex()]


def test_led():
    assert ours(Function.LED, codecs.encode_led(1, 100, 200, 3)) == FIXTURES["led"]


def test_buzzer():
    assert ours(Function.BUZZER, codecs.encode_buzzer(1500, 100, 100, 1)) == FIXTURES["buzzer"]


def test_motor_speed():
    payload = codecs.encode_motor_speeds([(1, 0.5), (2, -0.5), (3, 1.0), (4, -1.0)])
    assert ours(Function.MOTOR, payload) == FIXTURES["motor_speed"]


def test_motor_type():
    assert ours(Function.MOTOR, codecs.encode_motor_type(codecs.MOTOR_TYPE_JGB37)) == \
        FIXTURES["motor_type_jgb37"]


def test_oled():
    assert ours(Function.OLED, codecs.encode_oled_text(1, "HWR")) == FIXTURES["oled"]


def test_servo_set_forward_pose():
    payload = codecs.encode_servo_positions(
        1500, [(1, 500), (2, 701), (3, 120), (4, 300), (5, 500)])
    assert ours(Function.BUS_SERVO, payload) == FIXTURES["servo_set_forward_pose"]


def test_servo_stop():
    payload = codecs.encode_servo_stop([1, 2, 3, 4, 5, 10])
    assert ours(Function.BUS_SERVO, payload) == FIXTURES["servo_stop"]


def test_servo_torque():
    assert ours(Function.BUS_SERVO, codecs.encode_servo_torque(5, True)) == \
        FIXTURES["servo_torque_on"]
    assert ours(Function.BUS_SERVO, codecs.encode_servo_torque(5, False)) == \
        FIXTURES["servo_torque_off"]


def test_servo_read_requests():
    for sub, key in [(codecs.SERVO_READ_POSITION, "servo_read_position_request"),
                     (codecs.SERVO_READ_VIN, "servo_read_vin_request"),
                     (codecs.SERVO_READ_TEMP, "servo_read_temp_request")]:
        assert ours(Function.BUS_SERVO, codecs.encode_servo_read(sub, 4)) == FIXTURES[key]
