"""Generate golden-trace fixtures by driving the VENDOR SDK with a spy port.

The vendor SDK is executed as an oracle (never copied): each API call's TX
bytes are captured and written to hwr_core/tests/fixtures/golden_traces.json.
test_golden.py then asserts our clean-room codecs emit identical frames.

Run once on the PC (vendor tree required):
    python tools/gen_golden_traces.py
"""
import importlib.util
import json
import pathlib
import struct
import sys
import threading
import types

SDK_PATH = pathlib.Path(
    r"D:\robot\Hiwonder_software_stack\vendor\src_ros2\src\driver"
    r"\ros_robot_controller\ros_robot_controller\ros_robot_controller_sdk.py")
OUT_PATH = pathlib.Path(__file__).resolve().parents[1] / \
    "hwr_core" / "tests" / "fixtures" / "golden_traces.json"


def load_vendor_sdk():
    if "serial" not in sys.modules:  # SDK imports pyserial; not needed for TX capture
        stub = types.ModuleType("serial")
        stub.Serial = object
        sys.modules["serial"] = stub
    spec = importlib.util.spec_from_file_location("vendor_sdk", SDK_PATH)
    sdk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)
    return sdk


class SpyPort:
    def __init__(self):
        self.frames = []

    def write(self, buf):
        self.frames.append(bytes(buf))


class ReplyQueue:
    """Feeds the SDK's blocking read path a canned reply."""

    def __init__(self, reply: bytes):
        self.reply = reply

    def get(self, block=True, timeout=None):
        return self.reply


def main():
    sdk = load_vendor_sdk()
    board = sdk.Board.__new__(sdk.Board)  # skip __init__: no serial, no threads
    board.port = SpyPort()
    board.servo_read_lock = threading.Lock()

    cases = {}

    def capture(name, call):
        board.port.frames = []
        call()
        cases[name] = [f.hex() for f in board.port.frames]

    capture("led", lambda: board.set_led(0.1, 0.2, 3, 1))
    capture("buzzer", lambda: board.set_buzzer(1500, 0.1, 0.1, 1))
    capture("motor_speed", lambda: board.set_motor_speed(
        [[1, 0.5], [2, -0.5], [3, 1.0], [4, -1.0]]))
    capture("motor_type_jgb37", lambda: board.set_motor_type(0x01))
    capture("oled", lambda: board.set_oled_text(1, "HWR"))
    capture("servo_set_forward_pose", lambda: board.bus_servo_set_position(
        1.5, [[1, 500], [2, 701], [3, 120], [4, 300], [5, 500]]))
    capture("servo_stop", lambda: board.bus_servo_stop([1, 2, 3, 4, 5, 10]))
    capture("servo_torque_on", lambda: board.bus_servo_enable_torque(5, True))
    capture("servo_torque_off", lambda: board.bus_servo_enable_torque(5, False))

    board.bus_servo_queue = ReplyQueue(struct.pack("<BBbh", 4, 0x05, 0, 300))
    capture("servo_read_position_request", lambda: board.bus_servo_read_position(4))
    board.bus_servo_queue = ReplyQueue(struct.pack("<BBbH", 4, 0x07, 0, 7400))
    capture("servo_read_vin_request", lambda: board.bus_servo_read_vin(4))
    board.bus_servo_queue = ReplyQueue(struct.pack("<BBbB", 4, 0x09, 0, 35))
    capture("servo_read_temp_request", lambda: board.bus_servo_read_temp(4))

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(cases, indent=2) + "\n")
    print(f"wrote {len(cases)} traces -> {OUT_PATH}")
    for name, frames in cases.items():
        print(f"  {name}: {frames}")


if __name__ == "__main__":
    main()
