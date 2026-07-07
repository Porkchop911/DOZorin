import struct
import threading
import time

from hwr_core.rrc.board import Board
from hwr_core.rrc.frames import Function, encode
from hwr_core.rrc.transport import Transport


class FakeSerial:
    """Injectable serial double: TX capture + programmable RX stream."""

    def __init__(self):
        self.tx = bytearray()
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._closed = False

    # test-side
    def inject(self, data: bytes):
        with self._lock:
            self._rx += data

    # transport-side
    @property
    def in_waiting(self):
        with self._lock:
            return len(self._rx)

    def read(self, n):
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            with self._lock:
                if self._closed:
                    raise OSError("closed")
                if self._rx:
                    out = bytes(self._rx[:n])
                    del self._rx[:n]
                    return out
            time.sleep(0.001)
        return b""

    def write(self, data):
        with self._lock:
            self.tx += data
        return len(data)

    def close(self):
        with self._lock:
            self._closed = True


def make():
    fake = FakeSerial()
    return fake, Transport(fake)


def test_send_writes_full_frame():
    fake, t = make()
    try:
        t.send(Function.BUZZER, b"\x01\x02")
        assert bytes(fake.tx) == encode(Function.BUZZER, b"\x01\x02")
    finally:
        t.close()


def test_latest_and_wait_report():
    fake, t = make()
    try:
        assert t.latest(Function.SYS) is None
        fake.inject(encode(Function.SYS, b"\x04" + struct.pack("<H", 11999)))
        data = t.wait_report(Function.SYS, timeout=1.0)
        assert data == b"\x04" + struct.pack("<H", 11999)
        assert t.latest(Function.SYS)[0] == data
    finally:
        t.close()


def test_wait_report_timeout():
    _, t = make()
    try:
        start = time.monotonic()
        assert t.wait_report(Function.KEY, timeout=0.2) is None
        assert time.monotonic() - start < 1.0
    finally:
        t.close()


def test_wait_report_predicate_skips_nonmatching():
    fake, t = make()
    try:
        result = {}

        def waiter():
            result["data"] = t.wait_report(
                Function.BUS_SERVO, timeout=2.0, predicate=lambda d: d[1] == 0x05)

        th = threading.Thread(target=waiter)
        th.start()
        time.sleep(0.05)
        fake.inject(encode(Function.BUS_SERVO, struct.pack("<BBbH", 1, 0x07, 0, 7400)))
        fake.inject(encode(Function.BUS_SERVO, struct.pack("<BBbh", 1, 0x05, 0, 512)))
        th.join()
        assert result["data"] == struct.pack("<BBbh", 1, 0x05, 0, 512)
    finally:
        t.close()


def test_subscribe_streams_all_reports():
    fake, t = make()
    try:
        got = []
        t.subscribe(Function.IMU, lambda data, stamp: got.append(data))
        sample = struct.pack("<6f", 0, 0, 1, 0, 0, 0)
        for _ in range(5):
            fake.inject(encode(Function.IMU, sample))
        deadline = time.monotonic() + 1.0
        while len(got) < 5 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert len(got) == 5
    finally:
        t.close()


def test_board_battery_and_actuators():
    fake, t = make()
    board = Board(t)
    try:
        fake.inject(encode(Function.SYS, b"\x04" + struct.pack("<H", 12345)))
        assert board.battery_mv(timeout=1.0) == 12345

        board.stop_all_motors()
        expected = encode(Function.MOTOR, bytes([0x01, 4]) + b"".join(
            struct.pack("<Bf", i, 0.0) for i in range(4)))
        assert bytes(fake.tx) == expected
    finally:
        board.close()


def test_board_servo_read_request_response():
    fake, t = make()
    board = Board(t)
    try:
        def answer():
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if bytes(fake.tx) == encode(Function.BUS_SERVO, b"\x05\x04"):
                    fake.inject(encode(Function.BUS_SERVO,
                                       struct.pack("<BBbh", 4, 0x05, 0, 301)))
                    return
                time.sleep(0.002)

        th = threading.Thread(target=answer)
        th.start()
        assert board.servo_position(4, timeout=1.5) == 301
        th.join()
    finally:
        board.close()


def test_close_is_clean_and_idempotent():
    _, t = make()
    t.close()
    t.close()
