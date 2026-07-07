"""Frame codec for the RRC protocol (STM32 <-> host).

Wire format::

    0xAA 0x55 | function(1) | length(1) | data[length] | crc8

The CRC covers ``function, length, data``. Valid function ids are 0..10.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass

from .crc8 import crc8

HEADER1 = 0xAA
HEADER2 = 0x55
MAX_FUNCTION = 10
MAX_DATA = 255


class Function(enum.IntEnum):
    SYS = 0
    LED = 1
    BUZZER = 2
    MOTOR = 3
    PWM_SERVO = 4
    BUS_SERVO = 5
    KEY = 6
    IMU = 7
    GAMEPAD = 8
    SBUS = 9
    OLED = 10


@dataclass(frozen=True)
class Frame:
    function: int
    data: bytes


def encode(function: int, data: bytes = b"") -> bytes:
    """Build a complete wire frame."""
    if not 0 <= int(function) <= MAX_FUNCTION:
        raise ValueError(f"function {function} out of range 0..{MAX_FUNCTION}")
    if len(data) > MAX_DATA:
        raise ValueError(f"payload too long ({len(data)} > {MAX_DATA})")
    body = bytes([int(function), len(data)]) + bytes(data)
    return bytes([HEADER1, HEADER2]) + body + bytes([crc8(body)])


# decoder states
_SB1, _SB2, _FUNC, _LEN, _DATA, _CRC = range(6)


class Decoder:
    """Streaming RX decoder, tolerant of garbage and partial reads.

    Feed it bytes in any chunking; it returns completed frames and
    resynchronizes on the 0xAA 0x55 header after any violation
    (unknown function, CRC mismatch). Counters expose link health.
    """

    def __init__(self) -> None:
        self._state = _SB1
        self._function = 0
        self._length = 0
        self._data = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, chunk: bytes) -> list[Frame]:
        frames: list[Frame] = []
        for b in chunk:
            frame = self._feed_byte(b)
            if frame is not None:
                frames.append(frame)
        return frames

    def _resync(self, b: int) -> None:
        self.resyncs += 1
        # the offending byte may itself start a new header
        self._state = _SB2 if b == HEADER1 else _SB1

    def _feed_byte(self, b: int) -> Frame | None:
        if self._state == _SB1:
            if b == HEADER1:
                self._state = _SB2
        elif self._state == _SB2:
            if b == HEADER2:
                self._state = _FUNC
            elif b != HEADER1:  # 0xAA 0xAA 0x55 ... still syncs
                self._state = _SB1
        elif self._state == _FUNC:
            if b <= MAX_FUNCTION:
                self._function = b
                self._state = _LEN
            else:
                self._resync(b)
        elif self._state == _LEN:
            self._length = b
            self._data = bytearray()
            self._state = _DATA if b > 0 else _CRC
        elif self._state == _DATA:
            self._data.append(b)
            if len(self._data) == self._length:
                self._state = _CRC
        elif self._state == _CRC:
            body = bytes([self._function, self._length]) + bytes(self._data)
            if crc8(body) == b:
                self.frames_ok += 1
                self._state = _SB1
                return Frame(self._function, bytes(self._data))
            self.crc_errors += 1
            self._resync(b)
        return None
