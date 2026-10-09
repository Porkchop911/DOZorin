"""Serial transport: TX lock + RX thread feeding per-function mailboxes.

The serial object is injected (anything with ``read/write/in_waiting/close``),
so tests run against a fake. ``open_rrc()`` builds the real pyserial port
(1 Mbaud, RTS/DTR forced low before open — required by the board).
"""
from __future__ import annotations

import threading
import time
from typing import Callable

from .frames import Decoder, Frame, encode


class _Mailbox:
    __slots__ = ("data", "stamp", "seq")

    def __init__(self) -> None:
        self.data: bytes | None = None
        self.stamp = 0.0
        self.seq = 0


class Transport:
    """Owns the serial port: single writer, single RX thread."""

    def __init__(self, port) -> None:
        self._port = port
        self._tx_lock = threading.Lock()
        self._cond = threading.Condition()
        self._mail = {f: _Mailbox() for f in range(11)}
        self._subs: dict[int, list[Callable[[bytes, float], None]]] = {}
        self.callback_errors = 0
        self.last_callback_error: Exception | None = None
        self.decoder = Decoder()
        self._alive = True
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

    # ------------------------------------------------------------- TX

    def send(self, function: int, data: bytes = b"") -> None:
        frame = encode(function, data)
        with self._tx_lock:
            self._port.write(frame)

    # ------------------------------------------------------------- RX

    def _rx_loop(self) -> None:
        while self._alive:
            try:
                waiting = self._port.in_waiting
                chunk = self._port.read(waiting if waiting else 1)
            except Exception:
                if self._alive:
                    time.sleep(0.05)
                    continue
                return
            if not chunk:
                continue
            for frame in self.decoder.feed(chunk):
                self._dispatch(frame)

    def _dispatch(self, frame: Frame) -> None:
        now = time.monotonic()
        with self._cond:
            box = self._mail[frame.function]
            box.data, box.stamp = frame.data, now
            box.seq += 1
            self._cond.notify_all()
        for cb in self._subs.get(frame.function, ()):
            try:
                cb(frame.data, now)
            except Exception as exc:
                # A consumer must never be able to kill serial reception: the
                # same thread carries asynchronous sensor telemetry.
                self.callback_errors += 1
                self.last_callback_error = exc

    # ------------------------------------------------------------- API

    def latest(self, function: int) -> tuple[bytes, float] | None:
        """Most recent report for a function: (data, monotonic stamp)."""
        box = self._mail[function]
        return (box.data, box.stamp) if box.data is not None else None

    def wait_report(self, function: int, timeout: float = 1.0,
                    predicate: Callable[[bytes], bool] | None = None) -> bytes | None:
        """Block for the next report of ``function`` arriving after this call.

        With ``predicate``, keeps waiting until a matching report arrives
        (used for request/response reads on shared report channels).
        """
        deadline = time.monotonic() + timeout
        with self._cond:
            box = self._mail[function]
            seen = box.seq
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)
                if box.seq != seen:
                    seen = box.seq
                    if box.data is not None and (predicate is None or predicate(box.data)):
                        return box.data

    def subscribe(self, function: int, callback: Callable[[bytes, float], None]) -> None:
        """Register a callback fired from the RX thread for every report."""
        self._subs.setdefault(function, []).append(callback)

    def close(self) -> None:
        self._alive = False
        try:
            self._port.close()  # unblocks the reader
        except Exception:
            pass
        self._rx.join(timeout=2.0)


def open_rrc(device: str = "/dev/rrc", baudrate: int = 1_000_000,
             timeout: float = 0.1) -> Transport:
    import serial

    port = serial.Serial(None, baudrate, timeout=timeout)
    port.rts = False  # both lines must be low before open or the STM32 resets
    port.dtr = False
    port.setPort(device)
    port.open()
    return Transport(port)
