"""Battery gate and emergency-stop latch."""
from __future__ import annotations

import threading
from typing import Callable


class EStop:
    """Latched stop: any source trips it; motion stays refused until reset()."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._reason: str | None = None

    def trip(self, reason: str) -> None:
        with self._lock:
            if self._reason is None:
                self._reason = reason

    def reset(self) -> None:
        with self._lock:
            self._reason = None

    @property
    def tripped(self) -> bool:
        with self._lock:
            return self._reason is not None

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason


class BatteryGate:
    """<refuse_below: refuse new motion. <stop_below: hard stop via callback
    and latch (stays latched even if the reading recovers)."""

    def __init__(self, on_hard_stop: Callable[[], None] | None = None,
                 refuse_below_mv: int = 11000, stop_below_mv: int = 10500) -> None:
        self.refuse_below_mv = refuse_below_mv
        self.stop_below_mv = stop_below_mv
        self._on_hard_stop = on_hard_stop
        self._lock = threading.Lock()
        self._latest: int | None = None
        self._hard_latched = False

    def update(self, mv: int) -> None:
        """Feed a battery reading (called from the transport RX thread)."""
        fire = False
        with self._lock:
            self._latest = mv
            if mv < self.stop_below_mv and not self._hard_latched:
                self._hard_latched = True
                fire = True
        if fire and self._on_hard_stop is not None:
            try:
                self._on_hard_stop()
            except Exception:
                pass

    @property
    def latest_mv(self) -> int | None:
        with self._lock:
            return self._latest

    @property
    def hard_latched(self) -> bool:
        with self._lock:
            return self._hard_latched

    def allow_motion(self) -> bool:
        """True while the battery permits new motion commands. Unknown battery
        is allowed briefly (reports arrive ~1 Hz right after connect)."""
        with self._lock:
            if self._hard_latched:
                return False
            return self._latest is None or self._latest >= self.refuse_below_mv
