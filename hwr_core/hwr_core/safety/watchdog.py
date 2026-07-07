"""Command watchdog: if the command source dies mid-drive, stop the robot.

Runs its own daemon thread so it fires even if the caller hangs. Armed by
the first feed; disarmed explicitly on stop (a stopped robot needs no dog).
"""
from __future__ import annotations

import threading
import time
from typing import Callable


class Watchdog:
    def __init__(self, action: Callable[[], None], timeout: float = 0.4,
                 tick: float = 0.02) -> None:
        self._action = action
        self.timeout = timeout
        self._tick = tick
        self._lock = threading.Lock()
        self._deadline: float | None = None  # None = disarmed
        self.fired_count = 0
        self._alive = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def feed(self) -> None:
        """Push the deadline out; arms the watchdog if it was disarmed."""
        with self._lock:
            self._deadline = time.monotonic() + self.timeout

    def disarm(self) -> None:
        with self._lock:
            self._deadline = None

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._deadline is not None

    def _run(self) -> None:
        while self._alive:
            time.sleep(self._tick)
            fire = False
            with self._lock:
                if self._deadline is not None and time.monotonic() >= self._deadline:
                    self._deadline = None  # one shot until re-fed
                    self.fired_count += 1
                    fire = True
            if fire:
                try:
                    self._action()
                except Exception:
                    pass  # never let the safety thread die

    def close(self) -> None:
        self._alive = False
        self._thread.join(timeout=1.0)
