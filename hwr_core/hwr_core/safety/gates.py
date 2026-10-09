"""Emergency-stop latch."""
from __future__ import annotations

import threading


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
