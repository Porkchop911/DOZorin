"""Timeout-based velocity command mux with explicit source priority."""
from __future__ import annotations

from collections.abc import Mapping, Sequence

Command = tuple[float, float, float]


class CommandMux:
    def __init__(self, priority: Sequence[str], timeouts: Mapping[str, float]):
        self.priority = tuple(priority)
        if not self.priority or set(self.priority) != set(timeouts):
            raise ValueError("priority and timeout sources must match")
        if any(float(timeout) <= 0.0 for timeout in timeouts.values()):
            raise ValueError("source timeouts must be positive")
        self.timeouts = {source: float(timeouts[source]) for source in self.priority}
        self._commands: dict[str, tuple[Command, float]] = {}

    def update(self, source: str, command: Command, stamp: float) -> None:
        if source not in self.timeouts:
            raise KeyError(source)
        self._commands[source] = (tuple(float(v) for v in command), float(stamp))

    def select(self, now: float) -> tuple[str | None, Command]:
        for source in self.priority:
            entry = self._commands.get(source)
            if entry is None:
                continue
            command, stamp = entry
            age = now - stamp
            if 0.0 <= age <= self.timeouts[source]:
                return source, command
        return None, (0.0, 0.0, 0.0)

    def clear(self) -> None:
        self._commands.clear()
