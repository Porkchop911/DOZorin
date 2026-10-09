"""Validation applied at the final hardware-command boundary."""
from __future__ import annotations

import math
from collections.abc import Iterable


def validate_wheel_rps(values: Iterable[float], max_rps: float = 4.0
                       ) -> tuple[float, float, float, float]:
    """Return four safe finite wheel speeds or raise ``ValueError``."""
    rps = tuple(float(value) for value in values)
    if len(rps) != 4:
        raise ValueError(f"expected four wheel speeds, got {len(rps)}")
    if not math.isfinite(max_rps) or max_rps <= 0.0:
        raise ValueError("max_rps must be finite and positive")
    if not all(math.isfinite(value) for value in rps):
        raise ValueError("wheel speeds must be finite")
    if any(abs(value) > max_rps for value in rps):
        raise ValueError(f"wheel speed exceeds {max_rps} RPS hard limit")
    return rps
