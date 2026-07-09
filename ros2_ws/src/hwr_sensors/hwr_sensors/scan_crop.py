"""Scan band-pass, pure numpy (plan task 4.2 — replaces laser_filters).

The COIN-D6 sees 360° but the chassis blocks a ~140° rear sector; the
usable band is [1.25, 5.08] rad in the lidar frame (vendor bounds,
verified empirically in task 4.3). ROS-free so it unit-tests on the PC;
the node wraps it.

Handles the band wrapping across the scan's angle origin (a driver may
publish -pi..pi or 0..2pi): kept beams are re-ordered into one ascending
arc starting at the band's lower edge. Bearings are preserved modulo 2pi,
which is all any consumer (TF, matcher) uses.
"""
from __future__ import annotations

import math

import numpy as np

BAND_LO = 1.25              # rad, lidar frame
BAND_HI = 5.08
MIN_RANGE = 0.12            # m, self-hits below this become inf


def crop_scan(angle_min: float, angle_increment: float, ranges,
              lo: float = BAND_LO, hi: float = BAND_HI,
              min_range: float = MIN_RANGE) -> tuple[float, np.ndarray]:
    """Crop to the [lo, hi] arc (mod 2pi) and blank self-hits.

    Returns (new_angle_min, new_ranges): one contiguous ascending arc with
    the original angle_increment. new_angle_min + i*increment gives each
    kept beam's bearing, equal mod 2pi to its original bearing.
    """
    ranges = np.asarray(ranges, dtype=float)
    n = len(ranges)
    if n == 0 or angle_increment <= 0.0:
        return angle_min, ranges.copy()

    two_pi = 2.0 * math.pi
    angles = angle_min + np.arange(n) * angle_increment
    rel = np.mod(angles - lo, two_pi)           # arc distance past the lower edge
    width = (hi - lo) % two_pi
    keep = rel <= width + 1e-9
    if not keep.any():
        return lo, np.empty(0)

    idx = np.nonzero(keep)[0]
    idx = idx[np.argsort(rel[idx])]             # ascending along the arc
    out = ranges[idx].copy()
    out[~np.isfinite(out) | (out < min_range)] = np.inf
    return lo + float(rel[idx[0]]), out
