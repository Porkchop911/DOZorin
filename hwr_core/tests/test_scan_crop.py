"""Scan band-pass crop (hwr_sensors.scan_crop) — pure PC tests (plan 4.2).

The tricky case is the band wrapping across the driver's angle origin:
a -pi..pi scan splits the [1.25, 5.08] band into two index segments that
must come back out as one ascending arc with bearings preserved mod 2pi.
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "ros2_ws" / "src" / "hwr_sensors"))
from hwr_sensors.scan_crop import BAND_HI, BAND_LO, crop_indices, crop_scan  # noqa: E402

TWO_PI = 2.0 * math.pi
N = 720
INC = TWO_PI / N


def scene(bearing):
    """Synthetic room: range as a smooth function of absolute bearing."""
    return 2.0 + 0.5 * math.sin(bearing) + 0.25 * math.cos(2.0 * bearing)


def make_scan(angle_min):
    angles = angle_min + np.arange(N) * INC
    return np.array([scene(a) for a in angles])


def bearings_of(new_min, ranges):
    return np.mod(new_min + np.arange(len(ranges)) * INC, TWO_PI)


def test_zero_origin_scan_crops_to_band():
    new_min, out = crop_scan(0.0, INC, make_scan(0.0))
    assert len(out) == pytest.approx((BAND_HI - BAND_LO) / INC, abs=2)
    b = bearings_of(new_min, out)
    assert b.min() >= BAND_LO - 1e-6
    assert b.max() <= BAND_HI + 1e-6
    for bi, ri in zip(b, out):
        assert ri == pytest.approx(scene(bi), abs=1e-9)


def test_wrapped_origin_matches_zero_origin():
    # -pi..pi scan: the band splits across the array ends; output must be
    # the same physical beams as the 0..2pi case
    min_a, out_a = crop_scan(0.0, INC, make_scan(0.0))
    min_b, out_b = crop_scan(-math.pi, INC, make_scan(-math.pi))
    assert len(out_a) == len(out_b)
    pairs_a = sorted(zip(np.round(bearings_of(min_a, out_a), 6), out_a))
    pairs_b = sorted(zip(np.round(bearings_of(min_b, out_b), 6), out_b))
    for (ba, ra), (bb, rb) in zip(pairs_a, pairs_b):
        assert ba == pytest.approx(bb, abs=1e-5)
        assert ra == pytest.approx(rb, abs=1e-9)


def test_output_is_one_ascending_arc():
    new_min, out = crop_scan(-math.pi, INC, make_scan(-math.pi))
    # ascending by construction: angle i = new_min + i*INC; ranges must
    # follow the scene at those bearings (i.e. reordering was correct)
    b = bearings_of(new_min, out)
    for bi, ri in zip(b, out):
        assert ri == pytest.approx(scene(bi), abs=1e-9)


def test_self_hits_and_junk_become_inf():
    ranges = make_scan(0.0)
    idx_in_band = int((BAND_LO + 0.5) / INC)
    ranges[idx_in_band] = 0.05                    # closer than the chassis
    ranges[idx_in_band + 1] = np.nan
    ranges[idx_in_band + 2] = -1.0
    new_min, out = crop_scan(0.0, INC, ranges)
    j = int(round((idx_in_band * INC - new_min) / INC))
    assert np.isinf(out[j]) and np.isinf(out[j + 1]) and np.isinf(out[j + 2])
    assert np.isfinite(out[j + 3])


def test_band_outside_scan_returns_empty():
    # a hypothetical forward-only scan that never enters the band
    angles_n = 100
    new_min, out = crop_scan(0.0, 0.01, np.ones(angles_n), lo=2.0, hi=3.0)
    assert len(out) == 0


def test_empty_scan():
    new_min, out = crop_scan(0.0, INC, [])
    assert len(out) == 0


def test_crop_indices_preserve_parallel_arrays():
    intensities = np.arange(N)
    new_min, idx = crop_indices(-math.pi, INC, N)
    bearings = np.mod(new_min + np.arange(len(idx)) * INC, TWO_PI)
    original_bearings = np.mod(-math.pi + idx * INC, TWO_PI)
    assert np.allclose(bearings, original_bearings)
    assert np.array_equal(intensities[idx], idx)
