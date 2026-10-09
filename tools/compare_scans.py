#!/usr/bin/env python3
"""Correlate two captured base-frame lidar scans over a wide yaw window."""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from hwr_core.matching import CorrelativeMatcher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("reference")
    parser.add_argument("current")
    parser.add_argument("--prior-yaw-deg", type=float, default=0.0)
    parser.add_argument("--window-yaw-deg", type=float, default=55.0)
    args = parser.parse_args()
    reference = np.asarray(np.load(args.reference), dtype=float).reshape(-1, 2)
    current = np.asarray(np.load(args.current), dtype=float).reshape(-1, 2)
    matcher = CorrelativeMatcher(
        window_xy=0.12,
        window_theta=math.radians(args.window_yaw_deg),
        coarse_xy=0.03,
        coarse_theta=math.radians(1.0),
        fine_xy=0.005,
        fine_theta=math.radians(0.25),
    )
    matcher.set_reference(reference)
    result = matcher.match(
        current, prior=(0.0, 0.0, math.radians(args.prior_yaw_deg)))
    print(json.dumps({
        "reference_points": len(reference),
        "current_points": len(current),
        "dx_m": result.dx,
        "dy_m": result.dy,
        "yaw_rad": result.dtheta,
        "yaw_deg": math.degrees(result.dtheta),
        "score": result.score,
        "degenerate": result.degenerate,
        "valid": result.valid,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
