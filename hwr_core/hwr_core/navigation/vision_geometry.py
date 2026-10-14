"""Image-to-robot geometry for detector boxes (colour camera + depth image)."""
from __future__ import annotations

import math

import numpy as np


def object_range(depth: np.ndarray, box: tuple[float, float, float, float],
                 color_shape: tuple[int, int]) -> float | None:
    """Median depth (m) over the central third of a colour-image box, rescaled to the depth image."""
    sy, sx = depth.shape[0] / color_shape[0], depth.shape[1] / color_shape[1]
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    # shortcut: assumes colour and depth share a field of view (no registration), upgrade with extrinsics if ranges look off
    patch = depth[int((y1 + h / 3) * sy):int((y2 - h / 3) * sy) + 1,
                  int((x1 + w / 3) * sx):int((x2 - w / 3) * sx) + 1]
    valid = patch[(patch > 0.1) & (patch < 8.0)]
    return round(float(np.median(valid)), 2) if valid.size else None


def bearing(u: float, fx: float, cx: float) -> float:
    """Horizontal angle of image column u, left of the optical axis positive (rad)."""
    return -math.atan2(u - cx, fx)
