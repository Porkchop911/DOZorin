"""Image-to-robot geometry for detector boxes (colour camera + depth image)."""
from __future__ import annotations

import json
import math

import numpy as np


def object_range(depth: np.ndarray, box: tuple[float, float, float, float],
                 color_shape: tuple[int, int]) -> float | None:
    """Near (20th percentile) depth (m) over the central third of a colour-image box, rescaled to
    the depth image. Not the median: open objects (chairs) show the wall through them, live 2026-10-18
    a chair at ~0.9 m read 2.34 m."""
    sy, sx = depth.shape[0] / color_shape[0], depth.shape[1] / color_shape[1]
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    # shortcut: assumes colour and depth share a field of view (no registration), upgrade with extrinsics if ranges look off
    patch = depth[int((y1 + h / 3) * sy):int((y2 - h / 3) * sy) + 1,
                  int((x1 + w / 3) * sx):int((x2 - w / 3) * sx) + 1]
    valid = patch[(patch > 0.1) & (patch < 8.0)]
    return round(float(np.percentile(valid, 20)), 2) if valid.size else None


def bearing(u: float, fx: float, cx: float) -> float:
    """Horizontal angle of image column u, left of the optical axis positive (rad)."""
    return -math.atan2(u - cx, fx)


def parse_grounding(text: str, width: int, height: int, bottom: float = 0.07) -> list[dict]:
    """VLM grounding reply ([{"label", "point_2d" or "bbox_2d": 0-1000}], ``` fences allowed) ->
    pixel `center` (the point, or the box centre) and `box` when given.

    Hits in the bottom `bottom` of the image are dropped: the gripper fingertips sit there and read
    as a sock (live 2026-10-18)."""
    start, end = text.find("["), text.rfind("]")
    if start < 0 or end < start:
        return []
    try:
        items = json.loads(text[start:end + 1])
    except json.JSONDecodeError:
        return []
    numbers = lambda v, n: (isinstance(v, list) and len(v) == n  # noqa: E731
                            and all(isinstance(x, (int, float)) for x in v))
    found = []
    for item in items if isinstance(items, list) else []:
        if not isinstance(item, dict):
            continue
        hit = {"label": str(item.get("label", ""))}
        if numbers(item.get("bbox_2d"), 4):
            x1, y1, x2, y2 = (v * size / 1000 for v, size in zip(item["bbox_2d"], (width, height) * 2))
            if x2 <= x1 or y2 <= y1:
                continue
            hit["box"] = [round(x1), round(y1), round(x2), round(y2)]
            hit["center"] = [round((x1 + x2) / 2), round((y1 + y2) / 2)]
        if numbers(item.get("point_2d"), 2):
            hit["center"] = [round(item["point_2d"][0] * width / 1000),
                             round(item["point_2d"][1] * height / 1000)]
        if "center" not in hit or (hit.get("box", hit["center"])[-1] >= height * (1 - bottom)):
            continue
        found.append(hit)
    return found
