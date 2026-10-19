"""Image-to-robot geometry for detector boxes (colour camera + depth image)."""
from __future__ import annotations

import json
from collections import deque
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


def parse_grounding(text: str, width: int, height: int, bottom: float = 0.15,
                    strip_labels: tuple[str, ...] = ("sock",)) -> list[dict]:
    """VLM grounding reply ([{"label", "point_2d" or "bbox_2d": 0-1000}], ``` fences allowed) ->
    pixel `center` (the point, or the box centre) and `box` when given.

    Sock hits in the bottom-centre strip are dropped: the fingertips and the bumper plate read as a
    sock (live 2026-10-18); a real sock there is inside the 0.19 m pick guard anyway. Other labels
    stay: the truck 0.3 m ahead sits right there in the steep views (live 2026-10-19)."""
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
        if "center" not in hit:
            continue
        cx, cy = hit["center"]
        # Only the fingertips' patch (bottom centre): a truck at the bottom edge is real (2026-10-18).
        if hit["label"] in strip_labels and cy >= height * (1 - bottom) and 0.35 * width <= cx <= 0.75 * width:
            continue
        found.append(hit)
    return found


def depth_below(depth: np.ndarray, u: float, v: float, color_shape: tuple[int, int],
                max_px: int = 80, step: int = 4) -> tuple[float, float] | None:
    """(v, range) of the first valid depth at or below colour pixel (u, v), within max_px.

    Dark metal (the toy truck) leaves holes in the depth; the floor just below it is where the
    object stands, so its range is the object's near edge (live 2026-10-18)."""
    for dv in range(0, max_px + 1, step):
        r = object_range(depth, (u - 15, v + dv - 15, u + 15, v + dv + 15), color_shape)
        if r is not None:
            return v + dv, r
    return None


def region_samples(rgb: np.ndarray, seed: tuple[int, int], tol: float = 40.0, n: int = 12,
                   step: int = 4) -> list[tuple[int, int]]:
    """Up to n pixels spread over the connected region of seed-like colour around `seed` (u, v).

    A long sock lying from under the bumper to 0.35 m out has its one VLM point at the far end; the
    pick wants the part ~0.2 m out (live 2026-10-18). Flood fill on a `step`-downsampled image."""
    small = rgb[::step, ::step].astype(float)
    h, w = small.shape[:2]
    sv, su = min(seed[1] // step, h - 1), min(seed[0] // step, w - 1)
    ref = small[sv, su]
    seen = np.zeros((h, w), bool)
    seen[sv, su] = True
    queue, cells = deque([(sv, su)]), []
    while queue:
        v, u = queue.popleft()
        cells.append((v, u))
        for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nv, nu = v + dv, u + du
            if 0 <= nv < h and 0 <= nu < w and not seen[nv, nu] \
                    and np.linalg.norm(small[nv, nu] - ref) <= tol:
                seen[nv, nu] = True
                queue.append((nv, nu))
    cells.sort()  # top to bottom: the samples run along the object in the image
    pick = cells[::max(1, len(cells) // n)][:n]
    return [(u * step + step // 2, v * step + step // 2) for v, u in pick]
