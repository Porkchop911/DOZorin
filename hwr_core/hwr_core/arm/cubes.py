"""Colour-cube blobs (kit cubes) from a BGR frame via OpenCV LAB thresholds."""
from __future__ import annotations

import cv2
import numpy as np


def find_color_blobs(bgr: np.ndarray, ranges: dict, min_area: float = 150.0,
                     max_area: float = 15000.0) -> list[dict]:
    """[{class: '<color>_cube', box, center, corners, area}] per blob, largest first per colour.

    a/b are shifted so the image median (mostly floor) reads neutral: auto white balance
    moved red b by 54 between two lamps live, fixed boxes can't follow that."""
    lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.int16)
    for channel in (1, 2):
        lab[:, :, channel] += 128 - int(np.median(lab[:, :, channel]))
    lab = np.clip(lab, 0, 255).astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    found = []
    for color, limits in ranges.items():
        mask = cv2.inRange(lab, tuple(int(v) for v in limits["min"]),
                           tuple(int(v) for v in limits["max"]))
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel),
                                cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not min_area <= area <= max_area:
                continue
            # Cubes are compact (an oblique cube is a hexagon); shadows, glare and walls are not.
            hull = cv2.contourArea(cv2.convexHull(contour))
            width, height = cv2.minAreaRect(contour)[1]
            if area / max(hull, 1.0) < 0.75 or max(width, height) > 2.2 * max(min(width, height), 1.0):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            blobs.append({"class": f"{color}_cube", "conf": 1.0, "id": None,
                          "box": [x, y, x + w, y + h],
                          "center": [round(moments["m10"] / moments["m00"], 1),
                                     round(moments["m01"] / moments["m00"], 1)],
                          # Outline polygon, not minAreaRect: an oblique cube is a hexagon whose
                          # min-area box sits on the diagonal (live: green read -43 deg, was ~-5).
                          "outline": cv2.approxPolyDP(contour, 3.0, True).reshape(-1, 2).tolist(),
                          "area": round(area)})
        found += sorted(blobs, key=lambda b: -b["area"])
    return found
