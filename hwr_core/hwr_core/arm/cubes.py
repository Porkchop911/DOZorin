"""Colour-cube blobs (kit cubes) from a BGR frame via OpenCV LAB thresholds."""
from __future__ import annotations

import cv2
import numpy as np


def find_color_blobs(bgr: np.ndarray, ranges: dict, min_area: float = 150.0) -> list[dict]:
    """[{class: '<color>_cube', box, center, corners, area}] per blob, largest first per colour."""
    lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2LAB)
    kernel = np.ones((5, 5), np.uint8)
    found = []
    for color, limits in ranges.items():
        mask = cv2.inRange(lab, tuple(int(v) for v in limits["min"]),
                           tuple(int(v) for v in limits["max"]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            blobs.append({"class": f"{color}_cube", "conf": 1.0, "id": None,
                          "box": [x, y, x + w, y + h],
                          "center": [round(moments["m10"] / moments["m00"], 1),
                                     round(moments["m01"] / moments["m00"], 1)],
                          "corners": cv2.boxPoints(cv2.minAreaRect(contour)).round(1).tolist(),
                          "area": round(area)})
        found += sorted(blobs, key=lambda b: -b["area"])
    return found
