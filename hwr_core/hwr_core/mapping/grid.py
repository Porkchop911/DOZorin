"""Dynamically growing 2D log-odds occupancy grid."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class LogOddsGrid:
    def __init__(self, resolution: float = 0.05, size: int = 200,
                 origin: tuple[float, float] | None = None,
                 hit: float = 0.85, miss: float = -0.85):
        if resolution <= 0.0 or size < 2:
            raise ValueError("invalid grid geometry")
        self.resolution = float(resolution)
        self.origin = np.array(origin or (-size * resolution / 2,
                                          -size * resolution / 2), dtype=float)
        self.log_odds = np.zeros((size, size), dtype=np.float32)
        self.observed = np.zeros((size, size), dtype=bool)
        self.hit, self.miss = float(hit), float(miss)

    @classmethod
    def from_occupancy(
            cls, occupancy: np.ndarray, resolution: float,
            origin: tuple[float, float], *, hit: float = 0.85,
            miss: float = -0.85) -> "LogOddsGrid":
        """Rehydrate an exact saved grid without turning unknown into free."""
        values = np.asarray(occupancy)
        if values.ndim != 2 or not values.size:
            raise ValueError("occupancy must be a non-empty 2D array")
        if not np.isin(values, (-1, 0, 100)).all():
            raise ValueError("occupancy values must be -1, 0, or 100")
        grid = cls(resolution=resolution, size=2, origin=origin,
                   hit=hit, miss=miss)
        grid.log_odds = np.zeros(values.shape, dtype=np.float32)
        grid.observed = values >= 0
        grid.log_odds[values == 0] = -5.0
        grid.log_odds[values == 100] = 5.0
        return grid

    def world_to_cell(self, points) -> np.ndarray:
        return np.floor((np.asarray(points, dtype=float) - self.origin)
                        / self.resolution).astype(int)

    def cell_to_world(self, cells) -> np.ndarray:
        return self.origin + (np.asarray(cells, dtype=float) + 0.5) * self.resolution

    def _ensure(self, points) -> None:
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        cells = self.world_to_cell(pts)
        h, w = self.log_odds.shape
        left = max(0, -int(cells[:, 0].min()) + 10)
        bottom = max(0, -int(cells[:, 1].min()) + 10)
        right = max(0, int(cells[:, 0].max()) - w + 11)
        top = max(0, int(cells[:, 1].max()) - h + 11)
        if not any((left, right, bottom, top)):
            return
        self.log_odds = np.pad(self.log_odds, ((bottom, top), (left, right)))
        self.observed = np.pad(self.observed, ((bottom, top), (left, right)))
        self.origin -= np.array((left, bottom)) * self.resolution

    def integrate(self, pose: tuple[float, float, float], points) -> None:
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not len(points):
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array(((c, -s), (s, c)))
        endpoints = points @ rotation.T + (x, y)
        self._ensure(np.vstack((endpoints, (x, y))))
        start = self.world_to_cell(((x, y),))[0]
        for endpoint in self.world_to_cell(endpoints):
            ray = list(_bresenham(int(start[0]), int(start[1]),
                                   int(endpoint[0]), int(endpoint[1])))
            for cx, cy in ray[:-1]:
                self.log_odds[cy, cx] += self.miss
                self.observed[cy, cx] = True
            cx, cy = ray[-1]
            self.log_odds[cy, cx] += self.hit
            self.observed[cy, cx] = True
        np.clip(self.log_odds, -5.0, 5.0, out=self.log_odds)

    def occupancy(self, occupied_probability: float = 0.65,
                  free_probability: float = 0.35) -> np.ndarray:
        probability = 1.0 / (1.0 + np.exp(-self.log_odds))
        out = np.full(self.log_odds.shape, -1, dtype=np.int8)
        out[self.observed & (probability <= free_probability)] = 0
        out[self.observed & (probability >= occupied_probability)] = 100
        return out

    def occupied_points(self, center=None, radius: float | None = None) -> np.ndarray:
        """Return occupied cell centers in world coordinates.

        A local crop keeps scan-to-map likelihood fields bounded as a map grows.
        When either crop argument is omitted the complete occupied set is returned.
        """
        cells_yx = np.argwhere(self.occupancy() == 100)
        if not len(cells_yx):
            return np.empty((0, 2), dtype=float)
        points = self.cell_to_world(cells_yx[:, ::-1])
        if center is not None and radius is not None:
            center = np.asarray(center, dtype=float)
            points = points[np.sum((points - center) ** 2, axis=1) <= radius ** 2]
        return points

    def save(self, base_path: str | Path) -> tuple[Path, Path]:
        base = Path(base_path)
        pgm, metadata = base.with_suffix(".pgm"), base.with_suffix(".yaml")
        occ = self.occupancy()
        pixels = np.full(occ.shape, 205, dtype=np.uint8)
        pixels[occ == 0], pixels[occ == 100] = 254, 0
        with pgm.open("wb") as stream:
            stream.write(f"P5\n{pixels.shape[1]} {pixels.shape[0]}\n255\n".encode())
            stream.write(np.flipud(pixels).tobytes())
        metadata.write_text(yaml.safe_dump({
            "image": pgm.name, "resolution": self.resolution,
            "origin": [float(self.origin[0]), float(self.origin[1]), 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.35,
        }, sort_keys=False), encoding="utf-8")
        return pgm, metadata
