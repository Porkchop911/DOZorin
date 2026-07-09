"""2D correlative scan matcher (plan task 5.1, ADR-004).

The displacement source for odometry without encoders: given two point
sets (scan endpoints in each scan's base frame), find the rigid transform
that maps the new scan onto the reference scan.

Method (Olson-style):
  1. Rasterize the reference points into a Gaussian likelihood field.
  2. Coarse grid search over (dx, dy, dtheta) around an optional prior,
     then a fine refinement around the coarse peak. Scores are the mean
     point likelihood, bilinearly sampled, in [0, 1].
  3. Covariance from the score-weighted second moments of the whole
     search surface — honest about degeneracy (a corridor leaves the
     along-axis variance large; `degenerate` flags it).

ROS-free on purpose: hwr_odom, hwr_slam (scan-to-map) and hwr_loc
(relocalization) all reuse this core. Pure numpy; budget target is
~5 Hz on one Jetson core with a model-odom-seeded (small) window.

Frame convention: the result transforms new-scan points into the
reference frame, p_ref = R(dtheta) @ p_new + (dx, dy) — i.e. the robot
motion from the reference pose to the new pose, expressed in the
reference base frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def scan_to_points(angles: np.ndarray, ranges: np.ndarray, max_range: float,
                   sensor_yaw: float = 0.0,
                   sensor_xy: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Scan (sensor-frame angles/ranges) -> hit endpoints in the BASE frame.

    `sensor_yaw`/`sensor_xy` are the lidar mount on the chassis — for the
    real robot that is yaw=pi at (0.048156, 0); sim2d raycasts from the
    base origin, so only the yaw applies there. No-return beams
    (range >= max_range) are dropped.
    """
    angles = np.asarray(angles, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    hit = ranges < max_range
    bearings = sensor_yaw + angles[hit]
    return np.column_stack((sensor_xy[0] + ranges[hit] * np.cos(bearings),
                            sensor_xy[1] + ranges[hit] * np.sin(bearings)))


@dataclass
class MatchResult:
    dx: float
    dy: float
    dtheta: float
    score: float                    # mean point likelihood at optimum, 0..1
    cov: np.ndarray                 # 3x3 over (x, y, theta)
    degenerate: bool                # translation ill-constrained along one axis
    valid: bool = True              # False when there was nothing to match

    @property
    def delta(self) -> tuple[float, float, float]:
        return self.dx, self.dy, self.dtheta


class CorrelativeMatcher:
    """Build once per reference (keyframe) scan, match many scans against it."""

    def __init__(self,
                 resolution: float = 0.02,      # likelihood grid, m/cell
                 sigma: float = 0.05,           # Gaussian point smear, m
                 window_xy: float = 0.15,       # search half-window, m
                 window_theta: float = math.radians(10.0),
                 coarse_xy: float = 0.03,
                 coarse_theta: float = math.radians(1.0),
                 fine_xy: float = 0.005,
                 fine_theta: float = math.radians(0.25),
                 cov_temp: float = 0.05,        # softmax temperature on scores
                 degenerate_ratio: float = 16.0,
                 min_points: int = 10):
        self.resolution = resolution
        self.sigma = sigma
        self.window_xy = window_xy
        self.window_theta = window_theta
        self.coarse_xy = coarse_xy
        self.coarse_theta = coarse_theta
        self.fine_xy = fine_xy
        self.fine_theta = fine_theta
        self.cov_temp = cov_temp
        self.degenerate_ratio = degenerate_ratio
        self.min_points = min_points
        self._grid: np.ndarray | None = None
        self._org = np.zeros(2)

    # -------------------------------------------------------- reference

    def set_reference(self, points: np.ndarray) -> None:
        """Rasterize reference-scan endpoints into the likelihood field."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) < self.min_points:
            self._grid = None
            return
        pad = 3.0 * self.sigma + self.window_xy + 0.05
        lo = pts.min(axis=0) - pad
        hi = pts.max(axis=0) + pad
        w = int(math.ceil((hi[0] - lo[0]) / self.resolution)) + 1
        h = int(math.ceil((hi[1] - lo[1]) / self.resolution)) + 1
        grid = np.zeros((h, w), dtype=np.float32)
        r = int(math.ceil(3.0 * self.sigma / self.resolution))
        inv2s2 = 1.0 / (2.0 * self.sigma * self.sigma)
        for px, py in pts:
            cx = int(round((px - lo[0]) / self.resolution))
            cy = int(round((py - lo[1]) / self.resolution))
            x0, x1 = max(cx - r, 0), min(cx + r, w - 1)
            y0, y1 = max(cy - r, 0), min(cy + r, h - 1)
            xs = lo[0] + np.arange(x0, x1 + 1) * self.resolution
            ys = lo[1] + np.arange(y0, y1 + 1) * self.resolution
            d2 = (xs[None, :] - px) ** 2 + (ys[:, None] - py) ** 2
            np.maximum(grid[y0:y1 + 1, x0:x1 + 1],
                       np.exp(-d2 * inv2s2, dtype=np.float32),
                       out=grid[y0:y1 + 1, x0:x1 + 1])
        self._grid = grid
        self._org = lo

    # -------------------------------------------------------- matching

    def match(self, points: np.ndarray,
              prior: tuple[float, float, float] = (0.0, 0.0, 0.0)
              ) -> MatchResult:
        """Match new-scan points (base frame) against the reference.

        `prior` (e.g. from model odometry) centers the search window —
        seeding both extends effective range and speeds convergence.
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if self._grid is None or len(pts) < self.min_points:
            return MatchResult(0.0, 0.0, 0.0, 0.0, np.eye(3) * 1e3,
                               degenerate=True, valid=False)

        # covariance uses the coarse surface only: fine-pass samples cluster
        # around the peak and would double-count it, hiding flat (degenerate)
        # directions like a corridor axis
        cands: list[np.ndarray] = []
        scores: list[np.ndarray] = []

        # coarse pass around the prior
        best = self._search(pts,
                            center=np.asarray(prior, dtype=float),
                            half_xy=self.window_xy, step_xy=self.coarse_xy,
                            half_th=self.window_theta, step_th=self.coarse_theta,
                            cands=cands, scores=scores)
        # fine pass around the coarse peak
        best = self._search(pts,
                            center=best[:3],
                            half_xy=self.coarse_xy, step_xy=self.fine_xy,
                            half_th=self.coarse_theta, step_th=self.fine_theta,
                            cands=None, scores=None)

        cov, degenerate = self._covariance(np.vstack(cands),
                                           np.concatenate(scores))
        dth = math.atan2(math.sin(best[2]), math.cos(best[2]))
        return MatchResult(float(best[0]), float(best[1]), dth,
                           float(best[3]), cov, degenerate)

    def _search(self, pts: np.ndarray, center: np.ndarray,
                half_xy: float, step_xy: float,
                half_th: float, step_th: float,
                cands: list | None, scores: list | None) -> np.ndarray:
        """Exhaustive grid search; returns (dx, dy, dtheta, score) of the peak."""
        n = int(round(half_xy / step_xy))
        offs = np.arange(-n, n + 1) * step_xy
        ox, oy = np.meshgrid(offs, offs)
        trans = np.column_stack((ox.ravel(), oy.ravel())) + center[:2]
        m = int(round(half_th / step_th))
        thetas = center[2] + np.arange(-m, m + 1) * step_th

        best = np.array([center[0], center[1], center[2], -1.0])
        for th in thetas:
            c, s = math.cos(th), math.sin(th)
            rot = pts @ np.array([[c, s], [-s, c]])          # (R @ p.T).T
            coords = rot[None, :, :] + trans[:, None, :]     # (M, N, 2)
            sc = self._sample(coords).mean(axis=1)           # (M,)
            if cands is not None:
                cands.append(np.column_stack((trans, np.full(len(trans), th))))
                scores.append(sc)
            i = int(np.argmax(sc))
            if sc[i] > best[3]:
                best = np.array([trans[i, 0], trans[i, 1], th, sc[i]])
        return best

    def _sample(self, xy: np.ndarray) -> np.ndarray:
        """Bilinear likelihood lookup at world coords, 0 outside the grid."""
        g = (xy - self._org) / self.resolution
        gx, gy = g[..., 0], g[..., 1]
        x0 = np.floor(gx).astype(np.int64)
        y0 = np.floor(gy).astype(np.int64)
        fx, fy = gx - x0, gy - y0
        h, w = self._grid.shape
        valid = (x0 >= 0) & (x0 < w - 1) & (y0 >= 0) & (y0 < h - 1)
        x0 = np.clip(x0, 0, w - 2)
        y0 = np.clip(y0, 0, h - 2)
        g00 = self._grid[y0, x0]
        g01 = self._grid[y0, x0 + 1]
        g10 = self._grid[y0 + 1, x0]
        g11 = self._grid[y0 + 1, x0 + 1]
        v = (g00 * (1 - fx) + g01 * fx) * (1 - fy) \
            + (g10 * (1 - fx) + g11 * fx) * fy
        return np.where(valid, v, 0.0)

    def _covariance(self, cands: np.ndarray, scores: np.ndarray
                    ) -> tuple[np.ndarray, bool]:
        """Score-weighted second moments of the search surface."""
        w = np.exp((scores - scores.max()) / self.cov_temp)
        w /= w.sum()
        mu = w @ cands
        d = cands - mu
        cov = (w[:, None] * d).T @ d
        # numeric floor: half a fine step in each dimension
        cov += np.diag([self.fine_xy ** 2 / 4.0,
                        self.fine_xy ** 2 / 4.0,
                        self.fine_theta ** 2 / 4.0])
        evals = np.linalg.eigvalsh(cov[:2, :2])
        degenerate = bool(evals[1] / evals[0] > self.degenerate_ratio)
        return cov, degenerate
