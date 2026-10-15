"""Numpy Monte-Carlo localization against a saved occupancy grid."""
from __future__ import annotations

import heapq
import math

import numpy as np


def _wrap(values):
    return np.arctan2(np.sin(values), np.cos(values))


def localization_motion_consistent(
        previous_pose, pose, previous_odom, odom, *,
        max_xy_residual: float = 0.05,
        max_yaw_residual: float = math.radians(12.0),
        xy_residual_per_meter: float = 0.15,
        yaw_residual_per_radian: float = 0.15) -> bool:
    """Check that a map-pose update agrees with accumulated odometry motion.

    Tolerance grows with odometry travelled since the last accepted pose:
    a fixed bound locks out every update once wheel drift over a long leg
    exceeds it (2026-10-14 free roam: 269 rejections, loc 0.95 -> 0.25).
    """
    if previous_pose is None or previous_odom is None or odom is None:
        return True
    values = (*previous_pose, *pose, *previous_odom, *odom,
              max_xy_residual, max_yaw_residual)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if max_xy_residual < 0.0 or max_yaw_residual < 0.0:
        raise ValueError("localization residual limits must be non-negative")
    odom_dx, odom_dy = odom[0] - previous_odom[0], odom[1] - previous_odom[1]
    frame_yaw = previous_pose[2] - previous_odom[2]
    c, s = math.cos(frame_yaw), math.sin(frame_yaw)
    expected_dx = c * odom_dx - s * odom_dy
    expected_dy = s * odom_dx + c * odom_dy
    actual_dx, actual_dy = pose[0] - previous_pose[0], pose[1] - previous_pose[1]
    xy_residual = math.hypot(actual_dx - expected_dx, actual_dy - expected_dy)
    actual_yaw = math.atan2(math.sin(pose[2] - previous_pose[2]),
                            math.cos(pose[2] - previous_pose[2]))
    expected_yaw = math.atan2(math.sin(odom[2] - previous_odom[2]),
                              math.cos(odom[2] - previous_odom[2]))
    yaw_residual = abs(math.atan2(math.sin(actual_yaw - expected_yaw),
                                  math.cos(actual_yaw - expected_yaw)))
    xy_limit = max_xy_residual + xy_residual_per_meter * math.hypot(odom_dx, odom_dy)
    yaw_limit = max_yaw_residual + yaw_residual_per_radian * abs(expected_yaw)
    return xy_residual <= xy_limit and yaw_residual <= yaw_limit


def aggregate_scan_panorama(history, current_pose,
                            max_translation: float = 0.15,
                            stride: int = 4) -> np.ndarray:
    """Transform rotation-only scan history into the current base frame."""
    current = np.asarray(current_pose, dtype=float)
    if current.shape != (3,) or not np.isfinite(current).all():
        raise ValueError("current_pose must be a finite x/y/yaw triple")
    if max_translation < 0.0 or stride < 1:
        raise ValueError("panorama bounds are invalid")
    combined = []
    cc, ss = math.cos(current[2]), math.sin(current[2])
    inverse_rotation = np.array(((cc, ss), (-ss, cc)))
    for pose, points in history:
        pose = np.asarray(pose, dtype=float)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if (pose.shape != (3,) or not np.isfinite(pose).all()
                or not np.isfinite(points).all()
                or math.hypot(pose[0] - current[0],
                              pose[1] - current[1]) > max_translation):
            continue
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = points[::stride] @ np.array(((c, s), (-s, c)))
        world += pose[:2]
        combined.append((world - current[:2]) @ inverse_rotation.T)
    return (np.vstack(combined) if combined
            else np.empty((0, 2), dtype=float))


def distance_field(occupancy: np.ndarray, resolution: float,
                   max_distance: float = 2.0) -> np.ndarray:
    occ = np.asarray(occupancy)
    field = np.full(occ.shape, float(max_distance), dtype=np.float32)
    queue = []
    for y, x in np.argwhere(occ >= 65):
        field[y, x] = 0.0
        heapq.heappush(queue, (0.0, int(x), int(y)))
    neighbors = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))
    h, w = occ.shape
    while queue:
        distance, x, y = heapq.heappop(queue)
        if distance > field[y, x] + 1e-6 or distance >= max_distance:
            continue
        for dx, dy, cost in neighbors:
            nx, ny = x + dx, y + dy
            candidate = distance + cost * resolution
            if 0 <= nx < w and 0 <= ny < h and candidate < field[ny, nx]:
                field[ny, nx] = candidate
                heapq.heappush(queue, (candidate, nx, ny))
    return field


class ParticleFilter:
    def __init__(self, occupancy: np.ndarray, resolution: float,
                 origin: tuple[float, float], count: int = 500, seed: int = 0):
        self.occupancy = np.asarray(occupancy)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.count = int(count)
        self.rng = np.random.default_rng(seed)
        self.field = distance_field(self.occupancy, self.resolution)
        self.particles = np.zeros((self.count, 3))
        self.weights = np.full(self.count, 1.0 / self.count)
        self.mean_score = 0.0
        self.initialize_global()

    def initialize_global(self) -> None:
        free = np.argwhere(self.occupancy == 0)
        if not len(free):
            raise ValueError("map has no known free cells")
        chosen = free[self.rng.integers(0, len(free), self.count)]
        self.particles[:, 0] = self.origin[0] + (chosen[:, 1] + 0.5) * self.resolution
        self.particles[:, 1] = self.origin[1] + (chosen[:, 0] + 0.5) * self.resolution
        self.particles[:, 2] = self.rng.uniform(-math.pi, math.pi, self.count)
        self.weights.fill(1.0 / self.count)

    def initialize_pose(self, pose: tuple[float, float, float],
                        std: tuple[float, float, float] = (0.1, 0.1, 0.1)) -> None:
        self.particles = self.rng.normal(pose, std, size=(self.count, 3))
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]),
                                          np.cos(self.particles[:, 2]))
        self.weights.fill(1.0 / self.count)

    def initialize_hypotheses(self, poses: np.ndarray, scores: np.ndarray | None = None,
                              std: tuple[float, float, float] =
                              (0.10, 0.10, math.radians(7.5))) -> None:
        """Seed the cloud around coarse global-localization hypotheses."""
        poses = np.asarray(poses, dtype=float).reshape(-1, 3)
        if not len(poses):
            raise ValueError("at least one pose hypothesis is required")
        if scores is None:
            probabilities = np.full(len(poses), 1.0 / len(poses))
        else:
            scores = np.asarray(scores, dtype=float)
            weights = np.exp((scores - scores.max()) / 0.05)
            probabilities = weights / weights.sum()
        selected = self.rng.choice(len(poses), size=self.count, p=probabilities)
        self.particles = self.rng.normal(poses[selected], std)
        self.particles[:, 2] = _wrap(self.particles[:, 2])
        self.weights.fill(1.0 / self.count)
        self.mean_score = float(np.sum(probabilities *
                                      (np.asarray(scores) if scores is not None else 0.0)))

    def global_relocalize(self, scan_points: np.ndarray, xy_step: float = 0.20,
                          yaw_step: float = math.radians(15.0),
                          hypotheses: int = 8) -> tuple[np.ndarray, np.ndarray]:
        """Coarsely correlate a scan over free map cells, then seed MCL."""
        poses, scores = coarse_global_search(
            self.occupancy, self.field, self.resolution, self.origin,
            scan_points, xy_step=xy_step, yaw_step=yaw_step,
            hypotheses=hypotheses)
        self.initialize_hypotheses(poses, scores)
        return poses, scores

    def predict(self, delta: tuple[float, float, float],
                noise: tuple[float, float, float] = (0.02, 0.02, 0.03)) -> None:
        dx, dy, dtheta = delta
        translation = math.hypot(dx, dy)
        rotation = abs(dtheta)
        if translation <= 1e-12 and rotation <= 1e-12:
            return
        # ``predict`` receives odometry at 50 Hz. Fixed per-message noise would
        # accumulate while stationary and make a pure yaw appear to translate.
        # Scale standard deviation with sqrt(motion), giving approximately the
        # same total uncertainty regardless of how a motion is packetized.
        translation_scale = math.sqrt(translation + 0.05 * rotation)
        yaw_scale = math.sqrt(rotation + 0.10 * translation)
        standard_deviation = (
            abs(float(noise[0])) * translation_scale,
            abs(float(noise[1])) * translation_scale,
            abs(float(noise[2])) * yaw_scale,
        )
        noisy = self.rng.normal(
            (dx, dy, dtheta), standard_deviation, size=(self.count, 3))
        c, s = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        self.particles[:, 0] += c * noisy[:, 0] - s * noisy[:, 1]
        self.particles[:, 1] += s * noisy[:, 0] + c * noisy[:, 1]
        self.particles[:, 2] += noisy[:, 2]
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]),
                                          np.cos(self.particles[:, 2]))

    def update(self, scan_points: np.ndarray, sigma: float = 0.15) -> None:
        points = np.asarray(scan_points, dtype=float).reshape(-1, 2)
        if len(points) > 60:
            points = points[np.linspace(0, len(points) - 1, 60).astype(int)]
        if not len(points):
            return
        c, s = np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])
        wx = (self.particles[:, 0, None] + c[:, None] * points[:, 0]
              - s[:, None] * points[:, 1])
        wy = (self.particles[:, 1, None] + s[:, None] * points[:, 0]
              + c[:, None] * points[:, 1])
        ix = np.floor((wx - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((wy - self.origin[1]) / self.resolution).astype(int)
        valid = ((ix >= 0) & (iy >= 0) & (ix < self.field.shape[1])
                 & (iy < self.field.shape[0]))
        distances = np.full(ix.shape, 2.0)
        distances[valid] = self.field[iy[valid], ix[valid]]
        scores = np.exp(-0.5 * (distances / sigma) ** 2).mean(axis=1)
        self.mean_score = float(np.sum(self.weights * scores))
        log_weights = np.log(self.weights + 1e-300) + np.log(scores + 1e-12)
        log_weights -= log_weights.max()
        self.weights = np.exp(log_weights)
        total = self.weights.sum()
        self.weights = (self.weights / total if total > 0
                        else np.full(self.count, 1.0 / self.count))

    @property
    def effective_sample_size(self) -> float:
        return float(1.0 / np.sum(self.weights ** 2))

    @property
    def pose(self) -> tuple[float, float, float]:
        x = float(self.weights @ self.particles[:, 0])
        y = float(self.weights @ self.particles[:, 1])
        yaw = math.atan2(float(self.weights @ np.sin(self.particles[:, 2])),
                         float(self.weights @ np.cos(self.particles[:, 2])))
        return x, y, yaw

    @property
    def healthy(self) -> bool:
        return self.effective_sample_size >= 0.2 * self.count and self.mean_score >= 0.35

    def resample_if_needed(self, threshold: float = 0.5) -> bool:
        if self.effective_sample_size >= threshold * self.count:
            return False
        positions = (self.rng.random() + np.arange(self.count)) / self.count
        cumulative = np.cumsum(self.weights)
        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.count)
        return True


def coarse_global_search(occupancy: np.ndarray, field: np.ndarray,
                         resolution: float, origin, scan_points: np.ndarray,
                         xy_step: float = 0.20,
                         yaw_step: float = math.radians(15.0),
                         hypotheses: int = 8, sigma: float = 0.15
                         ) -> tuple[np.ndarray, np.ndarray]:
    """Return the strongest scan hypotheses over known free space.

    Candidate positions are spatially downsampled free cells; headings cover a
    full turn.  Scores use the same likelihood field as the particle filter.
    """
    occupancy = np.asarray(occupancy)
    field = np.asarray(field)
    points = np.asarray(scan_points, dtype=float).reshape(-1, 2)
    if len(points) > 60:
        points = points[np.linspace(0, len(points) - 1, 60).astype(int)]
    if not len(points):
        raise ValueError("scan contains no hit points")
    stride = max(1, int(round(xy_step / resolution)))
    free = np.argwhere(occupancy == 0)
    free = free[(free[:, 0] % stride == 0) & (free[:, 1] % stride == 0)]
    if not len(free):
        raise ValueError("map has no free global-search candidates")
    candidates = np.column_stack((origin[0] + (free[:, 1] + 0.5) * resolution,
                                  origin[1] + (free[:, 0] + 0.5) * resolution))
    yaws = np.arange(-math.pi, math.pi, yaw_step)
    best_poses = np.empty((0, 3), dtype=float)
    best_scores = np.empty(0, dtype=float)
    h, w = field.shape
    for yaw in yaws:
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = points @ np.array(((c, s), (-s, c)))
        world = candidates[:, None, :] + rotated[None, :, :]
        ix = np.floor((world[..., 0] - origin[0]) / resolution).astype(int)
        iy = np.floor((world[..., 1] - origin[1]) / resolution).astype(int)
        valid = (ix >= 0) & (iy >= 0) & (ix < w) & (iy < h)
        distances = np.full(ix.shape, 2.0)
        distances[valid] = field[iy[valid], ix[valid]]
        scores = np.exp(-0.5 * (distances / sigma) ** 2).mean(axis=1)
        keep = min(hypotheses, len(scores))
        indices = np.argpartition(scores, -keep)[-keep:]
        poses = np.column_stack((candidates[indices], np.full(keep, yaw)))
        best_poses = np.vstack((best_poses, poses))
        best_scores = np.concatenate((best_scores, scores[indices]))
        keep = min(hypotheses, len(best_scores))
        indices = np.argpartition(best_scores, -keep)[-keep:]
        best_poses, best_scores = best_poses[indices], best_scores[indices]
    order = np.argsort(best_scores)[::-1]
    return best_poses[order], best_scores[order]
