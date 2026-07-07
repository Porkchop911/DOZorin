"""sim2d — kinematic 2D simulator for the ROSOrin Pro (no ROS, pure python).

Planner/controller/SLAM code develops against this before robot time
(plan task 8.5, built early). Models:
  - mecanum body kinematics (exact, uses hwr_core.kinematics)
  - command slew (accel ramp) approximating the STM32 wheel loop
  - ray-cast lidar with the REAL mount: 180 deg rotated, ~219 deg usable FOV
    (rear band blind), 10 Hz frame semantics, optional range noise
  - occupancy maps from arrays or PGM+YAML (map_server compatible)

Deterministic given a seed. A ROS wrapper node can publish these as real
topics later; the core stays import-and-test friendly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# real robot constants
LIDAR_YAW_OFFSET = math.pi          # mounted 180 deg rotated
LIDAR_FOV_MIN = 1.25                # rad, vendor band-pass (in lidar frame)
LIDAR_FOV_MAX = 5.08
LIDAR_MAX_RANGE = 8.0               # m (COIN-D6 spec 12; conservative indoor)
LIDAR_BEAMS = 400                   # ~0.9 deg like the real unit
MAX_ACCEL = 0.5                     # m/s^2 slew, mirrors our Limits defaults
MAX_ALPHA = 2.0                     # rad/s^2


@dataclass
class OccupancyMap:
    """Row-major grid, True = occupied. origin = world xy of cell (0,0) corner."""
    grid: np.ndarray                # bool, shape (rows, cols); row 0 = min y
    resolution: float = 0.05        # m/cell
    origin: tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_pgm(cls, pgm_path: str, yaml_path: str) -> "OccupancyMap":
        import yaml
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        with open(pgm_path, "rb") as f:
            magic = f.readline().strip()
            if magic not in (b"P5", b"P2"):
                raise ValueError(f"not a PGM: {magic!r}")
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            w, h = map(int, line.split())
            maxval = int(f.readline())
            if magic == b"P5":
                data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
            else:
                data = np.array(f.read().split(), dtype=np.uint8).reshape(h, w)
        occ = (maxval - data.astype(float)) / maxval  # dark = occupied
        grid = np.flipud(occ > float(meta.get("occupied_thresh", 0.65)))
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
        return cls(grid=grid, resolution=float(meta["resolution"]), origin=(ox, oy))

    @classmethod
    def empty_room(cls, w_m: float = 4.0, h_m: float = 4.0,
                   resolution: float = 0.05) -> "OccupancyMap":
        """Walled empty rectangle centered on the origin."""
        cols, rows = int(w_m / resolution), int(h_m / resolution)
        grid = np.zeros((rows, cols), dtype=bool)
        grid[0, :] = grid[-1, :] = True
        grid[:, 0] = grid[:, -1] = True
        return cls(grid=grid, resolution=resolution,
                   origin=(-w_m / 2.0, -h_m / 2.0))

    def is_occupied(self, x: float, y: float) -> bool:
        c = int((x - self.origin[0]) / self.resolution)
        r = int((y - self.origin[1]) / self.resolution)
        if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
            return bool(self.grid[r, c])
        return True  # outside the map counts as solid

    def raycast(self, x: float, y: float, angle: float,
                max_range: float, step: float | None = None) -> float:
        """Distance to the first occupied cell along `angle`, capped at max_range."""
        step = step or self.resolution * 0.5
        dx, dy = math.cos(angle) * step, math.sin(angle) * step
        px, py, d = x, y, 0.0
        while d < max_range:
            px += dx
            py += dy
            d += step
            if self.is_occupied(px, py):
                return d
        return max_range


@dataclass
class Sim2D:
    world: OccupancyMap
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    seed: int = 0
    range_noise_std: float = 0.0    # m, per beam
    slip: float = 0.0               # fraction of commanded velocity lost (0..1)
    collided: bool = field(default=False, init=False)
    _vel: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    # ------------------------------------------------------------- motion

    def step(self, cmd: tuple[float, float, float], dt: float) -> None:
        """Advance one tick with body-frame command (vx, vy, wz)."""
        vx, vy, wz = self._vel
        cx, cy, cw = cmd
        dv, dw = MAX_ACCEL * dt, MAX_ALPHA * dt
        vx += np.clip(cx - vx, -dv, dv)
        vy += np.clip(cy - vy, -dv, dv)
        wz += np.clip(cw - wz, -dw, dw)
        self._vel = (vx, vy, wz)

        k = 1.0 - self.slip
        mid_yaw = self.yaw + 0.5 * wz * dt
        nx = self.x + (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * k * dt
        ny = self.y + (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * k * dt
        if self.world.is_occupied(nx, ny):
            self.collided = True
            self._vel = (0.0, 0.0, 0.0)   # wall stops you
            return
        self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt),
                              math.cos(self.yaw + wz * dt))

    @property
    def velocity(self) -> tuple[float, float, float]:
        return self._vel

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    # ------------------------------------------------------------- sensing

    def scan(self) -> tuple[np.ndarray, np.ndarray]:
        """(angles, ranges) in the LIDAR frame, real mount geometry:
        angles are the vendor band-pass window; the sensor itself sits
        yaw-pi on the chassis. Blind rear -> only ~219 deg of beams."""
        angles = np.linspace(LIDAR_FOV_MIN, LIDAR_FOV_MAX, LIDAR_BEAMS)
        ranges = np.empty(LIDAR_BEAMS)
        for i, a in enumerate(angles):
            world_angle = self.yaw + LIDAR_YAW_OFFSET + a
            r = self.world.raycast(self.x, self.y, world_angle, LIDAR_MAX_RANGE)
            if self.range_noise_std > 0:
                r = max(0.0, r + self.rng.normal(0.0, self.range_noise_std))
            ranges[i] = r
        return angles, ranges

    def scan_points_world(self) -> np.ndarray:
        """Scan endpoints in world frame, shape (N, 2) — SLAM/matcher food."""
        angles, ranges = self.scan()
        world_angles = self.yaw + LIDAR_YAW_OFFSET + angles
        hit = ranges < LIDAR_MAX_RANGE
        return np.column_stack((self.x + ranges[hit] * np.cos(world_angles[hit]),
                                self.y + ranges[hit] * np.sin(world_angles[hit])))
