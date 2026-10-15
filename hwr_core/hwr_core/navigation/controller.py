"""Holonomic carrot follower with structurally prohibited reverse vx."""
from __future__ import annotations

import math


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def sensor_sample_fresh(now: float, stamp: float,
                        max_age: float = 0.75) -> bool:
    """Return true only for a present, recent monotonic sensor sample."""
    values = float(now), float(stamp), float(max_age)
    if not all(math.isfinite(value) for value in values) or max_age <= 0.0:
        return False
    age = now - stamp
    return stamp > 0.0 and 0.0 <= age <= max_age


def translation_deadline_seconds(distance: float, *,
                                 minimum_speed: float = 0.05,
                                 fixed_overhead: float = 4.0,
                                 maximum: float = 10.0,
                                 rotation: float = 0.0,
                                 yaw_rate: float = 0.30) -> float:
    """Bound a translation goal even when localization appears to make progress.

    `rotation` (rad) is the in-place turning the controller must do (face the
    path, then the goal yaw); it gets its own budget outside the translation cap.
    """
    values = (float(distance), float(minimum_speed), float(fixed_overhead),
              float(maximum))
    if (not all(math.isfinite(value) for value in values)
            or distance < 0.0 or minimum_speed <= 0.0
            or fixed_overhead < 0.0 or maximum <= 0.0):
        raise ValueError("invalid translation deadline parameters")
    if not math.isfinite(float(rotation)) or rotation < 0.0 or yaw_rate <= 0.0:
        raise ValueError("invalid rotation budget parameters")
    return (min(maximum, fixed_overhead + distance / minimum_speed)
            + 1.5 * rotation / yaw_rate)


def limit_translation_target(
        start_xy: tuple[float, float], target_xy: tuple[float, float],
        max_distance: float,
) -> tuple[float, float]:
    """Clamp a committed target to a strict radius from the current pose."""
    values = (*start_xy, *target_xy, max_distance)
    if (not all(math.isfinite(float(value)) for value in values)
            or max_distance <= 0.0):
        raise ValueError("translation target and maximum must be finite and positive")
    dx = float(target_xy[0]) - float(start_xy[0])
    dy = float(target_xy[1]) - float(start_xy[1])
    distance = math.hypot(dx, dy)
    if distance <= max_distance:
        return float(target_xy[0]), float(target_xy[1])
    scale = max_distance / distance
    return (float(start_xy[0]) + dx * scale,
            float(start_xy[1]) + dy * scale)


def translation_goal_reached_or_passed(
        start_xy: tuple[float, float], current_xy: tuple[float, float],
        goal_xy: tuple[float, float], *, goal_tolerance: float = 0.02,
        overshoot_tolerance: float = 0.05,
        cross_track_tolerance: float = 0.04,
) -> bool:
    """Accept a bounded forward pass without ever commanding reverse.

    Scan matching can place a short leg a few centimetres beyond its point
    goal. Once the robot reaches the along-track terminal band, remains close
    to the planned line, and stays inside the independent odometry leash,
    returning to the point would require prohibited reverse motion. The nav
    node separately requires physical odometry evidence before success.
    """
    values = (*start_xy, *current_xy, *goal_xy, goal_tolerance,
              overshoot_tolerance, cross_track_tolerance)
    if (not all(math.isfinite(float(value)) for value in values)
            or goal_tolerance <= 0.0 or overshoot_tolerance < 0.0
            or cross_track_tolerance < 0.0):
        return False
    gx = float(goal_xy[0]) - float(start_xy[0])
    gy = float(goal_xy[1]) - float(start_xy[1])
    length = math.hypot(gx, gy)
    current_error = math.hypot(
        float(goal_xy[0]) - float(current_xy[0]),
        float(goal_xy[1]) - float(current_xy[1]))
    if current_error <= goal_tolerance:
        return True
    if length <= goal_tolerance:
        return False
    ux, uy = gx / length, gy / length
    cx = float(current_xy[0]) - float(start_xy[0])
    cy = float(current_xy[1]) - float(start_xy[1])
    along = cx * ux + cy * uy
    cross = abs(cx * uy - cy * ux)
    return (length - goal_tolerance <= along <= length + overshoot_tolerance
            and cross <= cross_track_tolerance)


def odometry_translation_within_limit(
        initial_distance: float,
        odom_start: tuple[float, float, float],
        odom_current: tuple[float, float, float], *,
        excursion_allowance: float = 0.05) -> bool:
    """Leash physical travel even when map localization moves or oscillates."""
    values = (initial_distance, *odom_start, *odom_current,
              excursion_allowance)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if initial_distance < 0.0 or excursion_allowance < 0.0:
        return False
    dx = float(odom_current[0]) - float(odom_start[0])
    dy = float(odom_current[1]) - float(odom_start[1])
    return math.hypot(dx, dy) <= initial_distance + excursion_allowance


def odometry_motion_confirmed(
        initial_distance: float, initial_yaw_error: float,
        odom_start: tuple[float, float, float],
        odom_current: tuple[float, float, float], *,
        distance_tolerance: float = 0.02,
        yaw_tolerance: float = math.radians(5.0),
        distance_noise_allowance: float = 0.005,
        distance_scale_allowance: float = 0.10) -> bool:
    """Require physical odometry evidence before declaring a goal reached.

    Map localization can move with prediction or particle convergence even when
    the chassis is stalled.  Fused scan/IMU odometry is an independent minimum
    confirmation: it need not exactly equal map progress, but it must account
    for the requested displacement outside the controller's tolerance band.
    Map and fused odometry disagree by a few percent of distance on a clean
    leg (0.283 vs 0.292 m, 2026-10-15), so the allowance scales with distance.
    """
    values = (initial_distance, initial_yaw_error, *odom_start, *odom_current,
              distance_tolerance, yaw_tolerance, distance_noise_allowance)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if (initial_distance < 0.0 or distance_tolerance <= 0.0
            or yaw_tolerance <= 0.0 or distance_noise_allowance < 0.0):
        return False
    dx = float(odom_current[0]) - float(odom_start[0])
    dy = float(odom_current[1]) - float(odom_start[1])
    moved = math.hypot(dx, dy)
    if initial_distance > distance_tolerance:
        required = max(
            0.01,
            initial_distance * (1.0 - distance_scale_allowance)
            - distance_tolerance - distance_noise_allowance)
        if moved < required:
            return False
    yaw_needed = abs(_wrap(initial_yaw_error))
    if yaw_needed > yaw_tolerance:
        turned = abs(_wrap(float(odom_current[2]) - float(odom_start[2])))
        required = max(math.radians(2.0), yaw_needed - yaw_tolerance)
        if turned < required:
            return False
    return True


def gyro_yaw_motion_confirmed(
        initial_yaw_error: float, gyro_yaw_start: float,
        gyro_yaw_current: float, *,
        yaw_tolerance: float = math.radians(5.0),
        yaw_noise_allowance: float = math.radians(1.0)) -> bool:
    """Confirm physical rotation from integrated bias-corrected gyro rate.

    Scan matching can under-report yaw in repetitive scenes by pulling the
    fused pose toward an ambiguous keyframe. The gyro rate remains independent
    physical evidence that the chassis actually turned. Translation continues
    to require fused odometry and is deliberately outside this helper.
    """
    values = (initial_yaw_error, gyro_yaw_start, gyro_yaw_current,
              yaw_tolerance, yaw_noise_allowance)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if yaw_tolerance <= 0.0 or yaw_noise_allowance < 0.0:
        return False
    needed = abs(_wrap(float(initial_yaw_error)))
    if needed <= yaw_tolerance:
        return True
    turned = abs(_wrap(float(gyro_yaw_current) - float(gyro_yaw_start)))
    required = max(
        math.radians(2.0), needed - yaw_tolerance - yaw_noise_allowance)
    return turned >= required


class HolonomicController:
    def __init__(self, lookahead: float = 0.3, max_speed: float = 0.15,
                 max_wz: float = 0.5, goal_tolerance: float = 0.02,
                 translation_heading_limit: float = math.radians(30.0)):
        self.lookahead = lookahead
        self.max_speed = max_speed
        self.max_wz = max_wz
        self.goal_tolerance = goal_tolerance
        self.translation_heading_limit = abs(float(translation_heading_limit))

    def yaw_command(self, yaw: float, goal_yaw: float) -> tuple[float, bool]:
        """Rotate toward a final yaw without introducing translation."""
        error = _wrap(goal_yaw - yaw)
        wz = max(-self.max_wz, min(self.max_wz, 1.5 * error))
        return wz, abs(error) < math.radians(5.0)

    def command(self, pose: tuple[float, float, float], path,
                goal_yaw: float | None = None) -> tuple[float, float, float, bool]:
        if not path:
            return 0.0, 0.0, 0.0, False
        x, y, yaw = pose
        gx, gy = path[-1]
        goal_distance = math.hypot(gx - x, gy - y)
        if goal_distance <= self.goal_tolerance:
            if goal_yaw is None:
                return 0.0, 0.0, 0.0, True
            wz, done = self.yaw_command(yaw, goal_yaw)
            return 0.0, 0.0, wz, done
        carrot = path[-1]
        for point in path:
            if math.hypot(point[0] - x, point[1] - y) >= self.lookahead:
                carrot = point
                break
        dx, dy = carrot[0] - x, carrot[1] - y
        desired = math.atan2(dy, dx)
        # For a short final leg, hold the preflighted approach heading instead
        # of continually aiming at the shrinking goal-point vector. With a
        # 10 cm leg, centimetre-scale lateral localization noise can otherwise
        # swing that vector by tens of degrees and make the robot rotate beside
        # the goal until its deadline. Longer routes still face the path carrot
        # until they enter the normal lookahead radius.
        steering_yaw = (
            goal_yaw
            if goal_yaw is not None and goal_distance <= self.lookahead
            else desired
        )
        heading_error = _wrap(steering_yaw - yaw)
        wz = max(-self.max_wz, min(self.max_wz, 1.5 * heading_error))
        # Keep autonomous translation inside the forward depth-perception cone.
        # Larger holonomic offsets rotate first so low side obstacles enter view.
        if abs(heading_error) > self.translation_heading_limit:
            return 0.0, 0.0, wz, False
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * dx + s * dy, -s * dx + c * dy
        norm = max(math.hypot(bx, by), 1e-9)
        speed = min(self.max_speed, goal_distance)
        vx = max(0.0, speed * bx / norm)
        vy = speed * by / norm
        return vx, vy, wz, False


class ReplanBudget:
    """Bound consecutive live-obstacle replans until meaningful progress."""

    def __init__(self, limit: int = 3):
        if int(limit) < 0:
            raise ValueError("replan limit must be non-negative")
        self.limit = int(limit)
        self.attempts = 0

    def request(self) -> bool:
        """Consume one replan attempt, returning false when exhausted."""
        if self.attempts >= self.limit:
            return False
        self.attempts += 1
        return True

    def reset(self) -> None:
        self.attempts = 0


class GoalProgress:
    """Track meaningful translational or near-goal angular progress."""

    def __init__(self, distance: float, yaw_error: float, now: float,
                 distance_step: float = 0.03,
                 yaw_step: float = math.radians(3.0)):
        self.best_distance = float(distance)
        self.best_yaw_error = abs(float(yaw_error))
        self.last_progress = float(now)
        self.distance_step = float(distance_step)
        self.yaw_step = float(yaw_step)

    def update(self, distance: float, yaw_error: float, near_goal: bool,
               now: float) -> bool:
        progressed = False
        distance = float(distance)
        yaw_error = abs(float(yaw_error))
        if distance < self.best_distance - self.distance_step:
            progressed = True
        self.best_distance = min(self.best_distance, distance)
        if near_goal:
            if yaw_error < self.best_yaw_error - self.yaw_step:
                progressed = True
            self.best_yaw_error = min(self.best_yaw_error, yaw_error)
        if progressed:
            self.last_progress = float(now)
        return progressed

    def stalled(self, now: float, timeout: float) -> bool:
        return float(now) - self.last_progress >= float(timeout)


class ForwardRecovery:
    """Bounded recovery commands; reverse translation is unrepresentable."""

    def __init__(self, rotate_seconds: float = 4.0, arc_seconds: float = 2.0,
                 rotate_speed: float = 0.35, arc_speed: float = 0.06,
                 arc_yaw_rate: float = 0.25):
        self.rotate_seconds = float(rotate_seconds)
        self.arc_seconds = float(arc_seconds)
        self.rotate_speed = abs(float(rotate_speed))
        self.arc_speed = max(0.0, float(arc_speed))
        self.arc_yaw_rate = float(arc_yaw_rate)

    def command(self, elapsed: float) -> tuple[float, float, float, str]:
        if elapsed < self.rotate_seconds:
            return 0.0, 0.0, self.rotate_speed, "rotate-scan"
        if elapsed < self.rotate_seconds + self.arc_seconds:
            return self.arc_speed, 0.0, self.arc_yaw_rate, "forward-arc"
        return 0.0, 0.0, 0.0, "replan"
