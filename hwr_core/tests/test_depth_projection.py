import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "ros2_ws" / "src" / "hwr_sensors"))
from hwr_sensors.depth_projection import (depth_to_ranges,
                                          floor_support_preflight,
                                          ground_support_profile,
                                          ground_relative_heights,
                                          invalid_gap_ranges)  # noqa: E402


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s, one_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [c + x*x*one_c, x*y*one_c - z*s, x*z*one_c + y*s],
        [y*x*one_c + z*s, c + y*y*one_c, y*z*one_c - x*s],
        [z*x*one_c - y*s, z*y*one_c + x*s, c + z*z*one_c],
    ])


def _urdf_transform(parent_link, child_link, positions=None):
    positions = positions or {}
    urdf = pathlib.Path(__file__).resolve().parents[2] / "ros2_ws" / "src" \
        / "hwr_description" / "urdf" / "hwr.urdf"
    by_child = {}
    for joint in ET.parse(urdf).getroot().findall("joint"):
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        matrix = np.eye(4)
        matrix[:3, :3] = _rpy_matrix(rpy)
        matrix[:3, 3] = xyz
        if joint.get("type") in ("revolute", "continuous"):
            axis = [float(v) for v in joint.find("axis").get("xyz").split()]
            motion = np.eye(4)
            motion[:3, :3] = _axis_rotation(axis, positions.get(joint.get("name"), 0.0))
            matrix = matrix @ motion
        by_child[joint.find("child").get("link")] = (
            joint.find("parent").get("link"), matrix)
    chain = []
    current = child_link
    while current != parent_link:
        current, matrix = by_child[current]
        chain.append(matrix)
    transform = np.eye(4)
    for matrix in reversed(chain):
        transform = transform @ matrix
    return transform


def optical_to_base(height=0.2):
    # optical x right/y down/z forward -> base x forward/y left/z up
    out = np.array([[0, 0, 1, 0], [-1, 0, 0, 0],
                    [0, -1, 0, height], [0, 0, 0, 1]], dtype=float)
    return out


def test_center_obstacle_projects_forward():
    depth = np.full((5, 5), np.nan)
    depth[2, 2] = 0.5
    ranges = depth_to_ranges(depth, (100, 100, 2, 2), optical_to_base(),
                             beam_count=5, angle_min=-0.5, angle_max=0.5,
                             stride=1)
    assert np.isfinite(ranges).sum() == 1
    assert ranges[2] == pytest.approx(0.5)


def test_live_depth_frame_uses_ros_optical_axis_convention():
    radians_per_pulse = math.radians(240.0 / 1000.0)
    forward_pulses = (500, 701, 120, 300, 500)
    positions = {f"joint{i}": (pulse - 500) * radians_per_pulse
                 for i, pulse in enumerate(forward_pulses, 1)}
    base_from_depth = _urdf_transform(
        "base_footprint", "depth_camera_link", positions)
    rotation = base_from_depth[:3, :3]
    # Optical +Z is robot-forward, +X is robot-right (-base Y), and +Y is
    # approximately down (-base Z). Small CAD/arm mounting tilts are allowed.
    assert rotation[:, 2] @ np.array([1.0, 0.0, 0.0]) > 0.98
    assert rotation[:, 0] @ np.array([0.0, -1.0, 0.0]) > 0.98
    assert rotation[:, 1] @ np.array([0.0, 0.0, -1.0]) > 0.98


def test_nearest_point_wins_per_beam():
    depth = np.full((5, 5), np.nan)
    depth[2, 2] = 0.8
    depth[2, 3] = 0.4
    ranges = depth_to_ranges(depth, (1000, 1000, 2, 2), optical_to_base(),
                             beam_count=3, angle_min=-0.2, angle_max=0.2,
                             stride=1)
    assert np.nanmin(ranges) == pytest.approx(math.hypot(0.4, 0.0004))


def test_floor_and_out_of_range_points_are_removed():
    depth = np.full((3, 3), np.nan)
    depth[1, 1] = 0.5
    assert np.isinf(depth_to_ranges(
        depth, (100, 100, 1, 1), optical_to_base(height=0.0),
        beam_count=5, stride=1)).all()
    depth[1, 1] = 4.0
    assert np.isinf(depth_to_ranges(
        depth, (100, 100, 1, 1), optical_to_base(),
        beam_count=5, stride=1)).all()


def test_tilted_floor_is_removed_but_low_obstacle_remains():
    height, width = 80, 80
    fx = fy = 80.0
    cx, cy = 40.0, 20.0
    rows = np.arange(height)[:, None]
    depth = np.full((height, width), np.nan)
    lower = rows[:, 0] > cy
    depth[lower, :] = 0.30 * fy / (rows[lower, :] - cy)
    # A 20 cm tall vertical leg at 0.6 m, five pixels wide.
    depth[28:53, 38:43] = 0.60
    angle = 0.12
    tilt = np.eye(4)
    tilt[:3, :3] = np.array([
        [math.cos(angle), 0.0, math.sin(angle)],
        [0.0, 1.0, 0.0],
        [-math.sin(angle), 0.0, math.cos(angle)],
    ])
    ranges = depth_to_ranges(
        depth, (fx, fy, cx, cy), tilt @ optical_to_base(height=0.30),
        beam_count=64, min_height=0.03, max_height=0.35, stride=1)
    finite = ranges[np.isfinite(ranges)]
    assert 1 <= len(finite) <= 6
    assert finite.min() == pytest.approx(0.60, abs=0.08)


def test_ground_estimator_falls_back_without_a_supported_plane():
    x = np.array([0.5, 0.6, 0.7])
    z = np.array([0.05, 0.10, 0.15])
    assert np.array_equal(
        ground_relative_heights(x, np.zeros(3), z, np.full(3, 90), 100), z)


def test_supported_invalid_floor_gap_becomes_an_obstacle():
    height, width = 80, 80
    fx = fy = 80.0
    cx, cy = 40.0, 20.0
    rows = np.arange(height)[:, None]
    depth = np.full((height, width), np.nan)
    lower = rows[:, 0] > cy
    depth[lower, :] = 0.30 * fy / (rows[lower, :] - cy)
    depth[30:42, 37:44] = np.nan
    gaps = invalid_gap_ranges(
        depth, (fx, fy, cx, cy), optical_to_base(height=0.30),
        beam_count=64, angle_min=-0.5, angle_max=0.5,
        min_range=0.15, max_range=3.0, stride=1)
    finite = gaps[np.isfinite(gaps)]
    assert len(finite) >= 2
    assert finite.min() < 1.3


def test_isolated_or_bottom_border_holes_are_ignored():
    depth = np.ones((40, 40), dtype=float)
    depth[22:28, 20] = np.nan
    depth[35:, 25:30] = np.nan
    gaps = invalid_gap_ranges(
        depth, (80, 80, 20, 10), optical_to_base(),
        beam_count=32, angle_min=-0.5, angle_max=0.5,
        min_range=0.15, max_range=3.0, stride=1)
    assert np.isinf(gaps).all()


def test_invalid_gap_inside_unreliable_side_border_is_ignored():
    height, width = 80, 80
    fx = fy = 80.0
    cx, cy = 40.0, 20.0
    rows = np.arange(height)[:, None]
    depth = np.full((height, width), np.nan)
    lower = rows[:, 0] > cy
    depth[lower, :] = 0.30 * fy / (rows[lower, :] - cy)
    # A multi-column hole would otherwise pass the support test, but the depth
    # camera's outer 5% is not reliable enough for inferring missing objects.
    depth[30:42, 1:4] = np.nan
    gaps = invalid_gap_ranges(
        depth, (fx, fy, cx, cy), optical_to_base(height=0.30),
        beam_count=64, angle_min=-0.7, angle_max=0.7,
        min_range=0.15, max_range=3.0, stride=1)
    assert np.isinf(gaps).all()


def test_measured_point_inside_side_border_is_still_an_obstacle():
    depth = np.full((80, 80), np.nan)
    depth[40, 2] = 0.5
    ranges = depth_to_ranges(
        depth, (80, 80, 40, 20), optical_to_base(height=0.2),
        beam_count=64, angle_min=-0.8, angle_max=0.8,
        min_height=0.03, max_height=0.35, stride=1)
    assert np.isfinite(ranges).sum() == 1


def test_projection_configuration_is_bounded():
    with pytest.raises(ValueError):
        depth_to_ranges(np.ones((2, 2)), (1, 1, 0, 0), np.eye(4), beam_count=65)


def _synthetic_floor(camera_height=0.30, lower_height=None, edge=0.65):
    height, width = 120, 160
    fx = fy = 120.0
    cx, cy = 80.0, 30.0
    rows = np.arange(height)[:, None]
    difference = rows - cy
    depth = np.full((height, width), np.nan)
    lower_rows = difference[:, 0] > 0
    upper = camera_height * fy / difference[lower_rows, 0, None]
    if lower_height is None:
        depth[lower_rows, :] = upper
    else:
        lower = (camera_height - lower_height) * fy / difference[lower_rows, 0, None]
        depth[lower_rows, :] = np.where(upper <= edge, upper, lower)
    return depth, (fx, fy, cx, cy), optical_to_base(camera_height)


def test_flat_floor_supports_short_forward_corridor():
    depth, intrinsics, transform = _synthetic_floor()
    support, dropoff, valid, inliers = ground_support_profile(
        depth, intrinsics, transform, beam_count=32,
        angle_min=-0.5, angle_max=0.5, min_range=0.20,
        max_range=1.20, stride=1)
    assert valid and inliers >= 64
    assert np.isinf(dropoff).all()
    assert floor_support_preflight(
        support, -0.5, 1.0 / 32, 0.50, margin_m=0.10)[0]


def test_down_step_truncates_continuous_floor_support():
    depth, intrinsics, transform = _synthetic_floor(lower_height=-0.20)
    support, _, valid, _ = ground_support_profile(
        depth, intrinsics, transform, beam_count=32,
        angle_min=-0.5, angle_max=0.5, min_range=0.20,
        max_range=1.20, stride=1)
    assert valid
    allowed, reasons = floor_support_preflight(
        support, -0.5, 1.0 / 32, 0.75, margin_m=0.10)
    assert not allowed
    assert "observed floor support" in reasons[0]


def test_floor_support_fails_closed_without_plane():
    support, dropoff, valid, inliers = ground_support_profile(
        np.full((20, 20), np.nan), (80, 80, 10, 5), optical_to_base(),
        beam_count=16, stride=1)
    assert not valid and inliers == 0
    assert np.array_equal(support, np.zeros(16))
    assert np.isinf(dropoff).all()
    assert not floor_support_preflight(
        support, -0.5, 1.0 / 16, 0.10)[0]


def test_floor_support_requires_corridor_wide_coverage():
    support = np.ones(21)
    support[7:14] = 0.0
    allowed, reasons = floor_support_preflight(
        support, -0.5, 0.05, 0.40,
        margin_m=0.10, half_width_m=0.20, required_fraction=0.80)
    assert not allowed
    assert "swept corridor" in reasons[0]


def test_full_fan_keeps_returns_behind_the_robot():
    # Camera turned to look straight back (yaw pi): an obstacle 0.5 m out lands behind the base.
    depth = np.full((5, 5), np.nan)
    depth[2, 2] = 0.5
    back = np.eye(4)
    back[:3, :3] = _rpy_matrix((0.0, 0.0, math.pi))
    transform = back @ optical_to_base()
    assert np.isinf(depth_to_ranges(depth, (100, 100, 2, 2), transform, beam_count=8,
                                    angle_min=-math.pi, angle_max=math.pi, stride=1)).all()
    full = depth_to_ranges(depth, (100, 100, 2, 2), transform, beam_count=8,
                           angle_min=-math.pi, angle_max=math.pi, stride=1, forward_only=False)
    assert np.isfinite(full).sum() == 1 and np.nanmin(full) == pytest.approx(0.5)
    assert np.argmin(full) in (0, 7)  # the +-pi beams
