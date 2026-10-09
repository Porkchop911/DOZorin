"""Pure depth-image projection into a compact base-frame range fan."""
from __future__ import annotations

import math

import numpy as np


def _estimate_ground_plane(x: np.ndarray, y: np.ndarray, z: np.ndarray,
                           image_rows: np.ndarray, image_height: int,
                           inlier_tolerance: float = 0.025
                           ) -> tuple[np.ndarray | None, int]:
    x, y, z, rows = (np.asarray(values, dtype=float)
                     for values in (x, y, z, image_rows))
    candidate = ((rows >= 0.55 * image_height) & (x >= 0.15)
                 & (np.hypot(x, y) <= 3.0) & (z >= -0.40) & (z <= 0.45))
    indices = np.flatnonzero(candidate)
    if len(indices) < 64:
        return None, 0
    if len(indices) > 2000:
        indices = indices[np.linspace(0, len(indices) - 1, 2000).astype(int)]
    design = np.column_stack((x[indices], y[indices], np.ones(len(indices))))
    values = z[indices]
    rng = np.random.default_rng(0)
    best = None
    best_count = 0
    for _ in range(64):
        sample = rng.choice(len(indices), 3, replace=False)
        try:
            plane = np.linalg.solve(design[sample], values[sample])
        except np.linalg.LinAlgError:
            continue
        if np.hypot(plane[0], plane[1]) > 0.60:
            continue
        inliers = np.abs(values - design @ plane) <= inlier_tolerance
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < max(64, int(0.08 * len(indices))):
        return None, best_count
    plane, *_ = np.linalg.lstsq(design[best], values[best], rcond=None)
    for _ in range(2):
        residual = np.abs(values - design @ plane)
        refined = residual <= inlier_tolerance
        if np.count_nonzero(refined) < 64:
            break
        plane, *_ = np.linalg.lstsq(design[refined], values[refined], rcond=None)
        best_count = int(np.count_nonzero(refined))
    return plane, best_count


def ground_relative_heights(x: np.ndarray, y: np.ndarray, z: np.ndarray,
                            image_rows: np.ndarray, image_height: int,
                            inlier_tolerance: float = 0.025) -> np.ndarray:
    """Return point heights above a robustly estimated floor plane.

    Arm/camera mounting tolerances make a fixed base-Z floor cut brittle.  Fit
    ``z = ax + by + c`` to the lower image with deterministic RANSAC, then use
    residual height.  If no sufficiently supported near-horizontal plane is
    present, retain the original base-frame heights as a conservative fallback.
    """
    x, y, z = (np.asarray(values, dtype=float) for values in (x, y, z))
    plane, _ = _estimate_ground_plane(
        x, y, z, image_rows, image_height, inlier_tolerance)
    if plane is None:
        return z.copy()
    return z - (plane[0] * x + plane[1] * y + plane[2])


def ground_support_profile(
        depth_m: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        base_from_camera: np.ndarray, beam_count: int = 32,
        angle_min: float = -math.radians(35.0),
        angle_max: float = math.radians(35.0),
        min_range: float = 0.20, max_range: float = 1.50,
        radial_bin_m: float = 0.05, floor_tolerance_m: float = 0.04,
        drop_threshold_m: float = 0.08, max_initial_gap_m: float = 0.75,
        stride: int = 2
        ) -> tuple[np.ndarray, np.ndarray, bool, int]:
    """Return continuous observed-floor range and downward edge per bearing.

    The first array is how far floor support is continuously observed. The
    second is the nearest point significantly below the fitted floor plane, or
    infinity. If no supported floor plane can be estimated, all support ranges
    are zero and ``plane_valid`` is false.
    """
    depth = np.asarray(depth_m, dtype=float)
    transform = np.asarray(base_from_camera, dtype=float)
    if depth.ndim != 2 or transform.shape != (4, 4):
        raise ValueError("depth must be HxW and transform must be 4x4")
    if (not 3 <= beam_count <= 64 or stride < 1 or angle_max <= angle_min
            or min_range < 0.0 or max_range <= min_range
            or radial_bin_m <= 0.0 or max_initial_gap_m < min_range):
        raise ValueError("invalid ground-support configuration")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    vv, uu = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    optical_z = depth[::stride, ::stride]
    valid = np.isfinite(optical_z) & (optical_z >= 0.15) & (optical_z <= 3.0)
    unsupported = np.zeros(beam_count, dtype=float)
    no_drop = np.full(beam_count, np.inf)
    if np.count_nonzero(valid) < 64:
        return unsupported, no_drop, False, 0
    z = optical_z[valid]
    u, v = uu[valid], vv[valid]
    optical = np.vstack(((u - cx) * z / fx, (v - cy) * z / fy,
                         z, np.ones_like(z)))
    base = transform @ optical
    x, y, height = base[0], base[1], base[2]
    plane, inliers = _estimate_ground_plane(x, y, height, v, depth.shape[0])
    if plane is None:
        return unsupported, no_drop, False, inliers
    residual = height - (plane[0] * x + plane[1] * y + plane[2])
    planar = np.hypot(x, y)
    angles = np.arctan2(y, x)
    angular = ((x > 0.0) & (angles >= angle_min) & (angles <= angle_max))
    scale = beam_count / (angle_max - angle_min)
    beams = np.clip(np.floor((angles - angle_min) * scale).astype(int),
                    0, beam_count - 1)

    bins = int(math.ceil((max_range - min_range) / radial_bin_m))
    support = np.zeros((beam_count, bins), dtype=bool)
    floor = (angular & (np.abs(residual) <= floor_tolerance_m)
             & (planar >= min_range) & (planar <= max_range))
    if np.any(floor):
        radial = np.clip(
            np.floor((planar[floor] - min_range) / radial_bin_m).astype(int),
            0, bins - 1)
        support[beams[floor], radial] = True
    # Sampling and perspective leave single-cell holes even on a continuous
    # floor. Dilate by one radial and one angular bin, but never bridge a wider
    # unsupported region.
    dilated = support.copy()
    for beam_offset in (-1, 0, 1):
        for radial_offset in (-1, 0, 1):
            if beam_offset == 0 and radial_offset == 0:
                continue
            src_b0, src_b1 = max(0, -beam_offset), min(beam_count, beam_count - beam_offset)
            dst_b0, dst_b1 = src_b0 + beam_offset, src_b1 + beam_offset
            src_r0, src_r1 = max(0, -radial_offset), min(bins, bins - radial_offset)
            dst_r0, dst_r1 = src_r0 + radial_offset, src_r1 + radial_offset
            dilated[dst_b0:dst_b1, dst_r0:dst_r1] |= support[
                src_b0:src_b1, src_r0:src_r1]

    supported_until = np.zeros(beam_count, dtype=float)
    max_initial_bin = int(math.ceil(
        (max_initial_gap_m - min_range) / radial_bin_m))
    for beam in range(beam_count):
        present = np.flatnonzero(dilated[beam])
        # The chassis/camera geometry occludes the floor immediately in front
        # of the robot. Current footprint support may bridge only to the first
        # observed floor when it begins within a bounded distance. Any later
        # unsupported run still terminates the profile.
        if not len(present) or present[0] > max_initial_bin:
            continue
        last = int(present[0])
        for index in present[1:]:
            if int(index) - last > 2:
                break
            last = int(index)
        supported_until[beam] = min(
            max_range, min_range + (last + 1) * radial_bin_m)

    downward = angular & (residual <= -drop_threshold_m) & (planar <= max_range)
    dropoff_at = no_drop.copy()
    if np.any(downward):
        np.minimum.at(dropoff_at, beams[downward], planar[downward])
    supported_until = np.minimum(supported_until, dropoff_at)
    return supported_until, dropoff_at, True, inliers


def floor_support_preflight(
        supported_until_m: np.ndarray, angle_min: float, angle_increment: float,
        distance_m: float, *, margin_m: float = 0.10,
        half_width_m: float = 0.25, required_fraction: float = 0.80
        ) -> tuple[bool, list[str]]:
    """Require observed floor support across a proposed forward corridor."""
    ranges = np.asarray(supported_until_m, dtype=float)
    numeric = (angle_min, angle_increment, distance_m, margin_m,
               half_width_m, required_fraction)
    if (ranges.ndim != 1 or len(ranges) < 3
            or not all(math.isfinite(value) for value in numeric)
            or angle_increment <= 0.0 or distance_m < 0.0 or margin_m < 0.0
            or half_width_m <= 0.0 or not 0.0 < required_fraction <= 1.0):
        raise ValueError("invalid floor-support preflight input")
    required = distance_m + margin_m
    if required <= 0.0:
        return True, []
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    corridor = np.abs(np.tan(angles) * required) <= half_width_m
    if np.count_nonzero(corridor) < 3:
        return False, ["floor-support field does not cover swept corridor"]
    observed = np.isfinite(ranges[corridor]) & (ranges[corridor] >= required)
    fraction = float(np.count_nonzero(observed) / np.count_nonzero(corridor))
    if fraction < required_fraction:
        return False, [
            f"observed floor support covers {fraction:.0%} of swept corridor; "
            f"requires {required_fraction:.0%} through {required:.2f} m"]
    return True, []


def invalid_gap_ranges(depth_m: np.ndarray,
                       intrinsics: tuple[float, float, float, float],
                       base_from_camera: np.ndarray, beam_count: int,
                       angle_min: float, angle_max: float,
                       min_range: float, max_range: float, stride: int,
                       min_run: int = 3,
                       side_margin_fraction: float = 0.05) -> np.ndarray:
    """Convert supported lower-image depth holes into conservative obstacles.

    Dark floor-level structures can absorb structured IR and appear as holes.
    A hole is accepted only when it is a multi-row run, does not touch the
    bottom or unreliable side borders, has valid depth around its boundary, and
    overlaps a run in a neighboring sampled column.  The side crop applies only
    to inferred holes: real measured obstacle points retain the full field of
    view.  A hole's range is the nearest valid 2-D boundary point, which errs
    toward stopping early.
    """
    depth = np.asarray(depth_m, dtype=float)
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    sampled = depth[::stride, ::stride]
    rows, cols = sampled.shape
    valid = np.isfinite(sampled) & (sampled >= min_range) & (sampled <= max_range)
    start_row = max(0, int(math.floor(cy / stride)))
    side_margin = max(2, int(math.ceil(side_margin_fraction * cols)))
    runs = []
    for column in range(cols):
        row = start_row
        while row < rows:
            if valid[row, column]:
                row += 1
                continue
            begin = row
            while row < rows and not valid[row, column]:
                row += 1
            end = row
            # Runs already open at the optical horizon are ordinary far-range
            # absence, not an occluding object. Require valid support above.
            if (end - begin >= min_run and end < rows and begin > start_row
                    and side_margin <= column < cols - side_margin):
                runs.append((column, begin, end))
    output = np.full(beam_count, np.inf)
    if not runs:
        return output
    transform = np.asarray(base_from_camera, dtype=float)
    scale = beam_count / (angle_max - angle_min)
    for index, (column, begin, end) in enumerate(runs):
        supported = any(
            abs(other_column - column) <= 2
            and other_column != column
            and min(end, other_end) - max(begin, other_begin) >= 2
            for other_column, other_begin, other_end in runs)
        if not supported:
            continue
        r0, r1 = max(start_row, begin - 2), min(rows, end + 2)
        c0, c1 = max(0, column - 2), min(cols, column + 3)
        rr, cc = np.mgrid[r0:r1, c0:c1]
        boundary_valid = valid[r0:r1, c0:c1]
        if np.count_nonzero(boundary_valid) < 4:
            continue
        z = sampled[r0:r1, c0:c1][boundary_valid]
        u = (cc[boundary_valid] * stride).astype(float)
        v = (rr[boundary_valid] * stride).astype(float)
        points = np.vstack(((u - cx) * z / fx, (v - cy) * z / fy,
                            z, np.ones_like(z)))
        base = transform @ points
        planar = np.hypot(base[0], base[1])
        forward = base[0] > 0.0
        if not np.any(forward):
            continue
        nearest = np.flatnonzero(forward)[int(np.argmin(planar[forward]))]
        distance = float(planar[nearest])
        angle = math.atan2(float(base[1, nearest]), float(base[0, nearest]))
        if not (min_range <= distance <= max_range
                and angle_min <= angle <= angle_max):
            continue
        beam = min(beam_count - 1, max(0, int((angle - angle_min) * scale)))
        output[beam] = min(output[beam], distance)
    return output


def depth_to_ranges(depth_m: np.ndarray, intrinsics: tuple[float, float, float, float],
                    base_from_camera: np.ndarray, beam_count: int = 64,
                    angle_min: float = -math.radians(37.0),
                    angle_max: float = math.radians(37.0),
                    min_height: float = 0.03, max_height: float = 0.35,
                    min_range: float = 0.15, max_range: float = 3.0,
                    stride: int = 4) -> np.ndarray:
    """Project optical-frame depth into minimum planar range per bearing."""
    depth = np.asarray(depth_m, dtype=float)
    transform = np.asarray(base_from_camera, dtype=float)
    if depth.ndim != 2 or transform.shape != (4, 4):
        raise ValueError("depth must be HxW and transform must be 4x4")
    if not 1 <= beam_count <= 64 or stride < 1 or angle_max <= angle_min:
        raise ValueError("invalid beam projection configuration")
    fx, fy, cx, cy = (float(v) for v in intrinsics)
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    vv, uu = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    if not valid.any():
        return np.full(beam_count, np.inf)
    z, u, v = z[valid], uu[valid], vv[valid]
    points = np.vstack(((u - cx) * z / fx, (v - cy) * z / fy,
                        z, np.ones_like(z)))
    base = transform @ points
    x, y = base[0], base[1]
    height = ground_relative_heights(x, y, base[2], v, depth.shape[0])
    ranges = np.hypot(x, y)
    angles = np.arctan2(y, x)
    keep = ((x > 0.0) & (height >= min_height) & (height <= max_height)
            & (ranges >= min_range) & (ranges <= max_range)
            & (angles >= angle_min) & (angles <= angle_max))
    output = np.full(beam_count, np.inf)
    if keep.any():
        scale = beam_count / (angle_max - angle_min)
        bins = np.floor((angles[keep] - angle_min) * scale).astype(int)
        bins = np.clip(bins, 0, beam_count - 1)
        np.minimum.at(output, bins, ranges[keep])
    gaps = invalid_gap_ranges(
        depth, intrinsics, transform, beam_count, angle_min, angle_max,
        min_range, max_range, stride)
    np.minimum(output, gaps, out=output)
    return output
