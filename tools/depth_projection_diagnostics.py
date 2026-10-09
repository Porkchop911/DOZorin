"""Explain which RGB-D pixels create each forward depth-range obstacle.

This is a read-only field diagnostic.  It deliberately mirrors the production
projection rather than publishing any ROS messages or commanding the robot.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from hwr_sensors.depth_projection import (
    ground_relative_heights,
)
from hwr_sensors.depth_to_ranges_node import _depth_metres, _matrix


class Capture(Node):
    def __init__(self, depth_topic: str, info_topic: str) -> None:
        super().__init__("hwr_depth_projection_diagnostics")
        self.info: CameraInfo | None = None
        self.depth_message: Image | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)

    def _on_info(self, message: CameraInfo) -> None:
        self.info = message

    def _on_depth(self, message: Image) -> None:
        if self.info is not None:
            self.depth_message = message


def _diagnose(depth: np.ndarray, intrinsics: tuple[float, float, float, float],
              transform: np.ndarray, *, beam_count: int, half_fov: float,
              min_height: float, max_height: float, min_range: float,
              max_range: float, stride: int) -> dict:
    fx, fy, cx, cy = intrinsics
    vv, uu = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    sampled = depth[::stride, ::stride]
    valid = np.isfinite(sampled) & (sampled >= min_range) & (sampled <= max_range)
    if not valid.any():
        return {"image_size": [int(depth.shape[1]), int(depth.shape[0])],
                "valid_sample_count": 0, "kept_point_count": 0,
                "point_beams": [], "gap_beams": []}

    z = sampled[valid]
    u = uu[valid].astype(float)
    v = vv[valid].astype(float)
    points = np.vstack(((u - cx) * z / fx, (v - cy) * z / fy,
                        z, np.ones_like(z)))
    base = transform @ points
    x, y, base_z = base[0], base[1], base[2]
    height = ground_relative_heights(x, y, base_z, v, depth.shape[0])
    ranges = np.hypot(x, y)
    angles = np.arctan2(y, x)
    keep = ((x > 0.0) & (height >= min_height) & (height <= max_height)
            & (ranges >= min_range) & (ranges <= max_range)
            & (angles >= -half_fov) & (angles <= half_fov))

    scale = beam_count / (2.0 * half_fov)
    bins = np.full(len(z), -1, dtype=int)
    bins[keep] = np.clip(
        np.floor((angles[keep] + half_fov) * scale).astype(int),
        0, beam_count - 1)
    point_beams = []
    for beam in np.unique(bins[keep]):
        candidates = np.flatnonzero(keep & (bins == beam))
        index = candidates[int(np.argmin(ranges[candidates]))]
        point_beams.append({
            "beam": int(beam),
            "u": int(u[index]),
            "v": int(v[index]),
            "depth_m": round(float(z[index]), 4),
            "base_x_m": round(float(x[index]), 4),
            "base_y_m": round(float(y[index]), 4),
            "base_z_m": round(float(base_z[index]), 4),
            "ground_height_m": round(float(height[index]), 4),
            "range_m": round(float(ranges[index]), 4),
            "bearing_deg": round(math.degrees(float(angles[index])), 2),
        })

    gap_beams = _gap_details(
        depth, intrinsics, transform, beam_count, -half_fov, half_fov,
        min_range, max_range, stride)
    height_by_pixel = {
        (int(pixel_u), int(pixel_v)): float(point_height)
        for pixel_u, pixel_v, point_height in zip(u, v, height)
    }
    for gap in gap_beams:
        pixel = (gap["boundary_u"], gap["boundary_v"])
        if pixel in height_by_pixel:
            gap["boundary_ground_height_m"] = round(
                height_by_pixel[pixel], 4)
    point_beams.sort(key=lambda item: item["range_m"])
    gap_beams.sort(key=lambda item: item["range_m"])
    return {
        "image_size": [int(depth.shape[1]), int(depth.shape[0])],
        "stride": int(stride),
        "valid_sample_count": int(np.count_nonzero(valid)),
        "kept_point_count": int(np.count_nonzero(keep)),
        "point_beams": point_beams,
        "gap_beams": gap_beams,
    }


def _gap_details(depth_m: np.ndarray,
                 intrinsics: tuple[float, float, float, float],
                 transform: np.ndarray, beam_count: int, angle_min: float,
                 angle_max: float, min_range: float, max_range: float,
                 stride: int, min_run: int = 3,
                 side_margin_fraction: float = 0.05) -> list[dict]:
    """Mirror invalid_gap_ranges while retaining the source-hole geometry."""
    depth = np.asarray(depth_m, dtype=float)
    fx, fy, cx, cy = intrinsics
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
            if (end - begin >= min_run and end < rows and begin > start_row
                    and side_margin <= column < cols - side_margin):
                runs.append((column, begin, end))

    scale = beam_count / (angle_max - angle_min)
    details = []
    for column, begin, end in runs:
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
        index = np.flatnonzero(forward)[int(np.argmin(planar[forward]))]
        distance = float(planar[index])
        angle = math.atan2(float(base[1, index]), float(base[0, index]))
        if not (min_range <= distance <= max_range
                and angle_min <= angle <= angle_max):
            continue
        beam = min(beam_count - 1, max(
            0, int((angle - angle_min) * scale)))
        details.append({
            "beam": int(beam),
            "range_m": round(distance, 4),
            "bearing_deg": round(math.degrees(angle), 2),
            "hole_column_px": int(column * stride),
            "hole_top_px": int(begin * stride),
            "hole_bottom_px": int(min(depth.shape[0] - 1, end * stride)),
            "boundary_u": int(u[index]),
            "boundary_v": int(v[index]),
            "boundary_depth_m": round(float(z[index]), 4),
            "base_x_m": round(float(base[0, index]), 4),
            "base_y_m": round(float(base[1, index]), 4),
            "base_z_m": round(float(base[2, index]), 4),
        })
    # Production merges candidates by beam, retaining the closest boundary.
    nearest = {}
    for item in details:
        beam = item["beam"]
        if beam not in nearest or item["range_m"] < nearest[beam]["range_m"]:
            nearest[beam] = item
    return sorted(nearest.values(), key=lambda item: item["range_m"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--beam-count", type=int, default=64)
    parser.add_argument("--fov-degrees", type=float, default=74.0)
    parser.add_argument("--min-height", type=float, default=0.03)
    parser.add_argument("--max-height", type=float, default=0.35)
    parser.add_argument("--min-range", type=float, default=0.15)
    parser.add_argument("--max-range", type=float, default=3.0)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--limit", type=int, default=20)
    parser.add_argument("--frames", type=int, default=1)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be at least 1")

    rclpy.init()
    node = Capture(args.depth_topic, args.info_topic)
    deadline = time.monotonic() + args.timeout
    try:
        while (rclpy.ok() and node.depth_message is None
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.depth_message is None or node.info is None:
            raise RuntimeError("no synchronized-enough depth and camera-info received")
        message = node.depth_message
        source_frame = message.header.frame_id or node.info.header.frame_id
        if not source_frame:
            raise RuntimeError("depth frame id is empty")
        tf = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                tf = node.tf_buffer.lookup_transform(
                    args.base_frame, source_frame, Time(),
                    timeout=Duration(seconds=0.1))
                break
            except TransformException:
                rclpy.spin_once(node, timeout_sec=0.1)
        if tf is None:
            raise RuntimeError(
                f"no TF connection from {source_frame} to {args.base_frame}")

        reports = []
        occupied = []
        last_stamp = None
        while (rclpy.ok() and len(reports) < args.frames
               and time.monotonic() < deadline):
            message = node.depth_message
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == last_stamp:
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            last_stamp = stamp
            k = node.info.k
            report = _diagnose(
                _depth_metres(message), (k[0], k[4], k[2], k[5]),
                _matrix(tf.transform), beam_count=args.beam_count,
                half_fov=math.radians(args.fov_degrees) / 2.0,
                min_height=args.min_height, max_height=args.max_height,
                min_range=args.min_range, max_range=args.max_range,
                stride=args.stride)
            report["frame_index"] = len(reports)
            report["point_beams"] = report["point_beams"][:args.limit]
            report["gap_beams"] = report["gap_beams"][:args.limit]
            reports.append(report)
            if report["point_beams"] or report["gap_beams"]:
                occupied.append(report)
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(reports) < args.frames:
            raise RuntimeError(
                f"received only {len(reports)}/{args.frames} requested frames")
        if args.frames == 1:
            report = reports[0]
        else:
            report = {
                "frames_captured": len(reports),
                "occupied_frame_count": len(occupied),
                "occupied_frames": occupied,
            }
        report["depth_frame"] = source_frame
        report["base_frame"] = args.base_frame
        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        elif args.frames > 1:
            print(f"frame={source_frame}->{args.base_frame} "
                  f"captured={report['frames_captured']} "
                  f"occupied={report['occupied_frame_count']}")
            for occupied_report in report["occupied_frames"]:
                print(f"frame_index={occupied_report['frame_index']}")
                for item in occupied_report["point_beams"]:
                    print("  point " + " ".join(
                        f"{key}={value}" for key, value in item.items()))
                for item in occupied_report["gap_beams"]:
                    print(f"  gap beam={item['beam']} range_m={item['range_m']}")
        else:
            print(f"frame={source_frame}->{args.base_frame} "
                  f"image={report['image_size'][0]}x{report['image_size'][1]} "
                  f"stride={report['stride']} valid={report['valid_sample_count']} "
                  f"kept={report['kept_point_count']}")
            print("nearest projected point per occupied beam:")
            for item in report["point_beams"]:
                print("  " + " ".join(f"{key}={value}" for key, value in item.items()))
            print("invalid-gap beams:")
            for item in report["gap_beams"]:
                print(f"  beam={item['beam']} range_m={item['range_m']}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
