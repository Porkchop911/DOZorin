"""Convert sparse RGB-D points into a base-frame forward LaserScan."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from hwr_sensors.depth_projection import depth_to_ranges, ground_support_profile


def _matrix(transform) -> np.ndarray:
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = (t.x, t.y, t.z)
    return out


def _depth_metres(msg: Image) -> np.ndarray:
    enc = msg.encoding.upper()
    if enc in ("16UC1", "MONO16"):
        dtype, scale = np.dtype(">u2" if msg.is_bigendian else "<u2"), 0.001
    elif enc == "32FC1":
        dtype, scale = np.dtype(">f4" if msg.is_bigendian else "<f4"), 1.0
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding}")
    row_values = msg.step // dtype.itemsize
    data = np.frombuffer(msg.data, dtype=dtype,
                         count=row_values * msg.height).reshape(msg.height, row_values)
    return data[:, :msg.width].astype(float) * scale


class DepthToRangesNode(Node):
    def __init__(self):
        super().__init__("hwr_depth_to_ranges")
        for name, default in (
            ("beam_count", 64), ("rate", 3.0), ("min_height", 0.03),
            ("max_height", 0.35), ("min_range", 0.15), ("max_range", 3.0),
            ("fov_degrees", 74.0), ("stride", 4),
            ("floor_support_beams", 32), ("floor_support_max_range", 1.5),
            ("floor_support_stride", 2)):
            self.declare_parameter(name, default)
        self.declare_parameter("base_frame", "base_footprint")
        self.info: CameraInfo | None = None
        self._last_publish = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LaserScan, "depth_ranges", qos)
        self.support_pub = self.create_publisher(LaserScan, "floor_support", qos)
        self.create_subscription(CameraInfo, "camera/depth/camera_info",
                                 self._on_info, qos)
        self.create_subscription(Image, "camera/depth/image_raw", self._on_depth, qos)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_depth(self, msg: Image) -> None:
        now = time.monotonic()
        rate = float(self.get_parameter("rate").value)
        if rate <= 0.0 or now - self._last_publish < 1.0 / rate or self.info is None:
            return
        source_frame = msg.header.frame_id or self.info.header.frame_id
        if not source_frame:
            self.get_logger().warning("depth frame id is empty", throttle_duration_sec=5.0)
            return
        base_frame = self.get_parameter("base_frame").value
        try:
            try:
                tf = self.tf_buffer.lookup_transform(
                    base_frame, source_frame, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05))
            except TransformException:
                # Aurora timestamps can lead joint-state TF by a fraction of a
                # second. At 3 Hz, the latest arm transform is the safe usable
                # fallback; navigation poses the arm forward before driving.
                tf = self.tf_buffer.lookup_transform(
                    base_frame, source_frame, Time(),
                    timeout=Duration(seconds=0.05))
            depth = _depth_metres(msg)
            k = self.info.k
            count = int(self.get_parameter("beam_count").value)
            half_fov = math.radians(float(self.get_parameter("fov_degrees").value)) / 2.0
            base_from_camera = _matrix(tf.transform)
            intrinsics = (k[0], k[4], k[2], k[5])
            ranges = depth_to_ranges(
                depth, intrinsics, base_from_camera,
                beam_count=count, angle_min=-half_fov, angle_max=half_fov,
                min_height=float(self.get_parameter("min_height").value),
                max_height=float(self.get_parameter("max_height").value),
                min_range=float(self.get_parameter("min_range").value),
                max_range=float(self.get_parameter("max_range").value),
                stride=int(self.get_parameter("stride").value))
            support_count = int(self.get_parameter("floor_support_beams").value)
            support_max = float(
                self.get_parameter("floor_support_max_range").value)
            support, _, plane_valid, plane_inliers = ground_support_profile(
                depth, intrinsics, base_from_camera,
                beam_count=support_count, angle_min=-half_fov,
                angle_max=half_fov, min_range=0.20,
                max_range=support_max,
                stride=int(self.get_parameter("floor_support_stride").value))
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f"depth projection skipped: {exc}",
                                      throttle_duration_sec=5.0)
            return
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = base_frame
        out.angle_min, out.angle_max = -half_fov, half_fov
        out.angle_increment = (2.0 * half_fov) / max(count - 1, 1)
        out.scan_time = 1.0 / rate
        out.range_min = float(self.get_parameter("min_range").value)
        out.range_max = float(self.get_parameter("max_range").value)
        out.ranges = ranges.astype(float).tolist()
        self.pub.publish(out)

        support_out = LaserScan()
        support_out.header.stamp = msg.header.stamp
        support_out.header.frame_id = base_frame
        support_width = 2.0 * half_fov / support_count
        support_out.angle_min = -half_fov + support_width / 2.0
        support_out.angle_max = half_fov - support_width / 2.0
        support_out.angle_increment = support_width
        support_out.scan_time = 1.0 / rate
        support_out.range_min = 0.0
        support_out.range_max = support_max
        support_out.ranges = support.astype(float).tolist()
        # Repeated validity makes a malformed/truncated intensity array fail
        # closed in consumers; the inlier count is diagnostic only.
        support_out.intensities = ([float(plane_inliers)] * support_count
                                   if plane_valid else [0.0] * support_count)
        self.support_pub.publish(support_out)
        self._last_publish = now


def main():
    rclpy.init()
    node = DepthToRangesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
