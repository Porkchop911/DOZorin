"""ROS occupancy mapper using fused odometry and the HWR log-odds grid."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from hwr_msgs.msg import LocalizationHealth
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from hwr_core.mapping import (LogOddsGrid, ScanMapMapper, compose, inverse,
                              load_map, mapping_scan_to_points)


class MapperNode(Node):
    def __init__(self):
        super().__init__("hwr_mapper")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 0.5)
        self.declare_parameter("map_path", "hwr_map")
        self.declare_parameter("keyframe_distance", 0.10)
        self.declare_parameter("keyframe_angle", math.radians(8.0))
        self.declare_parameter("minimum_match_score", 0.45)
        self.declare_parameter("maximum_mapping_range", 4.0)
        self.maximum_mapping_range = float(
            self.get_parameter("maximum_mapping_range").value)
        if not 1.0 <= self.maximum_mapping_range <= 8.0:
            raise ValueError("maximum_mapping_range must be 1.0..8.0 metres")
        map_path = Path(str(self.get_parameter("map_path").value))
        map_yaml = map_path.with_suffix(".yaml")
        state_path = map_path.with_suffix(".state.yaml")
        self.resume_pose = None
        if map_yaml.exists() and state_path.exists():
            occupancy, resolution, origin = load_map(map_yaml)
            state = yaml.safe_load(state_path.read_text(encoding="utf-8"))
            pose = state.get("map_pose") if isinstance(state, dict) else None
            if (not isinstance(pose, list) or len(pose) != 3
                    or not all(math.isfinite(float(value)) for value in pose)):
                raise ValueError("saved mapper state has no valid map_pose")
            grid = LogOddsGrid.from_occupancy(
                occupancy, resolution, origin)
            self.resume_pose = tuple(float(value) for value in pose)
            self.get_logger().info(
                f"resuming map {map_yaml} at pose {self.resume_pose}")
        else:
            grid = LogOddsGrid(float(self.get_parameter("resolution").value))
        self.mapper = ScanMapMapper(
            grid=grid,
            keyframe_dist=float(self.get_parameter("keyframe_distance").value),
            keyframe_angle=float(self.get_parameter("keyframe_angle").value),
            minimum_score=float(self.get_parameter("minimum_match_score").value))
        self.pose = None
        self.map_pose = self.resume_pose
        self.last_score = 0.0
        self.enabled = True
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(OccupancyGrid, "map", map_qos)
        self.pub_score = self.create_publisher(Float32, "mapping_match_score", 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "pose", 10)
        self.pub_health = self.create_publisher(LocalizationHealth,
                                                "localization_health", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_service(SetBool, "mapping_enabled", self._on_enabled)
        self.create_service(Trigger, "map_save", self._on_save)
        self.tf = TransformBroadcaster(self)
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(0.1, self._publish_state)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self.resume_pose is not None:
            # Odometry intentionally restarts from a new local origin. Anchor
            # that first sample at the saved physical map pose before any scan
            # can be matched or integrated.
            self.mapper.map_to_odom = compose(
                self.resume_pose, inverse(self.pose))
            self.resume_pose = None

    def _on_scan(self, msg: LaserScan) -> None:
        if not self.enabled or self.pose is None:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = mapping_scan_to_points(
            angles, ranges, msg.range_max, self.maximum_mapping_range,
            sensor_yaw=math.pi, sensor_xy=(0.048156, 0.0))
        update = self.mapper.process(self.pose, points)
        self.map_pose = update.pose
        self.last_score = update.score
        if update.integrated:
            score = Float32()
            score.data = float(update.score)
            self.pub_score.publish(score)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        grid = self.mapper.grid
        occupancy = grid.occupancy()
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = now, "map"
        msg.info.map_load_time = now
        msg.info.resolution = grid.resolution
        msg.info.width, msg.info.height = occupancy.shape[1], occupancy.shape[0]
        msg.info.origin.position.x, msg.info.origin.position.y = grid.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.ravel().astype(int).tolist()
        self.pub.publish(msg)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = now, "map"
        transform.child_frame_id = "odom"
        x, y, yaw = self.mapper.map_to_odom
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(transform)

    def _publish_state(self) -> None:
        if self.map_pose is None:
            return
        now = self.get_clock().now().to_msg()
        x, y, yaw = self.map_pose
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = now, "map"
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.05
        pose.pose.covariance[35] = 0.05
        self.pub_pose.publish(pose)
        health = LocalizationHealth()
        health.header = pose.header
        health.healthy = True
        health.effective_sample_size = 0.0
        health.mean_scan_score = self.last_score
        health.state = "mapping"
        self.pub_health.publish(health)

    def _on_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = "mapping enabled" if self.enabled else "mapping paused"
        return response

    def _on_save(self, request, response):
        try:
            path = Path(str(self.get_parameter("map_path").value))
            pgm, metadata = self.mapper.grid.save(path)
            pose = self.map_pose
            if pose is None and self.pose is not None:
                pose = compose(self.mapper.map_to_odom, self.pose)
            if pose is None:
                raise RuntimeError("map pose unavailable")
            state_path = path.with_suffix(".state.yaml")
            state_path.write_text(yaml.safe_dump({
                "map_pose": [float(value) for value in pose],
            }, sort_keys=False), encoding="utf-8")
            response.success = True
            response.message = (
                f"saved {pgm}, {metadata}, and {state_path}")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response


def main():
    rclpy.init()
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
