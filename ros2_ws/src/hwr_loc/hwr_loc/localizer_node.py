"""ROS wrapper for likelihood-field Monte-Carlo localization."""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from hwr_msgs.msg import LocalizationHealth
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from hwr_core.localization import (ParticleFilter, aggregate_scan_panorama,
                                   localization_motion_consistent)
from hwr_core.mapping import load_map
from hwr_core.matching import scan_to_points


def _yaw(q):
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class LocalizerNode(Node):
    def __init__(self):
        super().__init__("hwr_localizer")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("particle_count", 500)
        self.declare_parameter("auto_global_relocalize", False)
        map_path = self.get_parameter("map_yaml").value
        if not map_path:
            raise RuntimeError("map_yaml parameter is required")
        occupancy, resolution, origin = load_map(map_path)
        self.filter = ParticleFilter(occupancy, resolution, origin,
                                     int(self.get_parameter("particle_count").value))
        self.odom_pose = None
        self.dead_reckoning_pose = (0.0, 0.0, 0.0)
        self.last_odom_time = None
        self.last_scan = None
        self.ever_healthy = False
        self.unhealthy_streak = 0
        self.last_relocalize = 0.0
        self.relocalizing = False
        self.scan_history = deque(maxlen=200)
        self.last_consistent_pose = None
        self.last_consistent_odom = None
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "pose", 10)
        self.pub_health = self.create_publisher(LocalizationHealth,
                                                "localization_health", 10)
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos)
        self.create_service(Trigger, "global_relocalize", self._on_global)
        self.tf = TransformBroadcaster(self)

    def _on_odom(self, msg: Odometry) -> None:
        current = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                   _yaw(msg.pose.pose.orientation))
        self.odom_pose = current
        now = time.monotonic()
        if self.last_odom_time is not None:
            dt = now - self.last_odom_time
            if 0.0 < dt <= 0.5:
                twist = msg.twist.twist
                delta = (twist.linear.x * dt, twist.linear.y * dt,
                         twist.angular.z * dt)
                self.filter.predict(delta)
                x, y, yaw = self.dead_reckoning_pose
                midpoint = yaw + 0.5 * delta[2]
                c, s = math.cos(midpoint), math.sin(midpoint)
                self.dead_reckoning_pose = (
                    x + c * delta[0] - s * delta[1],
                    y + s * delta[0] + c * delta[1],
                    math.atan2(math.sin(yaw + delta[2]),
                               math.cos(yaw + delta[2])))
        self.last_odom_time = now

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = scan_to_points(angles, ranges, msg.range_max,
                                sensor_yaw=math.pi, sensor_xy=(0.048156, 0.0))
        if len(points) < 10:
            # A transient empty scan must degrade localization, never terminate
            # the process by entering coarse global search without hit points.
            if self.ever_healthy:
                self.unhealthy_streak += 1
            self._publish(msg.header.stamp, False,
                          self.filter.effective_sample_size)
            return
        self.last_scan = points
        self.scan_history.append(
            (time.monotonic(), self.dead_reckoning_pose, points.copy()))
        prior_particles = self.filter.particles.copy()
        prior_weights = self.filter.weights.copy()
        prior_score = self.filter.mean_score
        self.filter.update(points)
        ess = self.filter.effective_sample_size
        healthy = ess >= 0.2 * self.filter.count and self.filter.mean_score >= 0.35
        candidate_pose = self.filter.pose
        consistent = localization_motion_consistent(
            self.last_consistent_pose, candidate_pose,
            self.last_consistent_odom, self.dead_reckoning_pose)
        if not consistent:
            # Reject a scan-induced map-mode jump. Retain odometry prediction
            # from before this measurement so the next scan can still recover.
            self.filter.particles = prior_particles
            self.filter.weights = prior_weights
            self.filter.mean_score = prior_score
            ess = self.filter.effective_sample_size
            healthy = False
            self.get_logger().warning(
                "rejected localization update inconsistent with odometry",
                throttle_duration_sec=2.0)
        self.filter.resample_if_needed()
        if healthy:
            self.last_consistent_pose = self.filter.pose
            self.last_consistent_odom = self.dead_reckoning_pose
            self.ever_healthy = True
            self.unhealthy_streak = 0
            self.relocalizing = False
        elif self.ever_healthy:
            self.unhealthy_streak += 1
            now = time.monotonic()
            if (self.get_parameter("auto_global_relocalize").value
                    and self.unhealthy_streak >= 10
                    and now - self.last_relocalize >= 15.0):
                self.last_relocalize = now
                self.unhealthy_streak = 0
                try:
                    self._global_relocalize(points)
                    self.relocalizing = True
                except ValueError as exc:
                    # Input validity is checked above, but keep the ROS wrapper
                    # alive if a future filter rejects a degenerate scan.
                    self.get_logger().warning(
                        f"global relocalization deferred: {exc}")
        self._publish(msg.header.stamp, healthy, ess)

    def _publish(self, stamp, healthy: bool, ess: float) -> None:
        x, y, yaw = self.filter.pose
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = qz, qw
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.1
        pose.pose.covariance[35] = 0.1
        self.pub_pose.publish(pose)
        health = LocalizationHealth()
        health.header = pose.header
        health.healthy = healthy
        health.effective_sample_size = ess
        health.mean_scan_score = self.filter.mean_score
        health.state = ("relocalizing" if self.relocalizing else
                        "tracking" if health.healthy else "degraded")
        self.pub_health.publish(health)
        if self.odom_pose is not None:
            ox, oy, oyaw = self.odom_pose
            angle = yaw - oyaw
            c, s = math.cos(angle), math.sin(angle)
            tf = TransformStamped()
            tf.header = pose.header
            tf.child_frame_id = "odom"
            tf.transform.translation.x = x - (c * ox - s * oy)
            tf.transform.translation.y = y - (s * ox + c * oy)
            tf.transform.rotation.z = math.sin(angle / 2)
            tf.transform.rotation.w = math.cos(angle / 2)
            self.tf.sendTransform(tf)

    def _on_global(self, request, response):
        if self.last_scan is None or len(self.last_scan) < 10:
            response.success, response.message = False, "no usable scan received"
            return response
        try:
            now = time.monotonic()
            history = [(pose, points) for stamp, pose, points in self.scan_history
                       if now - stamp <= 35.0]
            panorama = aggregate_scan_panorama(
                history, self.dead_reckoning_pose)
            evidence = panorama if len(panorama) >= 60 else self.last_scan
            poses, scores = self._global_relocalize(evidence)
            response.success = True
            response.message = (f"seeded from {len(poses)} hypotheses; "
                                f"best score={scores[0]:.3f}; "
                                f"evidence_points={len(evidence)}")
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def _global_relocalize(self, points):
        poses, scores = self.filter.global_relocalize(points)
        self.last_consistent_pose = None
        self.last_consistent_odom = None
        self.relocalizing = True
        self.get_logger().info(
            f"global relocalization seeded: best={scores[0]:.3f} "
            f"at ({poses[0, 0]:.2f}, {poses[0, 1]:.2f}, {poses[0, 2]:.2f})")
        return poses, scores


def main():
    rclpy.init()
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
