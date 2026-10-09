"""Fused odometry from applied velocity, filtered gyro, and scan matching."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from hwr_core.matching import scan_to_points
from hwr_core.odometry import ScanOdometry

LIDAR_YAW = math.pi
LIDAR_XY = (0.048156, 0.0)


class OdomNode(Node):
    def __init__(self):
        super().__init__("hwr_odom")
        self.declare_parameter("predict_rate", 50.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.odom = ScanOdometry()
        self._command = (0.0, 0.0)
        self._gyro_z = 0.0
        self._last_predict = None
        self._last_scan = 0.0
        self._translation_active_until = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_scan = self.create_publisher(Odometry, "odom_scan", 10)
        self.pub_score = self.create_publisher(Float32, "scan_match_score", 10)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TwistStamped, "cmd_vel_applied", self._on_command, 10)
        self.create_subscription(Imu, "imu_filtered", self._on_imu, qos)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos)
        rate = float(self.get_parameter("predict_rate").value)
        self.create_timer(1.0 / rate, self._predict)

    def _on_command(self, msg: TwistStamped) -> None:
        self._command = (msg.twist.linear.x, msg.twist.linear.y)
        if math.hypot(*self._command) > 1e-4:
            # Retain translation observability through the first scan after a
            # command stops; scan matching runs more slowly than prediction.
            self._translation_active_until = time.monotonic() + 0.5

    def _on_imu(self, msg: Imu) -> None:
        self._gyro_z = msg.angular_velocity.z

    def _predict(self) -> None:
        now = self.get_clock().now()
        if self._last_predict is None:
            self._last_predict = now
            return
        dt = (now - self._last_predict).nanoseconds * 1e-9
        self._last_predict = now
        if not 0.0 < dt <= 0.5:
            return
        self.odom.predict(self._command[0], self._command[1], self._gyro_z, dt)
        self._publish(self.pub_odom, now)
        if self.get_parameter("publish_tf").value:
            self._publish_tf(now)

    def _on_scan(self, msg: LaserScan) -> None:
        now_mono = time.monotonic()
        rate = float(self.get_parameter("scan_rate").value)
        if rate <= 0.0 or now_mono - self._last_scan < 1.0 / rate:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        points = scan_to_points(angles, ranges, msg.range_max,
                                sensor_yaw=LIDAR_YAW, sensor_xy=LIDAR_XY)
        result = self.odom.process_scan(
            points,
            observe_translation=(
                now_mono <= self._translation_active_until))
        self._last_scan = now_mono
        if result is None:
            return
        score = Float32()
        score.data = float(result.score)
        self.pub_score.publish(score)
        if result.valid:
            self._publish(self.pub_scan, self.get_clock().now())

    def _message(self, now) -> Odometry:
        x, y, yaw = self.odom.pose
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.get_parameter("odom_frame").value
        msg.child_frame_id = self.get_parameter("base_frame").value
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        p = self.odom.fusion.P
        msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[5] = p[0]
        msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[11] = p[1]
        msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35] = p[2]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = self._command
        msg.twist.twist.angular.z = self._gyro_z
        return msg

    def _publish(self, publisher, now) -> None:
        publisher.publish(self._message(now))

    def _publish_tf(self, now) -> None:
        msg = self._message(now)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
