"""hwr_base — cmd_vel pipeline + model odometry.

Pipeline (20 Hz tick, not per-message, so stale sources decay to stop):
    teleop/agent/nav command mux -> NO-REVERSE clamp (vx >= 0,
    structural: the robot is rear-blind) -> velocity clamp -> accel ramp ->
    mecanum mixing -> wheel_speeds
    The one exception: cmd_vel_reverse (gateway /motion/back, plan/18.4) drives slow pure -x,
    only while no mux source is active.

Model odometry: integrates the APPLIED command (vx and vy — the vendor
drops vy!) with yaw from the IMU gyro (bias estimated at rest on startup).
Publishes odom_model + optional TF odom->base_footprint with honest
(large) covariances; Phase 5 fusion takes the TF over later.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from hwr_msgs.msg import WheelSpeeds
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from hwr_core.control import CommandMux
from hwr_core.kinematics import Calibration, wheel_speeds
from hwr_core.safety import Limits
from hwr_core.navigation.retrace import clamp_reverse

BIAS_SAMPLES = 100  # ~2 s of IMU at 50 Hz, robot must start at rest


class BaseNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_base")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("teleop_timeout", 0.35)
        self.declare_parameter("agent_timeout", 0.5)
        self.declare_parameter("nav_timeout", 0.5)
        self.declare_parameter("allow_reverse", False)  # keep False: rear-blind robot
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("cal_linear", 1.0)
        self.declare_parameter("cal_strafe", 1.0)
        self.declare_parameter("cal_angular", 1.0)

        self.limits = Limits()
        self.cal = Calibration(
            linear=float(self.get_parameter("cal_linear").value),
            strafe=float(self.get_parameter("cal_strafe").value),
            angular=float(self.get_parameter("cal_angular").value))

        self.pub_wheels = self.create_publisher(WheelSpeeds, "wheel_speeds", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom_model", 10)
        self.pub_applied = self.create_publisher(TwistStamped, "cmd_vel_applied", 10)
        self.pub_source = self.create_publisher(String, "cmd_vel_source", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.mux = CommandMux(
            ("teleop", "agent", "nav"),
            {source: float(self.get_parameter(f"{source}_timeout").value)
             for source in ("teleop", "agent", "nav")})
        for source in ("teleop", "agent", "nav"):
            self.create_subscription(
                Twist, f"cmd_vel_{source}",
                lambda msg, source=source: self._on_cmd_vel(source, msg), 10)
        # plan/18.4: reverse only arrives here (gateway /motion/back after a retrace budget and a
        # rear camera look), never through the mux, and only while no other source is active.
        self._reverse = (0.0, 0.0, 0.0)
        self._reverse_stamp = float("-inf")
        self.create_subscription(Twist, "cmd_vel_reverse", self._on_reverse, 10)
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "imu", self._on_imu, sensor_qos)

        self._applied = (0.0, 0.0, 0.0)
        self._active_source = None
        # odometry state
        self._x = self._y = self._yaw = 0.0
        self._gz = 0.0
        self._gz_bias = 0.0
        self._bias_buf: list[float] = []
        self._last_tick = None

        period = 1.0 / float(self.get_parameter("rate").value)
        self.create_timer(period, self._tick)

    # ------------------------------------------------------------- inputs

    def _on_cmd_vel(self, source: str, msg: Twist) -> None:
        stamp = self.get_clock().now().nanoseconds * 1e-9
        self.mux.update(source, (msg.linear.x, msg.linear.y, msg.angular.z), stamp)

    def _on_reverse(self, msg: Twist) -> None:
        self._reverse = clamp_reverse(msg.linear.x, msg.linear.y, msg.angular.z)
        self._reverse_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        gz = msg.angular_velocity.z
        if len(self._bias_buf) < BIAS_SAMPLES:
            self._bias_buf.append(gz)
            if len(self._bias_buf) == BIAS_SAMPLES:
                self._gz_bias = sum(self._bias_buf) / len(self._bias_buf)
                self.get_logger().info(f"gyro z bias: {self._gz_bias:+.5f} rad/s")
            return
        self._gz = gz - self._gz_bias

    # ------------------------------------------------------------- 20 Hz pipeline

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = 0.0 if self._last_tick is None else (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now

        # Dead sources fall through by priority; no fresh source ramps to stop.
        source, target = self.mux.select(now.nanoseconds * 1e-9)
        if source != self._active_source:
            self._active_source = source
            source_msg = String()
            source_msg.data = source or "none"
            self.pub_source.publish(source_msg)

        vx, vy, wz = target
        if not self.get_parameter("allow_reverse").value and vx < 0.0:
            vx = 0.0  # structural no-reverse: rear ~140 deg is lidar-blind
        if source is None and now.nanoseconds * 1e-9 - self._reverse_stamp <= 0.3:
            vx, vy, wz = self._reverse  # already clamp_reverse'd: pure -x, <= 0.06 m/s
        cmd = self.limits.clamp_velocity(vx, vy, wz)
        cmd = self.limits.clamp_accel(cmd, self._applied, dt)
        self._applied = cmd

        applied = TwistStamped()
        applied.header.stamp = now.to_msg()
        applied.header.frame_id = self.get_parameter("base_frame").value
        applied.twist.linear.x, applied.twist.linear.y = cmd[0], cmd[1]
        applied.twist.angular.z = cmd[2]
        self.pub_applied.publish(applied)

        msg = WheelSpeeds()
        msg.header.stamp = now.to_msg()
        msg.rps = [float(r) for r in wheel_speeds(*cmd, cal=self.cal)]
        self.pub_wheels.publish(msg)

        self._integrate_odom(cmd, dt, now)

    def _integrate_odom(self, cmd, dt: float, now) -> None:
        vx, vy, _ = cmd
        yaw_rate = self._gz  # gyro is the better wz source than the command
        # exact integration for constant twist over dt (midpoint heading is
        # adequate at 20 Hz and our speeds)
        mid_yaw = self._yaw + 0.5 * yaw_rate * dt
        self._x += (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
        self._y += (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt
        self._yaw = math.atan2(math.sin(self._yaw + yaw_rate * dt),
                               math.cos(self._yaw + yaw_rate * dt))

        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        qz, qw = math.sin(self._yaw / 2.0), math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = yaw_rate
        # honest covariances: open-loop model, no wheel feedback exists
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.005
        self.pub_odom.publish(odom)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = odom_frame
            tf.child_frame_id = base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
