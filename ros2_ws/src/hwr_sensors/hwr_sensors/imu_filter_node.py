"""ROS wrapper for the HWR complementary IMU filter."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

from hwr_sensors.imu_filter import ImuComplementaryFilter


class ImuFilterNode(Node):
    def __init__(self):
        super().__init__("hwr_imu_filter")
        self.declare_parameter("bias_samples", 100)
        self.declare_parameter("alpha", 0.98)
        self.filter = ImuComplementaryFilter(
            int(self.get_parameter("bias_samples").value),
            float(self.get_parameter("alpha").value))
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Imu, "imu_filtered", qos)
        self.create_subscription(Imu, "imu", self._on_imu, qos)
        self._announced = False

    def _on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = self.filter.update(
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z),
            (msg.angular_velocity.x, msg.angular_velocity.y,
             msg.angular_velocity.z), stamp)
        if q is None:
            return
        if not self._announced:
            self.get_logger().info(f"gyro bias established: {self.filter.bias}")
            self._announced = True
        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
        out.orientation_covariance[0] = out.orientation_covariance[4] = 0.01
        out.orientation_covariance[8] = 0.04
        out.angular_velocity.x = msg.angular_velocity.x - self.filter.bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - self.filter.bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - self.filter.bias[2]
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = msg.linear_acceleration.x
        out.linear_acceleration.y = msg.linear_acceleration.y
        out.linear_acceleration.z = msg.linear_acceleration.z
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
