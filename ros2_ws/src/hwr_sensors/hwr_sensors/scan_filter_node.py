"""hwr_sensors scan filter — our band-pass between the lidar binary and
everything downstream (plan 4.2, replaces laser_filters).

Sub:  scan_raw (sensor_msgs/LaserScan, best-effort)
Pub:  scan     (sensor_msgs/LaserScan, best-effort) — cropped to the
      usable ~219° sector, self-hits (<0.12 m) blanked to inf.
Jazzy-ready: plain rclpy.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan

from hwr_sensors.scan_crop import BAND_HI, BAND_LO, MIN_RANGE, crop_scan


class ScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_scan_filter")
        self.declare_parameter("lower_angle", BAND_LO)
        self.declare_parameter("upper_angle", BAND_HI)
        self.declare_parameter("min_range", MIN_RANGE)
        self.lo = float(self.get_parameter("lower_angle").value)
        self.hi = float(self.get_parameter("upper_angle").value)
        self.min_range = float(self.get_parameter("min_range").value)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LaserScan, "scan", qos)
        self.create_subscription(LaserScan, "scan_raw", self._on_scan, qos)

    def _on_scan(self, msg: LaserScan) -> None:
        new_min, ranges = crop_scan(msg.angle_min, msg.angle_increment,
                                    msg.ranges, self.lo, self.hi,
                                    self.min_range)
        out = LaserScan()
        out.header = msg.header
        out.angle_min = new_min
        out.angle_increment = msg.angle_increment
        out.angle_max = new_min + msg.angle_increment * max(len(ranges) - 1, 0)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = msg.range_max
        out.ranges = [float(r) for r in ranges]
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
