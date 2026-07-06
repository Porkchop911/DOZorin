import sys, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0  # degrees

rclpy.init()
n = Node("initpose_pub")
pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
import time
msg = PoseWithCovarianceStamped()
def fill():
    msg.header.frame_id = "map"
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    r = math.radians(yaw)
    msg.pose.pose.orientation.z = math.sin(r/2)
    msg.pose.pose.orientation.w = math.cos(r/2)
    cov = [0.0]*36
    cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.068
    msg.pose.covariance = cov
# publish a few times so amcl definitely gets it
for i in range(6):
    fill()
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(0.15)
print(f"published initialpose x={x} y={y} yaw={yaw}deg")
n.destroy_node(); rclpy.shutdown()
