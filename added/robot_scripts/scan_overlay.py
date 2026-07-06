import math, yaml, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import os; MAPY = os.environ.get("MAPY", "/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml")
with open(MAPY) as f: meta = yaml.safe_load(f)
res = meta["resolution"]; ox, oy = meta["origin"][0], meta["origin"][1]
img = np.array(Image.open("/home/ubuntu/ros2_ws/src/slam/maps/" + meta["image"]))
H, W = img.shape[:2]

rclpy.init()
n = Node("scan_overlay")
buf = tf2_ros.Buffer(); ls = tf2_ros.TransformListener(buf, n)
scan = {}
def cb(m): scan["m"] = m
n.create_subscription(LaserScan, "/scan", cb, 10)
import time
t0 = time.time()
while "m" not in scan and time.time()-t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
m = scan.get("m")
if m is None:
    print("NO SCAN"); n.destroy_node(); rclpy.shutdown(); raise SystemExit
frame = m.header.frame_id
# wait for tf map->lidar
tf = None
t0 = time.time()
while time.time()-t0 < 5:
    try:
        tf = buf.lookup_transform("map", frame, rclpy.time.Time())
        break
    except Exception:
        rclpy.spin_once(n, timeout_sec=0.2)
if tf is None:
    print("NO TF map->%s" % frame); n.destroy_node(); rclpy.shutdown(); raise SystemExit
tx = tf.transform.translation.x; ty = tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

xs=[]; ys=[]
ang = m.angle_min
for r in m.ranges:
    if math.isfinite(r) and m.range_min < r < m.range_max:
        lx = r*math.cos(ang); ly = r*math.sin(ang)
        wx = tx + lx*math.cos(yaw) - ly*math.sin(yaw)
        wy = ty + lx*math.sin(yaw) + ly*math.cos(yaw)
        xs.append(wx); ys.append(wy)
    ang += m.angle_increment

# map pixel coords: col = (wx-ox)/res, row = H-1-(wy-oy)/res
px = [(x-ox)/res for x in xs]
py = [H-1-(y-oy)/res for y in ys]
rx = (tx-ox)/res; ry = H-1-(ty-oy)/res

plt.figure(figsize=(10,10*H/W))
plt.imshow(img, cmap="gray", origin="upper")
plt.scatter(px, py, s=4, c="red", label="lidar scan")
plt.scatter([rx],[ry], s=120, c="lime", marker="o", edgecolors="black", label="robot")
plt.arrow(rx, ry, 12*math.cos(-yaw), 12*math.sin(-yaw), head_width=4, color="lime")
plt.legend(loc="upper right")
plt.title("scan (red) vs saved map  | robot map=(%.2f,%.2f) yaw=%.0fdeg" % (tx,ty,math.degrees(yaw)))
plt.savefig("/tmp/overlay.png", dpi=90, bbox_inches="tight")
print("robot map pose x=%.2f y=%.2f yaw=%.1fdeg  scan pts=%d" % (tx,ty,math.degrees(yaw),len(xs)))
n.destroy_node(); rclpy.shutdown()
