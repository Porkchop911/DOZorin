#!/usr/bin/env python3
# goal_pick.py <station> — print "x y yaw_deg" for a station approach pose that
# is FREE in the live global costmap with real clearance, facing the station.
import sys, math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

NOMINAL = {
    "drums":  (-0.01, -0.79, 150.0),
    "door":   (-1.55, -1.26, 267.0),
    "window": (0.84, -1.08, 353.0),
    "start":  (-0.22, 0.05, 44.0),
}

st = sys.argv[1]
nx, ny, nyaw = NOMINAL[st]

rclpy.init()
n = Node("goalpick")
grid = []
qos = QoSProfile(depth=1)
qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: grid.append(m), qos)
t0 = time.time()
while not grid and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.5)
if not grid:
    print("ERROR no costmap", file=sys.stderr); sys.exit(1)
g = grid[0]
res = g.info.resolution
ox, oy = g.info.origin.position.x, g.info.origin.position.y
W, H = g.info.width, g.info.height

def cost(x, y):
    i, j = int((x - ox) / res), int((y - oy) / res)
    if 0 <= i < W and 0 <= j < H:
        return g.data[j * W + i]
    return 100

def clear(x, y, rad=0.22, lim=60):
    k = int(rad / res)
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            if di * di + dj * dj > k * k:
                continue
            if cost(x + di * res, y + dj * res) >= lim:
                return False
    return True

best = None
# rings from close to far: nearest good standoff wins
r = 0.0
while r <= 1.1:
    cands = []
    steps = max(1, int(2 * math.pi * max(r, 0.05) / res))
    for a in range(steps):
        ang = 2 * math.pi * a / steps
        x, y = nx + r * math.cos(ang), ny + r * math.sin(ang)
        if cost(x, y) == 0 and clear(x, y):
            cands.append((x, y))
    if cands:
        # prefer candidate closest to nominal yaw direction opposite (approach from front)
        x, y = cands[0]
        best = (x, y)
        break
    r += 0.1
if best is None:
    print("ERROR no free approach", file=sys.stderr); sys.exit(2)
x, y = best
if math.hypot(nx - x, ny - y) > 0.05:
    yaw = math.degrees(math.atan2(ny - y, nx - x)) % 360
else:
    yaw = nyaw
print("%.2f %.2f %.0f" % (x, y, yaw))
n.destroy_node(); rclpy.shutdown()
