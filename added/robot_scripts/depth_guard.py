#!/usr/bin/env python3
# depth_guard.py — cheap thin-obstacle guard for Nav2.
# Condenses the arm camera depth image (arm LOCKED forward: servos 1=500,2=701,
# 3=120,4=300,5=500) into a sparse synthetic LaserScan (~64 beams, <=3 Hz) in
# base_footprint, floor-filtered, for the local costmap. Replaces the 12 Hz
# full-pointcloud source that starved the Jetson (nav container 84% CPU).
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

HFOV = 1.15          # rgb/depth horizontal FOV ~66deg
VFOV = 0.75          # vertical FOV ~43deg
CAM_H = 0.36         # camera height above floor (RANSAC-measured 2026-07-03)
CAM_TILT = 0.14      # ~8 deg downtilt
CAM_FWD = 0.10       # camera x offset ahead of base_footprint (approx)
H_MIN, H_MAX = 0.10, 1.20   # keep obstacles in this height band
R_MIN, R_MAX = 0.25, 2.5
BEAMS = 64
RATE = 3.0

class Guard(Node):
    def __init__(self):
        super().__init__("depth_guard")
        self.br = CvBridge(); self.last = 0.0
        self.pub = self.create_publisher(LaserScan, "/depth_scan", 5)
        self.create_subscription(Image, "/depth_cam/depth0/image_raw", self.cb, 5)
        self.get_logger().info("depth_guard up: %d beams @ %.1f Hz max" % (BEAMS, RATE))

    def cb(self, msg):
        t = time.time()
        if t - self.last < 1.0/RATE: return
        self.last = t
        dep = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
        if dep.size == 0: return
        if np.nanmax(dep) > 20.0: dep *= 0.001
        h, w = dep.shape[:2]
        # per-pixel vertical angle (down positive), point height above floor
        rows = (np.arange(h, dtype=np.float32)[:, None] / h - 0.5) * VFOV + CAM_TILT
        height = CAM_H - dep * np.sin(rows)
        valid = np.isfinite(dep) & (dep > R_MIN) & (dep < R_MAX) & (height > H_MIN) & (height < H_MAX)
        d = np.where(valid, dep, np.inf)
        # column-band minimum -> beams (left of image = positive bearing)
        band = w // BEAMS
        d = d[:, :band*BEAMS].reshape(h, BEAMS, band)
        rng = d.min(axis=(0, 2))
        beams = np.full(BEAMS, np.inf, dtype=np.float32)
        # beam i center: xfrac=(i+0.5)/BEAMS, bearing=(0.5-xfrac)*HFOV; publish angles ascending
        for i in range(BEAMS):
            r = rng[i]
            if np.isfinite(r):
                beams[BEAMS-1-i] = r * math.cos(CAM_TILT) + CAM_FWD
        s = LaserScan()
        s.header.stamp = msg.header.stamp
        s.header.frame_id = "base_footprint"
        s.angle_min = -HFOV/2; s.angle_max = HFOV/2
        s.angle_increment = HFOV/(BEAMS-1)
        s.range_min = R_MIN; s.range_max = R_MAX
        s.ranges = [float(x) if np.isfinite(x) else float("inf") for x in beams]
        self.pub.publish(s)

rclpy.init()
rclpy.spin(Guard())
