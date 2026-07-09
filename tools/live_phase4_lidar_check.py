"""Static lidar verification on the robot (plan 4.1/4.2 — no motion).

Run inside a takeover window with hwr_sensors lidar.launch.py up:
  - collects scan_raw and scan, prints geometry (frame, band, beam count)
  - verifies the filter cropped to the configured band with no self-hits
  - runs the correlative matcher on consecutive static scans: the robot
    is not moving, so the recovered delta must be ~zero with a high score
    — first contact between the Phase 5 matcher and real lidar data
  - dumps the last filtered scan to ~/hwr/evidence_scan.json

Exit 0 = all checks passed. Silent (no buzzer — printed output only).
"""
import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from hwr_core.matching import CorrelativeMatcher, scan_to_points

LIDAR_MOUNT_YAW = math.pi
LIDAR_MOUNT_XY = (0.048156, 0.0)
BAND_LO, BAND_HI = 1.25, 5.08
WANT_RAW, WANT_FILTERED = 2, 4

failures = []


def check(label, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label} {detail}")
    if not ok:
        failures.append(label)


class Collector(Node):
    def __init__(self):
        super().__init__("phase4_lidar_check")
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.raw, self.filtered = [], []
        self.create_subscription(LaserScan, "scan_raw",
                                 lambda m: self.raw.append(m), qos)
        self.create_subscription(LaserScan, "scan",
                                 lambda m: self.filtered.append(m), qos)

    def done(self):
        return len(self.raw) >= WANT_RAW and len(self.filtered) >= WANT_FILTERED


def describe(tag, m):
    r = np.asarray(m.ranges)
    finite = np.isfinite(r) & (r > 0)
    print(f"  {tag}: frame={m.header.frame_id} beams={len(r)} "
          f"angles=[{m.angle_min:.3f},{m.angle_max:.3f}] "
          f"inc={m.angle_increment:.5f} finite={finite.mean() * 100:.0f}% "
          f"range=[{r[finite].min():.2f},{r[finite].max():.2f}] m")


def main():
    rclpy.init()
    node = Collector()
    import time
    t0 = time.time()
    while not node.done() and time.time() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.5)

    print("=== collection ===")
    check("scan_raw received", len(node.raw) >= WANT_RAW,
          f"({len(node.raw)})")
    check("scan (filtered) received", len(node.filtered) >= WANT_FILTERED,
          f"({len(node.filtered)})")
    if failures:
        rclpy.shutdown()
        return 1

    raw, flt = node.raw[-1], node.filtered[-1]
    print("=== geometry ===")
    describe("scan_raw", raw)
    describe("scan    ", flt)
    check("frame is lidar_frame", flt.header.frame_id == "lidar_frame",
          flt.header.frame_id)
    check("filtered band lower", flt.angle_min >= BAND_LO - 0.02,
          f"{flt.angle_min:.3f}")
    check("filtered band upper", flt.angle_max <= BAND_HI + 0.02,
          f"{flt.angle_max:.3f}")
    check("filtered is a subset of raw", len(flt.ranges) < len(raw.ranges),
          f"{len(flt.ranges)} < {len(raw.ranges)}")
    r = np.asarray(flt.ranges)
    finite = np.isfinite(r)
    check("no self-hits pass the filter", (r[finite] >= 0.12).all(),
          f"min {r[finite].min():.3f} m")
    check("usable returns > 50%", finite.mean() > 0.5,
          f"{finite.mean() * 100:.0f}%")

    print("=== matcher vs real scans (robot static) ===")

    def points(m):
        n = len(m.ranges)
        angles = m.angle_min + np.arange(n) * m.angle_increment
        rr = np.asarray(m.ranges, dtype=float)
        rr[~np.isfinite(rr)] = 1e6              # dropped by max_range cut
        return scan_to_points(angles, rr, max_range=8.0,
                              sensor_yaw=LIDAR_MOUNT_YAW,
                              sensor_xy=LIDAR_MOUNT_XY)

    matcher = CorrelativeMatcher()
    matcher.set_reference(points(node.filtered[-2]))
    res = matcher.match(points(flt))
    print(f"  delta=({res.dx * 1000:.1f}, {res.dy * 1000:.1f}) mm, "
          f"{math.degrees(res.dtheta):.3f} deg  score={res.score:.3f} "
          f"degenerate={res.degenerate}")
    check("matcher valid", res.valid)
    check("static delta < 10 mm", math.hypot(res.dx, res.dy) < 0.010)
    check("static rotation < 0.5 deg", abs(res.dtheta) < math.radians(0.5))
    check("score > 0.7", res.score > 0.7, f"{res.score:.3f}")

    out = os.path.expanduser("~/hwr/evidence_scan.json")
    with open(out, "w") as f:
        json.dump({"frame_id": flt.header.frame_id,
                   "angle_min": flt.angle_min,
                   "angle_increment": flt.angle_increment,
                   "ranges": [None if not math.isfinite(x) else round(x, 4)
                              for x in flt.ranges]}, f)
    print(f"  scan sample -> {out}")

    rclpy.shutdown()
    print(f"=== {'ALL PASS' if not failures else 'FAILURES: ' + ', '.join(failures)} ===")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
