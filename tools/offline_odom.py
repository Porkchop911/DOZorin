"""Replay a ROS 2 bag through the framework-free Phase-5 odometry core."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from hwr_core.matching import scan_to_points
from hwr_core.odometry import ScanOdometry


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=Path("odom_report.json"))
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {name: get_message(types[name]) for name in
              ("/cmd_vel_applied", "/imu_filtered", "/scan") if name in types}
    missing = {"/cmd_vel_applied", "/imu_filtered", "/scan"} - set(wanted)
    if missing:
        parser.error(f"bag is missing required topics: {sorted(missing)}")

    odom = ScanOdometry()
    command = (0.0, 0.0)
    gyro_z = 0.0
    last_ns = None
    trajectory = []
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        if last_ns is not None:
            dt = (stamp_ns - last_ns) * 1e-9
            if 0.0 < dt <= 1.0:
                odom.predict(command[0], command[1], gyro_z, dt)
        last_ns = stamp_ns
        msg = deserialize_message(raw, wanted[topic])
        if topic == "/cmd_vel_applied":
            command = (msg.twist.linear.x, msg.twist.linear.y)
        elif topic == "/imu_filtered":
            gyro_z = msg.angular_velocity.z
        else:
            ranges = np.asarray(msg.ranges, dtype=float)
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            points = scan_to_points(angles, ranges, msg.range_max,
                                    sensor_yaw=math.pi,
                                    sensor_xy=(0.048156, 0.0))
            match = odom.process_scan(points)
            trajectory.append({
                "stamp_ns": stamp_ns,
                "pose": odom.pose,
                "score": None if match is None else match.score,
                "valid": None if match is None else match.valid,
            })

    report = {
        "bag": str(args.bag),
        "samples": trajectory,
        "final_pose": odom.pose,
        "accepted_matches": odom.fusion.accepted,
        "rejected_matches": odom.fusion.rejected,
        "keyframes": odom.keyframes,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "samples"},
                     indent=2))
    print(f"trajectory -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
