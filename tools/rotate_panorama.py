#!/usr/bin/env python3
"""Collect a bounded rotation-only scan panorama through the HWR gateway.

This tool deliberately cannot command linear motion.  The gateway lease is
shorter than half a second, so loss of this client stops yaw commands quickly;
the emergency stop is latched on every exit path.
"""
from __future__ import annotations

import argparse
import math
import os
import time

import requests


DEFAULT_URL = "http://192.168.0.223:8090"


class Gateway:
    def __init__(self, url: str, token: str, timeout: float = 2.0):
        self.url = url.rstrip("/")
        self.headers = {"Authorization": f"Bearer {token}"}
        self.timeout = timeout

    def request(self, method: str, path: str, payload=None) -> dict:
        response = requests.request(
            method, self.url + path, headers=self.headers, json=payload,
            timeout=self.timeout)
        if not response.ok:
            raise RuntimeError(
                f"gateway {path} returned HTTP {response.status_code}: "
                f"{response.text}")
        return response.json()


def parser() -> argparse.ArgumentParser:
    out = argparse.ArgumentParser(
        description="Perform one bounded, zero-translation panorama rotation")
    out.add_argument("--url", default=os.environ.get("HWR_GATEWAY_URL", DEFAULT_URL))
    out.add_argument("--token", default=os.environ.get("HWR_GATEWAY_TOKEN", ""))
    out.add_argument("--wz", type=float, default=0.22,
                     help="yaw command in rad/s; sign selects direction")
    out.add_argument("--turns", type=float, default=1.05,
                     help="slightly over one turn provides scan overlap")
    out.add_argument("--lease", type=float, default=0.40)
    out.add_argument("--period", type=float, default=0.20)
    return out


def main() -> int:
    args = parser().parse_args()
    if len(args.token) < 24:
        raise SystemExit("set HWR_GATEWAY_TOKEN or pass --token")
    values = (args.wz, args.turns, args.lease, args.period)
    if not all(math.isfinite(value) for value in values):
        raise SystemExit("all rotation parameters must be finite")
    if not 0.15 <= abs(args.wz) <= 0.50:
        raise SystemExit("abs(wz) must be in 0.15..0.50 rad/s")
    if not 0.05 <= args.turns <= 1.10:
        raise SystemExit("turns must be in 0.05..1.10")
    if not 0.20 <= args.lease <= 0.50:
        raise SystemExit("lease must be in 0.20..0.50 seconds")
    if not 0.10 <= args.period < args.lease:
        raise SystemExit("period must be at least 0.10 s and shorter than lease")

    target_rotation = args.turns * 2.0 * math.pi
    timeout = 33.5

    gateway = Gateway(args.url, args.token)
    context = gateway.request("GET", "/navigation/context")
    if context.get("current_goal") is not None:
        raise SystemExit("refusing panorama while a navigation goal exists")
    if context.get("drive_active"):
        raise SystemExit("refusing panorama while another drive lease is active")
    if not context.get("camera_fresh"):
        raise SystemExit("refusing panorama because the perception stream is stale")
    odometry = context.get("dead_reckoning", {})
    odom_pose = odometry.get("pose")
    if not odometry.get("fresh") or not odom_pose or len(odom_pose) != 3:
        raise SystemExit("refusing panorama because odometry is unavailable or stale")
    if not all(math.isfinite(float(value)) for value in odom_pose):
        raise SystemExit("refusing panorama because odometry is non-finite")
    start_xy = tuple(float(value) for value in odom_pose[:2])
    last_yaw = float(odom_pose[2])
    accumulated_yaw = 0.0

    reset = gateway.request("POST", "/estop/reset")
    if not reset.get("success"):
        raise RuntimeError(f"estop reset failed: {reset.get('message', 'unknown')}")

    started = time.monotonic()
    next_report = started
    try:
        while accumulated_yaw < target_rotation:
            elapsed = time.monotonic() - started
            if elapsed >= timeout:
                raise RuntimeError(
                    f"panorama timed out after {accumulated_yaw:.2f} rad")
            gateway.request("POST", "/drive", {
                "vx": 0.0,
                "vy": 0.0,
                "wz": args.wz,
                "duration": args.lease,
            })
            context = gateway.request("GET", "/navigation/context")
            odometry = context.get("dead_reckoning", {})
            current = odometry.get("pose")
            if not odometry.get("fresh") or not current or len(current) != 3:
                raise RuntimeError("odometry became unavailable during panorama")
            current = tuple(float(value) for value in current)
            if not all(math.isfinite(value) for value in current):
                raise RuntimeError("odometry became non-finite during panorama")
            if math.hypot(current[0] - start_xy[0],
                          current[1] - start_xy[1]) > 0.05:
                raise RuntimeError("linear odometry exceeded 0.05 m during panorama")
            yaw_delta = math.atan2(math.sin(current[2] - last_yaw),
                                   math.cos(current[2] - last_yaw))
            # HTTP polling can occasionally span more than one odometry sample;
            # the gateway's dead-reckoning state is itself continuously
            # integrated, so reject only a physically impossible discontinuity.
            if abs(yaw_delta) > 0.75:
                raise RuntimeError("odometry yaw jumped during panorama")
            accumulated_yaw += abs(yaw_delta)
            last_yaw = current[2]
            if time.monotonic() >= next_report:
                fraction = min(1.0, accumulated_yaw / target_rotation)
                print(f"panorama rotation {100.0 * fraction:5.1f}%", flush=True)
                next_report += 2.0
            time.sleep(args.period)
    finally:
        try:
            stopped = gateway.request("POST", "/stop")
            print(f"stop latched: {stopped.get('message', stopped)}", flush=True)
        except Exception as exc:
            # The last drive lease still expires independently within 0.5 s.
            print(f"stop request failed after rotation: {exc}", flush=True)

    print(f"panorama rotation complete in {time.monotonic() - started:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
