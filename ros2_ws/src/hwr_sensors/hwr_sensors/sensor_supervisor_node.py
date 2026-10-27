"""Liveness supervisor for closed/prebuilt sensor drivers."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from hwr_sensors.sensor_liveness import (CAMERA_USB_RESET_TARGETS, process_matches,
                                         stale_targets)


class SensorSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("hwr_sensor_supervisor")
        self.declare_parameter("startup_grace", 15.0)
        self.declare_parameter("lidar_timeout", 5.0)
        self.declare_parameter("camera_timeout", 10.0)
        self.declare_parameter("recovery_cooldown", 15.0)
        now = time.monotonic()
        self.started = now
        self.last_seen = {"lidar": now, "camera": now}
        self.last_recovery = {"lidar": float("-inf"),
                              "camera": float("-inf")}
        self.create_subscription(LaserScan, "scan_raw", self._on_lidar,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "camera/color/image_raw", self._on_camera,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._check)

    def _on_lidar(self, _message: LaserScan) -> None:
        self.last_seen["lidar"] = time.monotonic()

    def _on_camera(self, _message: Image) -> None:
        self.last_seen["camera"] = time.monotonic()

    @staticmethod
    def _find_process(target: str) -> int | None:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                raw = (entry / "cmdline").read_bytes()
                cmdline = tuple(part.decode(errors="replace")
                                for part in raw.split(b"\0") if part)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if process_matches(cmdline, target):
                return int(entry.name)
        return None

    def _recover(self, target: str, now: float) -> None:
        pid = self._find_process(target)
        if pid is None:
            self.get_logger().warning(
                f"stale {target}: exact driver process not found")
        else:
            os.kill(pid, signal.SIGTERM)
            self.get_logger().error(
                f"stale {target}: terminated pid {pid} for respawn")
        if target == "camera":
            # Reset only the Aurora 930 device itself.  The Realtek devices on
            # this robot are upstream hubs shared with the base and lidar.
            if pid is not None:
                time.sleep(0.5)
            for device in CAMERA_USB_RESET_TARGETS:
                try:
                    subprocess.run(
                        ["sudo", "-n", "usbreset", device], check=True,
                        timeout=5.0, capture_output=True, text=True)
                    self.get_logger().warning(
                        f"reset stale camera USB function {device}")
                except (OSError, subprocess.SubprocessError) as exc:
                    self.get_logger().error(
                        f"camera USB reset {device} failed: {exc}")
        self.last_recovery[target] = now
        self.last_seen[target] = now

    def _check(self) -> None:
        now = time.monotonic()
        timeouts = {
            "lidar": float(self.get_parameter("lidar_timeout").value),
            "camera": float(self.get_parameter("camera_timeout").value),
        }
        stale = stale_targets(
            now, self.started, self.last_seen, timeouts,
            float(self.get_parameter("startup_grace").value))
        cooldown = float(self.get_parameter("recovery_cooldown").value)
        stale = [target for target in stale
                 if now - self.last_recovery[target] >= cooldown]
        if not stale:
            return
        for target in stale:
            self._recover(target, now)


def main() -> None:
    rclpy.init()
    node = SensorSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
