"""Stop-first liveness supervisor for closed/prebuilt sensor drivers."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from hwr_sensors.sensor_liveness import (CAMERA_USB_RESET_TARGETS,
                                         process_matches, stale_targets)


class SensorSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("hwr_sensor_supervisor")
        self.declare_parameter("startup_grace", 15.0)
        self.declare_parameter("lidar_timeout", 2.0)
        self.declare_parameter("camera_timeout", 3.0)
        self.declare_parameter("recovery_cooldown", 15.0)
        now = time.monotonic()
        self.started = now
        self.last_seen = {"lidar": now, "camera": now}
        self.last_recovery = {"lidar": float("-inf"),
                              "camera": float("-inf")}
        self.estop_latched = None
        self.estop_pending = False
        self.estop_client = self.create_client(SetBool, "estop")
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "estop_state", self._on_estop,
                                 state_qos)
        self.create_subscription(LaserScan, "scan_raw", self._on_lidar,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "camera/color/image_raw", self._on_camera,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._check)

    def _on_estop(self, message: Bool) -> None:
        self.estop_latched = bool(message.data)

    def _on_lidar(self, _message: LaserScan) -> None:
        self.last_seen["lidar"] = time.monotonic()

    def _on_camera(self, _message: Image) -> None:
        self.last_seen["camera"] = time.monotonic()

    def _request_stop(self) -> None:
        if self.estop_pending or not self.estop_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = True
        self.estop_pending = True
        future = self.estop_client.call_async(request)
        future.add_done_callback(self._stop_complete)

    def _stop_complete(self, future) -> None:
        self.estop_pending = False
        try:
            response = future.result()
            if response is None or not response.success:
                self.get_logger().error("sensor recovery could not latch estop")
        except Exception as exc:
            self.get_logger().error(f"sensor recovery estop failed: {exc}")

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
                f"stale {target}: estop confirmed; terminated pid {pid} for respawn")
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
        if self.estop_latched is not True:
            self.get_logger().warning(
                f"stale sensors {stale}; requesting estop before recovery")
            self._request_stop()
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
