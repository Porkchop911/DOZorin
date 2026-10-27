"""Arm behavior services over the single hardware driver."""
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from hwr_msgs.msg import ServoCommand
from hwr_msgs.srv import AimCamera, ArmPose, ArmReach, Gripper
from rclpy.node import Node

from hwr_core.arm import ArmKinematics, CameraAim, joints_to_pulses


class ArmNode(Node):
    def __init__(self):
        super().__init__("hwr_arm")
        path = Path(get_package_share_directory("hwr_arm")) / "config/poses.yaml"
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.poses = {name: {int(k): int(v) for k, v in pose.items()}
                      for name, pose in config["poses"].items()}
        self.gripper = {name: int(value) for name, value in config["gripper"].items()}
        self.kinematics = ArmKinematics()
        self.aim = CameraAim()
        self.pub = self.create_publisher(ServoCommand, "servo_cmd", 10)
        self.create_service(ArmPose, "arm/pose", self._pose)
        self.create_service(ArmReach, "arm/reach", self._reach)
        self.create_service(AimCamera, "arm/aim", self._aim)
        self.create_service(Gripper, "gripper", self._gripper)

    def _send(self, positions, duration: int, response):
        duration = int(duration or 1500)
        if not 1 <= duration <= 65535:
            response.success, response.message = False, "invalid duration"
            return response
        if any(sid not in (1, 2, 3, 4, 5, 10) or not 0 <= pulse <= 1000
               for sid, pulse in positions.items()):
            response.success, response.message = False, "invalid servo target"
            return response
        msg = ServoCommand()
        msg.duration_ms = duration
        msg.ids = list(positions)
        msg.positions = [positions[sid] for sid in msg.ids]
        self.pub.publish(msg)
        response.success, response.message = True, "command accepted"
        return response

    def _pose(self, request, response):
        if request.name not in self.poses:
            response.success = False
            response.message = f"unknown pose {request.name!r}"
            return response
        return self._send(self.poses[request.name], request.duration_ms, response)

    def _reach(self, request, response):
        solution = self.kinematics.ik(request.x, request.y, request.z,
                                      request.tool_pitch, request.roll)
        if solution is None:
            response.success, response.message = False, "target unreachable or in keep-out"
            return response
        try:
            positions = joints_to_pulses(solution)
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            return response
        return self._send(positions, request.duration_ms, response)

    def _aim(self, request, response):
        try:
            pan, tilt = self.aim.pulses(request.bearing, request.elevation)
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            return response
        return self._send({1: pan, 4: tilt}, request.duration_ms, response)

    def _gripper(self, request, response):
        if request.command not in self.gripper:
            response.success = False
            response.message = f"gripper command must be one of {sorted(self.gripper)}"
            return response
        return self._send({10: self.gripper[request.command]},
                          request.duration_ms or 800, response)


def main():
    rclpy.init()
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
