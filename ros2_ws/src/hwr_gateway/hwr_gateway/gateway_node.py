"""FastAPI/WS command surface bridged to bounded ROS interfaces."""
from __future__ import annotations

import asyncio
import hmac
import json
import math
import os
import random
import re
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from fastapi import Depends, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from hwr_msgs.action import NavigateToPose
from hwr_msgs.msg import LocalizationHealth
from hwr_msgs.srv import AimCamera, ArmPose, ArmReach, Gripper
from hwr_core.arm import ArmKinematics, CameraAim, pulses_to_joints
from hwr_core.arm.manipulation import follow_step, locate_cube, plan_grasp
from hwr_core.control.closed_loop import (TurnTracker, approach_command, approach_speed,
                                          pick_target, progress_along, wrap)
from hwr_core.navigation import (directional_escape_profile,
                                 evaluate_object_reach,
                                 normalize_place_definition,
                                 rank_goal_candidates_by_clearance,
                                 reachable_goal_candidates)
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, JointState
from std_msgs.msg import Bool, String, UInt16
from std_srvs.srv import SetBool, Trigger

from .voice import Voice


class DriveRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    duration: float = 0.25


class GotoRequest(BaseModel):
    x: float | None = None
    y: float | None = None
    yaw: float = 0.0
    place: str | None = None


class PoseRequest(BaseModel):
    name: str
    duration_ms: int = 1500


class ReachRequest(BaseModel):
    x: float
    y: float
    z: float
    tool_pitch: float = 0.0
    roll: float = 0.0
    duration_ms: int = 1500


class AimRequest(BaseModel):
    bearing: float
    elevation: float = 0.0
    duration_ms: int = 1000


class GripperRequest(BaseModel):
    command: str
    duration_ms: int = 800


class SayRequest(BaseModel):
    text: str


class BeepRequest(BaseModel):
    frequency: int = 1500


class TurnRequest(BaseModel):
    degrees: float


class ModeRequest(BaseModel):
    mode: str
    map: str


MAP_NAME = re.compile(r"[a-z0-9_]{1,40}")
HWR_ROOT = Path.home() / "hwr"


class ApproachRequest(BaseModel):
    target: str = "person"
    stop_at_m: float = 0.8
    timeout_s: float = 30.0


class FindRequest(BaseModel):
    view: str = "look_down"    # "floor" sees ~1 m out (coarser): for searching, not grasping
    level: int = 0             # locate on the mid-plane of a cube with this many cubes under it


class PickRequest(BaseModel):
    color: str
    level: int = 0             # cubes under it (0 = on the floor, 2 = top of a 3-stack)


class PlaceRequest(BaseModel):
    color: str | None = None   # colour being placed (for the after-release check)
    pitch_deg: float | None = None  # pick pitch: placing at the same pitch sets the cube down level
    on: str | None = None      # colour of the cube (stack top) to place on
    level: int = 1             # stack height under the held cube (1 = on one cube)
    x: float | None = None     # or a floor point in base_link
    y: float | None = None


class FollowRequest(BaseModel):
    target: str = "person"     # YOLO class or "<color>_cube"
    seconds: float = 60.0
    distance_m: float = 0.8


class MoveRequest(BaseModel):
    meters: float
    direction: str = "forward"


# Body-frame heading per move direction; no "back": the base clamps vx >= 0 (no-reverse envelope).
MOVE_DIRECTIONS = {"forward": 0.0, "left": math.pi / 2, "right": -math.pi / 2}


def _model_dict(model) -> dict:
    return model.model_dump() if hasattr(model, "model_dump") else model.dict()


class GatewayNode(Node):
    def __init__(self):
        super().__init__("hwr_gateway")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("camera_pose_tolerance_rad", 0.10)
        self.declare_parameter("joint_state_timeout", 1.0)
        # Closed-loop /motion/* tuning (ros2 param set /hwr_gateway ... at runtime).
        self.declare_parameter("turn_max_wz", 0.5)
        self.declare_parameter("turn_min_wz", 0.15)
        self.declare_parameter("turn_gain", 1.5)
        self.declare_parameter("turn_tolerance_deg", 4.0)
        self.declare_parameter("move_max_v", 0.15)
        self.declare_parameter("move_min_v", 0.05)
        self.declare_parameter("move_gain", 1.0)
        self.declare_parameter("move_tolerance_m", 0.03)
        self.declare_parameter("move_clearance_m", 0.15)
        self.motion_lock = threading.Lock()
        self.motion_epoch = 0
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.voice = Voice()
        self.voltage = None
        self.battery_stamp = 0.0
        self.pose = None
        self.pose_stamp = 0.0
        self.odom_pose = None
        self.odom_stamp = 0.0
        self.odom_motion_pose = [0.0, 0.0, 0.0]
        self.last_odom_motion_stamp = None
        self.loc = {"healthy": False, "state": "unavailable", "score": 0.0}
        self.loc_stamp = 0.0
        self.map_grid = None
        self.costmap = None
        self.costmap_meta = None
        self.jpeg = None
        self.vision = None
        self.estop_latched = None
        self.last_jpeg = 0.0
        self.joint_positions = {}
        self.joint_stamp = 0.0
        self.drive_command = (0.0, 0.0, 0.0)
        self.drive_until = 0.0
        self.drive_was_active = False
        self.current_goal = None
        self.goal_handle = None
        self.token = self._load_token()
        audit = Path(os.environ.get("HWR_AUDIT_LOG",
                                   str(Path.home() / "hwr/state/gateway_audit.jsonl")))
        audit.parent.mkdir(parents=True, exist_ok=True)
        self.audit_path = audit
        places_path = Path(get_package_share_directory("hwr_nav")) / "config/places.yaml"
        self.places = yaml.safe_load(places_path.read_text(encoding="utf-8"))["places"]
        arm_path = Path(get_package_share_directory("hwr_arm")) / "config/poses.yaml"
        arm_config = yaml.safe_load(arm_path.read_text(encoding="utf-8"))
        forward_pulses = arm_config["poses"]["forward"]
        radians_per_pulse = math.radians(240.0 / 1000.0)
        self.arm_config = arm_config
        self.kin = ArmKinematics()
        self.camera_forward_target = {
            f"joint{int(sid)}": (int(pulse) - 500) * radians_per_pulse
            for sid, pulse in forward_pulses.items() if int(sid) in range(1, 6)
        }

        self.drive_pub = self.create_publisher(Twist, "cmd_vel_agent", 10)
        self.beep_pub = self.create_publisher(UInt16, "beep", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                callback_group=self.group)
        self.estop = self.create_client(SetBool, "estop", callback_group=self.group)
        self.explore = self.create_client(SetBool, "explore", callback_group=self.group)
        self.arm_pose = self.create_client(ArmPose, "arm/pose", callback_group=self.group)
        self.arm_reach = self.create_client(ArmReach, "arm/reach", callback_group=self.group)
        self.arm_aim = self.create_client(AimCamera, "arm/aim", callback_group=self.group)
        self.gripper = self.create_client(Gripper, "gripper", callback_group=self.group)
        self.map_save = self.create_client(Trigger, "map_save", callback_group=self.group)
        self.relocalize = self.create_client(Trigger, "global_relocalize",
                                             callback_group=self.group)
        self.create_subscription(BatteryState, "battery", self._battery, 10,
                                 callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._pose, 10,
                                 callback_group=self.group)
        self.create_subscription(Odometry, "odom", self._odom, 10,
                                 callback_group=self.group)
        self.create_subscription(LocalizationHealth, "localization_health", self._health,
                                 10, callback_group=self.group)
        self.create_subscription(OccupancyGrid, "costmap", self._costmap, 1,
                                 callback_group=self.group)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self._map, map_qos,
                                 callback_group=self.group)
        self.create_subscription(String, "vision/objects", self._vision, 1)
        self.create_subscription(Bool, "estop_state", self._estop_state, QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, "camera/color/image_raw", self._image, 1,
                                 callback_group=self.group)
        self.create_subscription(JointState, "joint_states", self._joint_state, 10,
                                 callback_group=self.group)
        self.create_timer(0.05, self._drive_tick, callback_group=self.group)
        self.app = self._make_app()
        config = uvicorn.Config(self.app,
                                host=str(self.get_parameter("bind").value),
                                port=int(self.get_parameter("port").value),
                                log_level="info")
        self.server = uvicorn.Server(config)
        self.server_thread = threading.Thread(target=self.server.run, daemon=True)
        self.server_thread.start()

    @staticmethod
    def _load_token() -> str:
        token = os.environ.get("HWR_GATEWAY_TOKEN", "").strip()
        token_path = Path.home() / ".config/hwr/token"
        if not token and token_path.exists():
            token = token_path.read_text(encoding="utf-8").strip()
        if len(token) < 24:
            raise RuntimeError("set HWR_GATEWAY_TOKEN or ~/.config/hwr/token (>=24 chars)")
        return token

    def _battery(self, msg: BatteryState) -> None:
        if msg.present and math.isfinite(msg.voltage):
            with self.lock:
                self.voltage, self.battery_stamp = float(msg.voltage), time.monotonic()

    def _pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        with self.lock:
            self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            self.pose_stamp = time.monotonic()

    def _odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        now = time.monotonic()
        with self.lock:
            self.odom_pose = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw]
            if self.last_odom_motion_stamp is not None:
                dt = now - self.last_odom_motion_stamp
                twist = msg.twist.twist
                values = (twist.linear.x, twist.linear.y, twist.angular.z)
                if 0.0 < dt <= 0.5 and all(math.isfinite(v) for v in values):
                    dx, dy, dyaw = (float(v) * dt for v in values)
                    x, y, motion_yaw = self.odom_motion_pose
                    midpoint = motion_yaw + 0.5 * dyaw
                    c, s = math.cos(midpoint), math.sin(midpoint)
                    self.odom_motion_pose = [
                        x + c * dx - s * dy,
                        y + s * dx + c * dy,
                        math.atan2(math.sin(motion_yaw + dyaw),
                                   math.cos(motion_yaw + dyaw)),
                    ]
            self.last_odom_motion_stamp = now
            self.odom_stamp = now

    def _health(self, msg: LocalizationHealth) -> None:
        with self.lock:
            self.loc = {"healthy": bool(msg.healthy), "state": msg.state,
                        "score": float(msg.mean_scan_score)}
            self.loc_stamp = time.monotonic()

    def _localization_state_locked(self, now: float) -> dict:
        loc = dict(self.loc)
        fresh = (now - self.pose_stamp <= 1.0
                 and now - self.loc_stamp <= 1.0)
        loc["fresh"] = fresh
        if not fresh:
            loc["healthy"] = False
            loc["state"] = "stale"
        return loc

    def _costmap(self, msg: OccupancyGrid) -> None:
        if not msg.data:
            return
        with self.lock:
            self.costmap = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)
            self.costmap_meta = (msg.info.resolution, msg.info.origin.position.x,
                                 msg.info.origin.position.y)

    def _map(self, msg: OccupancyGrid) -> None:
        if not msg.data:
            return
        with self.lock:
            self.map_grid = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)

    def _estop_state(self, msg: Bool) -> None:
        with self.lock:
            self.estop_latched = bool(msg.data)

    def _vision(self, msg: String) -> None:
        with self.lock:
            self.vision = json.loads(msg.data)

    def _image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_jpeg < 0.5 or msg.encoding not in ("rgb8", "bgr8", "mono8"):
            return
        channels = 1 if msg.encoding == "mono8" else 3
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = raw[:, :msg.width * channels].reshape(msg.height, msg.width,
                                                       channels) if channels == 3 else raw[:, :msg.width]
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if ok:
            with self.lock:
                self.jpeg, self.last_jpeg = encoded.tobytes(), now

    def _joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        positions = {name: float(position)
                     for name, position in zip(msg.name, msg.position)
                     if math.isfinite(position)}
        if not positions:
            return
        with self.lock:
            self.joint_positions = positions
            self.joint_stamp = time.monotonic()

    def _camera_forward_locked(self, now: float) -> bool:
        timeout = float(self.get_parameter("joint_state_timeout").value)
        tolerance = float(self.get_parameter("camera_pose_tolerance_rad").value)
        if now - self.joint_stamp > timeout:
            return False
        return all(
            name in self.joint_positions
            and abs(self.joint_positions[name] - target) <= tolerance
            for name, target in self.camera_forward_target.items()
        )

    def _drive_tick(self) -> None:
        with self.lock:
            active = time.monotonic() < self.drive_until
            command = self.drive_command if active else (0.0, 0.0, 0.0)
            publish = active or self.drive_was_active
            self.drive_was_active = active
            if not active:
                self.drive_command = command
        if not publish:
            return
        msg = Twist()
        # ROS 2 generated Python setters require ``float`` exactly.  Keep this
        # boundary defensive so a zero tuple can never terminate the gateway.
        msg.linear.x = float(command[0])
        msg.linear.y = float(command[1])
        msg.angular.z = float(command[2])
        self.drive_pub.publish(msg)

    def status(self) -> dict:
        with self.lock:
            now = time.monotonic()
            return {"battery_voltage": self.voltage,
                    "battery_fresh": now - self.battery_stamp <= 3.0,
                    "pose": self.pose,
                    "localization": self._localization_state_locked(now),
                    "current_goal": self.current_goal,
                    "drive_active": now < self.drive_until}

    def navigation_context(self) -> dict:
        """Battery-free state and hard-footprint rays for semantic advising."""
        with self.lock:
            now = time.monotonic()
            pose = None if self.pose is None else list(self.pose)
            odom_pose = None if self.odom_pose is None else list(self.odom_pose)
            odom_motion_pose = list(self.odom_motion_pose)
            odom_fresh = now - self.odom_stamp <= 0.5
            loc = self._localization_state_locked(now)
            goal = None if self.current_goal is None else dict(self.current_goal)
            drive_active = now < self.drive_until
            camera_fresh = now - self.last_jpeg <= 2.0
            camera_forward = self._camera_forward_locked(now)
            grid = None if self.costmap is None else self.costmap.copy()
            meta = self.costmap_meta
        clearances = {}
        escape_profiles = {}
        trapped_start = False
        if pose is not None and grid is not None and meta is not None:
            resolution, ox, oy = meta
            distances = np.arange(0.0, 1.5 + resolution / 2.0, resolution)
            for name, relative_degrees in {
                    "forward": 0.0, "front_left": 45.0, "left": 90.0,
                    "front_right": -45.0, "right": -90.0}.items():
                heading = pose[2] + math.radians(relative_degrees)
                xs = pose[0] + distances * math.cos(heading)
                ys = pose[1] + distances * math.sin(heading)
                ix = np.floor((xs - ox) / resolution).astype(int)
                iy = np.floor((ys - oy) / resolution).astype(int)
                inside = ((ix >= 0) & (iy >= 0) & (ix < grid.shape[1])
                          & (iy < grid.shape[0]))
                cx = ix.clip(0, grid.shape[1] - 1)
                cy = iy.clip(0, grid.shape[0] - 1)
                # OccupancyGrid scales the internal 0..255 costmap to 0..100.
                blocked = (~inside) | (grid[cy, cx] >= 79)
                first = np.flatnonzero(blocked)
                clearances[name] = (float(distances[first[0]])
                                    if len(first) else None)
                escape_profiles[name] = directional_escape_profile(
                    blocked, distances)
            trapped_start = bool(escape_profiles.get(
                "forward", {}).get("trapped"))
        return {"pose": pose,
                "odometry": {"pose": odom_pose, "fresh": odom_fresh},
                "dead_reckoning": {"pose": odom_motion_pose,
                                    "fresh": odom_fresh},
                "localization": loc, "current_goal": goal,
                "drive_active": drive_active, "camera_fresh": camera_fresh,
                "camera_forward": camera_forward,
                "hard_clearance_m": clearances,
                "trapped_start": trapped_start,
                "escape_profiles": escape_profiles,
                "clearance_limit_m": 1.5}

    def _audit(self, action: str, outcome: str, detail=None) -> None:
        record = {"time": time.time(), "action": action, "outcome": outcome,
                  "detail": detail or {}}
        with self.audit_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")

    def _call(self, client, request, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=0.2):
            raise HTTPException(503, "ROS service unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise HTTPException(504, "ROS service timed out")
        response = future.result()
        if response is None:
            raise HTTPException(500, "ROS service failed")
        return response

    def _goal_is_free(self, x: float, y: float) -> bool:
        with self.lock:
            if self.costmap is None or self.costmap_meta is None:
                return False
            resolution, ox, oy = self.costmap_meta
            cx, cy = math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)
            return (0 <= cy < self.costmap.shape[0] and 0 <= cx < self.costmap.shape[1]
                    and self.costmap[cy, cx] < 65)

    def _send_goal(self, x: float, y: float, yaw: float,
                   metadata: dict | None = None) -> None:
        request = NavigateToPose.Goal()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x, request.target.pose.position.y = x, y
        request.target.pose.orientation.z = math.sin(yaw / 2.0)
        request.target.pose.orientation.w = math.cos(yaw / 2.0)
        with self.lock:
            self.current_goal = {"x": x, "y": y, "yaw": yaw, "state": "sending"}
            if metadata:
                self.current_goal.update(metadata)
        future = self.nav.send_goal_async(request)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
            with self.lock:
                self.goal_handle = handle if handle.accepted else None
                self.current_goal["state"] = "active" if handle.accepted else "rejected"
            if handle.accepted:
                handle.get_result_async().add_done_callback(self._goal_result)
        except Exception as exc:
            with self.lock:
                self.current_goal = {"state": "error", "message": str(exc)}

    def _goal_result(self, future) -> None:
        try:
            result = future.result().result
            state = "succeeded" if result.success else "failed"
            message = result.message
        except Exception as exc:
            state, message = "error", str(exc)
        with self.lock:
            completed = dict(self.current_goal or {})
            pose = None if self.pose is None else list(self.pose)
            localization_healthy = bool(self.loc.get("healthy", False))
        if state == "succeeded" and completed.get("place_kind") == "object":
            if not localization_healthy or pose is None:
                state = "failed"
                message = "navigation pose reached but final localization is unavailable"
                completed["object_reached"] = False
            else:
                try:
                    reach = evaluate_object_reach(
                        pose, completed["object_x"], completed["object_y"],
                        completed["max_object_distance_m"],
                        object_end_x=completed.get("object_end_x"),
                        object_end_y=completed.get("object_end_y"))
                    completed.update(reach)
                    if not reach["object_reached"]:
                        state = "failed"
                        message = (
                            "navigation pose reached but actual object remains "
                            f"{reach['object_distance_m']:.3f} m away "
                            f"(limit {reach['max_object_distance_m']:.2f} m)")
                except (KeyError, TypeError, ValueError) as exc:
                    state = "failed"
                    message = f"object reach verification failed: {exc}"
                    completed["object_reached"] = False
        completed.update({"state": state, "message": message})
        with self.lock:
            self.current_goal = completed
            self.goal_handle = None
        self._audit("goto_result", state, completed)

    def _motion_start(self) -> list:
        with self.lock:
            if self.estop_latched:
                raise HTTPException(409, "estop latched; reset before drive")
            if self.odom_pose is None or time.monotonic() - self.odom_stamp > 0.5:
                raise HTTPException(503, "odometry stale")
            return list(self.odom_pose)

    def _first_blocked(self, x: float, y: float, heading: float, length: float):
        """(distance along path, cost 0..100 or None if off-map) of the first non-free cell, else None."""
        steps = max(1, math.ceil(length / 0.05))
        with self.lock:
            if self.costmap is None or self.costmap_meta is None:
                return (0.0, None)
            grid, (resolution, ox, oy) = self.costmap, self.costmap_meta
        for i in range(1, steps + 1):
            d = length * i / steps
            cx = math.floor((x + math.cos(heading) * d - ox) / resolution)
            cy = math.floor((y + math.sin(heading) * d - oy) / resolution)
            if not (0 <= cy < grid.shape[0] and 0 <= cx < grid.shape[1]):
                return (round(d, 2), None)
            # 79 = the "hard" level hard_clearance_m uses; 65 also blocks the soft inflation band
            # a robot parked near a wall sits in, which refused clear lidar paths live.
            if grid[cy, cx] >= 79:
                return (round(d, 2), int(grid[cy, cx]))
        return None

    def _path_is_free(self, x: float, y: float, heading: float, length: float) -> bool:
        return self._first_blocked(x, y, heading, length) is None

    def _motion_loop(self, step, timeout: float) -> str:
        """Command step(odom_pose) -> (vx, vy, wz) every 50 ms until it returns None.

        Aborts on /stop, estop, stale odometry or timeout; always ends with a zero command.
        A string from step is an abort reason."""
        if not self.motion_lock.acquire(blocking=False):
            raise HTTPException(409, "another motion is running")
        outcome = "done"
        try:
            with self.lock:
                epoch = self.motion_epoch
            deadline = time.monotonic() + timeout
            while True:
                time.sleep(0.05)
                now = time.monotonic()
                with self.lock:
                    if self.motion_epoch != epoch:
                        outcome = "stopped"
                    elif self.estop_latched:
                        outcome = "estop"
                    elif self.odom_pose is None or now - self.odom_stamp > 0.5:
                        outcome = "odometry stale"
                    odom = None if self.odom_pose is None else list(self.odom_pose)
                if outcome != "done":
                    break
                command = step(odom)
                if command is None:
                    break
                if isinstance(command, str):
                    outcome = command
                    break
                if now > deadline:
                    outcome = "timeout"
                    break
                with self.lock:
                    if self.motion_epoch != epoch:
                        outcome = "stopped"
                        break
                    self.drive_command = tuple(float(v) for v in command)
                    self.drive_until = now + 0.3
        finally:
            with self.lock:
                self.drive_command, self.drive_until = (0.0, 0.0, 0.0), 0.0
            self.motion_lock.release()
        return outcome

    def _make_app(self) -> FastAPI:
        app = FastAPI(title="HWR Gateway", version="0.1.0")
        security = HTTPBearer(auto_error=False)

        def auth(credentials: HTTPAuthorizationCredentials = Depends(security)):
            if credentials is None or not hmac.compare_digest(credentials.credentials,
                                                               self.token):
                raise HTTPException(401, "invalid bearer token")

        @app.get("/status", dependencies=[Depends(auth)])
        def status():
            return self.status()

        @app.get("/navigation/context", dependencies=[Depends(auth)])
        def navigation_context():
            return self.navigation_context()

        @app.post("/drive", dependencies=[Depends(auth)])
        def drive(body: DriveRequest):
            values = (body.vx, body.vy, body.wz, body.duration)
            if not all(math.isfinite(v) for v in values):
                raise HTTPException(422, "drive values must be finite")
            if not (0.0 <= body.vx <= 0.2 and abs(body.vy) <= 0.2
                    and abs(body.wz) <= 0.5 and 0.02 <= body.duration <= 0.5):
                raise HTTPException(422, "drive outside bounded no-reverse envelope")
            with self.lock:
                if self.estop_latched:
                    raise HTTPException(409, "estop latched; reset before drive")
                self.drive_command = (body.vx, body.vy, body.wz)
                self.drive_until = time.monotonic() + body.duration
            self._audit("drive", "accepted", _model_dict(body))
            return {"accepted": True, "expires_in": body.duration}

        @app.post("/motion/turn", dependencies=[Depends(auth)])
        def motion_turn(body: TurnRequest):
            if not math.isfinite(body.degrees) or not 0.0 < abs(body.degrees) <= 360.0:
                raise HTTPException(422, "degrees must be in -360..360 and nonzero")
            param = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            max_wz, min_wz = param("turn_max_wz"), param("turn_min_wz")
            start = self._motion_start()
            tracker = TurnTracker(math.radians(body.degrees), start[2],
                                  math.radians(param("turn_tolerance_deg")))

            def step(odom):
                remaining = tracker.update(odom[2])
                if tracker.done:
                    return None
                return (0.0, 0.0, approach_speed(remaining, max_wz, min_wz, param("turn_gain")))

            outcome = self._motion_loop(step, 3.0 * abs(tracker.target) / max_wz + 3.0)
            result = {"outcome": outcome, "requested_deg": body.degrees,
                      "turned_deg": round(math.degrees(tracker.turned), 1),
                      "pose": self.pose}
            self._audit("motion_turn", outcome, result)
            return result

        @app.post("/motion/move", dependencies=[Depends(auth)])
        def motion_move(body: MoveRequest):
            if body.direction not in MOVE_DIRECTIONS:
                raise HTTPException(422, f"direction must be one of {sorted(MOVE_DIRECTIONS)}")
            if not math.isfinite(body.meters) or not 0.0 < body.meters <= 2.0:
                raise HTTPException(422, "meters must be in 0..2")
            param = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            max_v, min_v = param("move_max_v"), param("move_min_v")
            clearance = param("move_clearance_m")
            body_heading = MOVE_DIRECTIONS[body.direction]
            with self.lock:
                camera_forward = self._camera_forward_locked(time.monotonic())
                pose = None if self.pose is None else list(self.pose)
                fresh = self._localization_state_locked(time.monotonic())["fresh"]
            if not camera_forward:
                raise HTTPException(409, "translation requires verified forward camera pose; call arm_pose('forward') first")
            if pose is None or not fresh:
                raise HTTPException(409, "map pose not fresh; relocalize first")
            if hit := self._first_blocked(pose[0], pose[1], pose[2] + body_heading,
                                          body.meters + clearance):
                raise HTTPException(409, f"path blocked in costmap at {hit[0]} m (cost {hit[1]})")
            start = self._motion_start()
            travelled = [0.0]

            def step(odom):
                travelled[0] = progress_along(start, odom[:2], body_heading)
                remaining = body.meters - travelled[0]
                if remaining <= param("move_tolerance_m"):
                    return None
                with self.lock:
                    now_pose = None if self.pose is None else list(self.pose)
                # shortcut: blocking check is the costmap ahead of the map pose, not raw scan; add a scan gate if it misses fast obstacles.
                if now_pose is not None and (hit := self._first_blocked(
                        now_pose[0], now_pose[1], now_pose[2] + body_heading,
                        min(remaining, 0.3) + clearance)):
                    return f"blocked at {hit[0]} m ahead (cost {hit[1]})"
                speed = approach_speed(remaining, max_v, min_v, param("move_gain"))
                # shortcut: no heading hold; add a wz correction if strafes drift in yaw.
                return (speed * math.cos(body_heading), speed * math.sin(body_heading), 0.0)

            outcome = self._motion_loop(step, 3.0 * body.meters / max_v + 3.0)
            result = {"outcome": outcome, "requested_m": body.meters, "direction": body.direction,
                      "travelled_m": round(travelled[0], 3), "pose": self.pose}
            self._audit("motion_move", outcome, result)
            return result

        @app.post("/motion/approach", dependencies=[Depends(auth)])
        def motion_approach(body: ApproachRequest):
            if not 0.3 <= body.stop_at_m <= 3.0 or not 1.0 <= body.timeout_s <= 120.0:
                raise HTTPException(422, "stop_at_m must be 0.3..3, timeout_s 1..120")
            param = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            clearance = param("move_clearance_m")
            with self.lock:
                camera_forward = self._camera_forward_locked(time.monotonic())
            if not camera_forward:
                raise HTTPException(409, "approach needs the camera forward (bearings are camera-relative); call arm_pose('forward') first")
            self._motion_start()
            state = {"track": None, "last_seen": time.monotonic(), "searched": 0.0,
                     "last_yaw": None, "no_range_since": None, "seen": None}

            def step(odom):
                now = time.monotonic()
                with self.lock:
                    frame = self.vision
                    pose = None if self.pose is None else list(self.pose)
                if frame is None or time.time() - frame["stamp"] > 2.0:
                    return "vision stale"
                item = pick_target(frame["objects"], body.target, state["track"])
                if item is None:
                    if state["seen"] is not None and now - state["last_seen"] > 3.0:
                        return "target lost"
                    if state["seen"] is None:
                        # Not seen yet: search by turning in place, one full circle at most.
                        if state["last_yaw"] is not None:
                            state["searched"] += abs(wrap(odom[2] - state["last_yaw"]))
                        state["last_yaw"] = odom[2]
                        if state["searched"] >= 2 * math.pi:
                            return "target not found"
                        return (0.0, 0.0, param("turn_max_wz") * 0.8)
                    return (0.0, 0.0, 0.0)  # briefly lost: hold still and wait for re-detection
                state["track"], state["last_seen"], state["seen"] = item.get("id"), now, item
                range_m = item.get("range_m")
                if range_m is None:
                    state["no_range_since"] = state["no_range_since"] or now
                    if now - state["no_range_since"] > 3.0:
                        return "no depth on target"
                else:
                    state["no_range_since"] = None
                command = approach_command(item["bearing_deg"], range_m, body.stop_at_m,
                                           param("turn_max_wz"), param("turn_min_wz"),
                                           param("move_max_v"), param("move_min_v"))
                if command is None:
                    return None
                vx, wz = command
                if vx > 0.0 and pose is not None and (hit := self._first_blocked(
                        pose[0], pose[1], pose[2], min(range_m - body.stop_at_m, 0.3) + clearance)):
                    # Depth range can see through open objects (chair frames); the costmap is the arbiter.
                    return f"blocked at {hit[0]} m ahead (cost {hit[1]}) before reaching stop_at_m"
                return (vx, 0.0, wz)

            outcome = self._motion_loop(step, body.timeout_s)
            seen = state["seen"] or {}
            result = {"outcome": outcome, "target": body.target, "track_id": seen.get("id"),
                      "last_range_m": seen.get("range_m"), "last_bearing_deg": seen.get("bearing_deg"),
                      "pose": self.pose}
            self._audit("motion_approach", outcome, result)
            return result

        def current_mode() -> dict:
            state = HWR_ROOT / "state"
            read = lambda name: (state / name).read_text().strip() if (state / name).exists() else None  # noqa: E731
            map_path = read("mode_session.map")
            return {"mode": read("mode_session.name"),
                    "map": Path(map_path).stem if map_path else None}

        @app.get("/maps", dependencies=[Depends(auth)])
        def maps():
            names = sorted(p.stem for p in (HWR_ROOT / "maps").glob("*.yaml")
                           if not p.name.endswith(".state.yaml"))
            return {"maps": names, **current_mode()}

        @app.post("/mode", dependencies=[Depends(auth)])
        def switch_mode(body: ModeRequest):
            # Trust boundary: the name reaches a shell script, so only [a-z0-9_] and no shell.
            if body.mode not in ("exploration", "navigation"):
                raise HTTPException(422, "mode must be exploration or navigation")
            if not MAP_NAME.fullmatch(body.map):
                raise HTTPException(422, "map name must be 1-40 chars of a-z, 0-9, _")
            path = HWR_ROOT / "maps" / body.map
            if body.mode == "navigation":
                path = path.with_suffix(".yaml")
                if not path.exists():
                    raise HTTPException(404, f"no saved map {body.map!r}")
            log = (HWR_ROOT / "state" / "mode_switch.log").open("a")
            # Own session: mode.sh restart kills this gateway's process group, the switcher must survive it.
            subprocess.Popen(["zsh", "-c", 'sleep 1; exec "$0" restart "$1" "$2"',
                              str(HWR_ROOT / "deploy/mode.sh"), body.mode, str(path)],
                             stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                             start_new_session=True)
            self._audit("mode_switch", "accepted", {"mode": body.mode, "map": body.map})
            return {"switching": True, "mode": body.mode, "map": body.map,
                    "note": "stack restarts; gateway is back in ~30-60 s with the estop latched (startup)"}

        @app.post("/map/save", dependencies=[Depends(auth)])
        def map_save():
            response = self._call(self.map_save, Trigger.Request(), timeout=15.0)
            self._audit("map_save", "accepted" if response.success else "rejected",
                        {"message": response.message})
            return {"success": response.success, "message": response.message}

        @app.get("/explore/status", dependencies=[Depends(auth)])
        def explore_status():
            journal = HWR_ROOT / "state" / "exploration.jsonl"
            last_start = last_finish = None
            if journal.exists():
                with journal.open("rb") as stream:
                    stream.seek(max(0, journal.stat().st_size - 200_000))
                    for line in stream.read().decode("utf-8", "replace").splitlines():
                        try:
                            event = json.loads(line)
                        except ValueError:
                            continue
                        if event.get("event") == "session_started":
                            last_start, last_finish = event, None
                        elif event.get("event") == "session_finished":
                            last_finish = event
            return {"running": last_start is not None and last_finish is None,
                    "started": last_start, "finished": last_finish}

        @app.post("/stop", dependencies=[Depends(auth)])
        def stop():
            with self.lock:
                self.motion_epoch += 1  # aborts a running /motion/* loop on its next tick
                self.drive_command, self.drive_until = (0.0, 0.0, 0.0), 0.0
                handle = self.goal_handle
            self.drive_pub.publish(Twist())
            if handle is not None:
                handle.cancel_goal_async()
            request = SetBool.Request()
            request.data = True
            response = self._call(self.estop, request)
            self._audit("stop", "accepted", {"message": response.message})
            return {"success": response.success, "message": response.message}

        @app.post("/estop/reset", dependencies=[Depends(auth)])
        def reset_estop():
            request = SetBool.Request()
            request.data = False
            response = self._call(self.estop, request)
            self._audit("estop_reset", "accepted" if response.success else "rejected",
                        {"message": response.message})
            return {"success": response.success, "message": response.message}

        @app.post("/localization/relocalize", dependencies=[Depends(auth)])
        def relocalize():
            # Global search over the whole map; no motion, so no estop check.
            response = self._call(self.relocalize, Trigger.Request(), timeout=20.0)
            self._audit("relocalize", "accepted" if response.success else "rejected",
                        {"message": response.message})
            return {"success": response.success, "message": response.message}

        @app.post("/beep", dependencies=[Depends(auth)])
        def beep(body: BeepRequest):
            if not 20 <= body.frequency <= 20000:
                raise HTTPException(422, "frequency must be in 20..20000 Hz")
            msg = UInt16()
            msg.data = body.frequency
            self.beep_pub.publish(msg)
            self._audit("beep", "accepted", {"frequency": body.frequency})
            return {"accepted": True}

        @app.post("/say", dependencies=[Depends(auth)])
        def say(body: SayRequest):
            text = body.text.strip()
            if not 0 < len(text) <= 400:
                raise HTTPException(422, "text must be 1..400 characters")
            if not self.voice.say(text):
                raise HTTPException(409, "speaker busy: 3 utterances already queued")
            self._audit("say", "accepted", {"text": text})
            return {"queued": True}

        @app.post("/goto", dependencies=[Depends(auth)])
        def goto(body: GotoRequest):
            with self.lock:
                now = time.monotonic()
                localization = self._localization_state_locked(now)
                healthy = localization["healthy"]
                camera_forward = self._camera_forward_locked(now)
            if not healthy:
                raise HTTPException(409, "localization is unhealthy")
            goal_metadata = None
            if body.place:
                place = self.places.get(body.place)
                if place is None:
                    raise HTTPException(404, "unknown place")
                if not place.get("verified", False):
                    raise HTTPException(409, "place is not verified on the current HWR map")
                try:
                    place = normalize_place_definition(body.place, place)
                except ValueError as exc:
                    raise HTTPException(409, f"invalid place definition: {exc}") from exc
                x, y, yaw = place["x"], place["y"], math.radians(place["yaw_deg"])
                goal_metadata = {"place": body.place, "place_kind": place["kind"]}
                if place["kind"] == "object":
                    goal_metadata.update({
                        "object_x": place["object_x"],
                        "object_y": place["object_y"],
                        "object_geometry": place["object_geometry"],
                        "max_object_distance_m": place["max_object_distance_m"],
                        "planned_object_distance_m":
                            place["planned_object_distance_m"],
                    })
                    if place["object_geometry"] == "segment":
                        goal_metadata.update({
                            "object_end_x": place["object_end_x"],
                            "object_end_y": place["object_end_y"],
                        })
            elif body.x is not None and body.y is not None:
                x, y, yaw = float(body.x), float(body.y), float(body.yaw)
            else:
                raise HTTPException(422, "provide place or x/y")
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise HTTPException(422, "goal must be finite")
            with self.lock:
                current_pose = self.pose
            rotation_only = (current_pose is not None
                             and math.hypot(x - current_pose[0],
                                            y - current_pose[1]) <= 0.06)
            if not rotation_only and not camera_forward:
                self._audit("goto", "rejected", {"reason": "camera not forward"})
                raise HTTPException(409, "translation requires verified forward camera pose; call arm_pose('forward') first")
            if not rotation_only and not self._goal_is_free(x, y):
                raise HTTPException(409, "goal rejected by map geofence")
            if not self.nav.wait_for_server(timeout_sec=0.2):
                raise HTTPException(503, "navigation action unavailable")
            self._send_goal(x, y, yaw, goal_metadata)
            detail = {"x": x, "y": y, "yaw": yaw, **(goal_metadata or {})}
            self._audit("goto", "accepted", detail)
            return {"accepted": True, "goal": [x, y, yaw],
                    **(goal_metadata or {})}

        def explore(enabled: bool):
            if enabled:
                with self.lock:
                    camera_forward = self._camera_forward_locked(time.monotonic())
                if not camera_forward:
                    self._audit("explore", "rejected",
                                {"reason": "camera not forward"})
                    raise HTTPException(
                        409, "exploration requires verified forward camera pose; call arm_pose('forward') first")
            request = SetBool.Request()
            request.data = enabled
            response = self._call(self.explore, request)
            self._audit("explore", "accepted" if response.success else "rejected",
                        {"enabled": enabled, "message": response.message})
            return {"success": response.success, "message": response.message}

        app.post("/explore/start", dependencies=[Depends(auth)])(lambda: explore(True))
        app.post("/explore/stop", dependencies=[Depends(auth)])(lambda: explore(False))

        @app.post("/arm/pose", dependencies=[Depends(auth)])
        def arm_pose(body: PoseRequest):
            request = ArmPose.Request(name=body.name, duration_ms=body.duration_ms)
            response = self._call(self.arm_pose, request)
            self._audit("arm_pose", "accepted" if response.success else "rejected",
                        _model_dict(body))
            if not response.success:
                raise HTTPException(409, response.message)
            return {"success": response.success, "message": response.message}

        @app.post("/arm/reach", dependencies=[Depends(auth)])
        def arm_reach(body: ReachRequest):
            request = ArmReach.Request(x=body.x, y=body.y, z=body.z,
                                       tool_pitch=body.tool_pitch, roll=body.roll,
                                       duration_ms=body.duration_ms)
            response = self._call(self.arm_reach, request)
            self._audit("arm_reach", "accepted" if response.success else "rejected",
                        _model_dict(body))
            if not response.success:
                raise HTTPException(409, response.message)
            return {"success": response.success, "message": response.message}

        @app.post("/arm/aim", dependencies=[Depends(auth)])
        def arm_aim(body: AimRequest):
            request = AimCamera.Request(bearing=body.bearing, elevation=body.elevation,
                                        duration_ms=body.duration_ms)
            response = self._call(self.arm_aim, request)
            self._audit("arm_aim", "accepted" if response.success else "rejected",
                        _model_dict(body))
            if not response.success:
                raise HTTPException(409, response.message)
            return {"success": response.success, "message": response.message}

        @app.post("/gripper", dependencies=[Depends(auth)])
        def gripper(body: GripperRequest):
            request = Gripper.Request(command=body.command, duration_ms=body.duration_ms)
            response = self._call(self.gripper, request)
            self._audit("gripper", "accepted" if response.success else "rejected",
                        _model_dict(body))
            if not response.success:
                raise HTTPException(409, response.message)
            return {"success": response.success, "message": response.message}

        arm_config = self.arm_config
        size = float(arm_config["cube_size"])
        roll_sign = float(arm_config.get("roll_sign", 1.0))
        clearance = float(arm_config.get("place_clearance", 0.005))
        side_trim = float(arm_config.get("place_side_trim", 0.0))

        radial_per_deg = float(arm_config.get("place_radial_per_deg", 0.0))

        def place_plan(spot: dict, z: float, yaw: float, **window):
            """plan_grasp for a release, corrected by the floor calibration: side trim, then a
            radial pull-back that grows with the tilt from vertical (replanned once)."""
            pan = math.atan2(spot["y"], spot["x"] - self.kin.pan_x)
            x = spot["x"] - side_trim * math.sin(pan)
            y = spot["y"] + side_trim * math.cos(pan)
            plan = plan_grasp(self.kin, x, y, z, yaw, roll_sign=roll_sign, **window)
            if plan is None or radial_per_deg == 0.0:
                return plan
            back = radial_per_deg * (90.0 + math.degrees(plan[2]))
            return plan_grasp(self.kin, x - back * math.cos(pan), y - back * math.sin(pan), z, yaw,
                              roll_sign=roll_sign, **window) or plan

        def refined_plan(color: str, spot: dict, z: float, top_z: float,
                         flattest: float = -math.radians(30), steepest: float = -math.pi / 2):
            """Hover 8 cm over the look_down estimate, re-locate the cube from almost overhead
            (mostly its top face: true edges for yaw, small parallax), then plan the descent."""
            plan = plan_grasp(self.kin, spot["x"], spot["y"], z, spot["yaw"], hover=0.08,
                              roll_sign=roll_sign, flattest=flattest, steepest=steepest)
            if plan is None:
                return None, spot
            reach(plan[0], 1200, 1.6)
            blobs, k = fresh_cubes(time.time())
            joints = pulses_to_joints(arm_pulses())
            near = [s for s in (locate_cube(b, k, joints, self.kin, top_z)
                                for b in blobs if b["class"] == f"{color}_cube") if s]
            near = [s for s in near if math.hypot(s["x"] - spot["x"], s["y"] - spot["y"]) < 0.05]
            if near:
                spot = {**spot, **min(near, key=lambda s: math.hypot(s["x"] - spot["x"],
                                                                   s["y"] - spot["y"])),
                        "refined": True}
            return plan_grasp(self.kin, spot["x"], spot["y"], z, spot["yaw"],
                              roll_sign=roll_sign, flattest=flattest, steepest=steepest), spot

        def seen_near(color: str, spot: dict, plane_z: float, within: float = 0.03):
            """Offset (radial, sideways m, about the pan axis) of `color` from spot, None if not
            within. Logged per place to find systematic centring error (2026-10-17)."""
            blobs, k = fresh_cubes(time.time())
            joints = pulses_to_joints(arm_pulses())
            for blob in blobs:
                if blob["class"] == f"{color}_cube":
                    s = locate_cube(blob, k, joints, self.kin, plane_z)
                    if s and math.hypot(s["x"] - spot["x"], s["y"] - spot["y"]) < within:
                        pan = math.atan2(spot["y"], spot["x"] - self.kin.pan_x)
                        dx, dy = s["x"] - spot["x"], s["y"] - spot["y"]
                        return [round(dx * math.cos(pan) + dy * math.sin(pan), 4),
                                round(-dx * math.sin(pan) + dy * math.cos(pan), 4)]
            return None

        def arm_do(client, request, settle: float):
            response = self._call(client, request)
            if not response.success:
                raise HTTPException(409, response.message)
            time.sleep(settle)

        def reach(pulses: dict, duration_ms: int, settle: float):
            joints = pulses_to_joints(pulses)
            x, y, z, pitch = self.kin.fk(joints)
            arm_do(self.arm_reach, ArmReach.Request(x=x, y=y, z=z, tool_pitch=pitch,
                                                    roll=joints.roll, duration_ms=duration_ms),
                   settle)

        def pose(name: str, settle: float = 1.8):
            arm_do(self.arm_pose, ArmPose.Request(name=name, duration_ms=1500), settle)

        def grip(command: str):
            arm_do(self.gripper, Gripper.Request(command=command, duration_ms=500), 0.8)

        def arm_pulses() -> dict:
            with self.lock:
                if time.monotonic() - self.joint_stamp > 1.0:
                    raise HTTPException(503, "arm joint readback stale")
                return {(int(name[-1]) if name.startswith("joint") else 10):
                        round(500 + value / math.radians(0.24))
                        for name, value in self.joint_positions.items()}

        def fresh_cubes(after: float) -> tuple:
            """Cube blobs from the first vision frame newer than `after` (wall clock)."""
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                with self.lock:
                    frame = self.vision
                if frame and frame["stamp"] > after and frame.get("k"):
                    return ([o for o in frame["objects"] if o["class"].endswith("_cube")],
                            frame["k"])
                time.sleep(0.1)
            raise HTTPException(503, "vision stale")

        def find(color, plane_z: float, view: str = "look_down") -> list:
            """look_down (or floor), settle, then base-frame cubes (largest blob per colour first)."""
            pose(view, 2.0)
            blobs, k = fresh_cubes(time.time())
            joints = pulses_to_joints(arm_pulses())
            out = []
            for blob in blobs:
                name = blob["class"][:-5]
                if color and name != color:
                    continue
                spot = locate_cube(blob, k, joints, self.kin, plane_z)
                if spot:
                    out.append({"color": name, **spot, "area": blob["area"]})
            return out

        @app.post("/arm/find_cubes", dependencies=[Depends(auth)])
        def arm_find_cubes(body: FindRequest = FindRequest()):
            if body.view not in ("look_down", "floor"):
                raise HTTPException(422, "view must be look_down or floor")
            if not 0 <= body.level <= 3:
                raise HTTPException(422, "level must be 0..3")
            cubes = find(None, size * body.level + size / 2, body.view)
            self._audit("arm_find_cubes", "done", {"cubes": cubes})
            return {"cubes": cubes}

        @app.post("/arm/pick", dependencies=[Depends(auth)])
        def arm_pick(body: PickRequest):
            if body.color not in arm_config["cubes"]:
                raise HTTPException(422, f"color must be one of {sorted(arm_config['cubes'])}")
            grip("open")
            if not 0 <= body.level <= 3:
                raise HTTPException(422, "level must be 0..3")
            centre = size * body.level + size / 2
            seen = find(body.color, centre)
            if not seen:
                result = {"outcome": "not seen", "color": body.color}
            else:
                # Live: -40 missed (B2 #5); -55 picks held but the tilted hold failed every
                # later place (S2 #1, #5). Pick steep or reposition (creep) first.
                # Off a stack, grip 8 mm high: at mid-height the finger tips caught the cube below
                # and the lift left it tilted (c5 2026-10-17 unstack, next pick missed).
                grasp_z = centre + (0.008 if body.level else 0.0)
                plan, cube = refined_plan(body.color, seen[0], grasp_z, centre + size / 2,
                                          flattest=-math.radians(70))
                if plan is None:
                    result = {"outcome": "out of reach", "cube": cube}
                else:
                    hover, grasp, pitch = plan
                    reach(hover, 1200, 1.4)
                    reach(grasp, 800, 1.0)
                    grip("grip")
                    reach(hover, 800, 1.0)
                    closed = arm_pulses().get(10)
                    target = int(arm_config["gripper"]["grip"])
                    held = closed is not None and closed < target - int(arm_config["grasp_margin"])
                    result = {"outcome": "holding" if held else "missed", "cube": cube,
                              "pitch_deg": round(math.degrees(pitch)), "gripper_readback": closed}
                    if not held:
                        grip("open")
            self._audit("arm_pick", result["outcome"], result)
            return result

        @app.post("/arm/place", dependencies=[Depends(auth)])
        def arm_place(body: PlaceRequest):
            if not 1 <= body.level <= 4:
                raise HTTPException(422, "level must be 1..4")
            if body.on:
                seen = find(body.on, size * body.level - size / 2)
                if not seen:
                    result = {"outcome": "target not seen", "on": body.on}
                    self._audit("arm_place", result["outcome"], result)
                    return result
                spot, z = seen[0], size * body.level + size / 2 + clearance
            elif body.x is not None and body.y is not None:
                spot, z = {"x": body.x, "y": body.y, "yaw": 0.0}, size / 2 + clearance
            else:
                raise HTTPException(422, "give on=<color> or x,y")
            # Live B2 #3 placed at -85/-80 deg and stood; #4 at -60/-55 and fell: place steep only.
            # S2 #1: picked at -55, placed at -75 -> cube came down 20 deg tilted and slid off.
            # With the pick pitch known, place within 10 deg of it: exact was unreachable (S2 #2),
            # 10 deg off stood (series 1 #3), 15-20 deg off fell (S2 #5, #1).
            # The floor can't topple a cube: no pitch window there (unstack spot was unreachable at
            # the pick pitch, 2026-10-16), steep enough not to drag the jaw across the floor.
            steepest, flattest = -math.pi / 2, -math.radians(70 if body.on else 45)
            if body.pitch_deg is not None and body.on:
                steepest = max(-math.pi / 2, math.radians(body.pitch_deg - 10))
                flattest = min(-math.radians(60), math.radians(body.pitch_deg + 10))
            if body.on:
                # No overhead re-locate here: at hover the held cube hides most of the target and
                # its visible sliver moved the spot 3.5 cm (c3 2026-10-17, green fell off blue).
                # From look_down the held cube sits at the image bottom, clear of the target.
                plan = place_plan(spot, z, spot["yaw"], flattest=flattest, steepest=steepest)
            else:
                plan = place_plan(spot, z, 0.0, flattest=flattest, steepest=steepest)
            if plan is None:
                result = {"outcome": "out of reach", "spot": spot}
            else:
                hover, release, pitch = plan
                reach(hover, 1200, 1.4)
                reach(release, 800, 1.0)
                grip("open")
                reach(hover, 800, 1.0)
                outcome = "placed"
                if body.color:  # verify from look_down (at hover the cube is cut off at the image edge)
                    pose("look_down", 2.0)
                    # Mid-height planes for the placed cube, as find() used for the target.
                    middle = size * body.level + size / 2 if body.on else size / 2
                    error = seen_near(body.color, spot, middle, within=0.04)
                    # Good places land within ~1.2 cm; one 2 cm off had fallen (c3 2026-10-17).
                    outcome = ("placed" if error is not None and math.hypot(*error) < 0.015
                               else "not on target")
                result = {"outcome": outcome, "spot": spot, "z": round(z, 3),
                          "pitch_deg": round(math.degrees(pitch))}
                if body.color:
                    result["error_radial_side"] = error
            self._audit("arm_place", result["outcome"], result)
            return result

        @app.post("/arm/follow", dependencies=[Depends(auth)])
        def arm_follow(body: FollowRequest):
            if not 3.0 <= body.seconds <= 300.0 or not 0.3 <= body.distance_m <= 3.0:
                raise HTTPException(422, "seconds 3..300, distance_m 0.3..3")
            self._motion_start()
            pose("forward", 1.8)
            aim = CameraAim()
            state = {"bearing": 0.0, "elevation": 0.0, "seen": None,
                     "last_seen": time.monotonic(), "frames": 0, "centred": 0}
            start = time.monotonic()
            clearance = float(self.get_parameter("move_clearance_m").value)

            def step(odom):
                now = time.monotonic()
                if now - start > body.seconds:
                    return None
                with self.lock:
                    frame = self.vision
                    map_pose = None if self.pose is None else list(self.pose)
                if frame is None or time.time() - frame["stamp"] > 2.0 or not frame.get("k"):
                    return "vision stale"
                item = pick_target(frame["objects"], body.target,
                                   (state["seen"] or {}).get("id"))
                if item is None:
                    if now - state["last_seen"] > 3.0:
                        return "target lost" if state["seen"] else "target not found"
                    return (0.0, 0.0, 0.0)
                state["seen"], state["last_seen"] = item, now
                box = item["box"]
                centre = item.get("center") or [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
                k = frame["k"]
                state["frames"] += 1
                state["centred"] += abs(math.atan2(centre[0] - k[2], k[0])) < math.radians(10)
                bearing, elevation, vx, wz = follow_step(centre, k, state["bearing"],
                                                         state["elevation"],
                                                         item.get("range_m"), body.distance_m)
                bearing = max(-aim.max_bearing, min(aim.max_bearing, bearing))
                elevation = max(aim.min_elevation, min(aim.max_elevation, elevation))
                state["bearing"], state["elevation"] = bearing, elevation
                try:
                    response = self._call(self.arm_aim, AimCamera.Request(
                        bearing=bearing, elevation=elevation, duration_ms=120), timeout=0.5)
                except HTTPException as exc:
                    return f"arm aim failed: {exc.detail}"
                if not response.success:
                    return f"arm refused: {response.message}"
                if vx > 0.0 and map_pose is not None and self._first_blocked(
                        map_pose[0], map_pose[1], map_pose[2], 0.3 + clearance):
                    vx = 0.0  # costmap says no: keep turning and tracking, don't drive
                return (vx, 0.0, wz)

            outcome = self._motion_loop(step, body.seconds + 5.0)
            try:
                pose("forward", 0.0)
            except HTTPException:
                pass
            seen = state["seen"] or {}
            result = {"outcome": outcome, "target": body.target,
                      "frames_tracked": state["frames"],
                      "centred_pct": round(100 * state["centred"] / max(1, state["frames"])),
                      "last_range_m": seen.get("range_m"), "pose": self.pose}
            self._audit("arm_follow", outcome, result)
            return result

        @app.get("/snapshot", dependencies=[Depends(auth)])
        def snapshot():
            with self.lock:
                jpeg = self.jpeg
            if jpeg is None:
                raise HTTPException(503, "camera image unavailable")
            return Response(jpeg, media_type="image/jpeg")

        @app.get("/wander/candidate", dependencies=[Depends(auth)])
        def wander_candidate(max_distance: float = 1.5):
            if not 0.5 <= max_distance <= 3.0:
                raise HTTPException(422, "max_distance must be 0.5..3.0 m")
            with self.lock:
                grid, meta = self.costmap, self.costmap_meta
                pose = None if self.pose is None else list(self.pose)
            if grid is None or pose is None:
                raise HTTPException(503, "costmap or pose unavailable")
            # Same 0..100 -> internal 0..255 conversion as tools/run_nav_soak.py.
            internal = np.clip(np.rint(grid.astype(float) * 2.55), 0, 255).astype(np.uint8)
            choices = reachable_goal_candidates(
                internal, meta[0], (meta[1], meta[2]), tuple(pose[:2]), count=12,
                min_distance=0.4, max_distance=max_distance, min_separation=0.2,
                seed=random.randrange(1 << 30), allow_trapped_start=True)
            if not choices:
                raise HTTPException(409, "no reachable free point in range")
            # Live obstacles close narrow routes mid-leg in clutter; favour open targets.
            ranked = rank_goal_candidates_by_clearance(
                internal, meta[0], (meta[1], meta[2]), choices)
            x, y, _ = random.choice(ranked[:3])
            # Arrive facing the direction of travel: no extra final turn.
            yaw = math.atan2(y - pose[1], x - pose[0])
            return {"x": x, "y": y, "yaw": yaw, "from": pose}

        @app.get("/vision/objects", dependencies=[Depends(auth)])
        def vision_objects():
            with self.lock:
                frame = self.vision
            if frame is None or time.time() - frame["stamp"] > 2.0:
                raise HTTPException(503, "vision unavailable or stale")
            return frame

        @app.get("/map.png", dependencies=[Depends(auth)])
        def map_png():
            with self.lock:
                grid = None if self.map_grid is None else self.map_grid.copy()
            if grid is None:
                raise HTTPException(503, "occupancy map unavailable")
            pixels = np.full(grid.shape, 127, dtype=np.uint8)
            known = grid >= 0
            pixels[known] = np.clip(255.0 - grid[known] * 2.55,
                                    0, 255).astype(np.uint8)
            ok, encoded = cv2.imencode(".png", np.flipud(pixels))
            if not ok:
                raise HTTPException(500, "map encoding failed")
            return Response(encoded.tobytes(), media_type="image/png")

        @app.websocket("/telemetry")
        async def telemetry(websocket: WebSocket):
            if not hmac.compare_digest(websocket.query_params.get("token", ""), self.token):
                await websocket.close(code=1008)
                return
            await websocket.accept()
            try:
                while True:
                    await websocket.send_json(self.status())
                    await asyncio.sleep(0.5)
            except WebSocketDisconnect:
                pass

        return app

    def shutdown(self) -> None:
        with self.lock:
            self.drive_command, self.drive_until = (0.0, 0.0, 0.0), 0.0
        self.drive_pub.publish(Twist())
        self.server.should_exit = True
        self.server_thread.join(timeout=3.0)


def main():
    rclpy.init()
    node = GatewayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
