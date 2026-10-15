"""FastAPI/WS command surface bridged to bounded ROS interfaces."""
from __future__ import annotations

import asyncio
import hmac
import json
import math
import os
import random
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


def _model_dict(model) -> dict:
    return model.model_dump() if hasattr(model, "model_dump") else model.dict()


class GatewayNode(Node):
    def __init__(self):
        super().__init__("hwr_gateway")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("camera_pose_tolerance_rad", 0.10)
        self.declare_parameter("joint_state_timeout", 1.0)
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

        @app.post("/stop", dependencies=[Depends(auth)])
        def stop():
            with self.lock:
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
                                       tool_pitch=body.tool_pitch,
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
