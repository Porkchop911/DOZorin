"""Choose reachable frontiers and dispatch bounded, journaled nav legs."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hwr_msgs.action import NavigateToPose
from hwr_msgs.msg import LocalizationHealth
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import LaserScan

from hwr_core.exploration import (find_frontiers, shorter_leg,
                                  restore_semantic_boundary_cooldowns)
from hwr_core.navigation import (SemanticPreflightClient,
                                 limit_translation_target,
                                 metric_rotation_preflight,
                                 staged_path_waypoint,
                                 staged_yaw_waypoints)
from hwr_sensors.depth_projection import floor_support_preflight


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class ExploreNode(Node):
    def __init__(self):
        super().__init__("hwr_explore")
        self.declare_parameter("minimum_frontier_cells", 8)
        self.declare_parameter("session_seconds", 900.0)
        self.declare_parameter("goal_timeout_seconds", 45.0)
        # Mapper publishes at 0.5 Hz; three seconds tolerates one delayed map
        # cycle while still rejecting a genuinely stale exploration state.
        self.declare_parameter("input_freshness_seconds", 3.0)
        self.declare_parameter("maximum_goal_leg_m", 0.10)
        self.declare_parameter("frontier_setback_m", 0.25)
        self.declare_parameter("frontier_cooldown_seconds", 120.0)
        self.declare_parameter("semantic_boundary_cooldown_seconds", 3600.0)
        self.declare_parameter("frontier_exclusion_radius_m", 0.35)
        self.declare_parameter("approach_bearing_cooldown_deg", 25.0)
        self.declare_parameter("translation_heading_tolerance_deg", 8.0)
        self.declare_parameter("maximum_rotation_stages_per_frontier", 6)
        self.declare_parameter("maximum_session_rotation_stages", 18)
        # Keep one extra depth/range cell beyond the 0.20 m hard footprint.
        self.declare_parameter("semantic_margin_m", 0.25)
        self.declare_parameter("semantic_timeout_seconds", 60.0)
        self.declare_parameter(
            "gateway_url", os.environ.get(
                "HWR_GATEWAY_URL", "http://127.0.0.1:8090"))
        self.declare_parameter(
            "vlm_url", os.environ.get(
                "LM_STUDIO_URL", "http://192.168.0.148:12340/v1"))
        self.declare_parameter(
            "vlm_model", os.environ.get("HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
        self.declare_parameter(
            "journal_path",
            os.path.expanduser("~/hwr/state/exploration.jsonl"))

        self.map = None
        self.map_resolution = None
        self.map_origin = None
        self.map_time = 0.0
        self.costmap = None
        self.costmap_resolution = None
        self.costmap_origin = None
        self.costmap_time = 0.0
        self.pose = None
        self.pose_time = 0.0
        self.localization_healthy = False
        self.localization_time = 0.0
        self.floor_support = None
        self.floor_support_angle_min = 0.0
        self.floor_support_angle_increment = 0.0
        self.floor_support_valid = False
        self.floor_support_time = 0.0
        self.enabled = False
        self.started = 0.0
        self.goal_handle = None
        self.goal_pending = False
        self.pending_request = None
        self.pending_generation = None
        self.reset_acknowledged = False
        self.reset_requested = 0.0
        self.goal_started = 0.0
        self.active_frontier = None
        self.committed_frontier = None
        self.committed_waypoint = None
        self.committed_rotation_stages = 0
        self.session_rotation_stages = 0
        self.active_waypoint = None
        self.active_kind = None
        self.stop_pending = False
        self.stop_future = None
        self.estop_latched = True
        self.estop_state_time = 0.0
        self.next_goal_after = 0.0
        self.failures = 0
        self.cooldowns: list[tuple[tuple[float, float], float]] = []
        self.bearing_cooldowns: list[tuple[float, float]] = []
        self.session_generation = 0
        self.state_lock = threading.RLock()
        self.geometry_mismatch_since = 0.0

        journal = Path(str(self.get_parameter("journal_path").value))
        journal.parent.mkdir(parents=True, exist_ok=True)
        self.journal_path = journal
        if journal.exists():
            try:
                records = [json.loads(line) for line in
                           journal.read_text(encoding="utf-8").splitlines()
                           if line.strip()]
                restored_goals, restored_bearings = (
                    restore_semantic_boundary_cooldowns(
                        records[-5000:],
                        float(self.get_parameter(
                            "semantic_boundary_cooldown_seconds").value),
                        now_wall=time.time(), now_monotonic=time.monotonic()))
                self.cooldowns.extend(restored_goals)
                self.bearing_cooldowns.extend(restored_bearings)
                if restored_goals or restored_bearings:
                    self._journal(
                        "semantic_boundaries_restored",
                        frontiers=len(restored_goals),
                        bearings=len(restored_bearings))
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                self.get_logger().warning(
                    f"semantic boundary restore skipped: {exc}")

        token = os.environ.get("HWR_GATEWAY_TOKEN", "")
        try:
            self.semantic = SemanticPreflightClient(
                str(self.get_parameter("gateway_url").value), token,
                str(self.get_parameter("vlm_url").value),
                str(self.get_parameter("vlm_model").value),
                timeout=float(
                    self.get_parameter("semantic_timeout_seconds").value),
                margin=float(self.get_parameter("semantic_margin_m").value),
                retries=1)
        except ValueError as exc:
            self.semantic = None
            self.get_logger().error(str(exc))

        self.sensor_group = ReentrantCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.mode_group = ReentrantCallbackGroup()
        self.nav = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self.control_group)
        self.save = self.create_client(
            Trigger, "map_save", callback_group=self.control_group)
        self.estop = self.create_client(
            SetBool, "estop", callback_group=self.control_group)
        map_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, "map", self._on_map, map_qos,
            callback_group=self.sensor_group)
        self.create_subscription(
            OccupancyGrid, "costmap", self._on_costmap, 1,
            callback_group=self.sensor_group)
        self.create_subscription(
            PoseWithCovarianceStamped, "pose", self._on_pose, 10,
            callback_group=self.sensor_group)
        self.create_subscription(LocalizationHealth, "localization_health",
                                 self._on_localization, 10,
                                 callback_group=self.sensor_group)
        self.create_subscription(Bool, "estop_state", self._on_estop_state,
                                 map_qos, callback_group=self.sensor_group)
        self.create_subscription(LaserScan, "floor_support",
                                 self._on_floor_support,
                                 qos_profile_sensor_data,
                                 callback_group=self.sensor_group)
        # Mode requests must remain responsive while a visual preflight blocks
        # the control timer. In particular, stop cannot wait behind an LLM.
        self.create_service(SetBool, "explore", self._on_mode,
                            callback_group=self.mode_group)
        self.create_timer(0.25, self._tick,
                          callback_group=self.control_group)
        self._journal("node_started")

    def _journal(self, event: str, **fields) -> None:
        record = {"time": time.time(), "event": event, **fields}
        try:
            with self.journal_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        except OSError as exc:
            self.get_logger().error(f"exploration journal write failed: {exc}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        if msg.data and msg.info.resolution > 0.0:
            occupancy = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width).astype(np.int8)
            with self.state_lock:
                self.map = occupancy
                self.map_resolution = float(msg.info.resolution)
                self.map_origin = (msg.info.origin.position.x,
                                   msg.info.origin.position.y)
                self.map_time = time.monotonic()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        if msg.data and msg.info.resolution > 0.0:
            values = np.asarray(msg.data, dtype=float).reshape(
                msg.info.height, msg.info.width)
            costmap = np.clip(np.rint(values * 2.55), 0, 255).astype(np.uint8)
            with self.state_lock:
                self.costmap = costmap
                self.costmap_resolution = float(msg.info.resolution)
                self.costmap_origin = (msg.info.origin.position.x,
                                       msg.info.origin.position.y)
                self.costmap_time = time.monotonic()

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        with self.state_lock:
            self.pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self.pose_time = time.monotonic()

    def _on_localization(self, msg: LocalizationHealth) -> None:
        with self.state_lock:
            self.localization_healthy = bool(msg.healthy)
            self.localization_time = time.monotonic()

    def _on_estop_state(self, msg: Bool) -> None:
        with self.state_lock:
            self.estop_latched = bool(msg.data)
            self.estop_state_time = time.monotonic()

    def _on_floor_support(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        intensities = np.asarray(msg.intensities, dtype=float)
        valid = (len(ranges) >= 3 and len(intensities) == len(ranges)
                 and np.all(np.isfinite(ranges))
                 and np.all(intensities >= 64.0)
                 and msg.angle_increment > 0.0)
        with self.state_lock:
            self.floor_support = ranges
            self.floor_support_angle_min = float(msg.angle_min)
            self.floor_support_angle_increment = float(msg.angle_increment)
            self.floor_support_valid = bool(valid)
            self.floor_support_time = time.monotonic()

    def _input_reason(self) -> str | None:
        with self.state_lock:
            now = time.monotonic()
            freshness = float(
                self.get_parameter("input_freshness_seconds").value)
            if self.map is None or now - self.map_time > freshness:
                return "map missing or stale"
            if self.costmap is None or now - self.costmap_time > freshness:
                return "costmap missing or stale"
            if self.pose is None or now - self.pose_time > freshness:
                return "pose missing or stale"
            if (not self.localization_healthy
                    or now - self.localization_time > freshness):
                return "localization unhealthy or stale"
            if (self.map.shape != self.costmap.shape
                    or abs(self.map_resolution - self.costmap_resolution) > 1e-9
                    or any(abs(a - b) > 1e-6 for a, b in zip(
                        self.map_origin, self.costmap_origin))):
                return "map and costmap geometry differ"
        return None

    def _floor_support_reason(self) -> str | None:
        """Return why translation floor support is unavailable.

        Floor-plane estimation can be view-dependent, particularly when the
        depth camera faces sunlit glass.  It must fail translation closed, but
        it must not prevent an otherwise metric-cleared in-place rotation that
        can move the camera away from the bad view.
        """
        with self.state_lock:
            freshness = float(
                self.get_parameter("input_freshness_seconds").value)
            if self.floor_support is None:
                return "floor-support profile missing"
            if time.monotonic() - self.floor_support_time > freshness:
                return "floor-support profile stale"
            if not self.floor_support_valid:
                return "floor-support profile invalid"
        return None

    def _on_mode(self, request, response):
        if request.data:
            if self.semantic is None:
                response.success = False
                response.message = "semantic preflight unavailable"
                self._journal("start_rejected", reason=response.message)
                return response
            reason = self._input_reason()
            if reason is not None and reason != "map and costmap geometry differ":
                response.success = False
                response.message = reason
                self._journal("start_rejected", reason=reason)
                return response
            if self.enabled:
                response.success = True
                response.message = "exploration already active"
                return response
            self.enabled = True
            self.session_generation += 1
            self.started = time.monotonic()
            self.failures = 0
            # Cooldowns deliberately survive bounded sector stop/start cycles.
            # Short navigation failures expire normally; semantic glass
            # boundaries remain excluded for their longer configured duration.
            self.committed_frontier = None
            self.committed_waypoint = None
            self.committed_rotation_stages = 0
            self.session_rotation_stages = 0
            self.geometry_mismatch_since = (
                time.monotonic() if reason is not None else 0.0)
            self._journal("session_started", pose=list(self.pose))
            response.success, response.message = True, "exploration started"
        else:
            self.session_generation += 1
            self._finish("stopped by request", save=False)
            response.success, response.message = True, "exploration stopped"
        return response

    def _trip_stop(self) -> None:
        if self.stop_pending or not self.estop.service_is_ready():
            return
        self.stop_pending = True
        future = self.estop.call_async(SetBool.Request(data=True))
        self.stop_future = future
        future.add_done_callback(self._stop_complete)

    def _stop_complete(self, future) -> None:
        if future is not self.stop_future:
            return
        self.stop_future = None
        self.stop_pending = False
        try:
            response = future.result()
            if not response.success:
                self._journal("stop_latch_failed", reason=response.message)
        except Exception as exc:
            self._journal("stop_latch_failed", reason=str(exc))

    def _save_map(self) -> None:
        if not self.save.service_is_ready():
            self._journal("map_save_unavailable")
            return
        future = self.save.call_async(Trigger.Request())
        future.add_done_callback(self._map_saved)

    def _map_saved(self, future) -> None:
        try:
            response = future.result()
            self._journal("map_save_result", success=bool(response.success),
                          message=response.message)
        except Exception as exc:
            self._journal("map_save_result", success=False, error=str(exc))

    def _finish(self, reason: str, save: bool = True) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_pending = False
        self.pending_request = None
        self.pending_generation = None
        self.reset_acknowledged = False
        self.reset_requested = 0.0
        self.goal_started = 0.0
        self.active_frontier = None
        self.committed_frontier = None
        self.committed_waypoint = None
        self.committed_rotation_stages = 0
        self.active_waypoint = None
        self.active_kind = None
        was_enabled = self.enabled
        self.enabled = False
        self.session_generation += 1
        self._trip_stop()
        if was_enabled:
            self._journal("session_finished", reason=reason,
                          failures=self.failures, save_requested=save)
        self.get_logger().info(reason)
        if save:
            self._save_map()

    def _active_exclusions(self, now: float) -> tuple[tuple[float, float], ...]:
        self.cooldowns = [(goal, expiry) for goal, expiry in self.cooldowns
                          if expiry > now]
        return tuple(goal for goal, _ in self.cooldowns)

    def _bearing_excluded(self, waypoint: tuple[float, float], now: float) -> bool:
        self.bearing_cooldowns = [
            (bearing, expiry) for bearing, expiry in self.bearing_cooldowns
            if expiry > now]
        dx, dy = waypoint[0] - self.pose[0], waypoint[1] - self.pose[1]
        if math.hypot(dx, dy) <= 0.03:
            return False
        bearing = math.atan2(dy, dx)
        tolerance = math.radians(float(
            self.get_parameter("approach_bearing_cooldown_deg").value))
        return any(abs(math.atan2(
            math.sin(bearing - rejected),
            math.cos(bearing - rejected))) <= tolerance
            for rejected, _ in self.bearing_cooldowns)

    def _cooldown_active_frontier(
            self, reason: str, *, duration: float | None = None,
            count_failure: bool = True) -> None:
        if self.active_frontier is not None:
            if duration is None:
                duration = float(
                    self.get_parameter("frontier_cooldown_seconds").value)
            self.cooldowns.append((self.active_frontier.goal,
                                   time.monotonic() + duration))
            if (self.active_waypoint is not None and self.pose is not None
                    and math.hypot(
                        self.active_waypoint[0] - self.pose[0],
                        self.active_waypoint[1] - self.pose[1]) > 0.03):
                bearing = math.atan2(
                    self.active_waypoint[1] - self.pose[1],
                    self.active_waypoint[0] - self.pose[0])
                self.bearing_cooldowns.append(
                    (bearing, time.monotonic() + duration))
            self._journal("frontier_failed", reason=reason,
                          frontier_goal=list(self.active_frontier.goal),
                          waypoint=(None if self.active_waypoint is None
                                    else list(self.active_waypoint)))
        if count_failure:
            self.failures += 1
        self.committed_frontier = None
        self.committed_waypoint = None
        self.committed_rotation_stages = 0

    def _tick(self) -> None:
        if not self.enabled:
            return
        generation = self.session_generation
        now = time.monotonic()
        if now - self.started >= float(
                self.get_parameter("session_seconds").value):
            self._finish("exploration session budget reached")
            return
        # A stop request issued after the previous goal must be acknowledged
        # before any new reset can be sent. This closes the async true/false
        # race that could re-latch stop during the next accepted goal.
        if self.stop_pending:
            return
        if self.goal_handle is not None:
            # Once navigation has accepted a goal, it owns live localization,
            # lidar/depth freshness, collision checking, and stop behavior.
            # A mapper resize may temporarily put map and costmap on adjacent
            # generations; that is a planning-boundary concern and must not
            # asynchronously trip stop during an otherwise valid short goal.
            if now - self.goal_started >= float(
                    self.get_parameter("goal_timeout_seconds").value):
                self._cooldown_active_frontier("goal timeout")
                self._finish("exploration goal timed out")
            return
        reason = self._input_reason()
        if reason is not None:
            if reason == "map and costmap geometry differ":
                if self.geometry_mismatch_since <= 0.0:
                    self.geometry_mismatch_since = now
                    self._journal("grid_sync_wait")
                if now - self.geometry_mismatch_since <= float(
                        self.get_parameter("input_freshness_seconds").value):
                    # The mapper may resize first; navigation republishes the
                    # matching costmap on its next 0.5 s cycle.  Never plan
                    # across generations, and keep the existing stop latch.
                    self._trip_stop()
                    return
            self._finish(reason)
            return
        if self.geometry_mismatch_since > 0.0:
            self._journal("grid_sync_restored")
            self.geometry_mismatch_since = 0.0
        if self.goal_pending:
            if self.reset_acknowledged and self.pending_request is not None:
                with self.state_lock:
                    reset_observed = (
                        not self.estop_latched
                        and now - self.estop_state_time <= 1.0)
                if reset_observed:
                    request = self.pending_request
                    request_generation = self.pending_generation
                    self.pending_request = None
                    self.pending_generation = None
                    self.reset_acknowledged = False
                    if (not self.enabled
                            or request_generation != self.session_generation):
                        self.goal_pending = False
                        self._trip_stop()
                        return
                    self._journal("estop_reset_observed_for_goal",
                                  kind=self.active_kind)
                    sent = self.nav.send_goal_async(request)
                    sent.add_done_callback(self._goal_response)
                elif now - self.reset_requested > 2.0:
                    self.goal_pending = False
                    self.pending_request = None
                    self.pending_generation = None
                    self.reset_acknowledged = False
                    self._finish("estop reset state was not observed")
                return
            return
        if now < self.next_goal_after:
            return

        waypoint = None
        if self.committed_frontier is None:
            frontiers = find_frontiers(
                self.map, self.map_resolution, self.map_origin, self.pose[:2],
                min_cells=int(
                    self.get_parameter("minimum_frontier_cells").value),
                costmap=self.costmap, robot_yaw=self.pose[2],
                setback_m=float(
                    self.get_parameter("frontier_setback_m").value),
                excluded_goals=self._active_exclusions(now),
                exclusion_radius_m=float(
                    self.get_parameter("frontier_exclusion_radius_m").value))
            if not frontiers:
                self._finish("exploration complete: no eligible frontier remains")
                return
            frontier = None
            waypoint = None
            for candidate in frontiers:
                try:
                    staged = staged_path_waypoint(
                        self.costmap, self.costmap_resolution,
                        self.costmap_origin, self.pose[:2], candidate.goal,
                        max_leg=float(self.get_parameter(
                            "maximum_goal_leg_m").value))
                except ValueError:
                    continue
                if not self._bearing_excluded(staged, now):
                    frontier, waypoint = candidate, staged
                    break
            if frontier is None:
                self._finish(
                    "exploration paused: no eligible local approach direction")
                return
            self.committed_frontier = frontier
            self.committed_waypoint = waypoint
            self.committed_rotation_stages = 0
        else:
            # Keep both the selected frontier and its local approach waypoint
            # through bounded rotations. Recomputing a 10 cm waypoint after
            # every turn lets centimetre-scale map-pose drift flip A* between
            # neighboring cells, so the robot can oscillate between headings
            # without attempting the already-validated translation.
            frontiers = [self.committed_frontier]
            waypoint = self.committed_waypoint
        if not self.nav.wait_for_server(timeout_sec=0.1):
            return

        frontier = self.committed_frontier
        if waypoint is None:
            try:
                waypoint = staged_path_waypoint(
                    self.costmap, self.costmap_resolution,
                    self.costmap_origin, self.pose[:2], frontier.goal,
                    max_leg=float(self.get_parameter(
                        "maximum_goal_leg_m").value))
                self.committed_waypoint = waypoint
            except ValueError as exc:
                self.active_frontier = frontier
                self.active_waypoint = None
                self._cooldown_active_frontier(str(exc))
                self.active_frontier = None
                self.next_goal_after = now + 0.5
                return

        # The waypoint is deliberately committed across rotation stages, but
        # map-pose x/y can drift by a centimetre while turning. Preserve its
        # bearing while clamping the dispatched leg to the strict live cap.
        waypoint = limit_translation_target(
            self.pose[:2], waypoint,
            float(self.get_parameter("maximum_goal_leg_m").value))

        yaw = math.atan2(waypoint[1] - self.pose[1],
                         waypoint[0] - self.pose[0])
        yaw_error = math.atan2(math.sin(yaw - self.pose[2]),
                               math.cos(yaw - self.pose[2]))
        kind = "translation"
        preflight = None
        heading_tolerance = math.radians(float(
            self.get_parameter("translation_heading_tolerance_deg").value))
        if abs(yaw_error) > heading_tolerance:
            per_frontier_limit = int(self.get_parameter(
                "maximum_rotation_stages_per_frontier").value)
            session_limit = int(self.get_parameter(
                "maximum_session_rotation_stages").value)
            if (self.committed_rotation_stages >= per_frontier_limit
                    or self.session_rotation_stages >= session_limit):
                self.active_frontier = frontier
                self.active_waypoint = waypoint
                if self.session_rotation_stages >= session_limit:
                    self._finish(
                        "exploration paused: rotation stage budget reached")
                else:
                    self._cooldown_active_frontier(
                        "mapped heading did not converge within bounded "
                        "rotation stages")
                    self.active_frontier = None
                    self.active_waypoint = None
                    self.next_goal_after = now + 2.0
                return
            try:
                context = self.semantic.context()
            except Exception as exc:
                self._finish(f"rotation context unavailable: {exc}")
                return
            if not self.enabled or generation != self.session_generation:
                self._trip_stop()
                return
            allowed, reasons = metric_rotation_preflight(context)
            preflight = {"allowed": allowed, "reasons": reasons,
                         "navigation_context": context}
            if not allowed:
                self.active_frontier = frontier
                self.active_waypoint = None
                self._cooldown_active_frontier(
                    "rotation preflight rejected: " + ", ".join(reasons))
                self.active_frontier = None
                self.next_goal_after = now + 2.0
                return
            yaw = staged_yaw_waypoints(self.pose[2], yaw)[0]
            waypoint = self.pose[:2]
            kind = "rotation"
        else:
            distance = math.hypot(waypoint[0] - self.pose[0],
                                  waypoint[1] - self.pose[1])
            floor_input_reason = self._floor_support_reason()
            if floor_input_reason is not None:
                self.active_frontier = frontier
                self.active_waypoint = waypoint
                self._journal(
                    "floor_support_rejected",
                    frontier_goal=list(frontier.goal),
                    waypoint=list(waypoint), reasons=[floor_input_reason])
                self._cooldown_active_frontier(
                    "floor-support preflight rejected: "
                    + floor_input_reason,
                    count_failure=False)
                self.active_frontier = None
                self.active_waypoint = None
                self.next_goal_after = now + 2.0
                return
            floor_allowed, floor_reasons = floor_support_preflight(
                self.floor_support, self.floor_support_angle_min,
                self.floor_support_angle_increment, distance,
                margin_m=0.10, half_width_m=0.25,
                required_fraction=0.80)
            if not floor_allowed:
                self.active_frontier = frontier
                self.active_waypoint = waypoint
                self._journal("floor_support_rejected",
                              frontier_goal=list(frontier.goal),
                              waypoint=list(waypoint), reasons=floor_reasons)
                self._cooldown_active_frontier(
                    "floor-support preflight rejected: "
                    + ", ".join(floor_reasons))
                self.active_frontier = None
                self.active_waypoint = None
                self.next_goal_after = now + 2.0
                return
            try:
                allowed, preflight = self.semantic.assess(distance)
            except Exception as exc:
                self._finish(f"semantic preflight unavailable: {exc}")
                return
            if not self.enabled or generation != self.session_generation:
                self._trip_stop()
                return
            glass_boundary = bool(
                (preflight.get("assessment") or {}).get(
                    "glass_boundary_in_corridor"))
            half = None if allowed or glass_boundary else shorter_leg(self.pose[:2], waypoint)
            if half is not None:
                # plan/18.3: clutter far ahead rejected whole legs; ask once for half of it.
                half_distance = math.hypot(half[0] - self.pose[0], half[1] - self.pose[1])
                try:
                    half_allowed, half_preflight = self.semantic.assess(half_distance)
                except Exception:
                    half_allowed = False
                if not self.enabled or generation != self.session_generation:
                    self._trip_stop()
                    return
                self._journal("half_leg_retry", frontier_goal=list(frontier.goal),
                              waypoint=list(half), allowed=half_allowed)
                if half_allowed:
                    allowed, preflight, waypoint, distance = True, half_preflight, half, half_distance
            if not allowed:
                self.active_frontier = frontier
                self.active_waypoint = waypoint
                self._journal("semantic_preflight_rejected",
                              frontier_goal=list(frontier.goal),
                              waypoint=list(waypoint), report=preflight)
                if glass_boundary:
                    self._journal(
                        "semantic_boundary_rejected", boundary="glass",
                        frontier_goal=list(frontier.goal),
                        waypoint=list(waypoint),
                        evidence=(preflight.get("assessment") or {}).get(
                            "glass_boundary_evidence", ""))
                self._cooldown_active_frontier(
                    "semantic glass boundary rejected" if glass_boundary else
                    "semantic/metric translation preflight rejected",
                    duration=(float(self.get_parameter(
                        "semantic_boundary_cooldown_seconds").value)
                              if glass_boundary else None),
                    count_failure=not glass_boundary)
                self.active_frontier = None
                self.active_waypoint = None
                self.next_goal_after = now + 2.0
                return
            preflight["floor_support_allowed"] = True
            preflight["floor_support_required_m"] = distance + 0.10
        request = NavigateToPose.Goal()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x, request.target.pose.position.y = waypoint
        request.target.pose.orientation.z = math.sin(yaw / 2.0)
        request.target.pose.orientation.w = math.cos(yaw / 2.0)
        self.active_frontier = frontier
        self.active_waypoint = waypoint
        self.active_kind = kind
        self.goal_pending = True
        self._journal(
            "goal_selected", kind=kind, pose=list(self.pose),
            waypoint=list(waypoint), target_yaw=yaw,
            frontier_goal=list(frontier.goal), score=frontier.score,
            information_gain=frontier.information_gain,
            path_length_m=frontier.path_length_m,
            clearance_m=frontier.clearance_m,
            heading_change_rad=frontier.heading_change_rad,
            eligible_frontiers=len(frontiers), preflight=preflight)
        if not self.enabled or generation != self.session_generation:
            self.goal_pending = False
            self.pending_request = None
            self.pending_generation = None
            self.reset_acknowledged = False
            self._trip_stop()
            return
        if not self.estop.wait_for_service(timeout_sec=0.1):
            self.goal_pending = False
            self._finish("estop reset service unavailable")
            return
        self.pending_request = request
        self.pending_generation = generation
        self.reset_requested = time.monotonic()
        self.reset_acknowledged = False
        reset = self.estop.call_async(SetBool.Request(data=False))
        reset.add_done_callback(self._reset_complete)

    def _reset_complete(self, future) -> None:
        if not self.enabled:
            self.goal_pending = False
            self.pending_request = None
            self.pending_generation = None
            self.reset_acknowledged = False
            self._trip_stop()
            return
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message or "estop reset rejected")
        except Exception as exc:
            self.goal_pending = False
            self.pending_request = None
            self.pending_generation = None
            self.reset_acknowledged = False
            self._finish(f"estop reset failed: {exc}")
            return
        # The reset service response can arrive before hwr_nav receives the
        # transient-local estop_state=false sample. The control timer dispatches
        # only after that subscriber acknowledgement is fresh.
        self.reset_acknowledged = True
        self._journal("estop_reset_acknowledged", kind=self.active_kind)

    def _goal_response(self, future) -> None:
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self._trip_stop()
            self._cooldown_active_frontier(f"goal send failed: {exc}")
            self.active_frontier = None
            self.active_waypoint = None
            self.active_kind = None
            self.next_goal_after = time.monotonic() + 2.0
            return
        if not self.enabled:
            if handle.accepted:
                handle.cancel_goal_async()
            self._trip_stop()
            self.active_frontier = None
            self.active_waypoint = None
            self.active_kind = None
            return
        if not handle.accepted:
            self._trip_stop()
            self._cooldown_active_frontier("goal rejected")
            self.active_frontier = None
            self.active_waypoint = None
            self.active_kind = None
            self.next_goal_after = time.monotonic() + 2.0
            return
        self.goal_handle = handle
        self.goal_started = time.monotonic()
        self._journal("goal_accepted", kind=self.active_kind,
                      waypoint=list(self.active_waypoint))
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        self._trip_stop()
        self.goal_handle = None
        self.goal_started = 0.0
        if not self.enabled:
            self.active_frontier = None
            self.active_waypoint = None
            self.active_kind = None
            return
        try:
            result = future.result().result
            if not result.success:
                self._cooldown_active_frontier(result.message or "navigation failed")
                self.get_logger().warning(f"frontier goal failed: {result.message}")
            else:
                self.failures = 0
                self._journal("goal_succeeded", kind=self.active_kind,
                              waypoint=list(self.active_waypoint),
                              frontier_goal=list(self.active_frontier.goal))
                if self.active_kind == "translation":
                    self._save_map()
                    self.committed_frontier = None
                    self.committed_waypoint = None
                    self.committed_rotation_stages = 0
                elif self.active_kind == "rotation":
                    self.committed_rotation_stages += 1
                    self.session_rotation_stages += 1
        except Exception as exc:
            self._cooldown_active_frontier(f"goal result failed: {exc}")
            self.get_logger().error(f"frontier result failed: {exc}")
        self.active_frontier = None
        self.active_waypoint = None
        self.active_kind = None
        if self.failures >= 5:
            self._finish("exploration aborted after five navigation failures")
        self.next_goal_after = time.monotonic() + 2.0


def main():
    rclpy.init()
    node = ExploreNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._finish("exploration node stopping", save=False)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
