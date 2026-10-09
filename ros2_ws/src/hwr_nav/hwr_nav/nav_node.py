"""Health-gated A* navigation with a no-reverse holonomic controller."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from hwr_msgs.action import NavigateToPose
from hwr_msgs.msg import LocalizationHealth
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from hwr_core.mapping import load_map
from hwr_core.navigation import (Costmap, ForwardRecovery, GoalProgress,
                                 HolonomicController, ReplanBudget, astar,
                                 gyro_yaw_motion_confirmed,
                                 odometry_motion_confirmed,
                                 odometry_translation_within_limit,
                                 sensor_sample_fresh, simplify_path,
                                 translation_deadline_seconds,
                                 translation_goal_reached_or_passed)


def _yaw(q):
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class NavNode(Node):
    def __init__(self):
        super().__init__("hwr_nav")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("inflation_radius", 0.30)
        self.declare_parameter("allow_forward_recovery", False)
        self.declare_parameter("max_live_replans", 0)
        self.declare_parameter("max_yaw_rate", 0.30)
        self.declare_parameter("localization_recovery_grace", 1.5)
        self.declare_parameter("motion_sensor_timeout", 0.75)
        self.declare_parameter("translation_excursion_allowance", 0.05)
        path = self.get_parameter("map_yaml").value
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.allow_forward_recovery = bool(
            self.get_parameter("allow_forward_recovery").value)
        self.max_live_replans = int(
            self.get_parameter("max_live_replans").value)
        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.localization_recovery_grace = float(
            self.get_parameter("localization_recovery_grace").value)
        self.motion_sensor_timeout = float(
            self.get_parameter("motion_sensor_timeout").value)
        self.translation_excursion_allowance = float(
            self.get_parameter("translation_excursion_allowance").value)
        if not 0.10 <= max_yaw_rate <= 0.50:
            raise ValueError("max_yaw_rate must be 0.10..0.50 rad/s")
        if not 0.0 <= self.localization_recovery_grace <= 3.0:
            raise ValueError("localization_recovery_grace must be 0..3 seconds")
        if not 0.25 <= self.motion_sensor_timeout <= 1.5:
            raise ValueError("motion_sensor_timeout must be 0.25..1.5 seconds")
        if not 0.02 <= self.translation_excursion_allowance <= 0.15:
            raise ValueError(
                "translation_excursion_allowance must be 0.02..0.15 metres")
        self.costmap = None
        if path:
            occupancy, resolution, origin = load_map(path)
            self.costmap = Costmap(occupancy, resolution, origin,
                                   self.inflation_radius)
        self.controller = HolonomicController(max_wz=max_yaw_rate)
        self.recovery = ForwardRecovery()
        self.pose = None
        self.odom_pose = None
        self.gyro_yaw = 0.0
        self.gyro_stamp = 0.0
        self._last_odom_twist_stamp = None
        self.health = False
        self.pose_stamp = self.health_stamp = self.odom_stamp = 0.0
        self.lidar_stamp = self.depth_ranges_stamp = 0.0
        self.goal_active = False
        # The driver starts latched and publishes transient-local state. Reject
        # goals until an explicit reset is observed.
        self.estop_latched = True
        self.group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, "costmap", 1)
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._on_pose, 10,
                                 callback_group=self.group)
        self.create_subscription(Odometry, "odom", self._on_odom, 10,
                                 callback_group=self.group)
        self.create_subscription(LocalizationHealth, "localization_health",
                                 self._on_health, 10, callback_group=self.group)
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "estop_state", self._on_estop,
                                 state_qos, callback_group=self.group)
        self.create_subscription(OccupancyGrid, "map", self._on_map, 1,
                                 callback_group=self.group)
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "scan", self._on_lidar_ranges, qos,
                                 callback_group=self.group)
        self.create_subscription(LaserScan, "depth_ranges",
                                 self._on_depth_ranges, qos,
                                 callback_group=self.group)
        self.action = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._execute,
            cancel_callback=lambda request: CancelResponse.ACCEPT,
            goal_callback=self._on_goal,
            callback_group=self.group)
        self.create_service(Trigger, "clear_costmap", self._on_clear,
                            callback_group=self.group)
        self.create_timer(0.5, self._publish_costmap, callback_group=self.group)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
        self.pose_stamp = time.monotonic()

    def _on_health(self, msg: LocalizationHealth) -> None:
        self.health = msg.healthy
        self.health_stamp = time.monotonic()
        if not self.health:
            self.pub.publish(Twist())

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        pose = msg.pose.pose
        self.odom_pose = (
            pose.position.x, pose.position.y, _yaw(pose.orientation))
        wz = float(msg.twist.twist.angular.z)
        if math.isfinite(wz):
            if self._last_odom_twist_stamp is not None:
                dt = now - self._last_odom_twist_stamp
                if 0.0 < dt <= 0.5:
                    self.gyro_yaw = _wrap(self.gyro_yaw + wz * dt)
            self._last_odom_twist_stamp = now
            self.gyro_stamp = now
        self.odom_stamp = now

    def _on_estop(self, msg: Bool) -> None:
        self.estop_latched = bool(msg.data)
        if self.estop_latched:
            self.pub.publish(Twist())

    def _on_map(self, msg: OccupancyGrid) -> None:
        if not msg.data or msg.info.resolution <= 0.0:
            return
        occupancy = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width).astype(np.int8)
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        if (self.costmap is None
                or self.costmap.occupancy.shape != occupancy.shape
                or self.costmap.resolution != msg.info.resolution
                or not np.allclose(self.costmap.origin, origin)):
            self.costmap = Costmap(occupancy, msg.info.resolution, origin,
                                   self.inflation_radius)
        else:
            self.costmap.occupancy = occupancy

    def _on_goal(self, request):
        if self.goal_active or self.estop_latched:
            return GoalResponse.REJECT
        self.goal_active = True
        return GoalResponse.ACCEPT

    def _on_clear(self, request, response):
        if self.costmap is None:
            response.success, response.message = False, "no map available"
            return response
        self.costmap.clear_live()
        response.success, response.message = True, "live costmap cleared"
        return response

    def _publish_costmap(self) -> None:
        if self.costmap is None:
            return
        grid = self.costmap.grid(time.monotonic())
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.costmap.resolution
        msg.info.width, msg.info.height = grid.shape[1], grid.shape[0]
        msg.info.origin.position.x, msg.info.origin.position.y = self.costmap.origin
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid is int8: reserve 100 for lethal and scale all costs.
        msg.data = np.rint(grid.astype(float) * (100.0 / 255.0)).astype(int).ravel().tolist()
        self.pub_costmap.publish(msg)

    def _localization_ok(self) -> bool:
        now = time.monotonic()
        return (self.pose is not None and self.health
                and now - self.pose_stamp <= 1.0
                and now - self.health_stamp <= 1.0)

    def _on_ranges(self, msg: LaserScan) -> None:
        if self.pose is None or self.costmap is None:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        good = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        sensor_yaw = math.pi if msg.header.frame_id == "lidar_frame" else 0.0
        bearings = angles[good] + sensor_yaw + self.pose[2]
        points = np.column_stack((self.pose[0] + ranges[good] * np.cos(bearings),
                                  self.pose[1] + ranges[good] * np.sin(bearings)))
        self.costmap.mark(points, time.monotonic(), ttl=1.0)

    def _on_lidar_ranges(self, msg: LaserScan) -> None:
        self.lidar_stamp = time.monotonic()
        self._on_ranges(msg)

    def _on_depth_ranges(self, msg: LaserScan) -> None:
        self.depth_ranges_stamp = time.monotonic()
        self._on_ranges(msg)

    def _world(self, cell):
        return tuple(self.costmap.origin + (np.asarray(cell) + 0.5)
                     * self.costmap.resolution)

    def _plan(self, goal):
        if self.costmap is None:
            return []
        grid = self.costmap.grid(time.monotonic())
        start = self.costmap.world_to_cell(self.pose[:2])
        end = self.costmap.world_to_cell(goal[:2])
        cells = simplify_path(
            grid, astar(grid, start, end, allow_trapped_start=True))
        return [self._world(cell) for cell in cells]

    def _command_is_clear(self, vx, vy, wz=0.0) -> bool:
        if self.costmap is None:
            return False
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return abs(wz) < 1e-6 or self._rotation_is_clear()
        now = time.monotonic()
        if not (sensor_sample_fresh(
                    now, self.lidar_stamp, self.motion_sensor_timeout)
                and sensor_sample_fresh(
                    now, self.depth_ranges_stamp, self.motion_sensor_timeout)):
            return False
        # One-second projected stopping point inside the inflated costmap.
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        point = (self.pose[0] + c * vx - s * vy,
                 self.pose[1] + s * vx + c * vy)
        grid = self.costmap.grid(now)
        start = self.costmap.world_to_cell(self.pose[:2])
        start_inside = (0 <= start[0] < grid.shape[1]
                        and 0 <= start[1] < grid.shape[0])
        trapped = (start_inside and grid[start[1], start[0]] >= 200)
        # A map-pose correction can put the stopped robot inside an inflated
        # plateau even when the physical chassis has not contacted anything.
        # Permit only a monotonic, non-obstacle, globally planned prefix out of
        # that plateau. The costmap helper rejects rising cost, flat 255 cells,
        # re-entry, and out-of-bounds motion. Rotation remains prohibited here.
        return self.costmap.segment_is_clear(
            self.pose[:2], point, now,
            allow_trapped_start=trapped,
            allow_partial_escape=trapped)

    def _rotation_is_clear(self) -> bool:
        """Require fresh 360-degree sensing and a free current footprint."""
        if self.costmap is None or self.pose is None:
            return False
        now = time.monotonic()
        return (sensor_sample_fresh(
                    now, self.lidar_stamp, self.motion_sensor_timeout)
                and self.costmap.segment_is_clear(
                    self.pose[:2], self.pose[:2], now))

    def _feedback(self, goal, state):
        feedback = NavigateToPose.Feedback()
        feedback.current_pose.header.frame_id = "map"
        feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
        feedback.current_pose.pose.position.x = self.pose[0]
        feedback.current_pose.pose.position.y = self.pose[1]
        feedback.current_pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        feedback.current_pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        feedback.distance_remaining = math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])
        feedback.state = state
        return feedback

    def _stop(self) -> None:
        self.pub.publish(Twist())

    def _recover(self, handle, goal) -> tuple[bool, str]:
        """Clear, rotate-scan, and try one collision-checked forward arc."""
        self._stop()
        if self.costmap is None:
            return False, "map unavailable"
        self.costmap.clear_live()
        started = time.monotonic()
        while True:
            if self.estop_latched:
                self._stop()
                return False, "estop latched during navigation"
            if handle.is_cancel_requested:
                self._stop()
                return False, "canceled"
            if not self._localization_ok():
                self._stop()
                return False, "localization health lost"
            vx, vy, wz, state = self.recovery.command(time.monotonic() - started)
            if state == "replan":
                self._stop()
                return True, state
            if state == "forward-arc" and not self.allow_forward_recovery:
                self._stop()
                return True, "rotate-scan complete; forward recovery disabled"
            if vx > 0.0 and not self._command_is_clear(vx, vy):
                self._stop()
                return True, "forward arc blocked; replanning"
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
            self.pub.publish(command)
            handle.publish_feedback(self._feedback(goal, f"recovery: {state}"))
            time.sleep(0.1)

    def _execute(self, handle):
        result = NavigateToPose.Result()
        try:
            if self.estop_latched:
                handle.abort()
                result.message = "estop latched"
                return result
            if self.costmap is None:
                handle.abort()
                result.message = "map unavailable"
                return result
            if not self._localization_ok():
                handle.abort()
                result.message = "localization unavailable, stale, or unhealthy"
                return result
            if (self.odom_pose is None or not sensor_sample_fresh(
                    time.monotonic(), self.odom_stamp,
                    self.motion_sensor_timeout)):
                handle.abort()
                result.message = "fused odometry unavailable or stale"
                return result
            target = handle.request.target.pose
            goal = (target.position.x, target.position.y, _yaw(target.orientation))
            if not all(math.isfinite(value) for value in goal):
                handle.abort()
                result.message = "goal contains a non-finite value"
                return result
            path, last_plan = [], 0.0
            initial_distance = math.hypot(goal[0] - self.pose[0],
                                          goal[1] - self.pose[1])
            initial_map_pose = self.pose
            initial_yaw_error = _wrap(goal[2] - self.pose[2])
            initial_odom = self.odom_pose
            initial_gyro_yaw = self.gyro_yaw
            rotation_only = initial_distance <= self.controller.goal_tolerance
            if rotation_only and not self._rotation_is_clear():
                handle.abort()
                result.message = "rotation footprint blocked or lidar stale"
                return result
            deadline = time.monotonic() + (
                120.0 if rotation_only else
                translation_deadline_seconds(initial_distance))
            progress = GoalProgress(initial_distance,
                                    _wrap(goal[2] - self.pose[2]),
                                    time.monotonic())
            recoveries = 0
            live_replans = ReplanBudget(self.max_live_replans)
            localization_lost_since = None
            while time.monotonic() < deadline:
                if self.estop_latched:
                    self._stop()
                    handle.abort()
                    result.message = "estop latched during navigation"
                    return result
                if not sensor_sample_fresh(
                        time.monotonic(), self.odom_stamp,
                        self.motion_sensor_timeout):
                    self._stop()
                    handle.abort()
                    result.message = "fused odometry lost or stale"
                    return result
                if (not rotation_only
                        and not odometry_translation_within_limit(
                            initial_distance, initial_odom, self.odom_pose,
                            excursion_allowance=
                            self.translation_excursion_allowance)):
                    self._stop()
                    handle.abort()
                    result.message = "physical translation leash exceeded"
                    return result
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = "canceled"
                    return result
                if not self._localization_ok():
                    # The health callback has already published zero velocity.
                    # A single weak scan during pure yaw is not evidence that
                    # the map pose is irrecoverable. Remain stopped for a short,
                    # bounded window and resume only after a fresh healthy
                    # sample; persistent loss still aborts.
                    self._stop()
                    if not rotation_only:
                        handle.abort()
                        result.message = "localization health lost during translation"
                        return result
                    now = time.monotonic()
                    if localization_lost_since is None:
                        localization_lost_since = now
                    if (now - localization_lost_since
                            >= self.localization_recovery_grace):
                        handle.abort()
                        result.message = "localization health lost or stale"
                        return result
                    handle.publish_feedback(self._feedback(
                        goal, "paused: awaiting healthy localization"))
                    time.sleep(0.05)
                    continue
                localization_lost_since = None
                now = time.monotonic()
                distance = math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])
                translation_terminal = (
                    not rotation_only and translation_goal_reached_or_passed(
                        initial_map_pose[:2], self.pose[:2], goal[:2],
                        goal_tolerance=self.controller.goal_tolerance))
                near_goal = (rotation_only or distance <=
                             self.controller.goal_tolerance + 0.03)
                if progress.update(distance, _wrap(goal[2] - self.pose[2]),
                                   near_goal, now):
                    live_replans.reset()
                needs_recovery = progress.stalled(now, 8.0)
                if now - last_plan >= 1.0:
                    # A rotate-only request intentionally holds the current
                    # position.  Localization can drift across a costmap cell
                    # between request construction and acceptance, making the
                    # sampled x/y appear to be an adjacent inflated goal cell.
                    # Pure yaw needs no translation path; keep a non-empty
                    # sentinel so the zero-vx/vy controller branch can run.
                    path = ([self.pose[:2]] if rotation_only
                            else self._plan(goal))
                    last_plan = now
                    if not path:
                        self._stop()
                        handle.abort()
                        result.message = "no collision-free path"
                        self.get_logger().warning(result.message)
                        return result
                if not needs_recovery and path:
                    if rotation_only:
                        if not self._rotation_is_clear():
                            self._stop()
                            handle.abort()
                            result.message = (
                                "rotation footprint blocked or lidar stale")
                            return result
                        wz, done = self.controller.yaw_command(self.pose[2], goal[2])
                        vx = vy = 0.0
                    elif translation_terminal:
                        # The bounded forward leg has reached or safely passed
                        # its along-track terminal band. Hold position and
                        # finish yaw; never rotate around to reverse toward a
                        # point that scan matching placed just behind us.
                        wz, done = self.controller.yaw_command(
                            self.pose[2], goal[2])
                        vx = vy = 0.0
                    else:
                        vx, vy, wz, done = self.controller.command(
                            self.pose, path, goal[2])
                    if done:
                        translation_confirmed = odometry_motion_confirmed(
                            initial_distance, 0.0, initial_odom,
                            self.odom_pose,
                            distance_tolerance=self.controller.goal_tolerance)
                        gyro_fresh = sensor_sample_fresh(
                            time.monotonic(), self.gyro_stamp,
                            self.motion_sensor_timeout)
                        yaw_confirmed = (
                            gyro_fresh and gyro_yaw_motion_confirmed(
                                initial_yaw_error, initial_gyro_yaw,
                                self.gyro_yaw))
                        if not translation_confirmed or not yaw_confirmed:
                            self._stop()
                            handle.abort()
                            result.message = (
                                "goal pose unconfirmed by fused translation "
                                "odometry and physical gyro yaw")
                            return result
                        handle.succeed()
                        result.success, result.message = True, "goal reached"
                        return result
                    if not self._command_is_clear(vx, vy, wz):
                        self._stop()
                        if not live_replans.request():
                            handle.abort()
                            result.message = "blocked after live-costmap replans"
                            self.get_logger().warning(result.message)
                            return result
                        path, last_plan = [], 0.0
                        handle.publish_feedback(
                            self._feedback(goal, "live obstacle: replanning"))
                        self.get_logger().info(
                            "live obstacle stopped command; replanning "
                            f"({live_replans.attempts}/{live_replans.limit})")
                        time.sleep(0.1)
                        continue
                if needs_recovery:
                    if near_goal:
                        self._stop()
                        handle.abort()
                        result.message = "final yaw stalled"
                        return result
                    if not rotation_only:
                        self._stop()
                        handle.abort()
                        result.message = "translation stalled"
                        return result
                    if recoveries >= 2:
                        handle.abort()
                        result.message = "stuck after two recoveries"
                        return result
                    recoveries += 1
                    recovered, reason = self._recover(handle, goal)
                    if not recovered:
                        if reason == "canceled":
                            handle.canceled()
                        else:
                            handle.abort()
                        result.message = reason
                        return result
                    path, last_plan = [], 0.0
                    progress = GoalProgress(
                        math.hypot(goal[0] - self.pose[0],
                                   goal[1] - self.pose[1]),
                        _wrap(goal[2] - self.pose[2]), time.monotonic())
                    continue
                command = Twist()
                command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
                self.pub.publish(command)
                handle.publish_feedback(self._feedback(goal, "tracking"))
                time.sleep(0.1)
            handle.abort()
            result.message = "navigation timeout"
            return result
        finally:
            self._stop()
            self.goal_active = False


def main():
    rclpy.init()
    node = NavNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
