"""Plan or execute a bounded, logged HWR navigation soak.

Dry-run is the default. Execution never resets estop; the human must explicitly
enable motion first. Every exit path attempts to re-latch estop.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from hwr_msgs.action import NavigateToPose
from hwr_msgs.msg import LocalizationHealth
from hwr_core.navigation import (assessment_from_visual_observation,
                                 metric_translation_preflight,
                                 rank_goal_candidates_by_clearance,
                                 reachable_goal_candidates,
                                 semantic_candidate_score,
                                 semantic_context_for_prompt)
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


SCHEMA_REPAIR_PROMPT = (
    "Your previous response did not satisfy the required JSON contract. "
    "Return only a corrected JSON object containing all 13 mandatory keys: "
    "visibility_sufficient, doorway_visible, person_visible, "
    "person_in_corridor, hazards, visual_corridor, "
    "glass_boundary_visible, glass_boundary_in_corridor, "
    "glass_boundary_evidence, recommendation, rationale, "
    "doorway_evidence, person_evidence. Never omit rationale or any other "
    "key. Preserve the observed safety facts; do not turn an uncertain or "
    "blocked scene into a clear scene merely to satisfy the schema."
)


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class SoakNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_navigation_soak")
        self.pose: tuple[float, float, float] | None = None
        self.pose_stamp = 0.0
        self.healthy = False
        self.health_stamp = 0.0
        self.grid: OccupancyGrid | None = None
        self.grid_stamp = 0.0
        self.estop_seen = False
        self.estop_latched = True
        self.reverse_samples = 0
        self.max_reverse = 0.0
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.estop = self.create_client(SetBool, "estop")
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._pose, 10)
        self.create_subscription(LocalizationHealth, "localization_health",
                                 self._health, 10)
        self.create_subscription(OccupancyGrid, "costmap", self._grid, 1)
        state_qos = QoSProfile(depth=1,
                               reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "estop_state", self._estop, state_qos)
        self.create_subscription(TwistStamped, "cmd_vel_applied",
                                 self._applied, 10)

    def _pose(self, message: PoseWithCovarianceStamped) -> None:
        p = message.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw(p.orientation))
        self.pose_stamp = time.monotonic()

    def _health(self, message: LocalizationHealth) -> None:
        self.healthy = bool(message.healthy)
        self.health_stamp = time.monotonic()

    def _grid(self, message: OccupancyGrid) -> None:
        if message.data and message.info.resolution > 0.0:
            self.grid = message
            self.grid_stamp = time.monotonic()

    def _estop(self, message: Bool) -> None:
        self.estop_seen = True
        self.estop_latched = bool(message.data)

    def _applied(self, message: TwistStamped) -> None:
        reverse = max(0.0, -float(message.twist.linear.x))
        self.max_reverse = max(self.max_reverse, reverse)
        if reverse > 1e-4:
            self.reverse_samples += 1

    def ready(self, *, require_enabled: bool) -> tuple[bool, str]:
        now = time.monotonic()
        if self.pose is None or now - self.pose_stamp > 1.0:
            return False, "localized pose missing or stale"
        if not self.healthy or now - self.health_stamp > 1.0:
            return False, "localization unhealthy or stale"
        if self.grid is None or now - self.grid_stamp > 2.0:
            return False, "costmap missing or stale"
        if not self.estop_seen:
            return False, "estop state unavailable"
        if require_enabled and self.estop_latched:
            return False, "estop is latched; this runner never resets it"
        if self.reverse_samples:
            return False, "negative forward command observed"
        return True, "ready"

    def spin_until_ready(self, timeout: float, *, require_enabled: bool) -> None:
        deadline = time.monotonic() + timeout
        reason = "state unavailable"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready, reason = self.ready(require_enabled=require_enabled)
            if ready:
                return
        raise RuntimeError(reason)

    def internal_grid(self) -> tuple[np.ndarray, float, tuple[float, float]]:
        message = self.grid
        values = np.asarray(message.data, dtype=float).reshape(
            message.info.height, message.info.width)
        internal = np.clip(np.rint(values * 2.55), 0, 255).astype(np.uint8)
        origin = (message.info.origin.position.x, message.info.origin.position.y)
        return internal, float(message.info.resolution), origin

    def trip_estop(self) -> bool:
        if not self.estop.wait_for_service(timeout_sec=2.0):
            return False
        future = self.estop.call_async(SetBool.Request(data=True))
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(future.done() and future.result().success)


class Journal:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields) -> None:
        record = {"time": time.time(), "event": event, **fields}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")


class SemanticPreflight:
    """Battery-free gateway snapshot/context judgment through a local VLM."""

    def __init__(self, gateway_url: str, token: str, vlm_url: str, model: str,
                 *, timeout: float = 60.0, margin: float = 0.20,
                 retries: int = 1):
        if not token:
            raise ValueError("HWR_GATEWAY_TOKEN is required for semantic preflight")
        self.gateway_url = gateway_url.rstrip("/")
        self.headers = {"Authorization": f"Bearer {token}"}
        self.vlm_url = vlm_url.rstrip("/")
        self.model = model
        self.timeout = timeout
        self.margin = margin
        self.retries = retries
        self.last_attempts = 0

    @staticmethod
    def _check(response: requests.Response) -> None:
        if not response.ok:
            raise RuntimeError(
                f"semantic preflight HTTP {response.status_code}: {response.text}")

    def _get(self, path: str, *, binary: bool = False):
        response = requests.get(self.gateway_url + path, headers=self.headers,
                                timeout=min(self.timeout, 15.0))
        self._check(response)
        return response.content if binary else response.json()

    def assess(self, distance: float, *, margin: float | None = None
               ) -> tuple[bool, dict]:
        context = self._get("/navigation/context")
        image = self._get("/snapshot", binary=True)
        prompt = (
            "You are the fail-closed advisory vision preflight for an attended "
            "indoor robot. The camera is aligned with a proposed short forward "
            f"translation of {distance:.2f} metres. Inspect the entire near-field "
            "floor and the robot-height volume above it. Report people, feet, "
            "wires/cables, chairs, thin poles, rack or furniture bases/legs, hanging "
            "fabric, and other overhangs. Metric sensor context remains "
            "authoritative and cannot be overruled. Do not identify people. "
            "Small dark rounded shapes permanently visible at the bottom edge "
            "are the robot's own gripper/base hardware, never human feet. "
            "Judge the visible forward corridor and a conservative swept envelope "
            "wider than the chassis, including the camera/arm overhang and any "
            "lateral correction. A visible person or chair entirely outside that "
            "envelope must still be reported by person_visible when applicable, "
            "but must not be put in hazards or person_in_corridor and does not by "
            "itself require hold. In particular, a seated person and chair wholly "
            "at the far left edge of a clear forward floor lane are outside the "
            "corridor. "
            "Treat windows, glazed doors, mirrors, and reflective planes as "
            "optical boundaries, not open passages. Set glass_boundary_visible "
            "true whenever one is visible and glass_boundary_in_corridor true "
            "when the proposed corridor terminates at, points through, or is "
            "dominated by it; that condition requires hold. "
            "Return only one JSON object with keys visibility_sufficient (boolean), "
            "doorway_visible (boolean), person_visible (boolean), "
            "person_in_corridor (boolean), hazards (array containing only "
            "corridor-intersecting hazards), visual_corridor (string), "
            "glass_boundary_visible (boolean), "
            "glass_boundary_in_corridor (boolean), "
            "glass_boundary_evidence (short string or empty), recommendation "
            "(one of hold, sensor_preflight, human_reposition), rationale "
            "(short string), doorway_evidence (short string or empty), and "
            "person_evidence (short string or empty). Use sensor_preflight only "
            "when the complete proposed near-field segment appears clear. Use "
            "hold for any observed or uncertain concern in the swept envelope. "
            "A person requires hold only when the person intersects or may enter "
            "that envelope. This is a visual classification, "
            "not final motion authorization: a separate metric layer evaluates "
            "exact chassis geometry and the immediate camera-occluded footprint. "
            "Do not demand a drawn overlay or additional unspecified context. "
            "visibility_sufficient asks only whether this image is clear enough to "
            "report visible near-field hazards; it does not authorize movement. "
            "When hazards is empty, no person is in the corridor, visual_corridor is clear, "
            "and you state no actual visibility uncertainty, set visibility_sufficient "
            "true and recommendation sensor_preflight. A hold must name its concrete "
            "observed or visibility "
            "concern in hazards, visual_corridor, or rationale. The fact that you "
            "do not command motion is not itself a reason to hold. "
            "A null hard-clearance value is explicit censored clear space out "
            "to clearance_limit_m, not absent telemetry; freshness is stated "
            "by the dedicated freshness fields. If the image is visually clear "
            "and those fields are fresh, use sensor_preflight rather than hold. "
            f"Battery-free navigation context: {json.dumps(semantic_context_for_prompt(context), separators=(',', ':'))}. "
            "All 13 requested keys are mandatory. Never omit rationale or any "
            "other key."
        )
        payload = {"model": self.model, "temperature": 0.1, "messages": [{
            "role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url":
                    "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")}},
            ]}]}
        self.last_attempts = 0

        def post_vlm(active_payload: dict) -> requests.Response:
            response = None
            last_error = None
            for attempt in range(self.retries + 1):
                self.last_attempts += 1
                try:
                    response = requests.post(
                        self.vlm_url + "/chat/completions", json=active_payload,
                        timeout=self.timeout)
                    break
                except requests.RequestException as exc:
                    last_error = exc
                    if attempt < self.retries:
                        time.sleep(0.25)
            if response is None:
                raise RuntimeError(
                    f"VLM unavailable after {self.last_attempts} attempts: "
                    f"{last_error}") from last_error
            self._check(response)
            return response

        def parse(response: requests.Response) -> tuple[dict, str]:
            raw = response.json()["choices"][0]["message"]["content"].strip()
            if raw.startswith("```"):
                raw = raw.split("\n", 1)[-1]
                raw = raw.rsplit("```", 1)[0].strip()
            return assessment_from_visual_observation(json.loads(raw)), raw

        response = post_vlm(payload)
        first_raw = response.json()["choices"][0]["message"]["content"].strip()
        try:
            assessment, raw = parse(response)
        except (json.JSONDecodeError, ValueError) as first_error:
            repair_payload = dict(payload)
            repair_payload["messages"] = [
                *payload["messages"],
                {"role": "assistant", "content": first_raw},
                {"role": "user", "content": SCHEMA_REPAIR_PROMPT},
            ]
            response = post_vlm(repair_payload)
            try:
                assessment, raw = parse(response)
            except (json.JSONDecodeError, ValueError) as exc:
                failed_raw = response.json()["choices"][0]["message"][
                    "content"].strip()
                raise ValueError(
                    "invalid VLM visual assessment after one schema repair: "
                    f"{exc}; first_error={first_error}; "
                    f"raw={failed_raw[:1200]}") from exc
        effective_margin = self.margin if margin is None else margin
        metric_ok, metric_reasons = metric_translation_preflight(
            context, distance, margin=effective_margin)
        semantic_ok = bool(assessment.get("safe_to_translate"))
        semantic_ok &= assessment.get("recommendation") == "sensor_preflight"
        reasons = list(metric_reasons)
        if not semantic_ok:
            reasons.extend(assessment.get("consistency_veto", []))
            if not assessment.get("consistency_veto"):
                reasons.append("semantic assessment rejected translation")
        report = {"assessment": assessment, "navigation_context": context,
                  "distance": distance, "metric_margin": effective_margin,
                  "vlm_attempts": self.last_attempts,
                  "metric_allowed": metric_ok, "reasons": sorted(set(reasons))}
        return semantic_ok and metric_ok, report


def _wait_future(node: Node, future, deadline: float) -> bool:
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def _goal_request(x: float, y: float, yaw: float, node: Node):
    request = NavigateToPose.Goal()
    request.target.header.frame_id = "map"
    request.target.header.stamp = node.get_clock().now().to_msg()
    request.target.pose.position.x = x
    request.target.pose.position.y = y
    request.target.pose.orientation.z = math.sin(yaw / 2.0)
    request.target.pose.orientation.w = math.cos(yaw / 2.0)
    return request


def _execute_goal(node: SoakNode, x: float, y: float, yaw: float,
                  timeout: float) -> tuple[bool, int, str]:
    deadline = time.monotonic() + timeout
    sent = node.client.send_goal_async(_goal_request(x, y, yaw, node))
    if not _wait_future(node, sent, min(deadline, time.monotonic() + 10.0)):
        raise RuntimeError("timed out sending goal")
    handle = sent.result()
    if not handle.accepted:
        return False, 0, "goal rejected"
    result_future = handle.get_result_async()
    abort_reason = None
    while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready, reason = node.ready(require_enabled=True)
        if not ready:
            abort_reason = reason
            break
    if abort_reason or not result_future.done():
        canceled = handle.cancel_goal_async()
        _wait_future(node, canceled, time.monotonic() + 2.0)
        raise RuntimeError(abort_reason or "goal timed out")
    wrapped = result_future.result()
    result = wrapped.result
    return bool(result.success), int(wrapped.status), result.message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true",
                        help="send goals; default is candidate-only dry-run")
    parser.add_argument("--goals", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20260723)
    parser.add_argument("--min-leg", type=float, default=0.05)
    parser.add_argument("--max-leg", type=float, default=0.10)
    parser.add_argument("--goal-timeout", type=float, default=15.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--semantic-preflight", action="store_true",
                        help="rotate, inspect with camera/VLM, then translate")
    parser.add_argument("--candidate-limit", type=int, default=10)
    parser.add_argument("--semantic-margin", type=float, default=0.20)
    parser.add_argument("--gateway-url", default=os.environ.get(
        "HWR_GATEWAY_URL", "http://127.0.0.1:8090"))
    parser.add_argument("--vlm-url", default=os.environ.get(
        "LM_STUDIO_URL", "http://127.0.0.1:1234/v1"))
    parser.add_argument("--vlm-model", default=os.environ.get(
        "HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
    parser.add_argument("--semantic-timeout", type=float, default=60.0)
    parser.add_argument("--semantic-retries", type=int, default=1)
    args = parser.parse_args()
    if not 1 <= args.goals <= 20:
        parser.error("--goals must be 1..20")
    if not 0.05 <= args.min_leg <= args.max_leg <= 0.15:
        parser.error("require 0.05 <= min-leg <= max-leg <= 0.15")
    if not 5.0 <= args.goal_timeout <= 180.0:
        parser.error("--goal-timeout must be 5..180 seconds")
    if args.execute and not args.semantic_preflight:
        parser.error("--execute requires --semantic-preflight")
    if not 1 <= args.candidate_limit <= 50:
        parser.error("--candidate-limit must be 1..50")
    if not 0.0 <= args.semantic_margin <= 0.50:
        parser.error("--semantic-margin must be 0..0.50 metres")
    if not 0 <= args.semantic_retries <= 3:
        parser.error("--semantic-retries must be 0..3")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = args.log or Path(f"state/navigation_soak_{stamp}.jsonl")
    journal = Journal(log_path)
    rclpy.init()
    node = SoakNode()
    visited: list[tuple[float, float]] = []
    succeeded = failed = 0
    semantic = None
    try:
        node.spin_until_ready(args.ready_timeout, require_enabled=args.execute)
        grid, resolution, origin = node.internal_grid()
        preview = reachable_goal_candidates(
            grid, resolution, origin, node.pose[:2], count=args.goals,
            min_distance=args.min_leg, max_distance=args.max_leg,
            min_separation=0.10, seed=args.seed,
            allow_trapped_start=True)
        if not args.execute:
            output = [{"index": index + 1, "x": x, "y": y,
                       "yaw_deg": math.degrees(yaw)}
                      for index, (x, y, yaw) in enumerate(preview)]
            print(json.dumps({"dry_run": True, "pose": node.pose,
                              "candidate_count": len(output),
                              "candidates": output}, indent=2))
            return 0 if len(output) == args.goals else 2
        if not node.client.wait_for_server(timeout_sec=args.ready_timeout):
            raise RuntimeError("navigation action server unavailable")
        semantic = SemanticPreflight(
            args.gateway_url, os.environ.get("HWR_GATEWAY_TOKEN", ""),
            args.vlm_url, args.vlm_model, timeout=args.semantic_timeout,
            margin=args.semantic_margin, retries=args.semantic_retries)
        journal.write("start", goals=args.goals, seed=args.seed,
                      min_leg=args.min_leg, max_leg=args.max_leg,
                      semantic_preflight=True, start_pose=node.pose)
        for index in range(args.goals):
            ready, reason = node.ready(require_enabled=True)
            if not ready:
                raise RuntimeError(reason)
            grid, resolution, origin = node.internal_grid()
            chosen = None
            rotation_blocked = False
            search_bands = [
                ("normal", args.min_leg, args.max_leg, args.semantic_margin),
            ]
            if args.min_leg > 0.20:
                escape_max = min(0.34, args.min_leg - 0.01)
                if escape_max >= 0.20:
                    search_bands.append((
                        "escape", 0.20, escape_max,
                        min(args.semantic_margin, 0.10)))
            candidate_number = 0
            for search_index, (search_mode, min_leg, max_leg,
                               metric_margin) in enumerate(search_bands):
                choices = reachable_goal_candidates(
                    grid, resolution, origin, node.pose[:2],
                    count=min(50, args.candidate_limit * 4),
                    min_distance=min_leg, max_distance=max_leg,
                    min_separation=0.10,
                    seed=args.seed + index + search_index * 10000,
                    allow_trapped_start=True,
                    excluded_xy=tuple(visited))
                choices = rank_goal_candidates_by_clearance(
                    grid, resolution, origin, choices)[:args.candidate_limit]
                allowed_in_band = []
                for x, y, _ in choices:
                    candidate_number += 1
                    start_x, start_y, _ = node.pose
                    distance = math.hypot(x - start_x, y - start_y)
                    bearing = math.atan2(y - start_y, x - start_x)
                    journal.write("candidate", index=index + 1,
                                  candidate=candidate_number,
                                  search_mode=search_mode,
                                  start=node.pose, target=[x, y, bearing],
                                  distance=distance,
                                  metric_margin=metric_margin)
                    rotation_allowed, rotation_report = semantic.assess(
                        min(0.10, distance), margin=metric_margin)
                    journal.write(
                        "rotation_preflight", index=index + 1,
                        candidate=candidate_number, search_mode=search_mode,
                        allowed=rotation_allowed, report=rotation_report)
                    if not rotation_allowed:
                        rotation_blocked = True
                        break
                    try:
                        rotate_ok, rotate_status, rotate_message = _execute_goal(
                            node, start_x, start_y, bearing,
                            args.goal_timeout)
                    except RuntimeError as exc:
                        journal.write(
                            "candidate_rejected", index=index + 1,
                            candidate=candidate_number,
                            search_mode=search_mode,
                            reason="rotate-only goal interrupted",
                            message=str(exc))
                        node.spin_until_ready(
                            args.ready_timeout, require_enabled=True)
                        continue
                    if not rotate_ok:
                        journal.write(
                            "candidate_rejected", index=index + 1,
                            candidate=candidate_number,
                            search_mode=search_mode,
                            reason="rotate-only goal failed",
                            status=rotate_status, message=rotate_message)
                        continue
                    settle = time.monotonic() + 0.75
                    while rclpy.ok() and time.monotonic() < settle:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    aligned_x, aligned_y, _ = node.pose
                    aligned_distance = math.hypot(x - aligned_x, y - aligned_y)
                    allowed, report = semantic.assess(
                        aligned_distance, margin=metric_margin)
                    journal.write(
                        "semantic_preflight", index=index + 1,
                        candidate=candidate_number, search_mode=search_mode,
                        allowed=allowed, target=[x, y, bearing], report=report)
                    if not allowed:
                        print(
                            f"goal {index + 1}: {search_mode} candidate "
                            f"{candidate_number} rejected - "
                            f"{', '.join(report['reasons'])}", flush=True)
                        rotation_blocked = True
                        break
                    score = semantic_candidate_score(
                        report["assessment"], report["navigation_context"],
                        aligned_distance)
                    allowed_in_band.append((
                        score, x, y, search_mode, metric_margin,
                        candidate_number))
                    journal.write(
                        "candidate_provisionally_allowed", index=index + 1,
                        candidate=candidate_number, search_mode=search_mode,
                        score=score, target=[x, y, bearing])
                    print(f"goal {index + 1}: {search_mode} candidate "
                          f"{candidate_number} provisionally allowed "
                          f"(score {score:.3f})", flush=True)
                if rotation_blocked:
                    break
                for (score, x, y, candidate_mode, candidate_margin,
                     selected_number) in sorted(allowed_in_band, reverse=True):
                    aligned_x, aligned_y, _ = node.pose
                    distance = math.hypot(x - aligned_x, y - aligned_y)
                    bearing = math.atan2(y - aligned_y, x - aligned_x)
                    rotation_allowed, rotation_report = semantic.assess(
                        min(0.10, distance), margin=candidate_margin)
                    journal.write(
                        "rotation_revalidation", index=index + 1,
                        candidate=selected_number,
                        search_mode=candidate_mode,
                        allowed=rotation_allowed, report=rotation_report)
                    if not rotation_allowed:
                        rotation_blocked = True
                        break
                    try:
                        rotate_ok, rotate_status, rotate_message = _execute_goal(
                            node, aligned_x, aligned_y, bearing,
                            args.goal_timeout)
                    except RuntimeError as exc:
                        journal.write(
                            "candidate_revalidation", index=index + 1,
                            candidate=selected_number,
                            search_mode=candidate_mode, allowed=False,
                            reason="final alignment interrupted",
                            message=str(exc))
                        node.spin_until_ready(
                            args.ready_timeout, require_enabled=True)
                        continue
                    if not rotate_ok:
                        journal.write(
                            "candidate_revalidation", index=index + 1,
                            candidate=selected_number,
                            search_mode=candidate_mode, allowed=False,
                            reason="final alignment failed",
                            status=rotate_status, message=rotate_message)
                        continue
                    settle = time.monotonic() + 0.75
                    while rclpy.ok() and time.monotonic() < settle:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    final_x, final_y, _ = node.pose
                    final_distance = math.hypot(x - final_x, y - final_y)
                    allowed, report = semantic.assess(
                        final_distance, margin=candidate_margin)
                    journal.write(
                        "candidate_revalidation", index=index + 1,
                        candidate=selected_number,
                        search_mode=candidate_mode, allowed=allowed,
                        score=score, target=[x, y, bearing], report=report)
                    if not allowed:
                        print(f"goal {index + 1}: candidate "
                              f"{selected_number} failed final revalidation",
                              flush=True)
                        rotation_blocked = True
                        break
                    chosen = (x, y, bearing, candidate_mode)
                    break
                if chosen is not None:
                    break
                if rotation_blocked:
                    break
            if chosen is None:
                message = (
                    "no semantically and metrically safe candidate in "
                    "bounded search")
                failed += 1
                journal.write(
                    "result", index=index + 1, success=False, status=6,
                    message=message, final_pose=node.pose,
                    safe_refusal=True)
                print(f"goal {index + 1}/{args.goals}: SAFE REFUSAL - "
                      f"{message}", flush=True)
                continue
            x, y, yaw, search_mode = chosen
            start_pose = node.pose
            journal.write("goal", index=index + 1, start=start_pose,
                          target=[x, y, yaw], search_mode=search_mode)
            ok, status, message = _execute_goal(
                node, x, y, yaw, args.goal_timeout)
            succeeded += int(ok)
            failed += int(not ok)
            journal.write("result", index=index + 1, success=ok,
                          status=status, message=message,
                          final_pose=node.pose)
            if ok:
                visited.append((x, y))
            print(f"goal {index + 1}/{args.goals}: "
                  f"{'PASS' if ok else 'FAIL'} - {message}", flush=True)
            if not ok:
                break
            settle = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.1)
        passed = succeeded >= math.ceil(0.9 * args.goals)
        journal.write("summary", succeeded=succeeded, failed=failed,
                      reverse_samples=node.reverse_samples,
                      max_reverse=node.max_reverse, passed=passed)
        print(json.dumps({"succeeded": succeeded, "failed": failed,
                          "reverse_samples": node.reverse_samples,
                          "max_reverse": node.max_reverse, "passed": passed,
                          "log": str(log_path)}, indent=2))
        return 0 if passed else 3
    except Exception as exc:
        journal.write("abort", error=str(exc), succeeded=succeeded, failed=failed,
                      reverse_samples=node.reverse_samples,
                      max_reverse=node.max_reverse)
        print(f"ABORT: {exc}")
        return 4
    finally:
        if args.execute:
            node.trip_estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
