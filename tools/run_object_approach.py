"""Ground an object and rank or execute close, staged approach poses.

Dry-run is the default. Execution is explicit, VLM-grounded, short-legged,
semantically preflighted, and stop-latched after every action. This tool never
reads battery state.
"""
from __future__ import annotations

import argparse
import base64
import cv2
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import time

import numpy as np
import rclpy
import requests
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from hwr_core.navigation import (metric_rotation_preflight,
                                 object_approach_candidates,
                                 point_to_segment_distance,
                                 project_depth_pixel, staged_path_waypoint,
                                 staged_yaw_waypoints)
from hwr_msgs.msg import LocalizationHealth
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from hwr_sensors.depth_to_ranges_node import _depth_metres
from run_nav_soak import SemanticPreflight


def _yaw(q) -> float:
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def _matrix(transform) -> np.ndarray:
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4)
    out[:3, :3] = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    out[:3, 3] = (t.x, t.y, t.z)
    return out


def _json_content(raw: str) -> dict:
    raw = raw.strip()
    if raw.startswith("```"):
        raw = raw.split("\n", 1)[-1].rsplit("```", 1)[0].strip()
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("VLM response must be a JSON object")
    return value


def _normalized_point(value, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field} must be [x, y]")
    point = float(value[0]), float(value[1])
    if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in point):
        raise ValueError(f"{field} must contain normalized finite coordinates")
    return point


def _vlm_request(vlm_url: str, model: str, prompt: str, image: bytes,
                 timeout: float) -> dict:
    payload = {"model": model, "temperature": 0.0, "messages": [{
        "role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url":
                "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")}},
        ]}]}
    response = requests.post(vlm_url.rstrip("/") + "/chat/completions",
                             json=payload, timeout=timeout)
    response.raise_for_status()
    return _json_content(response.json()["choices"][0]["message"]["content"])


def _marked_image(image: bytes, point: tuple[float, float]) -> bytes:
    frame = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("snapshot is not a decodable image")
    x = int(round(point[0] * (frame.shape[1] - 1)))
    y = int(round(point[1] * (frame.shape[0] - 1)))
    radius = max(10, min(frame.shape[:2]) // 30)
    # Keep the exact target pixels visible. A cross through a thin colored foot
    # can cover the evidence the audit model needs to distinguish it from floor.
    cv2.circle(frame, (x, y), radius, (0, 0, 255), 3)
    cv2.arrowedLine(frame, (x + 2 * radius, y - 2 * radius),
                    (x + radius // 2, y - radius // 2),
                    (0, 0, 255), 3, tipLength=0.35)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError("failed to encode marked grounding image")
    return encoded.tobytes()


def _marked_crop_image(image: bytes, point: tuple[float, float]) -> bytes:
    """Enlarge the neighborhood while retaining the exact proposed pixel."""
    frame = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("snapshot is not a decodable image")
    height, width = frame.shape[:2]
    x = int(round(point[0] * (width - 1)))
    y = int(round(point[1] * (height - 1)))
    half_width, half_height = max(40, width // 8), max(30, height // 6)
    x0, x1 = max(0, x - half_width), min(width, x + half_width)
    y0, y1 = max(0, y - half_height), min(height, y + half_height)
    crop = frame[y0:y1, x0:x1]
    local = ((x - x0) / max(x1 - x0 - 1, 1),
             (y - y0) / max(y1 - y0 - 1, 1))
    enlarged = cv2.resize(crop, (640, 480), interpolation=cv2.INTER_CUBIC)
    ok, encoded = cv2.imencode(".jpg", enlarged,
                               [cv2.IMWRITE_JPEG_QUALITY, 94])
    if not ok:
        raise RuntimeError("failed to encode grounding audit crop")
    return _marked_image(encoded.tobytes(), local)


def _grid_relocate(description: str, snapshot: bytes, vlm_url: str,
                   model: str, timeout: float) -> tuple[tuple[float, float], list[dict]]:
    """Use a labeled coarse grid and enlarged crop to recover bad coordinates."""
    frame = cv2.imdecode(np.frombuffer(snapshot, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("snapshot is not a decodable image")
    rows, columns = 6, 8
    height, width = frame.shape[:2]
    marked = frame.copy()
    for column in range(1, columns):
        x = round(column * width / columns)
        cv2.line(marked, (x, 0), (x, height - 1), (255, 255, 0), 1)
    for row in range(1, rows):
        y = round(row * height / rows)
        cv2.line(marked, (0, y), (width - 1, y), (255, 255, 0), 1)
    for row in range(rows):
        for column in range(columns):
            label = f"{chr(65 + column)}{row + 1}"
            cv2.putText(marked, label,
                        (round(column * width / columns) + 3,
                         round(row * height / rows) + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1,
                        cv2.LINE_AA)
    ok, encoded = cv2.imencode(".jpg", marked,
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError("failed to encode grounding grid")
    grid_prompt = (
        f"Locate the specifically requested physical material {description!r} "
        "in this labeled 8-column (A-H), 6-row (1-6) grid. Choose the one cell "
        "containing the clearest interior part of that material, not a connected "
        "structure or nearby object. Return only JSON with found (boolean), "
        "confidence (0..1), grid_cell (such as C5, or null), and rationale."
    )
    grid_result = _vlm_request(vlm_url, model, grid_prompt,
                               encoded.tobytes(), timeout)
    grid_result["grid_relocation"] = True
    match = re.fullmatch(r"([A-Ha-h])([1-6])",
                         str(grid_result.get("grid_cell", "")).strip())
    if (grid_result.get("found") is not True
            or float(grid_result.get("confidence", 0.0)) < 0.60 or match is None):
        raise RuntimeError("grid relocation failed: " + json.dumps(
            grid_result, separators=(",", ":")))
    column = ord(match.group(1).upper()) - ord("A")
    row = int(match.group(2)) - 1
    x0, x1 = round(column * width / columns), round((column + 1) * width / columns)
    y0, y1 = round(row * height / rows), round((row + 1) * height / rows)
    crop = frame[y0:y1, x0:x1]
    crop = cv2.resize(crop, (640, 480), interpolation=cv2.INTER_CUBIC)
    ok, encoded_crop = cv2.imencode(".jpg", crop,
                                    [cv2.IMWRITE_JPEG_QUALITY, 94])
    if not ok:
        raise RuntimeError("failed to encode grounding crop")
    crop_prompt = (
        f"This is an enlarged grid cell selected for target {description!r}. "
        "Return only JSON with found (boolean), confidence (0..1), target_point "
        "(normalized [x,y] well inside the specifically requested material in "
        "this crop, or null), and rationale. Reject if the requested material "
        "is not actually visible in the crop."
    )
    crop_result = _vlm_request(vlm_url, model, crop_prompt,
                               encoded_crop.tobytes(), timeout)
    crop_result["crop_refinement"] = True
    if (crop_result.get("found") is not True
            or float(crop_result.get("confidence", 0.0)) < 0.60):
        raise RuntimeError("crop refinement failed: " + json.dumps(
            crop_result, separators=(",", ":")))
    local = _normalized_point(crop_result.get("target_point"), "target_point")
    point = ((column + local[0]) / columns, (row + local[1]) / rows)
    crop_result["full_image_point"] = list(point)
    return point, [grid_result, crop_result]


def _audit_grounding_point(description: str, point: tuple[float, float],
                           snapshot: bytes, vlm_url: str, model: str,
                           timeout: float) -> tuple[tuple[float, float], list[dict]]:
    """Verify a visible marker is on target, allowing one audited correction."""
    audits = []
    seed_point = point
    for attempt in range(2):
        prompt = (
            "Audit a proposed object-grounding pixel. The center of the red "
            f"circle marks the exact proposed point for target {description!r}. The point "
            "must lie on the specifically requested physical material, not merely "
            "on a connected pole, frame, nearby object, floor, edge, gap, or "
            "background. A pole, frame, connector, or base is valid only when "
            "that exact component is explicitly named in the target description. "
            "Return only JSON with verified (boolean), confidence "
            "(0..1), rationale (short string), and corrected_point (a normalized "
            "[x,y] point well inside the requested material, or null)."
        )
        audit = _vlm_request(vlm_url, model, prompt,
                             _marked_image(snapshot, point), timeout)
        confidence = float(audit.get("confidence", 0.0))
        audit["confidence"] = confidence
        audit["audited_point"] = list(point)
        audits.append(audit)
        if audit.get("verified") is True and confidence >= 0.70:
            return point, audits
        close_prompt = (
            "Audit the same proposed grounding point in this enlarged crop. "
            "The center of the red circle is the exact pixel. Determine only "
            f"whether that center pixel lies on the physical material of "
            f"target {description!r}. Return only JSON with verified (boolean), "
            "confidence (0..1), rationale (short string), and corrected_point "
            "(null; crop coordinates are not accepted)."
        )
        close_audit = _vlm_request(
            vlm_url, model, close_prompt,
            _marked_crop_image(snapshot, point), timeout)
        close_confidence = float(close_audit.get("confidence", 0.0))
        close_audit["confidence"] = close_confidence
        close_audit["audited_point"] = list(point)
        close_audit["zoomed_audit"] = True
        audits.append(close_audit)
        if (close_audit.get("verified") is True
                and close_confidence >= 0.70):
            return point, audits
        correction = audit.get("corrected_point")
        if attempt == 0:
            if correction is None:
                try:
                    point, relocation = _grid_relocate(
                        description, snapshot, vlm_url, model, timeout)
                except RuntimeError as exc:
                    audits.append({"grid_relocation_error": str(exc)})
                    break
                audits.extend(relocation)
                continue
            if correction is not None:
                point = _normalized_point(correction, "corrected_point")
                continue
        break
    # A model can identify the correct object but repeatedly select a small
    # center feature or rim instead of the surrounding physical material. Try
    # a bounded ring around the original point; every candidate is independently
    # marker-audited, so this cannot silently drift onto a stand or background.
    for dx, dy in ((0.015, 0.05), (0.02, 0.04), (-0.02, 0.04),
                   (0.02, 0.06), (-0.04, 0.04), (0.04, 0.04),
                   (-0.05, 0.0), (0.05, 0.0), (0.0, 0.05),
                   (0.0, -0.05)):
        candidate = (min(1.0, max(0.0, seed_point[0] + dx)),
                     min(1.0, max(0.0, seed_point[1] + dy)))
        audit = _vlm_request(
            vlm_url, model, prompt, _marked_image(snapshot, candidate), timeout)
        confidence = float(audit.get("confidence", 0.0))
        audit.update({"confidence": confidence,
                      "audited_point": list(candidate),
                      "nearby_material_search": True})
        audits.append(audit)
        if audit.get("verified") is True and confidence >= 0.70:
            return candidate, audits
        close_audit = _vlm_request(
            vlm_url, model, close_prompt,
            _marked_crop_image(snapshot, candidate), timeout)
        close_confidence = float(close_audit.get("confidence", 0.0))
        close_audit.update({"confidence": close_confidence,
                            "audited_point": list(candidate),
                            "zoomed_audit": True,
                            "nearby_material_search": True})
        audits.append(close_audit)
        if (close_audit.get("verified") is True
                and close_confidence >= 0.70):
            return candidate, audits
    raise RuntimeError(
        "marker audit could not verify a point on the requested object: "
        + json.dumps(audits, separators=(",", ":")))


class LiveInputs(Node):
    def __init__(self) -> None:
        super().__init__("hwr_object_approach")
        self.pose = None
        self.pose_time = 0.0
        self.healthy = False
        self.health_time = 0.0
        self.stable_history = deque(maxlen=200)
        self.grid = None
        self.grid_time = 0.0
        self.depth = None
        self.depth_message = None
        self.info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, "pose", self._pose, 10)
        self.create_subscription(LocalizationHealth, "localization_health",
                                 self._health, 10)
        self.create_subscription(OccupancyGrid, "costmap", self._grid, 1)
        self.create_subscription(CameraInfo, "camera/depth/camera_info",
                                 self._info, qos_profile_sensor_data)
        self.create_subscription(Image, "camera/depth/image_raw", self._depth,
                                 qos_profile_sensor_data)

    def _pose(self, message):
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
        self.pose_time = time.monotonic()
        if self.healthy:
            self.stable_history.append((self.pose_time, self.pose))

    def _health(self, message):
        self.healthy = bool(message.healthy)
        self.health_time = time.monotonic()
        if not self.healthy:
            self.stable_history.clear()

    def _grid(self, message):
        if message.data and message.info.resolution > 0.0:
            self.grid = message
            self.grid_time = time.monotonic()

    def _info(self, message):
        self.info = message

    def _depth(self, message):
        try:
            self.depth = _depth_metres(message)
            self.depth_message = message
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)

    def wait_ready(self, timeout: float, *, require_depth: bool) -> None:
        deadline = time.monotonic() + timeout
        reason = "live inputs unavailable"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.pose is None or now - self.pose_time > 1.0:
                reason = "localized pose missing or stale"
                continue
            if not self.healthy or now - self.health_time > 1.0:
                reason = "localization unhealthy or stale"
                continue
            if self.grid is None or now - self.grid_time > 2.0:
                reason = "costmap missing or stale"
                continue
            if require_depth and (self.depth is None or self.info is None):
                reason = "depth image or calibration unavailable"
                continue
            return
        raise RuntimeError(reason)

    def refresh_depth(self, timeout: float = 3.0) -> None:
        """Capture calibration and a depth frame after the latest camera stop."""
        self.depth = None
        self.depth_message = None
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.depth is not None and self.info is not None:
                return
        raise RuntimeError("fresh depth frame unavailable")

    def wait_stable(self, timeout: float, *, duration: float = 2.0,
                    max_xy_jitter: float = 0.03,
                    max_yaw_jitter: float = math.radians(5.0)) -> None:
        """Require sustained healthy localization within a tight pose window."""
        deadline = time.monotonic() + timeout
        reason = "localization did not remain stable"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            samples = [(stamp, pose) for stamp, pose in self.stable_history
                       if now - stamp <= duration + 0.25]
            if not self.healthy or not samples:
                continue
            if samples[-1][0] - samples[0][0] < duration:
                continue
            anchor = samples[0][1]
            xy_jitter = max(math.hypot(pose[0] - anchor[0],
                                       pose[1] - anchor[1])
                            for _, pose in samples)
            yaw_jitter = max(abs(math.atan2(
                math.sin(pose[2] - anchor[2]),
                math.cos(pose[2] - anchor[2]))) for _, pose in samples)
            if xy_jitter <= max_xy_jitter and yaw_jitter <= max_yaw_jitter:
                return
            reason = (f"localization jitter xy={xy_jitter:.3f} m, "
                      f"yaw={math.degrees(yaw_jitter):.1f} deg")
        raise RuntimeError(reason)

    def internal_grid(self):
        message = self.grid
        values = np.asarray(message.data, dtype=float).reshape(
            message.info.height, message.info.width)
        grid = np.clip(np.rint(values * 2.55), 0, 255).astype(np.uint8)
        origin = (message.info.origin.position.x, message.info.origin.position.y)
        return grid, float(message.info.resolution), origin

    def map_from_depth_camera(self) -> np.ndarray:
        source = self.depth_message.header.frame_id or self.info.header.frame_id
        if not source:
            raise RuntimeError("depth frame id is empty")
        # The synchronous local-VLM request intentionally does not spin this
        # short-lived node. Allow enough bounded time afterward for both TF
        # broadcasters to be rediscovered and deliver a connected tree.
        deadline = time.monotonic() + 12.0
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                # Grounding is performed only while the gateway is stop-
                # latched and localization has remained stable.  A short-lived
                # process can receive a camera frame whose timestamp predates
                # its local TF cache, so use the latest fully connected tree.
                # Applying an unavailable historical transform would fail even
                # though the robot and camera have not moved.
                transform = self.tf_buffer.lookup_transform(
                    "map", source, Time(),
                    timeout=Duration(seconds=0.1))
                return _matrix(transform.transform)
            except TransformException as exc:
                last_error = exc
                try:
                    # Some publishers put the fixed camera extrinsic on /tf
                    # instead of /tf_static. A newly started listener can then
                    # have map->base and base->camera samples with no common
                    # timestamp. Since this caller is stop-latched and has
                    # already required stable localization, compose each edge
                    # at its own latest sample.
                    base_from_source = self.tf_buffer.lookup_transform(
                        "base_link", source, Time(),
                        timeout=Duration(seconds=0.1))
                    if self.pose is None:
                        raise RuntimeError("localized base pose unavailable")
                    x, y, yaw = self.pose
                    cosine, sine = math.cos(yaw), math.sin(yaw)
                    map_from_base = np.eye(4)
                    map_from_base[:3, :3] = np.array([
                        [cosine, -sine, 0.0],
                        [sine, cosine, 0.0],
                        [0.0, 0.0, 1.0],
                    ])
                    map_from_base[:3, 3] = (x, y, 0.0)
                    return (map_from_base
                            @ _matrix(base_from_source.transform))
                except TransformException as split_exc:
                    last_error = split_exc
                # Short-lived grounding processes can have pose/depth before
                # DDS has delivered the TF tree. Actively spin instead of
                # treating discovery latency as missing geometry.
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"map-to-depth transform unavailable after discovery wait: "
            f"{last_error}") from last_error

    def project_normalized(self, point: tuple[float, float],
                           patch_radius: int = 10) -> dict:
        image = self.depth
        u = int(round(point[0] * (image.shape[1] - 1)))
        v = int(round(point[1] * (image.shape[0] - 1)))
        x0, x1 = max(0, u - patch_radius), min(image.shape[1], u + patch_radius + 1)
        y0, y1 = max(0, v - patch_radius), min(image.shape[0], v + patch_radius + 1)
        values = image[y0:y1, x0:x1]
        valid_mask = np.isfinite(values) & (values >= 0.15) & (values <= 3.0)
        rows, columns = np.where(valid_mask)
        if len(rows) < 5:
            raise RuntimeError(
                f"insufficient valid depth around target pixel ({u}, {v})")
        # Thin poles and feet often occupy only a few pixels. Select the valid
        # samples nearest the requested surface point instead of allowing the
        # surrounding floor or background to dominate a rectangular median.
        absolute_u, absolute_v = columns + x0, rows + y0
        order = np.argsort((absolute_u - u) ** 2 + (absolute_v - v) ** 2)
        chosen = order[:min(25, len(order))]
        selected = values[rows[chosen], columns[chosen]]
        sample_u = int(round(float(np.median(absolute_u[chosen]))))
        sample_v = int(round(float(np.median(absolute_v[chosen]))))
        median = float(np.median(selected))
        mad = float(np.median(np.abs(selected - median)))
        k = self.info.k
        world = project_depth_pixel(
            (sample_u, sample_v), median, (k[0], k[4], k[2], k[5]),
            self.map_from_depth_camera())
        return {"normalized": list(point), "requested_pixel": [u, v],
                "depth_pixel": [sample_u, sample_v],
                "depth_m": median, "depth_mad_m": mad,
                "valid_samples": int(len(rows)),
                "selected_samples": int(len(selected)),
                "map_point": list(world)}


def ground_with_vlm(description: str, gateway_url: str, token: str,
                    vlm_url: str, model: str, timeout: float) -> dict:
    if not token:
        raise ValueError("HWR_GATEWAY_TOKEN is required for VLM grounding")
    headers = {"Authorization": f"Bearer {token}"}
    snapshot = requests.get(gateway_url.rstrip("/") + "/snapshot",
                            headers=headers, timeout=min(timeout, 15.0))
    snapshot.raise_for_status()
    prompt = (
        "Ground one physical navigation target in this robot-camera image. "
        f"Target description: {description!r}. Reject lookalikes and ambiguity. "
        "This is grounding only, never a motion decision. Put each surface point "
        "well inside a solid visible part of the target, not on an edge, gap, "
        "reflection, floor, occluder, or background. Use one point for a compact "
        "object. Use two well-separated points for an extended coplanar surface "
        "such as a window or door, only when both points have unambiguous target "
        "surface behind them. Return only JSON with keys identity_match (boolean), "
        "ambiguous (boolean), confidence (0..1), object_label (string), "
        "surface_points (array of one or two normalized [x,y] points), and "
        "rationale (short string)."
    )
    result = _vlm_request(vlm_url, model, prompt, snapshot.content, timeout)
    confidence = float(result.get("confidence", 0.0))
    if (result.get("identity_match") is not True
            or result.get("ambiguous") is not False or confidence < 0.60):
        raise RuntimeError(
            "target grounding rejected: identity must match unambiguously at "
            f"confidence >= 0.60; model={json.dumps(result, separators=(',', ':'))}")
    points = result.get("surface_points")
    if not isinstance(points, list) or not 1 <= len(points) <= 2:
        raise ValueError("surface_points must contain one or two points")
    proposed = [_normalized_point(point, f"surface_points[{index}]")
                for index, point in enumerate(points)]
    audited, audits = [], []
    for point in proposed:
        verified, point_audits = _audit_grounding_point(
            description, point, snapshot.content, vlm_url, model, timeout)
        audited.append(list(verified))
        audits.append(point_audits)
    result["proposed_surface_points"] = [list(point) for point in proposed]
    result["surface_points"] = audited
    result["marker_audits"] = audits
    result["confidence"] = confidence
    return result


def _place_target(name: str, path: Path):
    places = yaml.safe_load(path.read_text(encoding="utf-8"))["places"]
    place = places.get(name)
    if not isinstance(place, dict):
        raise ValueError(f"unknown place {name!r}")
    if place.get("kind") != "object":
        raise ValueError(f"place {name!r} is not an object")
    if place.get("verified") is not True:
        raise ValueError(f"place {name!r} is not physically verified")
    try:
        point = float(place["object_x"]), float(place["object_y"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"place {name!r} has no measured object anchor") from exc
    if not all(math.isfinite(value) for value in point):
        raise ValueError(f"place {name!r} object anchor is not finite")
    end = None
    if place.get("object_end_x") is not None or place.get("object_end_y") is not None:
        try:
            end = float(place["object_end_x"]), float(place["object_end_y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"place {name!r} has an incomplete object segment") from exc
        if not all(math.isfinite(value) for value in end):
            raise ValueError(f"place {name!r} object segment is not finite")
    return point, end, {"source": "place", "place": name,
                        "physically_verified": True}


class Gateway:
    """Authenticated, battery-free navigation gateway client."""

    TERMINAL = {"succeeded", "failed", "rejected", "error"}

    def __init__(self, url: str, token: str, timeout: float = 15.0):
        if not token:
            raise ValueError("HWR_GATEWAY_TOKEN is required")
        self.url = url.rstrip("/")
        self.headers = {"Authorization": f"Bearer {token}"}
        self.timeout = timeout

    @staticmethod
    def _check(response: requests.Response) -> None:
        if not response.ok:
            raise RuntimeError(
                f"gateway HTTP {response.status_code}: {response.text}")

    def context(self) -> dict:
        response = requests.get(self.url + "/navigation/context",
                                headers=self.headers, timeout=self.timeout)
        self._check(response)
        return response.json()

    def post(self, path: str, payload: dict | None = None) -> dict:
        response = requests.post(self.url + path, headers=self.headers,
                                 json=payload, timeout=self.timeout)
        self._check(response)
        return response.json()

    def stop(self) -> None:
        try:
            self.post("/stop")
        except Exception:
            pass

    def execute_goal(self, x: float, y: float, yaw: float,
                     timeout: float) -> dict:
        """Send one goal and always stop afterward."""
        try:
            self.post("/goto", {"x": x, "y": y, "yaw": yaw})
            deadline = time.monotonic() + timeout
            seen = False
            while time.monotonic() < deadline:
                goal = self.context().get("current_goal") or {}
                matches = (abs(float(goal.get("x", math.inf)) - x) <= 0.03
                           and abs(float(goal.get("y", math.inf)) - y) <= 0.03)
                seen = seen or matches
                if seen and goal.get("state") in self.TERMINAL:
                    if goal.get("state") != "succeeded":
                        raise RuntimeError(
                            f"navigation goal {goal.get('state')}: "
                            f"{goal.get('message', '')}")
                    return goal
                time.sleep(0.25)
            raise RuntimeError("navigation goal timed out")
        finally:
            self.stop()


def _ground_target(node: LiveInputs, args, token: str):
    grounding = ground_with_vlm(
        args.ground, args.gateway_url, token, args.vlm_url,
        args.vlm_model, args.vlm_timeout)
    # The robot is stopped while grounding. Capture depth after the visual call
    # so an old pre-rotation frame can never be fused with the current image.
    node.refresh_depth()
    projected = []
    projection_errors = []
    for point in grounding["surface_points"]:
        try:
            projected.append(node.project_normalized(tuple(point)))
        except RuntimeError as exc:
            projection_errors.append({"normalized": list(point),
                                      "error": str(exc)})
    if not projected:
        raise RuntimeError(
            "no audited target point had valid metric depth; "
            f"projection_errors={json.dumps(projection_errors, separators=(',', ':'))}; "
            f"VLM grounding={json.dumps(grounding, separators=(',', ':'))}")
    start = tuple(projected[0]["map_point"][:2])
    end = (tuple(projected[1]["map_point"][:2])
           if len(projected) == 2 else None)
    provenance = {"source": "vlm_depth_tf", "description": args.ground,
                  "grounding": grounding, "projected_points": projected,
                  "projection_errors": projection_errors}
    return start, end, provenance


def _execute_staged(node: LiveInputs, args, token: str) -> dict:
    gateway = Gateway(args.gateway_url, token)
    semantic = SemanticPreflight(
        args.gateway_url, token, args.vlm_url, args.vlm_model,
        timeout=args.vlm_timeout, margin=args.semantic_margin, retries=1)
    events = []
    motion_dispatched = False
    try:
        for stage in range(1, args.max_stages + 1):
            node.wait_ready(args.ready_timeout, require_depth=True)
            node.wait_stable(args.stability_timeout,
                             duration=args.stability_duration)
            if args.ground:
                object_start, object_end, provenance = _ground_target(
                    node, args, token)
            else:
                node.refresh_depth()
                object_start, object_end, provenance = _place_target(
                    args.place, args.places)
            target_end = object_start if object_end is None else object_end
            terminal_distance = point_to_segment_distance(
                node.pose[:2], object_start, target_end)
            event = {"stage": stage, "pose": list(node.pose),
                     "target": {**provenance,
                                "object_start": list(object_start),
                                "object_end": (None if object_end is None
                                               else list(object_end))},
                     "object_distance_m": terminal_distance}
            if terminal_distance <= 0.50:
                # A stored place anchor is navigation metadata, not physical
                # proof that the named object is currently within reach.  Only
                # a target freshly grounded in the live color image and then
                # projected with the corresponding depth/TF may close the
                # 50 cm arrival contract.
                if args.ground:
                    depths = [float(point["depth_m"])
                              for point in provenance["projected_points"]]
                    event["terminal_verified"] = True
                    event["verification_source"] = "live_vlm_depth_tf"
                    event["direct_target_depth_m"] = min(depths)
                    events.append(event)
                    return {
                        "state": "succeeded",
                        "motion_dispatched": motion_dispatched,
                        "object_reached": True,
                        "object_distance_m": terminal_distance,
                        "stages": stage - 1,
                        "events": events,
                    }

                event["terminal_verified"] = False
                event["verification_required"] = (
                    "stored place anchor requires live target grounding")
                events.append(event)
                return {
                    "state": "verification_required",
                    "motion_dispatched": motion_dispatched,
                    "object_reached": False,
                    "object_distance_m": terminal_distance,
                    "stages": stage - 1,
                    "events": events,
                }

            grid, resolution, origin = node.internal_grid()
            candidates = object_approach_candidates(
                grid, resolution, origin, node.pose[:2], object_start,
                object_end, min_distance=args.min_distance,
                max_distance=args.max_distance, limit=args.limit)
            if not candidates:
                raise RuntimeError("no reachable close approach candidate")

            # Rotation also sweeps the chassis and camera/arm assembly.  A
            # stationary full-envelope visual/metric check must pass before we
            # expose any candidate direction; the navigation node separately
            # requires a fresh lidar and a free current footprint.
            rotation_context = gateway.context()
            rotation_allowed, rotation_reasons = metric_rotation_preflight(
                rotation_context)
            event["pre_rotation_preflight"] = {
                "allowed": rotation_allowed,
                "reasons": rotation_reasons,
                "navigation_context": rotation_context,
            }
            if not rotation_allowed:
                events.append(event)
                raise RuntimeError("rotation preflight rejected current scene")

            selected = None
            rejection_reports = []
            for candidate_index, candidate in enumerate(candidates, 1):
                try:
                    waypoint = staged_path_waypoint(
                        grid, resolution, origin, node.pose[:2],
                        (candidate.x, candidate.y), max_leg=args.max_leg)
                except ValueError as exc:
                    rejection_reports.append({"candidate": candidate_index,
                                              "reason": str(exc)})
                    continue
                bearing = math.atan2(waypoint[1] - node.pose[1],
                                     waypoint[0] - node.pose[0])
                # Pure rotation is inspected separately and never implies
                # translation. The gateway permits current-cell rotation even
                # when the start lies in an inflation prefix.
                rotations = []
                for rotation_yaw in staged_yaw_waypoints(
                        node.pose[2], bearing):
                    motion_dispatched = True
                    rotations.append(gateway.execute_goal(
                        node.pose[0], node.pose[1], rotation_yaw,
                        args.goal_timeout))
                    node.wait_ready(args.ready_timeout, require_depth=True)
                    node.wait_stable(args.stability_timeout,
                                     duration=args.stability_duration)
                leg_distance = math.hypot(waypoint[0] - node.pose[0],
                                          waypoint[1] - node.pose[1])
                allowed, report = semantic.assess(
                    leg_distance, margin=args.semantic_margin)
                rejection_reports.append({"candidate": candidate_index,
                                          "waypoint": list(waypoint),
                                          "rotation": rotations,
                                          "semantic": report,
                                          "allowed": allowed})
                if allowed:
                    selected = candidate, waypoint, bearing, candidate_index
                    break
            event["candidate_attempts"] = rejection_reports
            if selected is None:
                events.append(event)
                raise RuntimeError(
                    "all reachable candidates failed metric/semantic preflight")

            candidate, waypoint, bearing, candidate_index = selected
            try:
                motion_dispatched = True
                result = gateway.execute_goal(
                    waypoint[0], waypoint[1], bearing, args.goal_timeout)
            except RuntimeError as exc:
                event.update({"selected_candidate": candidate_index,
                              "selected": candidate.as_dict(),
                              "waypoint": list(waypoint),
                              "translation_error": str(exc)})
                events.append(event)
                # Never choose another candidate after a physical leg fails.
                # Localization loss, a newly blocked path, or a timeout all
                # require a fresh attended run from a known-safe scene.
                raise
            event.update({"selected_candidate": candidate_index,
                          "selected": candidate.as_dict(),
                          "waypoint": list(waypoint),
                          "translation_result": result})
            events.append(event)
            # The next iteration obtains a fresh pose, target grounding,
            # costmap, and candidate list. A closed route therefore switches to
            # another candidate instead of continuing a stale trajectory.
        raise RuntimeError("maximum staged legs reached before object arrival")
    finally:
        gateway.stop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true",
                        help="run bounded stages; default is stationary dry-run")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--place")
    target.add_argument("--point", nargs=2, type=float, metavar=("X", "Y"))
    target.add_argument("--segment", nargs=4, type=float,
                        metavar=("AX", "AY", "BX", "BY"))
    target.add_argument("--ground", metavar="DESCRIPTION",
                        help="ground target with local VLM + live metric depth")
    parser.add_argument("--places", type=Path,
                        default=Path("ros2_ws/src/hwr_nav/config/places.yaml"))
    parser.add_argument("--min-distance", type=float, default=0.30)
    parser.add_argument("--max-distance", type=float, default=0.48)
    parser.add_argument("--limit", type=int, default=12)
    parser.add_argument("--max-leg", type=float, default=0.10)
    parser.add_argument("--max-stages", type=int, default=8)
    parser.add_argument("--max-failed-legs", type=int, choices=[1], default=1,
                        help="compatibility option; physical leg failures never retry")
    parser.add_argument("--goal-timeout", type=float, default=15.0)
    parser.add_argument("--semantic-preflight", action="store_true")
    parser.add_argument("--semantic-margin", type=float, default=0.20)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parser.add_argument("--stability-timeout", type=float, default=15.0)
    parser.add_argument("--stability-duration", type=float, default=2.0)
    parser.add_argument("--gateway-url", default=os.environ.get(
        "HWR_GATEWAY_URL", "http://127.0.0.1:8090"))
    parser.add_argument("--vlm-url", default=os.environ.get(
        "LM_STUDIO_URL", "http://127.0.0.1:1234/v1"))
    parser.add_argument("--vlm-model", default=os.environ.get(
        "HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
    parser.add_argument("--vlm-timeout", type=float, default=60.0)
    args = parser.parse_args()
    if args.execute and not (args.ground or args.place):
        parser.error("--execute requires --ground or a verified --place")
    if args.execute and not args.semantic_preflight:
        parser.error("--execute requires --semantic-preflight")
    if not 0.05 <= args.max_leg <= 0.15:
        parser.error("--max-leg must be 0.05..0.15 metres")
    if not 1 <= args.max_stages <= 12:
        parser.error("--max-stages must be 1..12")
    if not 1.0 <= args.stability_duration <= 5.0:
        parser.error("--stability-duration must be 1..5 seconds")

    rclpy.init()
    node = LiveInputs()
    try:
        node.wait_ready(args.ready_timeout, require_depth=bool(args.ground))
        token = os.environ.get("HWR_GATEWAY_TOKEN", "")
        if args.execute:
            result = _execute_staged(node, args, token)
            print(json.dumps({"dry_run": False, **result}, indent=2))
            return 0 if result.get("state") == "succeeded" else 2
        grounding = None
        if args.place:
            object_start, object_end, provenance = _place_target(
                args.place, args.places)
        elif args.point:
            object_start, object_end = tuple(args.point), None
            provenance = {"source": "explicit_point"}
        elif args.segment:
            object_start = tuple(args.segment[:2])
            object_end = tuple(args.segment[2:])
            provenance = {"source": "explicit_segment"}
        else:
            object_start, object_end, provenance = _ground_target(
                node, args, token)

        grid, resolution, origin = node.internal_grid()
        candidates = object_approach_candidates(
            grid, resolution, origin, node.pose[:2], object_start, object_end,
            min_distance=args.min_distance, max_distance=args.max_distance,
            limit=args.limit)
        output = {
            "dry_run": True,
            "motion_dispatched": False,
            "pose": list(node.pose),
            "target": {**provenance, "object_start": list(object_start),
                       "object_end": (None if object_end is None
                                      else list(object_end))},
            "candidate_count": len(candidates),
            "candidates": [{**candidate.as_dict(),
                            "yaw_deg": math.degrees(candidate.yaw)}
                           for candidate in candidates],
        }
        print(json.dumps(output, indent=2))
        return 0 if candidates else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
