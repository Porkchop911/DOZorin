"""Authenticated snapshot/context semantic preflight through a local VLM."""
from __future__ import annotations

import base64
import json
import time

import requests

from .semantic import (assessment_from_visual_observation,
                       metric_translation_preflight,
                       semantic_context_for_prompt)


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


class SemanticPreflightClient:
    """Fail-closed visual and metric check for a short forward translation."""

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

    def context(self) -> dict:
        return self._get("/navigation/context")

    def assess(self, distance: float, *, margin: float | None = None
               ) -> tuple[bool, dict]:
        context = self.context()
        image = self._get("/snapshot", binary=True)
        prompt = (
            "You are the fail-closed advisory vision preflight for an attended "
            "indoor robot. The camera is aligned with a proposed short forward "
            f"translation of {distance:.2f} metres. Inspect the entire near-field "
            "floor and the robot-height volume above it. Report people, feet, "
            "wires/cables, chairs, thin poles, rack or furniture bases/legs, "
            "hanging fabric, and other overhangs. Metric sensor context remains "
            "authoritative and cannot be overruled. Do not identify people. "
            "Small dark rounded shapes permanently visible at the bottom edge "
            "are the robot's own gripper/base hardware, never human feet. "
            "Judge the visible forward corridor and a conservative swept envelope "
            "wider than the chassis, including the camera/arm overhang and any "
            "lateral correction. A visible person or chair entirely outside that "
            "envelope must still be reported by person_visible when applicable, "
            "but must not be put in hazards or person_in_corridor and does not by "
            "itself require hold. A seated person and chair wholly at the far left "
            "edge of a clear forward floor lane are outside the corridor. "
            "Treat windows, glazed doors, mirrors, and other transparent or "
            "reflective planes as optical boundaries, never as open passages. "
            "Set glass_boundary_visible true whenever one is visible. Set "
            "glass_boundary_in_corridor true when the proposed forward corridor "
            "terminates at, points through, or is visually dominated by that "
            "boundary; in that case recommendation must be hold even if lidar "
            "reports censored-clear space. An ordinary open doorway with "
            "continuous floor is not a glass boundary. "
            "Return only one JSON object with keys visibility_sufficient (boolean), "
            "doorway_visible (boolean), person_visible (boolean), "
            "person_in_corridor (boolean), hazards (array containing only "
            "corridor-intersecting hazards), visual_corridor (string), "
            "glass_boundary_visible (boolean), "
            "glass_boundary_in_corridor (boolean), "
            "glass_boundary_evidence (short string or empty), "
            "recommendation (one of hold, sensor_preflight, human_reposition), "
            "rationale (short string), doorway_evidence (short string or empty), "
            "and person_evidence (short string or empty). Use sensor_preflight "
            "only when the complete proposed near-field segment appears clear. "
            "Use hold for any observed or uncertain concern in the swept envelope. "
            "A person requires hold only when the person intersects or may enter "
            "that envelope. This is a visual classification, not final motion "
            "authorization: a separate metric layer evaluates exact chassis "
            "geometry and the immediate camera-occluded footprint. "
            "visibility_sufficient asks only whether this image is clear enough "
            "to report visible near-field hazards; it does not authorize movement. "
            "When hazards is empty, no person is in the corridor, visual_corridor "
            "is clear, and you state no actual visibility uncertainty, set "
            "visibility_sufficient true and recommendation sensor_preflight. "
            "A hold must name its concrete observed or visibility concern. "
            "A null hard-clearance value means censored clear space out to the "
            "reported limit, not absent telemetry. "
            f"Navigation context: {json.dumps(semantic_context_for_prompt(context), separators=(',', ':'))}. "
            "All 13 requested keys are mandatory. Never omit rationale or any "
            "other key."
        )
        # Thinking models (Strata/Qwen3.8) reason for minutes on this prompt; servers without the flag ignore it.
        payload = {"model": self.model, "temperature": 0.1,
                   "chat_template_kwargs": {"enable_thinking": False}, "messages": [{
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
                raw = raw.split("\n", 1)[-1].rsplit("```", 1)[0].strip()
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
                  "metric_allowed": metric_ok,
                  "reasons": sorted(set(reasons))}
        return semantic_ok and metric_ok, report
