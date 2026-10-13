"""Text-friendly, bounded skills over the same audited gateway humans use."""
from __future__ import annotations

import base64
import json
import math
import os
import re
import time
from pathlib import Path

import requests


class GatewaySkills:
    _FLOOR_HAZARD_WORDS = ("wire", "cable", "cord", "clothing", "chair",
                           "obstruction", "clutter", "object on floor")

    @staticmethod
    def _without_explicit_negations(text: str, terms: tuple[str, ...]) -> str:
        for term in sorted(set(terms), key=len, reverse=True):
            text = re.sub(
                rf"\b(?:no|without|free of)\s+(?:visible\s+)?"
                rf"{re.escape(term)}s?\b", " ", text)
        return text

    def __init__(self, gateway_url: str | None = None, token: str | None = None,
                 dry_run: bool = False, max_actions: int = 20,
                 timeout: float = 10.0):
        self.url = (gateway_url or os.environ.get(
            "HWR_GATEWAY_URL", "http://127.0.0.1:8090")).rstrip("/")
        self.token = token or os.environ.get("HWR_GATEWAY_TOKEN", "")
        if not self.token and not dry_run:
            raise ValueError("HWR_GATEWAY_TOKEN is required")
        self.dry_run = bool(dry_run)
        self.max_actions = int(max_actions)
        self.timeout = float(timeout)
        self.actions = 0

    def _act(self, description: str, path: str, payload=None):
        if self.actions >= self.max_actions:
            raise RuntimeError("mission action budget exhausted")
        self.actions += 1
        if self.dry_run:
            return {"dry_run": True, "would": description}
        response = requests.post(self.url + path,
                                 headers={"Authorization": f"Bearer {self.token}"},
                                 json=payload, timeout=self.timeout)
        self._check(response)
        return response.json()

    def _get(self, path: str, binary: bool = False):
        response = requests.get(self.url + path,
                                headers={"Authorization": f"Bearer {self.token}"},
                                timeout=self.timeout)
        self._check(response)
        return response.content if binary else response.json()

    @staticmethod
    def _check(response) -> None:
        if response.ok:
            return
        try:
            detail = response.json().get("detail", response.text)
        except ValueError:
            detail = response.text
        raise RuntimeError(f"gateway HTTP {response.status_code}: {detail}")

    def goto_place(self, name: str):
        dispatch = self._act(f"navigate to {name}", "/goto", {"place": name})
        result = self._wait_goal()
        if (not self.dry_run and dispatch.get("place_kind") == "object"
                and result.get("state") == "succeeded"):
            distance = result.get("object_distance_m")
            limit = result.get("max_object_distance_m")
            reached = result.get("object_reached") is True
            valid = (isinstance(distance, (int, float))
                     and isinstance(limit, (int, float))
                     and math.isfinite(float(distance))
                     and math.isfinite(float(limit))
                     and 0.0 < float(limit) <= 0.50
                     and float(distance) <= float(limit))
            if not reached or not valid:
                result = dict(result)
                result.update({
                    "state": "failed", "object_reached": False,
                    "message": "gateway did not prove object proximity within 0.50 m",
                })
        return {"dispatch": dispatch, "result": result}

    def goto(self, x: float, y: float, yaw: float = 0.0):
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            raise ValueError("goal must be finite")
        dispatch = self._act(f"navigate to ({x}, {y}, {yaw})", "/goto",
                             {"x": x, "y": y, "yaw": yaw})
        return {"dispatch": dispatch, "result": self._wait_goal()}

    def explore(self, minutes: float):
        minutes = float(minutes)
        if not 0.1 <= minutes <= 15.0:
            raise ValueError("explore duration must be 0.1..15 minutes")
        started = self._act(f"explore for {minutes} minutes", "/explore/start")
        if self.dry_run:
            return started
        try:
            time.sleep(minutes * 60.0)
        finally:
            stopped = self._act("stop exploration", "/explore/stop")
        return {"started": started, "stopped": stopped}

    def look(self, direction: str = "center", bearing: float | None = None):
        bearings = {"left": math.radians(45), "center": 0.0,
                    "right": math.radians(-45)}
        if bearing is None:
            if direction not in bearings:
                raise ValueError("direction must be left, center, or right")
            bearing = bearings[direction]
        result = self._act(f"look at bearing {bearing}", "/arm/aim",
                           {"bearing": bearing, "elevation": 0.0,
                            "duration_ms": 1000})
        if self.dry_run:
            return result
        time.sleep(1.2)
        path = Path(f"hwr_view_{int(time.time())}.jpg").resolve()
        path.write_bytes(self._get("/snapshot", binary=True))
        return {"aim": result, "image": str(path)}

    def describe_view(self, question: str = "Describe what is visible."):
        if self.dry_run:
            return {"dry_run": True, "would": f"capture and ask VLM: {question}"}
        image = self._get("/snapshot", binary=True)
        return {"description": self._ask_vlm(image, question)}

    def _ask_vlm(self, image: bytes, prompt: str) -> str:
        endpoint = os.environ.get("LM_STUDIO_URL", "http://127.0.0.1:1234/v1").rstrip("/")
        model = os.environ.get("HWR_VLM_MODEL", "qwen/qwen3-vl-4b")
        payload = {"model": model, "temperature": 0.2, "messages": [{
            "role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url":
                    "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")}},
            ]}]}
        response = requests.post(endpoint + "/chat/completions", json=payload,
                                 timeout=60.0)
        response.raise_for_status()
        return response.json()["choices"][0]["message"]["content"]

    def assess_scene(self, question: str = "Is the visible route safe to translate into?"):
        """Return advisory semantic context; metric navigation guards stay authoritative."""
        if self.dry_run:
            return {"dry_run": True,
                    "would": f"combine navigation context and camera via VLM: {question}"}
        context = self._get("/navigation/context")
        image = self._get("/snapshot", binary=True)
        prompt = (
            "You are an advisory vision layer for an attended indoor robot. "
            "Metric sensor context is authoritative; never overrule a blocked "
            "clearance and never command motion. Do not identify people. "
            "A doorway is visible only when an open passage with floor "
            "continuing into another space is clearly seen; a window, curtain, "
            "blank wall, closed door, or furniture gap is not a doorway. A "
            "person is visible only when a human body is clearly present. "
            "Return only one JSON object with keys safe_to_translate (boolean), "
            "doorway_visible (boolean), person_visible (boolean), hazards "
            "(array of short strings), visual_corridor (string), recommendation "
            "(one of hold, sensor_preflight, human_reposition), and rationale "
            "(short string), doorway_evidence (short string or empty), and "
            "person_evidence (short string or empty). Use sensor_preflight when "
            "the visible near-field route appears safe but metric checks are "
            "still required. Use hold only for an observed or uncertain visual "
            "concern; the fact that you do not command motion is not a reason "
            "to hold.\n"
            f"Question: {question}\n"
            f"Navigation context: {json.dumps(context, separators=(',', ':'))}"
        )
        raw = self._ask_vlm(image, prompt).strip()
        if raw.startswith("```"):
            raw = raw.split("\n", 1)[-1]
            raw = raw.rsplit("```", 1)[0].strip()
        try:
            assessment = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"VLM returned invalid assessment JSON: {exc}") from exc
        required = {"safe_to_translate", "doorway_visible", "person_visible",
                    "hazards", "visual_corridor", "recommendation", "rationale"}
        required |= {"doorway_evidence", "person_evidence"}
        if not isinstance(assessment, dict) or not required <= assessment.keys():
            raise RuntimeError("VLM assessment is missing required fields")
        if assessment["recommendation"] not in {
                "hold", "sensor_preflight", "human_reposition"}:
            raise RuntimeError("VLM assessment has an invalid recommendation")
        hazard_text = " ".join(str(item).lower()
                               for item in assessment["hazards"])
        corridor = str(assessment["visual_corridor"]).lower()
        rationale = str(assessment["rationale"]).lower()
        corridor_veto_words = (
            "blocked", "clutter", "uncertain", "obstruction", "cable",
            "wire", "cord", "chair", "rack", "leg", "base", "pole",
            "person")
        checked_hazards = self._without_explicit_negations(
            hazard_text, self._FLOOR_HAZARD_WORDS)
        checked_corridor = self._without_explicit_negations(
            corridor, corridor_veto_words)
        positive_clear = (
            bool(assessment.get("safe_to_translate"))
            and not assessment["hazards"]
            and any(phrase in corridor
                    for phrase in ("clear", "open-floor", "open floor", "lane"))
            and not any(word in checked_corridor
                        for word in corridor_veto_words)
            and any(phrase in rationale for phrase in (
                "clearance sufficient", "no visual obstruction",
                "path is clear", "corridor is clear", "no hazards"))
        )
        if assessment["recommendation"] == "hold" and positive_clear:
            assessment["model_recommendation"] = "hold"
            assessment["recommendation"] = "sensor_preflight"
            assessment["consistency_correction"] = (
                "positive visual evidence contradicted model hold")
        vetoes = []
        if any(word in checked_hazards for word in self._FLOOR_HAZARD_WORDS):
            vetoes.append("reported floor hazard")
        if any(word in checked_corridor for word in corridor_veto_words):
            vetoes.append("corridor not visually clear")
        if assessment["recommendation"] in {"hold", "human_reposition"}:
            vetoes.append("model recommendation requires stopping")
        if assessment.get("safe_to_translate") and vetoes:
            assessment["model_safe_to_translate"] = True
            assessment["safe_to_translate"] = False
            assessment["consistency_veto"] = sorted(set(vetoes))
        return {"assessment": assessment, "navigation_context": context}

    def patrol(self, places: list[str]):
        if not 1 <= len(places) <= 10:
            raise ValueError("patrol requires 1..10 places")
        results = []
        for place in places:
            outcome = self.goto_place(place)
            results.append({"place": place, **outcome})
            if outcome["result"].get("state") != "succeeded":
                break
        return results

    def _wait_goal(self, timeout: float = 120.0):
        if self.dry_run:
            return {"dry_run": True, "state": "would-wait"}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            context = self._get("/navigation/context")
            goal = context.get("current_goal") or {}
            if goal.get("state") in ("succeeded", "failed", "rejected", "error"):
                return goal
            time.sleep(1.0)
        raise RuntimeError("navigation goal timed out")

    def arm_pose(self, name: str):
        return self._act(f"move arm to {name}", "/arm/pose",
                         {"name": name, "duration_ms": 1500})

    def pick_test(self):
        return {"open": self._act("open gripper", "/gripper",
                                  {"command": "open", "duration_ms": 800}),
                "close": self._act("close gripper", "/gripper",
                                   {"command": "close", "duration_ms": 800})}

    def stop(self):
        # Outside the action budget: halting must always be possible.
        if self.dry_run:
            return {"dry_run": True, "would": "stop motion and latch estop"}
        response = requests.post(self.url + "/stop",
                                 headers={"Authorization": f"Bearer {self.token}"},
                                 timeout=self.timeout)
        self._check(response)
        return response.json()

    def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0,
              seconds: float = 1.0):
        seconds = float(seconds)
        if not 0.05 <= seconds <= 5.0:
            raise ValueError("drive duration must be 0.05..5 seconds")
        body = {"vx": float(vx), "vy": float(vy), "wz": float(wz), "duration": 0.3}
        first = self._act(f"drive vx={vx} vy={vy} wz={wz} for {seconds}s", "/drive", body)
        if self.dry_run:
            return first
        # Gateway commands expire after 0.3 s; re-issue so motion stops on its own if this process dies.
        end = time.monotonic() + seconds
        while time.monotonic() + 0.2 < end:
            time.sleep(0.2)
            self._check(requests.post(self.url + "/drive", json=body, timeout=self.timeout,
                                      headers={"Authorization": f"Bearer {self.token}"}))
        time.sleep(max(0.0, end - time.monotonic()))
        self._check(requests.post(self.url + "/drive", json={**body, "vx": 0.0, "vy": 0.0,
                                  "wz": 0.0, "duration": 0.05}, timeout=self.timeout,
                                  headers={"Authorization": f"Bearer {self.token}"}))
        return {"drove_seconds": seconds, "command": body}

    def arm_aim(self, bearing: float, elevation: float = 0.0):
        return self._act(f"aim camera bearing={bearing} elevation={elevation}", "/arm/aim",
                         {"bearing": float(bearing), "elevation": float(elevation),
                          "duration_ms": 1000})

    def arm_reach(self, x: float, y: float, z: float, tool_pitch: float = 0.0):
        return self._act(f"reach arm to ({x}, {y}, {z})", "/arm/reach",
                         {"x": float(x), "y": float(y), "z": float(z),
                          "tool_pitch": float(tool_pitch), "duration_ms": 1500})

    def gripper(self, command: str):
        if command not in ("open", "close"):
            raise ValueError("gripper command must be open or close")
        return self._act(f"{command} gripper", "/gripper",
                         {"command": command, "duration_ms": 800})

    def reset_estop(self):
        # User decision 2026-10-13: the agent may clear the estop itself (counts against the budget).
        return self._act("reset estop", "/estop/reset")

    def where_am_i(self):
        if self.dry_run:
            return {"dry_run": True, "would": "read pose and localization"}
        context = self._get("/navigation/context")
        return {"pose": context["pose"],
                "localization": context["localization"]}

    def call(self, name: str, arguments: dict):
        allowed = {"goto_place", "goto", "explore", "look", "describe_view",
                   "assess_scene", "patrol", "arm_pose", "pick_test",
                   "where_am_i", "stop", "reset_estop",
                   "drive", "arm_aim", "arm_reach", "gripper"}
        if name not in allowed:
            raise ValueError(f"unknown skill {name!r}")
        return getattr(self, name)(**arguments)
