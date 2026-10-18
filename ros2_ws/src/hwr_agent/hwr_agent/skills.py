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


def _detour(target: dict, blockers: list, clear: float = 0.15, side: float = 0.17) -> float | None:
    """Smallest sideways shift (m, + = left) that puts every blocker >= clear off the robot's
    centre line while the target stays within the arm's side reach (|y| <= side). Base frame."""
    for step in range(2, 31, 2):
        for shift in (step / 100, -step / 100):
            if abs(target["y"] - shift) <= side and all(abs(b["y"] - shift) >= clear for b in blockers):
                return shift
    return None


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
        # life_loop starts a fresh MCP process per tick, so roam_step's "already seen" lives on disk.
        self._roam_file = Path(os.environ.get("HWR_ROAM_STATE",
                                              Path.home() / ".hwr_roam_seen.json"))

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

    def _motion(self, description: str, path: str, payload: dict, seconds: float):
        if self.actions >= self.max_actions:
            raise RuntimeError("mission action budget exhausted")
        self.actions += 1
        if self.dry_run:
            return {"dry_run": True, "would": description}
        # The gateway runs the loop and answers when it ends; allow its own timeout plus slack.
        response = requests.post(self.url + path, json=payload, timeout=seconds + 10.0,
                                 headers={"Authorization": f"Bearer {self.token}"})
        self._check(response)
        return response.json()

    def turn(self, degrees: float):
        degrees = float(degrees)
        if not 0.0 < abs(degrees) <= 360.0:
            raise ValueError("turn degrees must be in -360..360 and nonzero")
        return self._motion(f"turn {degrees} deg", "/motion/turn", {"degrees": degrees},
                            3.0 * math.radians(abs(degrees)) / 0.15 + 3.0)

    def move(self, meters: float, direction: str = "forward"):
        meters = float(meters)
        if direction not in ("forward", "left", "right"):
            raise ValueError("move direction must be forward, left or right (the base cannot reverse)")
        if not 0.0 < meters <= 2.0:
            raise ValueError("move meters must be in 0..2")
        return self._motion(f"move {meters} m {direction}", "/motion/move",
                            {"meters": meters, "direction": direction}, 3.0 * meters / 0.05 + 3.0)

    def back(self, meters: float = 0.1):
        """Back up <= 0.15 m, only along the straight path just driven forward, after the arm camera
        looked behind on both sides (plan/18.4). Refused after a turn or strafe."""
        meters = float(meters)
        if not 0.0 < meters <= 0.15:
            raise ValueError("back meters must be in 0..0.15")
        return self._motion(f"back up {meters} m", "/motion/back", {"meters": meters},
                            3.0 * meters / 0.05 + 15.0)

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

    def wander(self, max_distance: float = 1.5):
        if self.dry_run:
            return {"dry_run": True, "would": f"go to a random reachable point within {max_distance} m"}
        recovered = []
        if not self.where_am_i()["localization"].get("healthy"):
            recovered.append("relocalized before start")
            self.relocalize()
        # Deterministic recovery so the LLM never has to: camera not forward -> arm forward,
        # unplannable pick -> fresh pick, localization-related failure -> relocalize once.
        for _ in range(3):
            target = self._get(f"/wander/candidate?max_distance={float(max_distance)}")
            try:
                outcome = self.goto(target["x"], target["y"], target["yaw"])
            except RuntimeError as exc:
                if "forward camera pose" not in str(exc) or "arm forward" in recovered:
                    raise
                recovered.append("arm forward")
                self.arm_pose("forward")
                time.sleep(1.6)
                continue
            message = str(outcome["result"].get("message", ""))
            if message == "no collision-free path":
                continue
            if (outcome["result"].get("state") != "succeeded" and "locali" in message
                    and "relocalized after failure" not in recovered):
                recovered.append("relocalized after failure")
                self.relocalize()
                continue
            break
        return {"target": target, "recovered": recovered, **outcome}

    def list_maps(self):
        if self.dry_run:
            return {"dry_run": True, "would": "list saved maps and the active one"}
        return self._get("/maps")

    @staticmethod
    def _map_name(name: str) -> str:
        if not re.fullmatch(r"[a-z0-9_]{1,40}", str(name)):
            raise ValueError("map name must be 1-40 chars of a-z, 0-9, _ (e.g. living_room)")
        return str(name)

    def _switch(self, mode: str, name: str, timeout: float = 150.0) -> dict:
        """Restart the stack in mode on map name, wait until it answers, clear its startup estop."""
        self._act(f"switch to {mode} on map {name}", "/mode", {"mode": mode, "map": name})
        time.sleep(5.0)  # let the old gateway die before polling
        deadline = time.monotonic() + timeout
        while True:
            try:
                state = self._get("/maps")
                if state.get("mode") == mode and state.get("map") == name:
                    self._get("/status")
                    break
            except (RuntimeError, requests.RequestException):
                pass
            if time.monotonic() > deadline:
                raise RuntimeError(f"stack did not come back in {mode} mode within {timeout:.0f} s")
            time.sleep(3.0)
        # The driver latches the estop at every stack start (by design); this switch caused it.
        self.reset_estop()
        return state

    def use_map(self, name: str):
        name = self._map_name(name)
        if self.dry_run:
            return {"dry_run": True, "would": f"restart in navigation on {name} and relocalize"}
        self._switch("navigation", name)
        time.sleep(3.0)
        return {"map": name, "relocalize": self.relocalize(), **self.where_am_i()}

    def map_room(self, name: str, minutes: float = 10.0):
        """Map a new area end to end: exploration mode, explore until done or timeout, save, navigate on it."""
        name, minutes = self._map_name(name), float(minutes)
        if not 1.0 <= minutes <= 30.0:
            raise ValueError("mapping minutes must be 1..30")
        if self.dry_run:
            return {"dry_run": True, "would": f"map {name} for up to {minutes} min, save, switch to it"}
        existing = name in self._get("/maps")["maps"]
        self._switch("exploration", name)
        self.arm_pose("forward")
        time.sleep(2.0)
        before = self._get("/explore/status").get("coverage") or {}
        started = time.monotonic()
        self._act("start exploration", "/explore/start")
        status = {}
        while time.monotonic() - started < minutes * 60.0:
            time.sleep(10.0)
            status = self._get("/explore/status")
            if status.get("finished"):
                break
        else:
            self._act("stop exploration (time limit)", "/explore/stop")
            time.sleep(2.0)
            status = self._get("/explore/status")
        # explore only saves on natural completion; always save explicitly.
        saved = self._act("save map", "/map/save")
        result = {"map": name, "extended_existing": existing,
                  "explored_minutes": round((time.monotonic() - started) / 60.0, 1),
                  "explore_end": (status.get("finished") or {}).get("reason", "unknown"),
                  "frontier_rejections": status.get("rejections", []),
                  "half_leg_retries": status.get("half_leg_retries"),
                  "known_m2": (status.get("coverage") or {}).get("known_m2"),
                  "known_m2_gained": round(((status.get("coverage") or {}).get("known_m2") or 0.0)
                                           - (before.get("known_m2") or 0.0), 2),
                  "saved": saved}
        if not saved.get("success"):
            result["navigation"] = "not switched: map save failed"
            return result
        result["navigation"] = self.use_map(name)
        return result

    def approach(self, target: str = "person", stop_at_m: float = 0.8, timeout_s: float = 30.0):
        stop_at_m, timeout_s = float(stop_at_m), float(timeout_s)
        if not 0.3 <= stop_at_m <= 3.0 or not 1.0 <= timeout_s <= 120.0:
            raise ValueError("stop_at_m must be 0.3..3, timeout_s 1..120")
        return self._motion(f"approach {target} to {stop_at_m} m", "/motion/approach",
                            {"target": str(target), "stop_at_m": stop_at_m,
                             "timeout_s": timeout_s}, timeout_s)

    CUBE_COLORS = ("red", "green", "blue")

    def find_cubes(self, view: str = "look_down", level: int = 0, pan_deg: float = 0.0):
        if view not in ("look_down", "floor"):
            raise ValueError("view must be look_down or floor")
        if not abs(float(pan_deg)) <= 60:
            raise ValueError("pan_deg must be -60..60")
        payload = {"view": view, **({"level": int(level)} if level else {}),
                   **({"pan_deg": float(pan_deg)} if pan_deg else {})}
        return self._motion("look for cubes", "/arm/find_cubes", payload, 12.0)

    def look_around(self, target: str | None = None, view: str = "floor", pans=(0, 50, -50),
                    level: int = 0):
        """Search by swinging the arm (camera) left/right; the body stays put. Cubes come back in
        the base frame (first sighting per colour); any other target is matched against the
        YOLO classes with bearing = arm pan + bearing in the image. Stops early once `target`
        is seen, then re-centres the arm."""
        if self.dry_run:
            return {"dry_run": True, "would": f"look around for {target or 'cubes'}"}
        cube = target is None or target in self.CUBE_COLORS
        found = {}
        for pan in pans:
            if cube:
                for c in self.find_cubes(view, level, pan_deg=pan).get("cubes", []):
                    found.setdefault(c["color"], {**c, "pan_deg": pan})
            else:
                self.arm_aim(math.radians(pan))
                time.sleep(1.2)  # fresh detections after the swing
                try:
                    objects = self._get("/vision/objects")["objects"]
                except RuntimeError:
                    objects = []
                for o in objects:
                    if o["class"] == target:
                        found.setdefault(target, {**o, "bearing_deg": pan + (o.get("bearing_deg") or 0.0),
                                                  "pan_deg": pan})
            if target in found or len(found) == len(self.CUBE_COLORS):
                break
        if pan:
            self.arm_pose("forward")
        return {"found": list(found.values()), "target_seen": None if target is None else target in found}

    def pick(self, color: str, level: int = 0):
        if color not in self.CUBE_COLORS:
            raise ValueError(f"color must be one of {self.CUBE_COLORS}")
        payload = {"color": color, "level": int(level)}
        result = self._motion(f"pick the {color} cube", "/arm/pick", payload, 20.0)
        # Floor picks are near-vertical only (gateway, reach <= 0.223 m): bring the cube to 0.205 m.
        if result.get("outcome") == "out of reach" and self._into_reach(result["cube"], reach_x=0.205,
                                                                        target=color):
            result = self._motion(f"pick the {color} cube", "/arm/pick", payload, 20.0)
            result["repositioned"] = True
        return result

    def place(self, on: str | None = None, level: int = 1,
              x: float | None = None, y: float | None = None, color: str | None = None,
              pitch_deg: float | None = None):
        payload = {"on": on, "level": int(level), "x": x, "y": y, "color": color,
                   "pitch_deg": pitch_deg}
        result = self._motion(f"place on {on or (x, y)}", "/arm/place", payload, 20.0)
        if result.get("outcome") == "out of reach" and self._into_reach(result["spot"], held=color):
            result = self._motion(f"place on {on or (x, y)}", "/arm/place", payload, 20.0)
            result["repositioned"] = True
        return result

    def _into_reach(self, spot: dict, reach_x: float = 0.22, keep_x: float = 0.19,
                    held: str | None = None, detour: bool = True, target: str | None = None) -> bool:
        """Turn to face a base-frame spot and creep (<= 0.15 m) until it sits ~reach_x ahead.

        The creep bypasses the costmap (it inflates the cubes themselves), so it is capped by every
        cube in view: none may end closer than keep_x (bumper ~0.125 m). Live 2026-10-16 the
        uncapped creep drove into the nearer cubes. The base can't reverse, so a cube too close is
        handled by strafing away, and a cube behind another by strafing around it."""
        if self.dry_run:
            return False
        moved = False
        if math.hypot(spot["x"], spot["y"]) < keep_x:
            # Live 2026-10-17 a cube beside the jaw needed a human otherwise.
            side = 1.0 if spot["y"] >= 0 else -1.0
            step = min(math.sqrt(max(reach_x ** 2 - spot["x"] ** 2, 0.0)) - abs(spot["y"]), 0.15)
            if step < 0.02 or not self._strafe(-side * step):
                return False
            spot, moved = {"x": spot["x"], "y": spot["y"] + side * step}, True
        bearing = math.degrees(math.atan2(spot["y"], spot["x"]))
        distance = math.hypot(spot["x"], spot["y"])
        # Steep-only grasps end ~0.25 m; +0.04 slack here once called 0.275 "close" (S3 #1).
        if abs(bearing) < 8.0 and distance <= reach_x + 0.02:
            return moved  # in front and close: moving forward won't help
        if abs(bearing) >= 8.0:
            self._arm_clear()
            self.turn(bearing)  # in place; needs no forward camera
            spot, moved = {"x": distance, "y": 0.0}, True
        # The held cube shows at the image bottom (S2 #4 refused the creep because of it).
        cubes = [c for c in self.find_cubes().get("cubes", []) if held is None or c.get("color") != held]
        # S5 #4/#5 2026-10-17: the turn undershot and the target, seen ~6 cm left of the estimate, was
        # taken for a blocker -> strafed away from it -> "target not seen". Use where it is now.
        seen = [c for c in cubes if target is not None and c.get("color") == target]
        if seen:
            spot = {"x": seen[0]["x"], "y": seen[0]["y"]}
        cubes = [c for c in cubes if target is None or c.get("color") != target]
        # Corridor = robot half-width ~0.1 m + half a cube.
        corridor = [c for c in cubes if abs(c["y"]) < 0.13]
        nearest = min([c["x"] for c in corridor] + [spot["x"]])
        creep = min(spot["x"] - reach_x, nearest - keep_x, 0.15)
        # Detour whenever another cube caps the creep short of the target (c1 2026-10-17: a few-cm
        # creep per call next to the 2-stack, then "out of reach" twice and give up).
        if nearest - keep_x < min(spot["x"] - reach_x, 0.15) - 0.01:
            blockers = [c for c in corridor if c["x"] < spot["x"] - 0.05]
            shift = _detour(spot, blockers) if detour and blockers and spot["x"] - reach_x >= 0.02 else None
            if shift is None or not self._strafe(shift):
                if creep >= 0.02:
                    self._arm_clear()
                    self.drive(vx=0.1, seconds=creep / 0.1)  # as close as the blocker allows
                    return True
                return moved  # can't get closer without hitting another cube
            # Live 2026-10-17: red sat right behind the 2-stack. Step beside it and creep past it;
            # the arm takes the target off to the side (a turn would line the stack up again).
            cubes = [c for c in self.find_cubes().get("cubes", [])
                     if held is None or c.get("color") != held]
            seen = [c for c in cubes if target is not None and c.get("color") == target]
            spot = {"x": seen[0]["x"], "y": seen[0]["y"]} if seen else {"x": spot["x"], "y": spot["y"] - shift}
            cubes = [c for c in cubes if target is None or c.get("color") != target]
            nearest = min([c["x"] for c in cubes if abs(c["y"]) < 0.13] + [1.0])  # target is aside now
            # Stop with the target on the reach circle: a fixed x 0.14 put blue 0.165 m out (live
            # 2026-10-17 evening), inside the gateway's 0.19 m own-wheel guard; dead ahead it is reach_x.
            stop = math.sqrt(max(reach_x ** 2 - spot["y"] ** 2, 0.0))
            creep = min(spot["x"] - stop, nearest - keep_x, 0.25)
            if creep >= 0.02:
                self._arm_clear()
                self.drive(vx=0.1, seconds=creep / 0.1)
            return True
        if creep < 0.02:
            return moved  # already as close as reach_x
        self._arm_clear()
        self.drive(vx=0.1, seconds=creep / 0.1)
        return True

    def _arm_clear(self):
        # S5 #3 2026-10-17: a turn in look_down swept the held cube through the 2-stack.
        self.arm_pose("forward")
        time.sleep(2.0)

    def _strafe(self, meters: float) -> bool:
        """Sideways (+ = left) through move(): costmap-checked, which needs the forward camera."""
        self._arm_clear()
        try:
            self.move(abs(meters), "left" if meters > 0 else "right")
        except RuntimeError:
            return False
        return True

    def follow(self, target: str = "person", seconds: float = 60.0, distance_m: float = 0.8):
        seconds, distance_m = float(seconds), float(distance_m)
        if not 3.0 <= seconds <= 300.0 or not 0.3 <= distance_m <= 3.0:
            raise ValueError("seconds 3..300, distance_m 0.3..3")
        return self._motion(f"follow {target} for {seconds} s", "/arm/follow",
                            {"target": str(target), "seconds": seconds,
                             "distance_m": distance_m}, seconds + 15.0)

    def _pose(self):
        # Odometry [x, y, yaw]: live 2026-10-17 the degraded map pose froze x/y over ~1 m of
        # driving; odom drift over a few short moves is small.
        return self._get("/navigation/context")["odometry"]["pose"]

    @staticmethod
    def _to_world(pose, spot):
        x, y, yaw = pose
        return (x + math.cos(yaw) * spot["x"] - math.sin(yaw) * spot["y"],
                y + math.sin(yaw) * spot["x"] + math.cos(yaw) * spot["y"])

    @staticmethod
    def _to_base(pose, point):
        x, y, yaw = pose
        dx, dy = point[0] - x, point[1] - y
        return {"x": math.cos(yaw) * dx + math.sin(yaw) * dy,
                "y": -math.sin(yaw) * dx + math.cos(yaw) * dy}

    def stack_cubes(self, order: list[str] | None = None, max_rounds: int = 8):
        """Stack the cubes into one tower. With `order` (bottom first) it follows it; without, it
        picks the base itself (the cube nearest the arm's sweet spot) and stacks the nearest
        remaining cube each round. The body turns/creeps as needed (stack tracked in map frame);
        a missed pick is retried and a fallen place puts that cube back in the pool."""
        if order is not None:
            order = list(order)
            if not 2 <= len(order) <= 3 or len(set(order)) != len(order) or \
                    not set(order) <= set(self.CUBE_COLORS):
                raise ValueError(f"order must be 2-3 distinct colours from {self.CUBE_COLORS}")
        if self.dry_run:
            return {"dry_run": True, "would": f"stack {order or 'any order'}"}
        log, world = [], {}

        def look():
            pose = self._pose()
            for cube in self.find_cubes().get("cubes", []):
                if cube["color"] not in stack:
                    world[cube["color"]] = self._to_world(pose, cube)
            return pose

        stack = []
        pose = look()
        for i in range(3):  # search: the arm sweeps the floor view (~1 m out, +-80 deg) here, then the body turns
            if set(order or self.CUBE_COLORS) <= set(world):
                break
            if i:
                self.turn(120)
                pose = self._pose()
            for cube in self.look_around()["found"]:
                world.setdefault(cube["color"], self._to_world(pose, cube))
        wanted = order or sorted(world)
        if len(wanted) < 2 or not set(wanted) <= set(world):
            return {"outcome": f"cubes not seen: {sorted(set(order or self.CUBE_COLORS) - set(world))}",
                    "log": log}
        # Build on the farthest cube in reach-ish range: the others are then nearer and get carried
        # out to it past at most one cube (c8 2026-10-17: a near base left red behind two cubes).
        dist = {c: math.hypot(*self._to_base(pose, world[c]).values()) for c in wanted}
        near = [c for c in wanted if dist[c] <= 0.40]
        base = order[0] if order else max(near, key=dist.get) if near else min(wanted, key=dist.get)
        stack, stack_xy = [base], world.pop(base)
        last = None
        for _ in range(max_rounds):
            todo = [c for c in wanted if c not in stack]
            if not todo:
                break
            pose = self._pose()
            color = todo[0] if order else min(
                todo, key=lambda c: math.hypot(*self._to_base(pose, world[c]).values()))
            picked = self.pick(color)
            for _ in range(4):  # far (found by the search): creep up in capped steps
                if picked.get("outcome") != "not seen" or color not in world or                         not self._into_reach(self._to_base(self._pose(), world[color]), reach_x=0.205):
                    break
                picked = self.pick(color)
            log.append({"pick": color, **picked})
            if picked.get("outcome") != "holding":
                if last == (color, picked.get("outcome")):
                    break  # same failure twice: no progress, report instead of looping
                last = (color, picked.get("outcome"))
                look()
                continue
            for _ in range(2):  # still holding: get the stack in view/reach and try again
                here = self._to_base(self._pose(), stack_xy)
                # The tower's top reads off on the floor plane; only a lone base is a reliable fix.
                self._into_reach(here, reach_x=0.22, held=color,
                                 target=stack[0] if len(stack) == 1 else None)
                placed = self.place(on=stack[-1], level=len(stack), color=color,
                                    pitch_deg=picked.get("pitch_deg"))
                log.append({"place_on": stack[-1], **placed})
                if placed.get("outcome") not in ("target not seen", "out of reach"):
                    break
                if placed.get("outcome") == "target not seen":
                    # The tower position is odometry since the last place: sweep the arm for its
                    # top cube (on its own level's plane) before the second try.
                    pose = self._pose()
                    found = self.look_around(stack[-1], level=len(stack) - 1)["found"]
                    if found:
                        stack_xy = self._to_world(pose, found[0])
            if placed.get("outcome") == "placed":
                last = None
                stack.append(color)
                if placed.get("spot"):
                    stack_xy = self._to_world(self._pose(), placed["spot"])  # tower as last seen
            elif placed.get("outcome") in ("target not seen", "out of reach"):
                # Never open mid-air (live 2026-10-17 that dropped cubes "for no reason"):
                # set it down in front and put it back in the pool.
                log.append({"set_down": color, **self.place(x=0.22, y=0.0, color=color)})
                # S5 #4 2026-10-17: lost the tower, then set down/picked/missed it 3x for 90 s.
                if last == (color, placed.get("outcome")):
                    break
                last = (color, placed.get("outcome"))
                look()
            else:
                look()  # released but not on the stack: on the floor, back in the pool
        self.arm_pose("forward")
        done = len(stack) == len(wanted)
        if done:
            # The tool once said "stacked" with a cube on the floor beside it (2026-10-17, twice,
            # once ~5 cm apart): every stack colour still visible must sit at the top cube's spot.
            # Each cube located on its own level's plane (all on the floor plane, the top cube
            # read ~4 cm off and failed a standing tower, c5 2026-10-17).
            at = {c: {k["color"]: k for k in self.find_cubes(level=i).get("cubes", [])}.get(c)
                  for i, c in enumerate(stack)}
            top = at[stack[-1]]
            apart = [c for c in stack[:-1] if at[c] is not None and top is not None and
                     math.hypot(at[c]["x"] - top["x"], at[c]["y"] - top["y"]) > 0.035]
            if top is None or apart:
                done = False
                log.append({"final_check": "failed", "top_seen": top is not None, "apart": apart})
            self.arm_pose("forward")
        return {"outcome": "stacked" if done else f"stack of {len(stack)} unverified"
                if len(stack) == len(wanted) else f"stack of {len(stack)}", "stack": stack,
                "log": log}

    def unstack_cubes(self, stack: list[str]):
        """Take a tower (bottom first) apart top-down onto the floor 0.10 m either side of it.
        Stops at the first failure and never moves the body: live 2026-10-16 an ad-hoc reset kept
        going after a failed place and dropped a cube out of view."""
        stack = list(stack)
        if not 2 <= len(stack) <= 3 or len(set(stack)) != len(stack) or \
                not set(stack) <= set(self.CUBE_COLORS):
            raise ValueError(f"stack must be 2-3 distinct colours from {self.CUBE_COLORS}")
        if self.dry_run:
            return {"dry_run": True, "would": f"unstack {stack} onto the floor"}
        log, sides = [], None
        for level in range(len(stack) - 1, 0, -1):
            color = stack[level]
            picked = self._motion(f"pick the {color} cube", "/arm/pick",
                                  {"color": color, "level": level}, 20.0)
            log.append({"pick": color, **picked})
            if picked.get("outcome") != "holding":
                break
            tower = picked["cube"]
            if sides is None:  # nearer-the-centre side first; the tower itself stays put
                sides = sorted((0.10, -0.10), key=lambda d: abs(tower["y"] + d))
            # Not closer than 0.19 m: near the bumper places land up to 2.6 cm off (c8 2026-10-17).
            spot = {"x": max(tower["x"], 0.19), "y": tower["y"] + sides.pop(0)}
            placed = self._motion(f"put the {color} cube down", "/arm/place",
                                  {"level": 1, **spot, "color": color}, 20.0)
            log.append({"place_at": spot, **placed})
            if placed.get("outcome") != "placed":
                break
        else:
            self.arm_pose("forward")
            return {"outcome": "unstacked", "log": log}
        return {"outcome": f"stopped: {log[-1].get('outcome')}", "log": log}

    def roam_step(self, max_distance: float = 1.5):
        """One idle-life tick: sense, wander, sense; the VLM runs only when something new shows up."""
        if self.dry_run:
            return {"dry_run": True, "would": "seen_objects, wander, seen_objects, describe if new"}
        before = self._classes()
        try:
            moved = self.wander(max_distance)
            move = {"state": moved["result"].get("state"), "message": moved["result"].get("message"),
                    "recovered": moved["recovered"]}
        except RuntimeError as exc:
            move = {"state": "refused", "message": str(exc)}
        after = self._classes()
        try:
            anchored = self.anchor_places().get("anchored", [])
        except (RuntimeError, requests.RequestException):  # unhealthy pose etc.: roam on without
            anchored = []
        seen = before | after
        try:
            last_seen = json.loads(self._roam_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            last_seen = {}
        now = time.time()
        # "New" = not seen in the last 30 min, so a returning person is news again.
        new = sorted(c for c in seen if now - last_seen.get(c, 0.0) > 1800.0)
        last_seen.update({c: now for c in seen})
        self._roam_file.write_text(json.dumps(last_seen), encoding="utf-8")
        result = {"move": move, "pose": self.where_am_i()["pose"],
                  "objects": sorted(seen), "new_objects": new,
                  "anchored": [a["name"] for a in anchored]}
        if new:
            result["view"] = self.describe_view(
                f"In one or two sentences: what are the {', '.join(new)} here and what is around them?")["description"]
        return result

    def _classes(self) -> set:
        try:
            return {item["class"] for item in self._get("/vision/objects")["objects"]}
        except RuntimeError:  # vision stale: roam on, just without object news
            return set()

    def seen_objects(self):
        if self.dry_run:
            return {"dry_run": True, "would": "read tracked objects"}
        frame = self._get("/vision/objects")
        lines = []
        for item in frame["objects"]:
            where = []
            if item.get("range_m") is not None:
                where.append(f"{item['range_m']} m")
            if item.get("bearing_deg") is not None:
                side = "left" if item["bearing_deg"] > 0 else "right"
                where.append(f"{abs(item['bearing_deg'])} deg {side}" if item["bearing_deg"] else "ahead")
            ident = f" #{item['id']}" if item.get("id") is not None else ""
            lines.append(f"{item['class']}{ident} ({item['conf']}) " + ", ".join(where))
        return {"objects": lines or ["nothing detected"], "infer_ms": frame["infer_ms"],
                "note": "bearing is relative to where the camera points"}

    def say(self, text: str):
        return self._act(f"say: {text}", "/say", {"text": str(text)})

    def reset_estop(self):
        # User decision 2026-10-13: the agent may clear the estop itself (counts against the budget).
        return self._act("reset estop", "/estop/reset")

    def relocalize(self):
        return self._act("global relocalize", "/localization/relocalize")

    def mark_place(self, name: str):
        return self._act(f"mark the current pose as place {name}", "/places/mark", {"name": name})

    def anchor_places(self):
        """Remember where the static objects in view are (auto object places, plan/18.1). Looks
        with the camera tilted up: live 2026-10-18 YOLO missed the chair from the forward view."""
        self.arm_pose("ceiling")
        try:
            if not self.dry_run:
                time.sleep(2.5)  # arm settles (1.5 s move) and vision publishes a fresh frame
            return self._act("anchor the objects in view as places", "/places/anchor")
        finally:
            self.arm_pose("forward")

    def list_places(self):
        if self.dry_run:
            return {"dry_run": True, "would": "list places on the loaded map"}
        return self._get("/places")

    def where_am_i(self):
        if self.dry_run:
            return {"dry_run": True, "would": "read pose and localization"}
        context = self._get("/navigation/context")
        return {"pose": context["pose"],
                "localization": context["localization"]}

    def call(self, name: str, arguments: dict):
        allowed = {"goto_place", "goto", "explore", "look", "describe_view",
                   "assess_scene", "patrol", "arm_pose", "pick_test",
                   "where_am_i", "relocalize", "stop", "reset_estop",
                   "drive", "arm_aim", "arm_reach", "gripper",
                   "say", "seen_objects", "wander", "turn", "move", "back",
                   "approach", "roam_step", "map_room", "use_map", "list_maps",
                   "find_cubes", "look_around", "pick", "place", "follow", "stack_cubes",
                   "unstack_cubes", "mark_place", "list_places", "anchor_places"}
        if name not in allowed:
            raise ValueError(f"unknown skill {name!r}")
        return getattr(self, name)(**arguments)
