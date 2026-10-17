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

    def find_cubes(self):
        return self._motion("look down for cubes", "/arm/find_cubes", {}, 10.0)

    def pick(self, color: str, level: int = 0):
        if color not in self.CUBE_COLORS:
            raise ValueError(f"color must be one of {self.CUBE_COLORS}")
        payload = {"color": color, "level": int(level)}
        result = self._motion(f"pick the {color} cube", "/arm/pick", payload, 20.0)
        if result.get("outcome") == "out of reach" and self._into_reach(result["cube"]):
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
                    held: str | None = None) -> bool:
        """Turn to face a base-frame spot and creep (<= 0.15 m) until it sits ~reach_x ahead.

        The creep bypasses the costmap (it inflates the cubes themselves), so it is capped by every
        cube in view: none may end closer than keep_x (bumper ~0.125 m). Live 2026-10-16 the
        uncapped creep drove into the nearer cubes."""
        if self.dry_run:
            return False
        moved = False
        if math.hypot(spot["x"], spot["y"]) < keep_x:
            # Too close to grasp and the base can't reverse: strafe away from it, then face it
            # (live 2026-10-17 a cube beside the jaw needed a human otherwise).
            side = 1.0 if spot["y"] >= 0 else -1.0
            step = min(math.sqrt(max(reach_x ** 2 - spot["x"] ** 2, 0.0)) - abs(spot["y"]), 0.15)
            if step < 0.02:
                return False
            self.arm_pose("forward")  # move() needs the verified forward camera
            try:
                self.move(step, "right" if side > 0 else "left")
            except RuntimeError:
                return False  # costmap says the side is blocked
            spot, moved = {"x": spot["x"], "y": spot["y"] + side * step}, True
        bearing = math.degrees(math.atan2(spot["y"], spot["x"]))
        distance = math.hypot(spot["x"], spot["y"])
        # Steep-only grasps end ~0.25 m; +0.04 slack here once called 0.275 "close" (S3 #1).
        if abs(bearing) < 8.0 and distance <= reach_x + 0.02:
            return moved  # in front and close: moving forward won't help
        if abs(bearing) >= 8.0:
            self.turn(bearing)  # in place; needs no forward camera
            spot, moved = {"x": distance, "y": 0.0}, True
        # The held cube shows at the image bottom (S2 #4 refused the creep because of it).
        cubes = [c for c in self.find_cubes().get("cubes", []) if held is None or c.get("color") != held]
        # Corridor = robot half-width ~0.1 m + half a cube.
        nearest = min([c["x"] for c in cubes if abs(c["y"]) < 0.13] + [spot["x"]])
        creep = min(spot["x"] - reach_x, nearest - keep_x, 0.15)
        if creep < 0.02:
            return moved  # can't get closer without hitting another cube
        self.drive(vx=0.1, seconds=creep / 0.1)
        return True

    def follow(self, target: str = "person", seconds: float = 60.0, distance_m: float = 0.8):
        seconds, distance_m = float(seconds), float(distance_m)
        if not 3.0 <= seconds <= 300.0 or not 0.3 <= distance_m <= 3.0:
            raise ValueError("seconds 3..300, distance_m 0.3..3")
        return self._motion(f"follow {target} for {seconds} s", "/arm/follow",
                            {"target": str(target), "seconds": seconds,
                             "distance_m": distance_m}, seconds + 15.0)

    def _pose(self):
        return self.where_am_i()["pose"]  # map [x, y, yaw]

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
            if not 2 <= len(order) <= 3 or len(set(order)) != len(order) or                     not set(order) <= set(self.CUBE_COLORS):
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
        wanted = order or sorted(world)
        if not set(wanted) <= set(world):
            return {"outcome": f"cubes not seen: {sorted(set(wanted) - set(world))}", "log": log}
        base = order[0] if order else min(wanted, key=lambda c: math.hypot(
            self._to_base(pose, world[c])["x"] - 0.22, self._to_base(pose, world[c])["y"]))
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
            if picked.get("outcome") == "not seen" and color in world:
                self._into_reach(self._to_base(pose, world[color]), reach_x=0.22)
                picked = self.pick(color)
            log.append({"pick": color, **picked})
            if picked.get("outcome") != "holding":
                if last == (color, picked.get("outcome")):
                    break  # same failure twice: no progress, report instead of looping
                last = (color, picked.get("outcome"))
                look()
                continue
            last = None
            here = self._to_base(self._pose(), stack_xy)
            self._into_reach(here, reach_x=0.22, held=color)
            placed = self.place(on=stack[-1], level=len(stack), color=color,
                                pitch_deg=picked.get("pitch_deg"))
            log.append({"place_on": stack[-1], **placed})
            if placed.get("outcome") == "placed":
                stack.append(color)
            else:
                self.gripper("open")
                look()  # the cube is on the floor somewhere: back in the pool
        self.arm_pose("forward")
        done = len(stack) == len(wanted)
        return {"outcome": "stacked" if done else f"stack of {len(stack)}", "stack": stack,
                "log": log}

    def unstack_cubes(self, stack: list[str]):
        """Take a tower (bottom first) apart top-down onto the floor 0.10 m either side of it.
        Stops at the first failure and never moves the body: live 2026-10-16 an ad-hoc reset kept
        going after a failed place and dropped a cube out of view."""
        stack = list(stack)
        if not 2 <= len(stack) <= 3 or len(set(stack)) != len(stack) or                 not set(stack) <= set(self.CUBE_COLORS):
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
            spot = {"x": tower["x"], "y": tower["y"] + sides.pop(0)}
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
                  "objects": sorted(seen), "new_objects": new}
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
                   "say", "seen_objects", "wander", "turn", "move",
                   "approach", "roam_step", "map_room", "use_map", "list_maps",
                   "find_cubes", "pick", "place", "follow", "stack_cubes",
                   "unstack_cubes"}
        if name not in allowed:
            raise ValueError(f"unknown skill {name!r}")
        return getattr(self, name)(**arguments)
