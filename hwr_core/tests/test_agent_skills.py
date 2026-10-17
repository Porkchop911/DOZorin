import math
import importlib.util
import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/hwr_agent"))

from hwr_agent import GatewaySkills  # noqa: E402
import hwr_agent.skills as skills_mod  # noqa: E402

CLI_SPEC = importlib.util.spec_from_file_location("hwr_cli", ROOT / "cli/hwr.py")
HWR_CLI = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(HWR_CLI)


def test_dry_run_never_needs_token_or_network():
    skills = GatewaySkills(dry_run=True)
    result = skills.goto(1.0, 2.0, 0.5)
    assert result["dispatch"]["dry_run"]
    assert result["result"]["state"] == "would-wait"


def test_action_budget_and_explore_bounds():
    skills = GatewaySkills(dry_run=True, max_actions=1)
    skills.arm_pose("forward")
    with pytest.raises(RuntimeError, match="budget"):
        skills.arm_pose("floor")
    assert skills.stop()["dry_run"]  # stop never consumes or respects the budget
    with pytest.raises(ValueError):
        GatewaySkills(dry_run=True).explore(20)
    with pytest.raises(ValueError):
        GatewaySkills(dry_run=True).drive(0.1, seconds=10)
    with pytest.raises(ValueError):
        GatewaySkills(dry_run=True).gripper("crush")


def test_openai_manifest_matches_callable_skills():
    manifest = json.loads((ROOT / "ros2_ws/src/hwr_agent/config/tools.json").read_text())
    skills = GatewaySkills(dry_run=True)
    assert manifest
    assert all(callable(getattr(skills, item["function"]["name"], None))
               for item in manifest)


def test_agent_exposes_no_power_tool_or_general_status_read(monkeypatch):
    manifest = json.loads((ROOT / "ros2_ws/src/hwr_agent/config/tools.json").read_text())
    names = {item["function"]["name"] for item in manifest}
    skills = GatewaySkills(dry_run=True)
    assert "battery" not in names
    assert not hasattr(skills, "battery")
    with pytest.raises(ValueError, match="unknown skill"):
        skills.call("battery", {})

    live = GatewaySkills(token="test")
    requested = []

    def fake_get(path, binary=False):
        requested.append(path)
        return {"pose": [1.0, 2.0, 0.0], "localization": {"healthy": True}}

    monkeypatch.setattr(live, "_get", fake_get)
    assert live.where_am_i()["pose"] == [1.0, 2.0, 0.0]
    assert requested == ["/navigation/context"]


def test_goal_polling_uses_navigation_context(monkeypatch):
    skills = GatewaySkills(token="test")
    requested = []

    def fake_get(path, binary=False):
        requested.append(path)
        return {"current_goal": {"state": "succeeded"}}

    monkeypatch.setattr(skills, "_get", fake_get)
    assert skills._wait_goal()["state"] == "succeeded"
    assert requested == ["/navigation/context"]


def test_named_object_goal_fails_closed_without_proximity_proof(monkeypatch):
    skills = GatewaySkills(token="test")
    monkeypatch.setattr(
        skills, "_act",
        lambda *args, **kwargs: {"accepted": True, "place_kind": "object"})
    monkeypatch.setattr(skills, "_wait_goal",
                        lambda: {"state": "succeeded"})

    result = skills.goto_place("window")["result"]
    assert result["state"] == "failed"
    assert result["object_reached"] is False
    assert "within 0.50 m" in result["message"]


def test_named_object_goal_accepts_gateway_proximity_proof(monkeypatch):
    skills = GatewaySkills(token="test")
    monkeypatch.setattr(
        skills, "_act",
        lambda *args, **kwargs: {"accepted": True, "place_kind": "object"})
    monkeypatch.setattr(skills, "_wait_goal", lambda: {
        "state": "succeeded", "object_reached": True,
        "object_distance_m": 0.42, "max_object_distance_m": 0.50,
    })

    result = skills.goto_place("drums")["result"]
    assert result["state"] == "succeeded"
    assert result["object_distance_m"] == 0.42


def test_scene_assessment_combines_battery_free_context_and_snapshot(monkeypatch):
    skills = GatewaySkills(token="test")
    requested = []
    context = {"pose": [0.0, 0.0, 0.0],
               "hard_clearance_m": {"forward": 1.25}}

    def fake_get(path, binary=False):
        requested.append((path, binary))
        return b"jpeg" if binary else context

    def fake_ask_vlm(image, prompt):
        assert image == b"jpeg"
        assert '"forward":1.25' in prompt
        return """```json
{"safe_to_translate":true,"doorway_visible":false,
 "person_visible":false,"hazards":["cable"],
 "visual_corridor":"uncertain","recommendation":"sensor_preflight",
 "rationale":"metric and semantic checks should agree",
 "doorway_evidence":"","person_evidence":""}
```"""

    monkeypatch.setattr(skills, "_get", fake_get)
    monkeypatch.setattr(skills, "_ask_vlm", fake_ask_vlm)
    result = skills.assess_scene()
    assert not result["assessment"]["safe_to_translate"]
    assert result["assessment"]["model_safe_to_translate"]
    assert result["assessment"]["consistency_veto"] == [
        "corridor not visually clear", "reported floor hazard"]
    assert result["navigation_context"] == context
    assert requested == [("/navigation/context", False), ("/snapshot", True)]


def test_cli_scene_assessment_applies_same_consistency_veto():
    assessment = HWR_CLI._parse_assessment(json.dumps({
        "safe_to_translate": True,
        "hazards": ["wire on floor"],
        "visual_corridor": "clear floor with clutter",
        "recommendation": "sensor_preflight",
    }))
    assert not assessment["safe_to_translate"]
    assert assessment["model_safe_to_translate"]
    assert assessment["consistency_veto"] == [
        "corridor not visually clear", "reported floor hazard"]


def test_cli_vetoes_obstruction_reported_in_corridor_text():
    assessment = HWR_CLI._parse_assessment(json.dumps({
        "safe_to_translate": True,
        "hazards": [],
        "visual_corridor": "clear floor path with furniture obstruction",
        "recommendation": "sensor_preflight",
        "rationale": "sensor verification required",
    }))
    assert not assessment["safe_to_translate"]
    assert assessment["model_safe_to_translate"]
    assert assessment["consistency_veto"] == ["corridor not visually clear"]


def test_cli_does_not_veto_explicitly_absent_obstruction():
    assessment = HWR_CLI._parse_assessment(json.dumps({
        "safe_to_translate": True,
        "hazards": [],
        "visual_corridor": "wooden floor with no visible obstructions",
        "recommendation": "sensor_preflight",
        "rationale": "path is clear",
    }))
    assert assessment["safe_to_translate"]
    assert "consistency_veto" not in assessment


def test_cli_vetoes_hazards_admitted_only_in_corridor_text():
    assessment = HWR_CLI._parse_assessment(json.dumps({
        "safe_to_translate": True,
        "hazards": [],
        "visual_corridor": "floor with chair base, rack leg, and cable visible",
        "recommendation": "sensor_preflight",
        "rationale": "metric preflight remains required",
    }))
    assert not assessment["safe_to_translate"]
    assert assessment["consistency_veto"] == ["corridor not visually clear"]


def test_cli_corrects_contradictory_hold_with_positive_clear_evidence():
    assessment = HWR_CLI._parse_assessment(json.dumps({
        "safe_to_translate": True,
        "hazards": [],
        "visual_corridor": "wood-floor lane",
        "recommendation": "hold",
        "rationale": "no hazards; no motion commanded",
    }))
    assert assessment["safe_to_translate"]
    assert assessment["model_recommendation"] == "hold"
    assert assessment["recommendation"] == "sensor_preflight"
    assert "consistency_veto" not in assessment


def test_agent_corrects_contradictory_hold_with_positive_clear_evidence(
        monkeypatch):
    skills = GatewaySkills(token="test")
    context = {"pose": [0.0, 0.0, 0.0],
               "hard_clearance_m": {"forward": 1.4}}

    monkeypatch.setattr(
        skills, "_get",
        lambda path, binary=False: b"jpeg" if binary else context)
    monkeypatch.setattr(skills, "_ask_vlm", lambda image, prompt: json.dumps({
        "safe_to_translate": True,
        "doorway_visible": False,
        "person_visible": False,
        "hazards": [],
        "visual_corridor": "wood-floor lane",
        "recommendation": "hold",
        "rationale": "no hazards; no motion commanded",
        "doorway_evidence": "",
        "person_evidence": "",
    }))

    assessment = skills.assess_scene()["assessment"]
    assert assessment["safe_to_translate"]
    assert assessment["model_recommendation"] == "hold"
    assert assessment["recommendation"] == "sensor_preflight"
    assert "consistency_veto" not in assessment


def test_seen_objects_reads_vision_endpoint_as_text(monkeypatch):
    skills = GatewaySkills(token="test")
    monkeypatch.setattr(skills, "_get", lambda path, binary=False: {
        "infer_ms": 12, "objects": [
            {"class": "person", "conf": 0.9, "id": 3, "range_m": 1.2, "bearing_deg": 20},
            {"class": "cup", "conf": 0.4, "id": None, "bearing_deg": 0}]})
    assert skills.seen_objects()["objects"] == ["person #3 (0.9) 1.2 m, 20 deg left",
                                                "cup (0.4) ahead"]


def test_turn_and_move_are_bounded_and_refuse_reverse():
    skills = GatewaySkills(dry_run=True)
    assert skills.turn(180)["dry_run"] and skills.move(0.5, "left")["dry_run"]
    for bad in (lambda: skills.turn(0), lambda: skills.turn(400),
                lambda: skills.move(3.0), lambda: skills.move(0.5, "back")):
        with pytest.raises(ValueError):
            bad()


def test_roam_step_describes_only_new_objects(tmp_path, monkeypatch):
    monkeypatch.setenv("HWR_ROAM_STATE", str(tmp_path / "seen.json"))
    skills = GatewaySkills(token="test")
    frames = iter([["chair"], ["chair", "person"], ["chair"], ["chair", "person"]])
    monkeypatch.setattr(skills, "_get", lambda path, binary=False: {
        "objects": [{"class": c} for c in next(frames)]})
    monkeypatch.setattr(skills, "wander", lambda d: {"result": {"state": "succeeded"}, "recovered": []})
    monkeypatch.setattr(skills, "where_am_i", lambda: {"pose": [0, 0, 0]})
    monkeypatch.setattr(skills, "anchor_places", lambda: {"anchored": [{"name": "chair"}]})
    asked = []
    monkeypatch.setattr(skills, "describe_view", lambda q: asked.append(q) or {"description": "x"})
    first = skills.roam_step()
    assert first["new_objects"] == ["chair", "person"] and len(asked) == 1
    assert first["anchored"] == ["chair"]
    second = GatewaySkills(token="test")  # fresh process, same state file
    monkeypatch.setattr(second, "_get", skills._get)
    monkeypatch.setattr(second, "wander", skills.wander)
    monkeypatch.setattr(second, "where_am_i", skills.where_am_i)
    monkeypatch.setattr(second, "describe_view", skills.describe_view)
    monkeypatch.setattr(second, "anchor_places", skills.anchor_places)
    assert second.roam_step()["new_objects"] == [] and len(asked) == 1


def test_wander_points_arm_forward_and_relocalizes_without_the_llm(monkeypatch):
    skills = GatewaySkills(token="test")
    calls = []
    monkeypatch.setattr(skills, "where_am_i", lambda: {"localization": {"healthy": True}})
    monkeypatch.setattr(skills, "_get", lambda path, binary=False: {"x": 1, "y": 0, "yaw": 0})
    monkeypatch.setattr("time.sleep", lambda s: None)
    results = iter([RuntimeError("gateway HTTP 409: translation requires verified forward camera pose"),
                    {"state": "failed", "message": "localization health lost during translation"},
                    {"state": "succeeded"}])

    def goto(x, y, yaw):
        r = next(results)
        if isinstance(r, Exception):
            raise r
        return {"result": r}
    monkeypatch.setattr(skills, "goto", goto)
    monkeypatch.setattr(skills, "arm_pose", lambda n: calls.append("arm " + n))
    monkeypatch.setattr(skills, "relocalize", lambda: calls.append("reloc"))
    out = skills.wander()
    assert out["result"]["state"] == "succeeded"
    assert calls == ["arm forward", "reloc"]
    assert out["recovered"] == ["arm forward", "relocalized after failure"]


def test_map_tools_validate_names_before_touching_the_robot():
    skills = GatewaySkills(dry_run=True)
    assert skills.map_room("living_room", 5)["dry_run"]
    assert skills.use_map("room_20261013")["dry_run"]
    for bad in ("../etc", "Room", "a b", "x;rm -rf", ""):
        with pytest.raises(ValueError):
            skills.map_room(bad)
        with pytest.raises(ValueError):
            skills.use_map(bad)
    with pytest.raises(ValueError):
        skills.map_room("ok", 60)


class FakeArm:
    """Scripted gateway: cubes on the floor in base frame at pose (0, 0, 0)."""

    def __init__(self, skills, cubes, picks, places, final=None):
        self.calls, self.cubes, self.picks, self.places = [], cubes, list(picks), list(places)
        self.final = final  # what look_down shows once every scripted place is used up
        skills._motion = self.motion
        skills._pose = lambda: [0.0, 0.0, 0.0]
        skills._act = lambda description, path, payload=None: self.calls.append((path, payload))
        skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: self.calls.append(("/drive", seconds))

    def motion(self, description, path, payload, seconds):
        self.calls.append((path, payload))
        if path == "/arm/find_cubes":
            cubes = self.cubes(payload) if callable(self.cubes) else self.cubes
            if self.final is not None and not self.places:
                cubes = self.final
            return {"cubes": [dict(c) for c in cubes]}
        if path == "/arm/pick":
            return dict(self.picks.pop(0))
        if path == "/arm/place":
            return dict(self.places.pop(0))
        return {"outcome": "done"}


CUBES = [{"color": "red", "x": 0.30, "y": 0.0}, {"color": "green", "x": 0.22, "y": 0.10},
         {"color": "blue", "x": 0.20, "y": -0.09}]


def test_stack_cubes_any_order_picks_base_and_nearest_first():
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES, [{"outcome": "holding", "pitch_deg": -80}] * 2,
                   [{"outcome": "placed"}] * 2, final=[{"color": "green", "x": 0.3, "y": 0.0}])
    result = skills.stack_cubes()
    assert result["outcome"] == "stacked"
    assert result["stack"] == ["red", "blue", "green"]  # red farthest (base); then nearest-first
    places = [p for path, p in fake.calls if path == "/arm/place"]
    assert [(p["on"], p["level"], p["pitch_deg"]) for p in places] == [("red", 1, -80),
                                                                         ("blue", 2, -80)]


def test_stack_cubes_retries_after_a_miss_and_a_fall():
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES,
                   [{"outcome": "missed"}] + [{"outcome": "holding", "pitch_deg": -80}] * 3,
                   [{"outcome": "not on target"}, {"outcome": "placed"}, {"outcome": "placed"}],
                   final=[{"color": "blue", "x": 0.3, "y": 0.0}, {"color": "green", "x": 0.31, "y": 0.02}])
    result = skills.stack_cubes(["red", "green", "blue"])
    assert result["outcome"] == "stacked" and result["stack"] == ["red", "green", "blue"]
    assert [p for p, _ in fake.calls].count("/arm/pick") == 4


def test_stack_cubes_gives_up_after_max_rounds():
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES, [{"outcome": "missed"}] * 3, [])
    result = skills.stack_cubes(["red", "blue"], max_rounds=3)
    assert result["outcome"] == "stack of 1"
    assert [p for p, _ in fake.calls].count("/arm/pick") == 2  # same failure twice -> stop
    with pytest.raises(ValueError):
        skills.stack_cubes(["red", "red"])


def test_into_reach_creep_never_closes_on_another_cube(monkeypatch):
    skills = GatewaySkills(token="t")
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills.arm_pose = lambda name: None
    drives = []
    skills.find_cubes = lambda: {"cubes": [{"x": 0.30, "y": 0.0}, {"x": 0.24, "y": -0.08}]}
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: drives.append(vx * seconds)
    assert skills._into_reach({"x": 0.30, "y": 0.0}, detour=False)
    assert drives == [pytest.approx(0.05)]  # target wants 0.05, the 0.24 cube allows 0.05
    drives.clear()
    skills.find_cubes = lambda: {"cubes": [{"x": 0.36, "y": 0.0}, {"x": 0.20, "y": 0.05}]}
    assert not skills._into_reach({"x": 0.36, "y": 0.0}, detour=False) and drives == []


def test_into_reach_ignores_the_held_cube(monkeypatch):
    skills = GatewaySkills(token="t")
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills.arm_pose = lambda name: None
    drives = []
    skills.find_cubes = lambda: {"cubes": [{"color": "green", "x": 0.15, "y": 0.0},
                                           {"color": "red", "x": 0.30, "y": 0.0}]}
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: drives.append(vx * seconds)
    assert skills._into_reach({"x": 0.30, "y": 0.0}, held="green")
    assert drives == [pytest.approx(0.08)]  # to reach_x 0.22; the held green at 0.15 is ignored


def test_unstack_cubes_top_first_beside_the_tower_and_stops_on_failure():
    skills = GatewaySkills(token="t")
    tower = {"x": 0.22, "y": 0.03}
    fake = FakeArm(skills, CUBES, [{"outcome": "holding", "cube": tower}] * 2,
                   [{"outcome": "placed"}] * 2)
    assert skills.unstack_cubes(["red", "green", "blue"])["outcome"] == "unstacked"
    picks = [(p["color"], p["level"]) for path, p in fake.calls if path == "/arm/pick"]
    places = [(p["x"], round(p["y"], 2), p["color"]) for path, p in fake.calls if path == "/arm/place"]
    assert picks == [("blue", 2), ("green", 1)]
    assert places == [(0.22, -0.07, "blue"), (0.22, 0.13, "green")]
    fake = FakeArm(skills, CUBES, [{"outcome": "holding", "cube": tower}] * 2,
                   [{"outcome": "not on target"}])
    result = skills.unstack_cubes(["red", "green", "blue"])
    assert result["outcome"] == "stopped: not on target"
    assert [p for p, _ in fake.calls].count("/arm/pick") == 1 and "/drive" not in dict(fake.calls)
    with pytest.raises(ValueError):
        skills.unstack_cubes(["red"])


def test_into_reach_strafes_away_from_a_cube_too_close_then_faces_it(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    calls = []
    skills.arm_pose = lambda name: calls.append(("pose", name))
    skills.move = lambda meters, direction="forward": calls.append(("move", round(meters, 3), direction))
    skills.turn = lambda degrees: calls.append(("turn", round(degrees)))
    skills.find_cubes = lambda: {"cubes": []}
    assert skills._into_reach({"x": 0.14, "y": 0.10})  # beside the left jaw
    assert calls == [("pose", "forward"), ("move", 0.07, "right"), ("pose", "forward"), ("turn", 50)]


def test_into_reach_strafes_around_a_cube_in_the_way(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    calls = []
    views = [{"cubes": [{"color": "green", "x": 0.20, "y": 0.0}, {"color": "red", "x": 0.36, "y": 0.0}]},
             {"cubes": [{"color": "green", "x": 0.20, "y": -0.16}, {"color": "red", "x": 0.36, "y": -0.16}]}]
    skills.arm_pose = lambda name: None
    skills.move = lambda meters, direction="forward": calls.append(("move", round(meters, 2), direction))
    skills.turn = lambda degrees: calls.append(("turn", round(degrees)))
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: calls.append(("drive", round(vx * seconds, 2)))
    skills.find_cubes = lambda: views.pop(0)
    assert skills._into_reach({"x": 0.36, "y": 0.0})
    # Stack 0.16 off the centre line; creep until red sits on the 0.22 reach circle, no turn.
    assert calls == [("move", 0.16, "left"), ("drive", 0.21)]
    assert skills_mod._detour({"x": 0.36, "y": 0.0}, [{"x": 0.2, "y": 0.0}]) == 0.16
    assert skills_mod._detour({"x": 0.36, "y": 0.0}, [{"x": 0.2, "y": 0.1}, {"x": 0.2, "y": -0.1}]) is None


def test_stack_cubes_searches_around_for_cubes_out_of_view():
    skills = GatewaySkills(token="t")
    turns = []
    skills.turn = lambda degrees: turns.append(degrees)
    seen = {"n": 0}

    def view(payload):  # only red in front; green + blue show up in the floor view after 1 turn
        if payload["view"] == "floor":
            seen["n"] += 1
            return [c for c in CUBES if c["color"] != "red"] if seen["n"] >= 4 else []
        return [CUBES[0]]

    fake = FakeArm(skills, view, [{"outcome": "holding", "pitch_deg": -80}] * 2, [{"outcome": "placed"}] * 2,
                   final=[{"color": "green", "x": 0.3, "y": 0.0}])
    result = skills.stack_cubes()
    assert turns == [120] and result["outcome"] == "stacked"
    floor = [p.get("pan_deg", 0) for path, p in fake.calls if path == "/arm/find_cubes" and p["view"] == "floor"]
    assert floor == [0, 50, -50] * 2  # the arm sweeps before the body turns


def test_look_around_swings_the_arm_and_stops_when_the_target_is_seen(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    aims, poses = [], []
    skills.arm_aim = lambda bearing, elevation=0.0: aims.append(round(math.degrees(bearing)))
    skills.arm_pose = poses.append
    skills.drive = skills.turn = skills.move = lambda *a, **k: pytest.fail("the body moved")
    frames = [[], [{"class": "bottle", "bearing_deg": -10.0}]]
    skills._get = lambda path: {"objects": frames.pop(0)}
    result = skills.look_around("bottle")
    assert aims == [0, 50] and poses == ["forward"]
    assert result["target_seen"] and result["found"][0]["bearing_deg"] == 40.0


def test_stack_cubes_looks_up_before_turning():
    skills = GatewaySkills(token="t")
    skills.turn = lambda degrees: pytest.fail("turned although the floor view here shows the cubes")
    FakeArm(skills, lambda payload: CUBES if payload["view"] == "floor" else [],
            [{"outcome": "holding", "pitch_deg": -80}] * 2, [{"outcome": "placed"}] * 2,
            final=[{"color": "green", "x": 0.3, "y": 0.0}])
    assert skills.stack_cubes()["outcome"] == "stacked"


def test_stack_cubes_never_drops_a_held_cube_and_checks_the_tower():
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES, [{"outcome": "holding", "pitch_deg": -80}] * 2,
                   [{"outcome": "target not seen"}, {"outcome": "placed"}, {"outcome": "placed"}],
                   final=[{"color": "blue", "x": 0.3, "y": 0.0}, {"color": "red", "x": 0.45, "y": 0.1}])
    result = skills.stack_cubes(["red", "green", "blue"])
    assert not any(path == "/gripper" for path, _ in fake.calls)
    assert [p["on"] for path, p in fake.calls if path == "/arm/place"] == ["red", "red", "green"]
    assert result["outcome"] == "stack of 3 unverified"  # red seen 0.18 m from the top cube


def test_stack_cubes_sweeps_the_arm_for_a_lost_tower(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES, [{"outcome": "holding", "pitch_deg": -90}],
                   [{"outcome": "target not seen"}, {"outcome": "placed"}],
                   final=[{"color": "red", "x": 0.3, "y": 0.0}])
    skills.turn = lambda degrees: pytest.fail("the body turned to find the tower")
    skills.stack_cubes(["red", "green"])
    sweeps = [p.get("pan_deg", 0) for path, p in fake.calls
              if path == "/arm/find_cubes" and p["view"] == "floor"]
    first_place = next(i for i, (path, _) in enumerate(fake.calls) if path == "/arm/place")
    after = [p for path, p in fake.calls[first_place:] if path == "/arm/find_cubes" and p["view"] == "floor"]
    assert after and after[0] == {"view": "floor"}  # red is the base: level 0, seen at pan 0 -> stop
    assert sweeps


def test_into_reach_detours_when_a_cube_only_partly_caps_the_creep(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    calls = []
    views = [{"cubes": [{"x": 0.30, "y": 0.0}, {"x": 0.21, "y": -0.08}]},  # creep capped at 0.02
             {"cubes": [{"x": 0.30, "y": -0.15}, {"x": 0.21, "y": -0.23}]}]
    skills.arm_pose = lambda name: None
    skills.move = lambda meters, direction="forward": calls.append(("move", round(meters, 2), direction))
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: calls.append(("drive", round(vx * seconds, 2)))
    skills.find_cubes = lambda: views.pop(0)
    assert skills._into_reach({"x": 0.30, "y": 0.0})
    assert calls[0][0] == "move" and calls[1] == ("drive", 0.1)  # red ends 0.22 out, not under the jaws


def test_unstack_stops_at_first_failed_place_and_never_moves_the_body(monkeypatch):
    skills = GatewaySkills(token="test")
    calls = []

    def fake_motion(description, path, payload, seconds):
        calls.append((path, payload))
        if path == "/arm/pick":
            return {"outcome": "holding", "cube": {"x": 0.15, "y": 0.04}}
        return {"outcome": "fell"}

    monkeypatch.setattr(skills, "_motion", fake_motion)
    for body_move in ("turn", "move", "drive", "_into_reach"):
        monkeypatch.setattr(skills, body_move, lambda *a, **k: pytest.fail("body moved"))
    result = skills.unstack_cubes(["red", "green", "blue"])
    assert result["outcome"] == "stopped: fell"
    assert [p for p, _ in calls] == ["/arm/pick", "/arm/place"]  # no second pick after the failure
    assert calls[0][1] == {"color": "blue", "level": 2}  # top first
    spot = calls[1][1]
    assert spot["x"] == 0.19  # never closer than 0.19 m (bumper misses)
    assert spot["y"] == pytest.approx(-0.06)  # tower at +0.04: the side nearer the centre first


def _stack_fixture(monkeypatch, cubes, place_outcomes):
    skills = GatewaySkills(token="test")
    log = {"picks": [], "places": []}
    pending = list(place_outcomes)
    monkeypatch.setattr(skills, "_pose", lambda: [0.0, 0.0, 0.0])
    monkeypatch.setattr(skills, "find_cubes", lambda *a, **k: {"cubes": [dict(c) for c in cubes]})
    monkeypatch.setattr(skills, "arm_pose", lambda name: {})
    monkeypatch.setattr(skills, "turn", lambda d: pytest.fail("search turn with all cubes in view"))
    monkeypatch.setattr(skills, "_into_reach", lambda *a, **k: True)

    def pick(color, level=0):
        log["picks"].append(color)
        return {"outcome": "holding", "pitch_deg": -80.0}

    def place(on=None, level=1, x=None, y=None, color=None, pitch_deg=None):
        log["places"].append({"on": on, "x": x, "color": color})
        return {"outcome": pending.pop(0) if on is not None else "placed"}

    monkeypatch.setattr(skills, "pick", pick)
    monkeypatch.setattr(skills, "place", place)
    return skills, log


def test_stack_builds_on_the_farthest_cube_within_reach_range(monkeypatch):
    cubes = [{"color": "red", "x": 0.20, "y": 0.0}, {"color": "green", "x": 0.35, "y": 0.05},
             {"color": "blue", "x": 0.60, "y": 0.0}]  # blue beyond 0.40 m: not a base candidate
    skills, log = _stack_fixture(monkeypatch, cubes, ["placed", "placed"])
    result = skills.stack_cubes()
    assert result["stack"][0] == "green"
    assert log["picks"][0] == "red"  # nearest remaining cube is carried first


def test_stack_never_opens_mid_air_when_the_tower_is_lost(monkeypatch):
    cubes = [{"color": "red", "x": 0.20, "y": 0.0}, {"color": "green", "x": 0.30, "y": 0.0}]
    skills, log = _stack_fixture(monkeypatch, cubes, ["target not seen", "target not seen"] * 8)
    result = skills.stack_cubes(order=["green", "red"], max_rounds=1)
    set_downs = [p for p in log["places"] if p["on"] is None]
    assert set_downs == [{"on": None, "x": 0.22, "color": "red"}]  # set down in front, on the floor
    assert result["stack"] == ["green"] and result["outcome"] == "stack of 1"


def test_into_reach_lifts_the_arm_before_moving_the_body(monkeypatch):
    skills = GatewaySkills(token="t")
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    calls = []
    skills.arm_pose = lambda name: calls.append(("arm", name))
    skills.turn = lambda degrees: calls.append(("turn", degrees))
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: calls.append(("drive", vx * seconds))
    skills.find_cubes = lambda: {"cubes": []}
    assert skills._into_reach({"x": 0.30, "y": 0.10})  # held cube hangs at stack height in look_down
    assert [c[0] for c in calls] == ["arm", "turn", "arm", "drive"]


def test_stack_cubes_stops_when_the_tower_is_lost_twice(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    skills.turn = lambda degrees: None
    lost = {"outcome": "target not seen"}
    fake = FakeArm(skills, CUBES[:2], [{"outcome": "holding", "pitch_deg": -80}] * 8,
                   [lost, lost, {"outcome": "placed"}] * 4)
    result = skills.stack_cubes(order=["red", "green"])
    assert result["outcome"] == "stack of 1"
    assert sum(1 for path, _ in fake.calls if path == "/arm/pick") == 2


def test_into_reach_never_detours_around_its_own_target(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    calls = []
    skills.arm_pose = lambda name: None
    skills.turn = lambda degrees: calls.append(("turn", round(degrees)))
    skills.move = lambda meters, direction="forward": calls.append(("move", direction))
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: calls.append(("drive", round(vx * seconds, 3)))
    # S5 #5: estimate 0.30 m ahead after the turn, blue really at (0.225, 0.063)
    skills.find_cubes = lambda: {"cubes": [{"color": "blue", "x": 0.225, "y": 0.063}]}
    skills._into_reach({"x": 0.29, "y": 0.08}, held="red", target="blue")
    assert ("move", "right") not in calls and ("move", "left") not in calls


def test_into_reach_short_strafe_does_not_creep_the_target_under_the_jaws(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    calls = []
    # Cal 2026-10-17: red at the corridor edge, strafe left asked 0.04 m, went 0.01; blue still ahead.
    views = [{"cubes": [{"color": "red", "x": 0.197, "y": -0.128}, {"color": "blue", "x": 0.422, "y": -0.016}]},
             {"cubes": [{"color": "red", "x": 0.192, "y": -0.148}, {"color": "blue", "x": 0.422, "y": -0.047}]}]
    skills.arm_pose = lambda name: None
    skills.move = lambda meters, direction="forward": calls.append(("move", direction))
    skills.turn = lambda degrees: calls.append(("turn", round(degrees)))
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: calls.append(("drive", round(vx * seconds, 3)))
    skills.find_cubes = lambda: views.pop(0)
    skills._into_reach({"x": 0.422, "y": -0.016}, target="blue")
    drives = [c[1] for c in calls if c[0] == "drive"]
    assert drives and math.hypot(0.422 - sum(drives), 0.047) >= 0.22 - 1e-6  # on the reach circle


def test_map_room_reports_area_gained(monkeypatch):
    monkeypatch.setattr("hwr_agent.skills.time.sleep", lambda s: None)
    skills = GatewaySkills(token="t")
    statuses = iter([{"coverage": {"known_m2": 4.0}},
                     {"finished": {"reason": "exploration complete"}, "coverage": {"known_m2": 10.5},
                      "half_leg_retries": {"tried": 2, "passed": 1}}])
    skills._get = lambda path: {"maps": []} if path == "/maps" else next(statuses)
    skills._switch = lambda mode, name: None
    skills.arm_pose = lambda name: None
    skills._act = lambda description, path, payload=None: {"success": True}
    skills.use_map = lambda name: "switched"
    result = skills.map_room("room_x", 5)
    assert result["known_m2"] == 10.5 and result["known_m2_gained"] == 6.5
    assert result["half_leg_retries"] == {"tried": 2, "passed": 1}
