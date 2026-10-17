import importlib.util
import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/hwr_agent"))

from hwr_agent import GatewaySkills  # noqa: E402

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
    asked = []
    monkeypatch.setattr(skills, "describe_view", lambda q: asked.append(q) or {"description": "x"})
    first = skills.roam_step()
    assert first["new_objects"] == ["chair", "person"] and len(asked) == 1
    second = GatewaySkills(token="test")  # fresh process, same state file
    monkeypatch.setattr(second, "_get", skills._get)
    monkeypatch.setattr(second, "wander", skills.wander)
    monkeypatch.setattr(second, "where_am_i", skills.where_am_i)
    monkeypatch.setattr(second, "describe_view", skills.describe_view)
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

    def __init__(self, skills, cubes, picks, places):
        self.calls, self.cubes, self.picks, self.places = [], cubes, list(picks), list(places)
        skills._motion = self.motion
        skills.where_am_i = lambda: {"pose": [0.0, 0.0, 0.0], "localization": {}}
        skills._act = lambda description, path, payload=None: self.calls.append((path, payload))
        skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: self.calls.append(("/drive", seconds))

    def motion(self, description, path, payload, seconds):
        self.calls.append((path, payload))
        if path == "/arm/find_cubes":
            return {"cubes": [dict(c) for c in self.cubes]}
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
                   [{"outcome": "placed"}] * 2)
    result = skills.stack_cubes()
    assert result["outcome"] == "stacked"
    assert result["stack"] == ["red", "blue", "green"]  # red nearest (0.22, 0); then nearest-first
    places = [p for path, p in fake.calls if path == "/arm/place"]
    assert [(p["on"], p["level"], p["pitch_deg"]) for p in places] == [("red", 1, -80),
                                                                         ("blue", 2, -80)]


def test_stack_cubes_retries_after_a_miss_and_a_fall():
    skills = GatewaySkills(token="t")
    fake = FakeArm(skills, CUBES,
                   [{"outcome": "missed"}] + [{"outcome": "holding", "pitch_deg": -80}] * 3,
                   [{"outcome": "not on target"}, {"outcome": "placed"}, {"outcome": "placed"}])
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


def test_into_reach_creep_never_closes_on_another_cube():
    skills = GatewaySkills(token="t")
    drives = []
    skills.find_cubes = lambda: {"cubes": [{"x": 0.30, "y": 0.0}, {"x": 0.24, "y": -0.08}]}
    skills.drive = lambda vx=0.0, vy=0.0, wz=0.0, seconds=1.0: drives.append(vx * seconds)
    assert skills._into_reach({"x": 0.30, "y": 0.0})
    assert drives == [pytest.approx(0.05)]  # target wants 0.05, the 0.24 cube allows 0.05
    drives.clear()
    skills.find_cubes = lambda: {"cubes": [{"x": 0.36, "y": 0.0}, {"x": 0.20, "y": 0.05}]}
    assert not skills._into_reach({"x": 0.36, "y": 0.0}) and drives == []


def test_into_reach_ignores_the_held_cube():
    skills = GatewaySkills(token="t")
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
