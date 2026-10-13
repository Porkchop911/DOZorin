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
