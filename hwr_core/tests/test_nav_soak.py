import math

import numpy as np
import pytest

from hwr_core.navigation import (assessment_from_visual_observation,
                                 bounded_escape_evidence,
                                 directional_escape_profile,
                                 metric_rotation_preflight,
                                 metric_translation_preflight,
                                 normalize_scene_assessment,
                                 rank_goal_candidates_by_clearance,
                                 reclassify_out_of_corridor_hazards,
                                 reachable_goal_candidates,
                                 semantic_candidate_score,
                                 semantic_context_for_prompt)


def test_reachable_goal_candidates_are_free_bounded_and_deterministic():
    grid = np.zeros((30, 30), dtype=np.uint8)
    grid[:, 16] = 255
    grid[15, 16] = 0
    kwargs = dict(resolution=0.1, origin=(0.0, 0.0), start_xy=(1.05, 1.55),
                  count=8, min_distance=0.3, max_distance=0.9, seed=7)
    first = reachable_goal_candidates(grid, **kwargs)
    second = reachable_goal_candidates(grid, **kwargs)
    assert first == second and len(first) == 8
    for x, y, yaw in first:
        assert 0.3 <= math.hypot(x - 1.05, y - 1.55) <= 0.9
        assert grid[math.floor(y / 0.1), math.floor(x / 0.1)] == 0
        assert -math.pi <= yaw <= math.pi


def test_unknown_band_is_not_used_as_a_path():
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[:, 10] = 180
    goals = reachable_goal_candidates(
        grid, 0.1, (0.0, 0.0), (0.55, 1.05), count=30,
        min_distance=0.3, max_distance=1.3, seed=3)
    assert goals
    assert all(x < 1.0 for x, _, _ in goals)


def test_goal_candidate_input_validation():
    grid = np.zeros((5, 5), dtype=np.uint8)
    with pytest.raises(ValueError):
        reachable_goal_candidates(grid, 0.0, (0.0, 0.0), (0.0, 0.0))
    with pytest.raises(ValueError):
        reachable_goal_candidates(grid, 0.1, (0.0, 0.0), (0.0, 0.0),
                                  min_distance=1.0, max_distance=0.5)


def test_goal_clearance_ranking_prefers_open_target_over_wall_target():
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[:, 10] = 255
    near_wall = (0.85, 0.55, 0.0)
    open_space = (0.25, 1.55, 0.0)
    ranked = rank_goal_candidates_by_clearance(
        grid, 0.1, (0.0, 0.0), [near_wall, open_space])
    assert ranked == [open_space, near_wall]


def test_semantic_preflight_vetoes_person_and_overhang_contradictions():
    assessment = normalize_scene_assessment({
        "safe_to_translate": True,
        "doorway_visible": False,
        "person_visible": True,
        "hazards": ["drying rack base and hanging clothes"],
        "visual_corridor": "open floor with rack base obstruction",
        "recommendation": "sensor_preflight",
        "rationale": "metric clearance appears sufficient",
        "doorway_evidence": "",
        "person_evidence": "seated person",
    })
    assert not assessment["safe_to_translate"]
    assert assessment["model_safe_to_translate"]
    assert assessment["consistency_veto"] == [
        "corridor not visually clear",
        "reported floor or overhang hazard"]


def test_visual_observation_is_not_motion_authority_and_stays_fail_closed():
    clear = {
        "visibility_sufficient": True,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "glass_boundary_visible": False,
        "glass_boundary_in_corridor": False,
        "glass_boundary_evidence": "",
        "hazards": [],
        "visual_corridor": "clear floor",
        "recommendation": "sensor_preflight",
        "rationale": "path is clear",
        "doorway_evidence": "",
        "person_evidence": "",
    }
    assert assessment_from_visual_observation(clear)["safe_to_translate"]
    outside_person = {
        **clear,
        "person_visible": True,
        "person_in_corridor": False,
        "rationale": "path is clear; seated person remains outside corridor",
        "person_evidence": "person at far left edge outside the path",
    }
    assert assessment_from_visual_observation(
        outside_person)["safe_to_translate"]
    assert not assessment_from_visual_observation(
        {**outside_person, "person_in_corridor": True})["safe_to_translate"]
    blocked = {**clear, "hazards": ["chair leg"]}
    assessment = assessment_from_visual_observation(blocked)
    assert not assessment["safe_to_translate"]
    assert "reported floor or overhang hazard" in assessment["consistency_veto"]
    assert not assessment_from_visual_observation(
        {**clear, "visibility_sufficient": False})["safe_to_translate"]
    with pytest.raises(ValueError):
        assessment_from_visual_observation(
            {**clear, "visibility_sufficient": "yes"})
    assert not assessment_from_visual_observation(
        {**clear, "rationale": "clear but dark near the endpoint"}
    )["safe_to_translate"]


def test_visual_observation_corrects_only_explicit_visibility_contradiction():
    observation = {
        "visibility_sufficient": False,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "glass_boundary_visible": False,
        "glass_boundary_in_corridor": False,
        "glass_boundary_evidence": "",
        "hazards": [],
        "visual_corridor": "clear",
        "recommendation": "sensor_preflight",
        "rationale": (
            "No visible hazards in swept envelope; camera forward view clear "
            "and sensor data fresh."),
        "doorway_evidence": "",
        "person_evidence": "",
    }
    assessment = assessment_from_visual_observation(observation)
    assert assessment["safe_to_translate"]
    assert assessment["visibility_sufficient"]
    assert assessment["model_visibility_sufficient"] is False
    assert "visibility_consistency_correction" in assessment

    uncertain = dict(observation)
    uncertain["rationale"] = (
        "No visible hazards in swept envelope, but low-light hides the endpoint.")
    assessment = assessment_from_visual_observation(uncertain)
    assert not assessment["safe_to_translate"]
    assert assessment["visibility_sufficient"] is False
    assert "visibility_consistency_correction" not in assessment


def test_visual_observation_vetoes_glass_boundary_in_corridor():
    observation = {
        "visibility_sufficient": True,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "glass_boundary_visible": True,
        "glass_boundary_in_corridor": True,
        "glass_boundary_evidence": "bright glazed plane closes the lane",
        "hazards": [],
        "visual_corridor": "clear floor terminating at a window",
        "recommendation": "sensor_preflight",
        "rationale": "floor is clear before the reflective boundary",
        "doorway_evidence": "",
        "person_evidence": "",
    }
    assessment = assessment_from_visual_observation(observation)
    assert not assessment["safe_to_translate"]
    assert "glass or mirror boundary in proposed corridor" in (
        assessment["consistency_veto"])
    assert not bounded_escape_evidence(assessment, 0.10)


def test_semantic_preflight_allows_person_outside_center_corridor():
    assessment = normalize_scene_assessment({
        "safe_to_translate": True,
        "doorway_visible": False,
        "person_visible": True,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear open floor lane",
        "recommendation": "sensor_preflight",
        "rationale": "path is clear; person remains outside the corridor",
        "doorway_evidence": "",
        "person_evidence": "seated well to the left of the lane",
    })
    assert assessment["safe_to_translate"]
    assert "consistency_veto" not in assessment


def test_semantic_preflight_does_not_veto_explicitly_absent_obstruction():
    assessment = normalize_scene_assessment({
        "safe_to_translate": True,
        "doorway_visible": False,
        "person_visible": False,
        "hazards": [],
        "visual_corridor": "wood floor with no visible obstructions",
        "recommendation": "sensor_preflight",
        "rationale": "path is clear",
        "doorway_evidence": "",
        "person_evidence": "",
    })
    assert assessment["safe_to_translate"]
    assert "consistency_veto" not in assessment


def test_semantic_preflight_never_turns_model_false_into_true():
    assessment = normalize_scene_assessment({
        "safe_to_translate": False,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear open floor lane",
        "recommendation": "sensor_preflight",
        "rationale": "No hazards intersecting the corridor.",
        "doorway_evidence": "",
        "person_evidence": "",
    })
    assert not assessment["safe_to_translate"]
    assert "consistency_correction" not in assessment


def test_semantic_preflight_does_not_correct_false_flag_under_uncertainty():
    assessment = normalize_scene_assessment({
        "safe_to_translate": False,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear floor in a dark corridor",
        "recommendation": "sensor_preflight",
        "rationale": "No hazards visible, but visibility is low-light.",
        "doorway_evidence": "",
        "person_evidence": "",
    })
    assert not assessment["safe_to_translate"]
    assert "consistency_correction" not in assessment


def test_semantic_preflight_vetoes_person_claim_hidden_by_false_boolean():
    assessment = normalize_scene_assessment({
        "safe_to_translate": False,
        "doorway_visible": False,
        "person_visible": True,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear",
        "recommendation": "hold",
        "rationale": "Feet visible indicate person in corridor, triggering hold.",
        "doorway_evidence": "",
        "person_evidence": "feet visible in foreground",
    })
    assert assessment["person_in_corridor"]
    assert assessment["model_person_in_corridor"] is False
    assert not bounded_escape_evidence(assessment, 0.25)


def test_metric_translation_preflight_uses_battery_free_context():
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": None,
        "drive_active": False,
        "hard_clearance_m": {"forward": 0.75},
        "clearance_limit_m": 1.5,
    }
    assert metric_translation_preflight(context, 0.50) == (True, [])
    allowed, reasons = metric_translation_preflight(context, 0.60)
    assert not allowed
    assert reasons == ["insufficient forward clearance"]


def test_metric_translation_preflight_accepts_censored_clearance_limit():
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": None,
        "drive_active": False,
        "hard_clearance_m": {"forward": None},
        "clearance_limit_m": 1.5,
    }
    assert metric_translation_preflight(context, 0.75) == (True, [])


def test_semantic_prompt_context_explains_censored_clearance_without_mutation():
    context = {
        "camera_fresh": True,
        "hard_clearance_m": {"forward": None},
        "clearance_limit_m": 1.5,
    }
    annotated = semantic_context_for_prompt(context)
    assert annotated is not context
    assert "clearance_semantics" not in context
    assert "not missing sensor data" in (
        annotated["clearance_semantics"]["null_hard_clearance"])
    assert "1.50 m" in annotated["clearance_semantics"]["null_hard_clearance"]
    assert annotated["hard_clearance_m"]["forward"] is None


@pytest.mark.parametrize("state", [
    "succeeded", "failed", "rejected", "cancelled", "error",
])
def test_metric_translation_preflight_ignores_completed_goal_record(state):
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": {"state": state},
        "drive_active": False,
        "hard_clearance_m": {"forward": 0.75},
        "clearance_limit_m": 1.5,
    }
    assert metric_translation_preflight(context, 0.50) == (True, [])


@pytest.mark.parametrize("goal", [
    {"state": "sending"}, {"state": "active"}, {}, "unknown",
])
def test_metric_translation_preflight_blocks_live_or_unknown_goal(goal):
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": goal,
        "drive_active": False,
        "hard_clearance_m": {"forward": 0.75},
        "clearance_limit_m": 1.5,
    }
    assert metric_translation_preflight(context, 0.50) == (
        False, ["another navigation goal is active"])


def test_metric_translation_preflight_allows_only_outward_trapped_escape():
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": None,
        "drive_active": False,
        "hard_clearance_m": {"forward": 0.0},
        "clearance_limit_m": 1.5,
        "trapped_start": True,
        "escape_profiles": {"forward": {
            "trapped": True, "blocked_prefix_m": 0.15,
            "clearance_m": 0.70}},
    }
    assert metric_translation_preflight(
        context, 0.30, margin=0.10) == (True, [])
    context["escape_profiles"]["forward"]["clearance_m"] = 0.35
    assert not metric_translation_preflight(
        context, 0.30, margin=0.10)[0]


@pytest.mark.parametrize("camera_forward", [False, None])
def test_metric_translation_preflight_requires_verified_forward_camera(
        camera_forward):
    context = {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": camera_forward,
        "current_goal": None,
        "drive_active": False,
        "hard_clearance_m": {"forward": 1.0},
        "clearance_limit_m": 1.5,
    }
    assert metric_translation_preflight(context, 0.25) == (
        False, ["camera not in verified forward pose"])


def test_metric_rotation_preflight_uses_stationary_360_degree_state():
    context = {
        "localization": {"healthy": True},
        "current_goal": {"state": "failed"},
        "drive_active": False,
        "trapped_start": False,
        # A forward hazard does not prove an in-place turn is blocked; the nav
        # node checks fresh 360-degree lidar and the current footprint.
        "hard_clearance_m": {"forward": 0.35},
    }
    assert metric_rotation_preflight(context) == (True, [])
    context["trapped_start"] = True
    assert metric_rotation_preflight(context) == (
        False, ["current footprint is blocked"])


def test_bounded_escape_allows_visible_person_outside_corridor():
    assessment = {
        "safe_to_translate": False,
        "person_visible": True,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear open floor lane",
        "recommendation": "hold",
    }
    assert bounded_escape_evidence(assessment, 0.32)
    assert not bounded_escape_evidence(assessment, 0.36)


def test_bounded_escape_rejects_visibility_uncertainty_and_corridor_hazards():
    uncertain = {
        "person_visible": False,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear center with dark shadows and glare",
    }
    hazard = {**uncertain, "visual_corridor": "clear lane",
              "hazards": ["thin pole"]}
    assert not bounded_escape_evidence(uncertain, 0.25)
    assert not bounded_escape_evidence(hazard, 0.25)

    rationale_only = {
        "person_visible": False,
        "person_in_corridor": False,
        "hazards": [],
        "visual_corridor": "clear",
        "rationale": "Uncertain visibility due to glare and low-light; sensor "
                     "context not authoritative.",
    }
    assert not bounded_escape_evidence(rationale_only, 0.24)


def test_short_corridor_never_reclassifies_reported_objects_as_safe():
    assessment = {
        "safe_to_translate": False,
        "person_visible": False,
        "person_in_corridor": False,
        "hazards": ["thin pole", "hanging fabric"],
        "visual_corridor": "clear",
        "recommendation": "sensor_preflight",
        "rationale": "No person in corridor; hazards are outside or not "
                     "intersecting proposed path.",
    }
    corrected = reclassify_out_of_corridor_hazards(assessment, 0.339)
    assert corrected["hazards"] == ["thin pole", "hanging fabric"]
    assert "model_hazards" not in corrected
    assert not bounded_escape_evidence(corrected, 0.339)
    assert reclassify_out_of_corridor_hazards(
        assessment, 0.40)["hazards"] == assessment["hazards"]


def test_semantic_candidate_score_prefers_open_confident_person_free_lane():
    confident = {"person_in_corridor": False, "model_safe_to_translate": True}
    corrected = {"person_in_corridor": True,
                  "bounded_escape_correction": "bounded correction"}
    open_context = {"hard_clearance_m": {"forward": 1.2},
                    "clearance_limit_m": 1.5}
    tight_context = {"hard_clearance_m": {"forward": 0.5},
                     "clearance_limit_m": 1.5}
    assert semantic_candidate_score(confident, open_context, 0.4) > (
        semantic_candidate_score(corrected, tight_context, 0.25))


def test_directional_escape_profile_distinguishes_exit_and_reblock():
    distances = np.arange(0.0, 0.6, 0.1)
    profile = directional_escape_profile(
        [True, True, False, False, True, True], distances)
    assert profile == {"trapped": True, "blocked_prefix_m": 0.2,
                       "clearance_m": 0.4}
    clear = directional_escape_profile([False] * 6, distances)
    assert clear == {"trapped": False, "blocked_prefix_m": 0.0,
                     "clearance_m": None}
