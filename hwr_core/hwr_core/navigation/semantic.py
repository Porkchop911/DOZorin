"""Pure semantic and metric preflight helpers for bounded navigation."""
from __future__ import annotations

import math
import re

import numpy as np


FLOOR_HAZARD_WORDS = (
    "wire", "cable", "cord", "clothing", "chair", "obstruction", "clutter",
    "object on floor", "rack", "pole", "leg", "base",
)
CORRIDOR_VETO_WORDS = (
    "blocked", "clutter", "uncertain", "obstruction", "cable", "wire",
    "cord", "chair", "rack", "leg", "base", "pole", "person",
)
POSITIVE_CORRIDOR_WORDS = ("clear", "open-floor", "open floor", "lane")
POSITIVE_RATIONALE_PHRASES = (
    "clearance sufficient", "no visual obstruction", "path is clear",
    "corridor is clear", "no hazards",
)
VISIBILITY_UNCERTAINTY_WORDS = (
    "dark", "glare", "shadow", "occluded", "not visible", "cannot see",
    "except", "uncertain", "low-light", "low light", "not authoritative",
)


def semantic_context_for_prompt(context: dict) -> dict:
    """Annotate censored clearance values without changing live sensor data.

    The navigation API uses ``null`` to mean that no obstacle was observed out
    to its reporting limit.  Language models commonly mistake that for missing
    telemetry, so make the wire contract explicit in the prompt payload.
    """
    if not isinstance(context, dict):
        raise ValueError("navigation context must be an object")
    result = dict(context)
    try:
        limit = float(context.get("clearance_limit_m"))
    except (TypeError, ValueError):
        limit = math.nan
    limit_text = (f"{limit:.2f} m" if math.isfinite(limit) and limit > 0.0
                  else "the stated reporting limit")
    result["clearance_semantics"] = {
        "null_hard_clearance": (
            "no hard obstacle was detected out to " + limit_text
            + "; this is censored clear space, not missing sensor data"),
        "numeric_hard_clearance": (
            "nearest detected hard obstacle distance from the robot"),
        "freshness": (
            "use the dedicated camera/localization freshness fields to decide "
            "whether sensor data is available"),
    }
    return result


def _without_explicitly_negated_terms(text: str, terms: tuple[str, ...]) -> str:
    """Remove simple, explicit negations before conservative keyword checks."""
    result = text
    for term in sorted(set(terms), key=len, reverse=True):
        escaped = re.escape(term)
        result = re.sub(
            rf"\b(?:no|without|free of)\s+(?:visible\s+)?{escaped}s?\b",
            " ", result)
    return result


def normalize_scene_assessment(assessment: dict) -> dict:
    """Fail closed on contradictory VLM fields while preserving raw claims."""
    if not isinstance(assessment, dict):
        raise ValueError("scene assessment must be an object")
    required = {
        "safe_to_translate", "doorway_visible", "person_visible", "hazards",
        "visual_corridor", "recommendation", "rationale",
        "doorway_evidence", "person_evidence",
    }
    if not required <= assessment.keys():
        raise ValueError("scene assessment is missing required fields")
    if assessment["recommendation"] not in {
            "hold", "sensor_preflight", "human_reposition"}:
        raise ValueError("scene assessment has an invalid recommendation")
    if not isinstance(assessment["hazards"], list):
        raise ValueError("scene assessment hazards must be an array")

    result = dict(assessment)
    result.setdefault("glass_boundary_visible", False)
    result.setdefault("glass_boundary_in_corridor", False)
    result.setdefault("glass_boundary_evidence", "")
    if not isinstance(result["glass_boundary_visible"], bool):
        raise ValueError("glass_boundary_visible must be boolean")
    if not isinstance(result["glass_boundary_in_corridor"], bool):
        raise ValueError("glass_boundary_in_corridor must be boolean")
    if not isinstance(result["glass_boundary_evidence"], str):
        raise ValueError("glass_boundary_evidence must be a string")
    if (result["glass_boundary_in_corridor"]
            and not result["glass_boundary_visible"]):
        result["model_glass_boundary_visible"] = False
        result["glass_boundary_visible"] = True
        result["glass_boundary_consistency_correction"] = (
            "a corridor glass boundary is necessarily visible")
    hazard_text = " ".join(str(item).lower() for item in result["hazards"])
    corridor = str(result["visual_corridor"]).lower()
    checked_hazards = _without_explicitly_negated_terms(
        hazard_text, FLOOR_HAZARD_WORDS)
    checked_corridor = _without_explicitly_negated_terms(
        corridor, CORRIDOR_VETO_WORDS)
    rationale = str(result["rationale"]).lower()
    person_text = f"{rationale} {str(result.get('person_evidence', '')).lower()}"
    positive_person_phrases = (
        "person in corridor", "person in the corridor",
        "person intersects corridor", "person intersects the corridor",
        "person blocks corridor", "person blocks the corridor",
    )
    negative_person_phrases = (
        "no person in corridor", "no person in the corridor",
        "person outside corridor", "person outside the corridor",
        "person remains outside", "person not in corridor",
        "person not in the corridor",
    )
    person_text_claims_corridor = (
        any(phrase in person_text for phrase in positive_person_phrases)
        and not any(phrase in person_text for phrase in negative_person_phrases)
    )
    if person_text_claims_corridor and not result.get("person_in_corridor"):
        result["model_person_in_corridor"] = bool(
            result.get("person_in_corridor"))
        result["person_in_corridor"] = True
        result["person_corridor_correction"] = (
            "model rationale or evidence placed a person in the corridor")
    person_visible = bool(result.get("person_visible"))
    person_in_corridor = bool(result.get("person_in_corridor"))
    vetoes: list[str] = []
    if result["hazards"]:
        if any(word in checked_hazards for word in FLOOR_HAZARD_WORDS):
            vetoes.append("reported floor or overhang hazard")
        else:
            vetoes.append("reported visual hazard")
    if person_in_corridor:
        vetoes.append("person in proposed corridor")
    if result["glass_boundary_in_corridor"]:
        vetoes.append("glass or mirror boundary in proposed corridor")
    if any(word in checked_corridor for word in CORRIDOR_VETO_WORDS):
        vetoes.append("corridor not visually clear")
    if result["recommendation"] in {"hold", "human_reposition"}:
        vetoes.append("model recommendation requires stopping")
    if vetoes:
        if result.get("safe_to_translate"):
            result["model_safe_to_translate"] = True
        result["safe_to_translate"] = False
        result["consistency_veto"] = sorted(set(vetoes))
    return result


def assessment_from_visual_observation(observation: dict) -> dict:
    """Convert a VLM's visual fact into the existing fail-closed assessment.

    The model reports whether the pictured path looks clear, not whether robot
    motion is authorized.  Deterministic semantic and metric gates retain that
    responsibility.
    """
    if not isinstance(observation, dict):
        raise ValueError("visual observation must be an object")
    if not isinstance(observation.get("visibility_sufficient"), bool):
        raise ValueError("visual observation must contain visibility_sufficient")
    for field in ("glass_boundary_visible", "glass_boundary_in_corridor"):
        if not isinstance(observation.get(field), bool):
            raise ValueError(f"visual observation must contain boolean {field}")
    if not isinstance(observation.get("glass_boundary_evidence"), str):
        raise ValueError(
            "visual observation must contain string glass_boundary_evidence")
    assessment = dict(observation)
    hazards = observation.get("hazards")
    corridor = str(observation.get("visual_corridor", "")).lower()
    rationale = str(observation.get("rationale", "")).lower()
    scene_text = f"{corridor} {rationale}"
    # Some small VLMs intermittently emit visibility_sufficient=false while
    # explicitly stating that the full swept envelope is visible and contains
    # no hazards. Correct only that narrow factual contradiction. Any reported
    # hazard, person, hold, or visibility-uncertainty wording still fails closed.
    if (
        observation["visibility_sufficient"] is False
        and isinstance(hazards, list) and not hazards
        and not observation.get("person_in_corridor")
        and not observation.get("glass_boundary_in_corridor")
        and "no visible hazards" in rationale
        and any(word in corridor for word in POSITIVE_CORRIDOR_WORDS)
        and not any(word in scene_text for word in VISIBILITY_UNCERTAINTY_WORDS)
        and observation.get("recommendation") == "sensor_preflight"
    ):
        assessment["model_visibility_sufficient"] = False
        assessment["visibility_sufficient"] = True
        assessment["visibility_consistency_correction"] = (
            "model explicitly reported no visible hazards in the swept "
            "envelope and no visibility uncertainty")
    observed_clear = (
        assessment["visibility_sufficient"]
        and isinstance(hazards, list) and not hazards
        and not observation.get("person_in_corridor")
        and not observation.get("glass_boundary_in_corridor")
        and any(word in corridor for word in POSITIVE_CORRIDOR_WORDS)
        and not any(word in scene_text for word in VISIBILITY_UNCERTAINTY_WORDS)
        and assessment.get("recommendation") == "sensor_preflight"
    )
    assessment["visual_path_clear"] = observed_clear
    assessment["safe_to_translate"] = observed_clear
    return normalize_scene_assessment(assessment)


def metric_translation_preflight(
        context: dict, distance: float, *, margin: float = 0.20,
) -> tuple[bool, list[str]]:
    """Check battery-free live context for a forward-aligned bounded leg."""
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("distance must be finite and positive")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("margin must be finite and non-negative")
    reasons: list[str] = []
    localization = context.get("localization") or {}
    if not localization.get("healthy"):
        reasons.append("localization unhealthy")
    if not context.get("camera_fresh"):
        reasons.append("camera stale")
    if context.get("camera_forward") is not True:
        reasons.append("camera not in verified forward pose")
    goal = context.get("current_goal")
    terminal_goal_states = {"succeeded", "failed", "rejected", "cancelled",
                            "error"}
    if (goal is not None
            and (not isinstance(goal, dict)
                 or goal.get("state") not in terminal_goal_states)):
        reasons.append("another navigation goal is active")
    if context.get("drive_active"):
        reasons.append("drive is active during preflight")
    hard = context.get("hard_clearance_m") or {}
    clearance = hard.get("forward")
    if clearance is None:
        clearance = context.get("clearance_limit_m")
    try:
        clearance = float(clearance)
    except (TypeError, ValueError):
        reasons.append("forward clearance unavailable")
    else:
        sufficient = math.isfinite(clearance) and clearance >= distance + margin
        if not sufficient and context.get("trapped_start"):
            profile = (context.get("escape_profiles") or {}).get(
                "forward") or {}
            prefix = profile.get("blocked_prefix_m")
            escape_clearance = profile.get("clearance_m")
            try:
                prefix = float(prefix)
                escape_clearance = (math.inf if escape_clearance is None
                                    else float(escape_clearance))
            except (TypeError, ValueError):
                pass
            else:
                sufficient = (math.isfinite(prefix)
                              and distance >= prefix
                              and escape_clearance >= distance + margin)
        if not sufficient:
            reasons.append("insufficient forward clearance")
    return not reasons, reasons


def metric_rotation_preflight(context: dict) -> tuple[bool, list[str]]:
    """Check stationary state before a zero-translation rotation request.

    The navigation node performs the authoritative fresh-lidar/current-footprint
    check for every yaw command. A forward camera cannot observe the full 360°
    rotation envelope, so a forward-translation VLM assessment is not evidence
    for or against an in-place turn.
    """
    reasons: list[str] = []
    localization = context.get("localization") or {}
    if not localization.get("healthy"):
        reasons.append("localization unhealthy")
    goal = context.get("current_goal")
    terminal_goal_states = {"succeeded", "failed", "rejected", "cancelled",
                            "error"}
    if (goal is not None
            and (not isinstance(goal, dict)
                 or goal.get("state") not in terminal_goal_states)):
        reasons.append("another navigation goal is active")
    if context.get("drive_active"):
        reasons.append("drive is active during preflight")
    if context.get("trapped_start"):
        reasons.append("current footprint is blocked")
    return not reasons, reasons


def bounded_escape_evidence(assessment: dict, distance: float,
                            *, max_distance: float = 0.35) -> bool:
    """Recognize a narrowly scoped clear micro-step despite a generic VLM hold."""
    if not math.isfinite(distance) or not 0.0 < distance <= max_distance:
        return False
    hazards = assessment.get("hazards")
    if not isinstance(hazards, list) or hazards:
        return False
    if assessment.get("person_in_corridor"):
        return False
    if assessment.get("glass_boundary_in_corridor"):
        return False
    corridor = str(assessment.get("visual_corridor", "")).lower()
    if not any(word in corridor for word in POSITIVE_CORRIDOR_WORDS):
        return False
    rationale = str(assessment.get("rationale", "")).lower()
    scene_text = f"{corridor} {rationale}"
    if any(word in corridor for word in CORRIDOR_VETO_WORDS):
        return False
    if any(word in scene_text for word in VISIBILITY_UNCERTAINTY_WORDS):
        return False
    return True


def reclassify_out_of_corridor_hazards(
        assessment: dict, distance: float, *, max_distance: float = 0.35,
) -> dict:
    """Preserve every reported hazard; visual geometry cannot clear one."""
    del distance, max_distance
    return dict(assessment)


def semantic_candidate_score(assessment: dict, context: dict,
                             distance: float) -> float:
    """Rank already-allowed candidates by open-space and semantic confidence."""
    hard = context.get("hard_clearance_m") or {}
    clearance = hard.get("forward")
    if clearance is None:
        clearance = context.get("clearance_limit_m", distance)
    try:
        clearance = float(clearance)
    except (TypeError, ValueError):
        clearance = distance
    if clearance <= 0.0 and context.get("trapped_start"):
        escape_clearance = ((context.get("escape_profiles") or {}).get(
            "forward") or {}).get("clearance_m")
        if escape_clearance is None:
            escape_clearance = context.get("clearance_limit_m", distance)
        try:
            clearance = float(escape_clearance)
        except (TypeError, ValueError):
            clearance = distance
    surplus = max(0.0, clearance - distance)
    score = surplus + 0.20 * min(distance, 1.0)
    if assessment.get("person_in_corridor"):
        score -= 0.40
    if assessment.get("bounded_escape_correction"):
        score -= 0.20
    if assessment.get("model_safe_to_translate") is True:
        score += 0.20
    return score


def directional_escape_profile(blocked, distances) -> dict:
    """Describe a ray that may begin inside an inflated blocked component."""
    blocked = np.asarray(blocked, dtype=bool).reshape(-1)
    distances = np.asarray(distances, dtype=float).reshape(-1)
    if not blocked.size or blocked.shape != distances.shape:
        raise ValueError("blocked and distances must be equal non-empty vectors")
    if not np.isfinite(distances).all() or np.any(np.diff(distances) < 0.0):
        raise ValueError("distances must be finite and nondecreasing")
    first_blocked = np.flatnonzero(blocked)
    if not blocked[0]:
        return {"trapped": False, "blocked_prefix_m": 0.0,
                "clearance_m": (float(distances[first_blocked[0]])
                                if len(first_blocked) else None)}
    first_free = np.flatnonzero(~blocked)
    if not len(first_free):
        return {"trapped": True, "blocked_prefix_m": None,
                "clearance_m": 0.0}
    free_index = int(first_free[0])
    reblocked = np.flatnonzero(blocked[free_index + 1:])
    clearance = (float(distances[free_index + 1 + reblocked[0]])
                 if len(reblocked) else None)
    return {"trapped": True,
            "blocked_prefix_m": float(distances[free_index]),
            "clearance_m": clearance}
