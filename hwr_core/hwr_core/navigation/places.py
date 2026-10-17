"""Validation and terminal reach checks for named physical places."""
from __future__ import annotations

import math
import os
import re
from collections.abc import Mapping, Sequence
from pathlib import Path

import yaml

from .object_approach import closest_point_on_segment


MAX_OBJECT_REACH_DISTANCE_M = 0.50


def _finite_number(value, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def normalize_place_definition(name: str, place: Mapping) -> dict:
    """Validate a place and return normalized numeric values.

    Object places separate the collision-free navigation pose (``x/y``) from
    a measured point on the physical target (``object_x/object_y``). This
    prevents a good camera view from being mistaken for physical arrival.
    """
    if not isinstance(place, Mapping):
        raise ValueError(f"place {name!r} must be a mapping")
    kind = str(place.get("kind", "")).strip().lower()
    if kind not in {"object", "reference"}:
        raise ValueError(f"place {name!r} kind must be object or reference")

    normalized = dict(place)
    normalized.update({
        "kind": kind,
        "x": _finite_number(place.get("x"), f"place {name!r} x"),
        "y": _finite_number(place.get("y"), f"place {name!r} y"),
        "yaw_deg": _finite_number(
            place.get("yaw_deg"), f"place {name!r} yaw_deg"),
    })
    if kind == "object":
        normalized["object_x"] = _finite_number(
            place.get("object_x"), f"place {name!r} object_x")
        normalized["object_y"] = _finite_number(
            place.get("object_y"), f"place {name!r} object_y")
        limit = _finite_number(place.get("max_object_distance_m"),
                               f"place {name!r} max_object_distance_m")
        if not 0.0 < limit <= MAX_OBJECT_REACH_DISTANCE_M:
            raise ValueError(
                f"place {name!r} max_object_distance_m must be > 0 and <= "
                f"{MAX_OBJECT_REACH_DISTANCE_M:.2f}")
        normalized["max_object_distance_m"] = limit
        end_values = place.get("object_end_x"), place.get("object_end_y")
        if any(value is not None for value in end_values):
            if any(value is None for value in end_values):
                raise ValueError(
                    f"place {name!r} object segment requires both "
                    "object_end_x and object_end_y")
            normalized["object_end_x"] = _finite_number(
                end_values[0], f"place {name!r} object_end_x")
            normalized["object_end_y"] = _finite_number(
                end_values[1], f"place {name!r} object_end_y")
            normalized["object_geometry"] = "segment"
        else:
            normalized["object_geometry"] = "point"
        nearest = closest_point_on_segment(
            (normalized["x"], normalized["y"]),
            (normalized["object_x"], normalized["object_y"]),
            (normalized.get("object_end_x", normalized["object_x"]),
             normalized.get("object_end_y", normalized["object_y"])))
        planned_distance = math.hypot(normalized["x"] - nearest[0],
                                      normalized["y"] - nearest[1])
        if planned_distance > limit:
            raise ValueError(
                f"place {name!r} approach pose is {planned_distance:.3f} m "
                f"from the object (limit {limit:.2f} m)")
        normalized["planned_object_distance_m"] = planned_distance
    return normalized


def evaluate_object_reach(pose: Sequence[float], object_x: float,
                          object_y: float, max_distance_m: float, *,
                          object_end_x: float | None = None,
                          object_end_y: float | None = None) -> dict:
    """Measure robot-center distance to an object and apply the strict limit."""
    if pose is None or len(pose) < 2:
        raise ValueError("a localized robot pose is required")
    px = _finite_number(pose[0], "robot pose x")
    py = _finite_number(pose[1], "robot pose y")
    ox = _finite_number(object_x, "object_x")
    oy = _finite_number(object_y, "object_y")
    if (object_end_x is None) != (object_end_y is None):
        raise ValueError("object segment requires both object_end_x and object_end_y")
    ex = ox if object_end_x is None else _finite_number(
        object_end_x, "object_end_x")
    ey = oy if object_end_y is None else _finite_number(
        object_end_y, "object_end_y")
    limit = _finite_number(max_distance_m, "max_object_distance_m")
    if not 0.0 < limit <= MAX_OBJECT_REACH_DISTANCE_M:
        raise ValueError(
            "max_object_distance_m must be > 0 and <= "
            f"{MAX_OBJECT_REACH_DISTANCE_M:.2f}")
    nearest = closest_point_on_segment((px, py), (ox, oy), (ex, ey))
    distance = math.hypot(px - nearest[0], py - nearest[1])
    return {
        "object_distance_m": distance,
        "max_object_distance_m": limit,
        "object_reached": distance <= limit,
        "object_closest_x": nearest[0],
        "object_closest_y": nearest[1],
    }


PLACE_NAME = re.compile(r"[a-z0-9_]{1,40}")


def places_file(map_yaml: str | Path) -> Path:
    """Places live beside their map (``room.yaml`` -> ``room.places.yaml``): coordinates are per map frame."""
    path = Path(map_yaml)
    return path.with_name(f"{path.stem}.places.yaml")


def load_places(path: str | Path) -> dict:
    path = Path(path)
    if not path.exists():
        return {}
    return (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("places") or {}


def mark_place(path: str | Path, name: str, pose: Sequence[float]) -> dict:
    """Store the robot's current localized pose as a verified reference place (overwrites ``name``)."""
    if not PLACE_NAME.fullmatch(name):
        raise ValueError("place name must be 1-40 chars of a-z, 0-9, _")
    if pose is None or len(pose) < 3:
        raise ValueError("a localized robot pose is required")
    x, y = _finite_number(pose[0], "pose x"), _finite_number(pose[1], "pose y")
    yaw_deg = math.degrees(_finite_number(pose[2], "pose yaw")) % 360.0
    place = {"kind": "reference", "x": round(x, 3), "y": round(y, 3),
             "yaw_deg": round(yaw_deg, 1), "verified": True}
    path = Path(path)
    places = load_places(path)
    places[name] = place
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(yaml.safe_dump({"places": places}, sort_keys=True), encoding="utf-8")
    os.replace(tmp, path)  # atomic: a crash mid-write never loses the other places
    return place
