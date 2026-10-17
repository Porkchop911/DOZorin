import math

import pytest

from hwr_core.navigation import (evaluate_object_reach,
                                 normalize_place_definition)


def object_place(**overrides):
    place = {
        "kind": "object", "x": 1.0, "y": 2.0, "yaw_deg": 90,
        "object_x": 1.2, "object_y": 2.0,
        "max_object_distance_m": 0.50, "verified": True,
    }
    place.update(overrides)
    return place


def test_object_place_keeps_approach_and_physical_anchor_separate():
    place = normalize_place_definition("window", object_place())
    assert (place["x"], place["y"]) == (1.0, 2.0)
    assert (place["object_x"], place["object_y"]) == (1.2, 2.0)
    assert place["planned_object_distance_m"] == pytest.approx(0.2)


def test_exactly_fifty_centimetres_counts_as_reached():
    result = evaluate_object_reach([0.0, 0.0, 0.0], 0.3, 0.4, 0.50)
    assert result["object_reached"]
    assert result["object_distance_m"] == pytest.approx(0.50)


def test_more_than_fifty_centimetres_does_not_count_as_reached():
    result = evaluate_object_reach([0.0, 0.0, 0.0], 0.5001, 0.0, 0.50)
    assert not result["object_reached"]


@pytest.mark.parametrize("bad_limit", [0.0, -0.1, 0.5001, math.inf, math.nan])
def test_object_limit_must_be_positive_and_never_exceed_fifty_cm(bad_limit):
    with pytest.raises(ValueError, match="max_object_distance_m"):
        normalize_place_definition(
            "drums", object_place(max_object_distance_m=bad_limit))


@pytest.mark.parametrize("missing", ["object_x", "object_y"])
def test_object_place_without_measured_anchor_fails_closed(missing):
    place = object_place()
    place[missing] = None
    with pytest.raises(ValueError, match=missing):
        normalize_place_definition("door", place)


def test_visibility_or_approach_pose_does_not_override_object_distance():
    result = evaluate_object_reach(
        [0.0, 0.0, 0.0], 1.0, 0.0, 0.50)
    assert not result["object_reached"]
    assert result["object_distance_m"] == pytest.approx(1.0)


def test_configured_approach_pose_outside_radius_fails_before_dispatch():
    with pytest.raises(ValueError, match="approach pose is 1.000 m"):
        normalize_place_definition(
            "window", object_place(x=0.0, y=0.0,
                                   object_x=1.0, object_y=0.0))


def test_extended_object_uses_distance_to_physical_segment():
    place = normalize_place_definition(
        "window", object_place(
            x=1.5, y=0.4, object_x=0.0, object_y=0.0,
            object_end_x=2.0, object_end_y=0.0))
    assert place["object_geometry"] == "segment"
    assert place["planned_object_distance_m"] == pytest.approx(0.4)
    result = evaluate_object_reach(
        (1.5, 0.3, 0.0), 0.0, 0.0, 0.50,
        object_end_x=2.0, object_end_y=0.0)
    assert result["object_reached"]
    assert result["object_distance_m"] == pytest.approx(0.3)
    assert (result["object_closest_x"], result["object_closest_y"]) == (
        pytest.approx(1.5), pytest.approx(0.0))


def test_extended_object_requires_complete_segment():
    with pytest.raises(ValueError, match="requires both"):
        normalize_place_definition(
            "door", object_place(object_end_x=2.0, object_end_y=None))


def test_point_object_remains_backward_compatible():
    place = normalize_place_definition("drums", object_place())
    assert place["object_geometry"] == "point"
    result = evaluate_object_reach((1.0, 2.0), 1.2, 2.0, 0.50)
    assert result["object_reached"]
    assert result["object_closest_x"] == pytest.approx(1.2)


def test_places_are_stored_per_map_and_marked_from_pose(tmp_path):
    from hwr_core.navigation import load_places, mark_place, places_file
    path = places_file(tmp_path / "room_20261015.yaml")
    assert path.name == "room_20261015.places.yaml"
    assert load_places(path) == {}
    mark_place(path, "sofa", (1.0, -2.0, -math.pi / 2))
    mark_place(path, "door", (0.5, 0.25, 0.0))
    places = load_places(path)
    assert places["sofa"] == {"kind": "reference", "x": 1.0, "y": -2.0,
                              "yaw_deg": 270.0, "verified": True}
    assert normalize_place_definition("door", places["door"])["x"] == 0.5
    mark_place(path, "sofa", (3.0, 0.0, 0.0))
    assert load_places(path)["sofa"]["x"] == 3.0 and "door" in load_places(path)
    with pytest.raises(ValueError, match="name"):
        mark_place(path, "../etc", (0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="pose"):
        mark_place(path, "x", None)
