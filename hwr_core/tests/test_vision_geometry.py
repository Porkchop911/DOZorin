import math

import numpy as np

from hwr_core.navigation.vision_geometry import (bearing, depth_below, object_range, parse_grounding,
                                                 region_samples)


def test_bearing_left_positive():
    assert bearing(320, 500, 320) == 0
    assert bearing(0, 500, 320) > 0 and bearing(640, 500, 320) < 0
    assert math.isclose(bearing(820, 500, 320), -math.pi / 4)


def test_object_range_rescales_and_ignores_invalid():
    depth = np.zeros((240, 320))
    depth[60:120, 80:160] = 1.5          # object in the half-resolution depth image
    depth[90, 120] = 0.0                  # dropout inside the patch is ignored
    assert object_range(depth, (160, 120, 320, 240), (480, 640)) == 1.5
    assert object_range(np.zeros((240, 320)), (160, 120, 320, 240), (480, 640)) is None


def test_object_range_prefers_the_object_over_the_background_seen_through_it():
    depth = np.full((240, 320), 2.3)
    depth[::3, :] = 0.9  # chair frame covers a third of the patch, the wall shows through the rest
    assert object_range(depth, (0, 0, 640, 480), (480, 640)) == 0.9


def test_parse_grounding_scales_boxes_and_drops_the_gripper_at_the_bottom_edge():
    reply = ('```json\n[{"label": "sock", "bbox_2d": [0, 312, 494, 630]},\n'
             ' {"label": "sock", "bbox_2d": [657, 938, 744, 1000]}]\n```')
    found = parse_grounding(reply, 640, 400)
    assert found == [{"label": "sock", "box": [0, 125, 316, 252], "center": [158, 188]}]
    assert parse_grounding("no socks here", 640, 400) == []
    assert parse_grounding('[{"label": "sock", "bbox_2d": [1, 2]}]', 640, 400) == []
    points = parse_grounding('[{"label": "sock", "point_2d": [349, 378]},'
                             ' {"label": "sock", "point_2d": [700, 960]}]', 640, 400)
    assert points == [{"label": "sock", "center": [223, 151]}]  # 2nd: the gripper strip
    assert parse_grounding('[{"label": "sock", "point_2d": [500, 900]}]', 640, 400) == []  # bumper plate
    assert parse_grounding('[{"label": "toy truck", "point_2d": [500, 900]}]', 640, 400)  # a truck stays
    truck = parse_grounding('[{"label": "toy truck", "bbox_2d": [640, 825, 880, 1000]}]', 640, 400)
    assert truck and truck[0]["center"] == [486, 365]  # bottom edge but right of the fingertips


def test_depth_below_skips_the_hole_on_a_dark_object_to_the_floor_in_front():
    depth = np.zeros((400, 640))
    depth[268:, :] = 0.95  # floor in front of the truck; the truck itself (rows < 268) has no return
    assert depth_below(depth, 329, 248, (400, 640)) == (264, 0.95)  # first window that reaches the floor
    assert depth_below(np.zeros((400, 640)), 329, 248, (400, 640)) is None


def test_region_samples_spread_over_the_sock_not_the_floor():
    rgb = np.full((400, 640, 3), 200, np.uint8)  # light floor
    rgb[100:400, 300:340] = 40                   # dark sock from the image top half to the bottom edge
    samples = region_samples(rgb, (320, 120))
    assert len(samples) == 12
    assert all(300 <= u < 340 and 100 <= v < 400 for u, v in samples)
    assert max(v for _, v in samples) - min(v for _, v in samples) > 200  # spread along it
