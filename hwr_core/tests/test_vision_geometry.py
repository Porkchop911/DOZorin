import math

import numpy as np

from hwr_core.navigation.vision_geometry import bearing, object_range


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
