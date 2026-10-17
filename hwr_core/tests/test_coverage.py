import numpy as np
import pytest

from hwr_core.exploration import map_coverage, shorter_leg


def test_map_coverage_counts_known_cells():
    grid = np.full((10, 10), -1)
    grid[:4, :5] = 0
    grid[4, :5] = 100
    assert map_coverage(grid, 0.05) == {"known_m2": 0.06, "free_m2": 0.05, "occupied_m2": 0.01}


def test_shorter_leg_halves_only_long_legs():
    assert shorter_leg((0.0, 0.0), (0.8, 0.0)) == pytest.approx((0.4, 0.0))
    assert shorter_leg((0.0, 0.0), (0.4, 0.0)) is None   # already short
    assert shorter_leg((1.0, 1.0), (1.0, 1.5)) is None   # half would be 0.25 < 0.30
