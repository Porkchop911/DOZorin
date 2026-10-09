import numpy as np
import yaml

from hwr_core.mapping import LogOddsGrid, load_map


def test_ray_marks_free_space_and_hit():
    grid = LogOddsGrid(resolution=0.1, size=20)
    grid.integrate((0, 0, 0), np.array([[0.5, 0.0]]))
    start, middle, end = grid.world_to_cell([(0, 0), (0.3, 0), (0.5, 0)])
    occ = grid.occupancy()
    assert occ[start[1], start[0]] == 0
    assert occ[middle[1], middle[0]] == 0
    assert occ[end[1], end[0]] == 100


def test_grid_grows_without_changing_world_cells():
    grid = LogOddsGrid(resolution=0.1, size=4, origin=(0, 0))
    grid.integrate((0.1, 0.1, 0), [[0.2, 0]])
    grid.integrate((0.1, 0.1, 0), [[-2.0, -2.0]])
    cell = grid.world_to_cell([(0.3, 0.1)])[0]
    assert grid.observed[cell[1], cell[0]]


def test_log_odds_saturate():
    grid = LogOddsGrid(resolution=0.1, size=20)
    for _ in range(100):
        grid.integrate((0, 0, 0), [[0.5, 0]])
    assert grid.log_odds.max() <= 5.0
    assert grid.log_odds.min() >= -5.0


def test_save_pgm_yaml(tmp_path):
    grid = LogOddsGrid(resolution=0.1, size=10)
    grid.integrate((0, 0, 0), [[0.4, 0]])
    pgm, meta = grid.save(tmp_path / "map")
    assert pgm.read_bytes().startswith(b"P5\n")
    data = yaml.safe_load(meta.read_text())
    assert data["resolution"] == 0.1
    assert data["image"] == "map.pgm"
    loaded, resolution, origin = load_map(meta)
    assert resolution == 0.1
    assert origin == tuple(grid.origin)
    assert np.array_equal(loaded, grid.occupancy())


def test_grid_rehydrates_saved_occupancy_exactly():
    occupancy = np.array([
        [-1, -1, 100, 100],
        [-1, 0, 0, 100],
        [0, 0, -1, -1],
    ], dtype=np.int8)
    grid = LogOddsGrid.from_occupancy(
        occupancy, 0.05, (-1.25, 2.5))

    assert grid.log_odds.shape == occupancy.shape
    assert tuple(grid.origin) == (-1.25, 2.5)
    assert np.array_equal(grid.occupancy(), occupancy)
