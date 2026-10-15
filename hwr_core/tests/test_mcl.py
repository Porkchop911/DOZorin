import math

import numpy as np
import pytest

from hwr_core.localization import (ParticleFilter, aggregate_scan_panorama,
                                   coarse_global_search,
                                   distance_field,
                                   localization_motion_consistent)


def test_distance_field_from_obstacle():
    occ = np.zeros((5, 5), dtype=np.int8)
    occ[2, 2] = 100
    field = distance_field(occ, 0.1)
    assert field[2, 2] == 0
    assert field[2, 3] == pytest.approx(0.1)
    assert field[3, 3] == pytest.approx(0.1 * np.sqrt(2))


def make_filter(count=200):
    occ = np.zeros((20, 20), dtype=np.int8)
    occ[[0, -1], :] = occ[:, [0, -1]] = 100
    return ParticleFilter(occ, 0.1, (-1, -1), count=count, seed=4)


def test_pose_initialization_and_motion():
    filt = make_filter()
    filt.initialize_pose((0, 0, 0), std=(0, 0, 0))
    filt.predict((0.2, 0, 0), noise=(0, 0, 0))
    assert filt.pose == pytest.approx((0.2, 0.0, 0.0))


def test_zero_odometry_updates_do_not_create_stationary_random_walk():
    filt = make_filter(count=500)
    filt.initialize_pose((0.2, -0.1, 0.3), std=(0, 0, 0))
    before = filt.particles.copy()
    for _ in range(500):
        filt.predict((0.0, 0.0, 0.0))
    assert np.array_equal(filt.particles, before)


def test_packetized_pure_yaw_has_bounded_translation_uncertainty():
    filt = make_filter(count=5000)
    filt.initialize_pose((0.0, 0.0, 0.0), std=(0, 0, 0))
    for _ in range(100):
        filt.predict((0.0, 0.0, 0.006))
    assert np.std(filt.particles[:, 0]) < 0.006
    assert np.std(filt.particles[:, 1]) < 0.006
    assert np.std(filt.particles[:, 2]) < 0.03


def test_localization_motion_consistency_accepts_odom_aligned_update():
    assert localization_motion_consistent(
        (1.0, 2.0, 0.5), (1.08, 2.03, 0.6),
        (0.0, 0.0, 0.2), (0.085, 0.0, 0.3))


def test_localization_motion_consistency_rejects_stationary_mode_jump():
    assert not localization_motion_consistent(
        (1.0, 2.0, 0.5), (1.08, 2.02, -0.8),
        (0.0, 0.0, 0.2), (0.0, 0.0, 0.2))


def test_localization_motion_consistency_tolerates_drift_over_long_leg():
    # 1.5 m odom leg with 10 % wheel under-report: 15 cm residual must pass,
    # while the same residual with no motion is still a mode jump.
    assert localization_motion_consistent(
        (0.0, 0.0, 0.0), (1.65, 0.0, 0.0), (0.0, 0.0, 0.0), (1.5, 0.0, 0.0))
    assert not localization_motion_consistent(
        (0.0, 0.0, 0.0), (0.15, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_scan_panorama_aligns_rotated_views_in_current_base_frame():
    landmark = np.array([[1.0, 0.0]])
    # At yaw 0 the landmark is forward. At yaw +90 degrees it is to the right
    # in that historical base frame. Both must align in the current yaw-90 frame.
    history = [
        ((0.0, 0.0, 0.0), landmark),
        ((0.0, 0.0, math.pi / 2), np.array([[0.0, -1.0]])),
    ]
    combined = aggregate_scan_panorama(
        history, (0.0, 0.0, math.pi / 2), stride=1)
    assert combined == pytest.approx(np.array([[0.0, -1.0], [0.0, -1.0]]))


def test_scan_panorama_excludes_translated_history():
    combined = aggregate_scan_panorama(
        [((0.3, 0.0, 0.0), np.array([[1.0, 0.0]]))],
        (0.0, 0.0, 0.0), max_translation=0.15)
    assert combined.shape == (0, 2)


def test_update_normalizes_weights():
    filt = make_filter()
    filt.update(np.array([[0.9, 0], [0, 0.9]]))
    assert filt.weights.sum() == pytest.approx(1.0)
    assert np.isfinite(filt.weights).all()


def test_low_ess_resamples_systematically():
    filt = make_filter(count=10)
    filt.weights[:] = 0
    filt.weights[3] = 1
    assert filt.resample_if_needed()
    assert np.allclose(filt.weights, 0.1)
    assert np.all(filt.particles == filt.particles[0])


def test_hypothesis_initialization_concentrates_cloud():
    filt = make_filter(count=1000)
    filt.initialize_hypotheses(np.array([[0.4, -0.2, 0.3]]), std=(0.02, 0.02, 0.02))
    assert filt.pose == pytest.approx((0.4, -0.2, 0.3), abs=0.01)


def test_coarse_global_search_finds_asymmetric_pose():
    occ = np.zeros((50, 60), dtype=np.int8)
    occ[[0, -1], :] = 100
    occ[:, [0, -1]] = 100
    occ[8:35, 12] = 100
    occ[35, 12:42] = 100
    occ[12:28, 45] = 100
    resolution, origin = 0.1, (-3.0, -2.5)
    field = distance_field(occ, resolution)
    obstacle_cells = np.argwhere(occ == 100)
    world = np.column_stack((origin[0] + (obstacle_cells[:, 1] + 0.5) * resolution,
                             origin[1] + (obstacle_cells[:, 0] + 0.5) * resolution))
    truth = np.array((0.05, -0.15, math.radians(30.0)))
    near = world[np.linalg.norm(world - truth[:2], axis=1) < 2.5]
    c, s = math.cos(truth[2]), math.sin(truth[2])
    scan = (near - truth[:2]) @ np.array(((c, -s), (s, c)))
    poses, scores = coarse_global_search(occ, field, resolution, origin, scan,
                                         xy_step=0.2, yaw_step=math.radians(15))
    assert scores[0] > 0.75
    assert np.linalg.norm(poses[0, :2] - truth[:2]) <= 0.16
    angle_error = math.atan2(math.sin(poses[0, 2] - truth[2]),
                             math.cos(poses[0, 2] - truth[2]))
    assert abs(angle_error) <= math.radians(8.0)
