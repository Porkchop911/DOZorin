import numpy as np

from hwr_core.arm.cubes import find_color_blobs

RANGES = {"red": {"min": [0, 152, 108], "max": [255, 255, 255]},
          "green": {"min": [51, 0, 125], "max": [255, 111, 255]},
          "blue": {"min": [0, 0, 0], "max": [255, 142, 120]}}


def test_finds_each_cube_once_on_grey_floor():
    image = np.full((400, 640, 3), 128, np.uint8)
    image[100:140, 100:140] = (0, 0, 220)    # red (BGR)
    image[200:240, 300:340] = (0, 200, 0)    # green
    image[50:90, 500:540] = (200, 120, 40)   # blue (kit blue is dull; pure blue has a > 142)
    blobs = find_color_blobs(image, RANGES)
    assert sorted(b["class"] for b in blobs) == ["blue_cube", "green_cube", "red_cube"]
    red = next(b for b in blobs if b["class"] == "red_cube")
    assert red["center"] == [119.5, 119.5]
    assert len(red["corners"]) == 4


def test_ignores_specks():
    image = np.full((400, 640, 3), 128, np.uint8)
    image[10:14, 10:14] = (0, 0, 220)
    assert find_color_blobs(image, RANGES) == []


def test_live_frames_two_lamps():
    # Real look_down frames from 2026-10-16 (warm lamp, then a cool one); see plan/16 session log.
    import pathlib
    import cv2
    import yaml
    root = pathlib.Path(__file__).resolve().parents[2]
    ranges = yaml.safe_load((root / "ros2_ws/src/hwr_arm/config/poses.yaml").read_text())["cubes"]
    for name, expected in [("find4", 3), ("find5", 3), ("find1", 0), ("find2", 0)]:
        blobs = find_color_blobs(cv2.imread(str(root / f"evidence/phase14/{name}.jpg")), ranges)
        assert sorted(b["class"] for b in blobs) == (
            ["blue_cube", "green_cube", "red_cube"] if expected else []), name
