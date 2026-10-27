import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "ros2_ws" / "src" / "hwr_sensors"))
from hwr_sensors.sensor_liveness import (CAMERA_USB_RESET_TARGETS,  # noqa: E402
                                         KNOWN_USB_HUB_IDS,
                                         process_matches, stale_targets)


def test_process_match_requires_exact_node_marker_and_executable():
    lidar = ("/opt/vendor/lib/sclidar", "--ros-args", "-r",
             "__node:=hwr_lidar")
    assert process_matches(lidar, "lidar")
    assert not process_matches(lidar[:-1], "lidar")
    assert not process_matches(("/usr/bin/python3", *lidar), "lidar")


def test_stale_targets_respect_startup_grace_and_individual_timeouts():
    last = {"lidar": 10.0, "camera": 19.0}
    timeouts = {"lidar": 2.0, "camera": 3.0}
    assert stale_targets(20.0, 10.0, last, timeouts, 15.0) == []
    assert stale_targets(30.0, 10.0, last, timeouts, 15.0) == [
        "lidar", "camera"]


def test_camera_recovery_can_never_reset_a_shared_usb_hub():
    assert CAMERA_USB_RESET_TARGETS == ("3251:1930",)
    assert set(CAMERA_USB_RESET_TARGETS).isdisjoint(KNOWN_USB_HUB_IDS)
