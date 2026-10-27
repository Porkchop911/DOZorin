"""ROS-free helpers for bounded external sensor process recovery."""
from __future__ import annotations


PROCESS_MARKERS = {
    "lidar": ("sclidar", "__node:=hwr_lidar"),
    "camera": ("aurora930_node", "__node:=hwr_depth_camera"),
}

# `lsusb` identifies the Aurora 930 itself as 3251:1930.  The Realtek IDs
# 0bda:5489 and 0bda:0489 are upstream hubs and must never be reset: doing so
# disconnects the base controller and lidar along with the camera.
CAMERA_USB_RESET_TARGETS = ("3251:1930",)
KNOWN_USB_HUB_IDS = frozenset({"0bda:5489", "0bda:0489", "1a86:8091"})


def process_matches(cmdline: tuple[str, ...], target: str) -> bool:
    executable, node_marker = PROCESS_MARKERS[target]
    return (bool(cmdline)
            and executable in cmdline[0]
            and node_marker in cmdline)


def stale_targets(now: float, started: float, last_seen: dict[str, float],
                  timeouts: dict[str, float], startup_grace: float) -> list[str]:
    if now - started < startup_grace:
        return []
    return [target for target, timeout in timeouts.items()
            if now - last_seen.get(target, started) > timeout]
