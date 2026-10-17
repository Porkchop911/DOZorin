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


def may_release_estop(tripped_at: float | None, recovered_at: float, last_scan_at: float,
                      estop_latched: bool | None, now: float, lidar_timeout: float) -> bool:
    """Release only the supervisor's own estop, and only once a REAL scan arrived after the respawn.

    Recovery fakes `last_seen` to start its cooldown, so freshness alone would release a dead lidar.
    """
    return (tripped_at is not None and estop_latched is True
            and recovered_at >= tripped_at
            and recovered_at < last_scan_at
            and now - last_scan_at <= lidar_timeout)
