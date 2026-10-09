# Phase 4 live perception — 2026-07-22

- Full graph: lidar ~9.94 Hz, RGB ~14.9 Hz, depth ranges ~2.8 Hz, filtered IMU
  ~50 Hz, scan odometry ~4.2 Hz.
- Fixed partial joint-state publication that disconnected the articulated
  `depth_camera_link` TF subtree. After rebuild, `base_footprint` to camera TF
  resolved and depth ranges published continuously.
- Captured real 640×400 center/left/right frames. The camera sweep observed the
  window, curtains, cabinet, traversable floor, office chair, nearby furniture,
  cables, and a person without attempting identification.
- Added and live-tested Windows `hwr describe`; it fetched an authenticated
  gateway snapshot and `qwen/qwen3-vl-4b` produced a correct scene description
  through localhost LM Studio.

Artifacts: `color_live_20260722.jpg`, `color_forward_20260722.jpg`,
`color_sweep_left_20260722.jpg`, `color_sweep_right_20260722.jpg`, and
`color_ros_quarter_turn_20260722.jpg`.
