# Phase 6 partial live mapping — 2026-07-22

- Mapping ran with lidar, depth, IMU, fused odometry, camera, and rosbag capture.
- One attended ~90° base turn physically completed; the camera view and map both
  changed, and the bounded teleop publisher ended with explicit zero commands.
- Later base commands were refused by an internal actuation gate without an
  estop latch. No further motion was attempted.
- The partial 200×200, 0.05 m map was saved as
  `hwr_map_partial_20260722.{pgm,yaml}` and exported through the corrected
  gateway `/map.png` mapping-mode path.
- The map covers only the start pose and one turn. It is evidence of live
  scan-to-map operation, but is deliberately not accepted for localization or
  navigation.
- Hardware was returned to the factory stack; scan and web recovery passed.

No navigation goal was issued in this session.
