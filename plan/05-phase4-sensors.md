# Phase 4 — Sensors under our stack

**Objective.** Lidar and depth camera publishing under **our** launch, frames, and filtering — the perception inputs every autonomy phase consumes. Vendor binaries are wrapped (ADR-002/006), the processing around them is ours.

**Prerequisites.** Phase 3 (TF/URDF). During takeover the factory lidar/camera nodes are down — we start our own instances of the driver binaries.

## Subtasks

- [ ] **4.1 Lidar bring-up.** `hwr_sensors/launch/lidar.launch.py`: run the installed `sclidar_ros2` node (port `/dev/lidar`, 230400, `version=4`) publishing `scan_raw` in `lidar_frame` (our URDF provides the 180° mount TF — verify the driver doesn't double-compensate; if it does, fix OUR urdf/launch combo until walls land where reality is).
- [ ] **4.2 Our scan band-pass node.** `hwr_sensors/scan_filter.py`: crop to the valid ~219° sector (start from vendor bounds 1.25–5.08 rad, then verify empirically by boxing the robot with a known target), drop <0.12 m self-hits, publish `scan`. Replaces `laser_filters`. Unit tests with synthetic scans.
- [ ] **4.3 Empirical FOV mapping.** With the robot boxed in on 3 sides at known distances: record a bag, plot polar; confirm usable sector, blind sector, and that **base-forward objects appear forward** after TF. Adjust filter bounds; write the final numbers into `90-clean-room-reference.md`.
- [ ] **4.4 Depth camera bring-up.** Launch wrapping the Deptrum node with our remaps (`camera/depth/image_raw`, `camera/color/image_raw`, camera_info); pointcloud OFF (CPU). Verify optical frame TF against a hand-held target.
- [ ] **4.5 depth_to_ranges node.** Ours: subsample the depth image (arm at forward-look pose) into ≤64 forward range beams ≤3 Hz in `base_footprint`, floor-plane filtered — the thin-obstacle bumper for the costmap (chair legs, cables that the lidar plane misses). Config: beam count, height band, rate.
- [ ] **4.6 IMU filter node.** Ours: gyro bias estimation at rest + complementary yaw filter; publish `imu_filtered` (orientation filled). Replaces vendor `imu_filter`. Validate: 60 s stationary yaw drift measurement, and 4×90° box turns vs lidar-visible wall angles.
- [ ] **4.7 (stretch, non-blocking) Own COIN-D6 UART driver.** Sniff `/dev/lidar` (factory stack down): capture raw stream, identify sync/header, packet length, per-point angle/dist/intensity encoding, checksum; compare against public COIN-D6/YDLidar-style docs. Implement `hwr_sensors/coin_d6.py` decoder + LaserScan publisher; A/B against the binary node on the same physical scene. Swap behind a launch arg. Timebox 1 day; park with findings if the format resists.

## Acceptance & evidence

- **Test:** RViz overlay — walls at 1 m within ~3 cm of tape-measured truth, front-is-front; scan covers the measured usable sector only; depth_to_ranges flags a chair leg at 0.5 m (bag + plot); IMU stationary drift <1°/min after filtering.
- **Evidence:** `evidence/phase4/`: annotated RViz screenshot vs photo of the boxed-in setup, polar plot, chair-leg detection plot, drift log.

## Safety / rollback

- Sensors only; no motion beyond slow attended teleop for data capture. Camera work with arm at forward-look pose set via Phase 1 servo path.

**Effort.** 1–1.5 days (+1 day stretch 4.7).

**Open questions.** Does `scan_raw` from the binary already embed any angular offset (vendor's `version=4` param)? Settled by 4.3. Aurora node CPU cost with RGB+depth at 12 fps under our stack — measure with `top` during 4.4.

## Status (2026-07-09) — 4.1/4.2 authored + PC-tested; on-robot run blocked on charge

`hwr_sensors` package done on the PC: `lidar.launch.py` (sclidar binary wrapped per ADR-006: port `/dev/lidar`, 230400, version=4, frame `lidar_frame`, `/scan`→`scan_raw`) + our `scan_filter_node` over ROS-free `scan_crop.py` (band 1.25–5.08 rad, self-hits <0.12 m → inf, handles the band wrapping across the driver's angle origin; 6 PC tests, suite 110/110). Added to build.sh.

Check tools staged: `tools/static_phase4_lidar.zsh` (rates + geometry) runs `tools/live_phase4_lidar_check.py` — includes **matcher-on-real-scans**: consecutive static scans through the Phase 5 correlative matcher must give ~zero delta at high score, and dumps a scan sample for evidence.

**Blocked 2026-07-09:** battery read 10.83–10.86 V (< 11.0 V floor) after ~1.5 h idle-on; robot gracefully shut down (user-approved) for an off-power charge. **Next session: sync → build → takeover → `static_phase4_lidar.zsh` → handback** (~5 min total, no motion).
