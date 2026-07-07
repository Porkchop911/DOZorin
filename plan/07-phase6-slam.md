# Phase 6 — Hand-rolled SLAM

**Objective.** Build maps with our own code: log-odds occupancy grid + scan-to-map matching to kill drift, saved in the standard PGM+YAML format. "Lite pose-graph" only if plain scan-to-map proves insufficient for the flat.

**Prerequisites.** Phase 5 (fused odom + matcher core). Develop on bags; one live mapping run at the end.

## Subtasks

- [ ] **6.1 Occupancy grid.** `hwr_slam/grid.py` (ROS-free): log-odds grid, 0.05 m, lazily grown; integrate a posed scan by raycast (Bresenham, numpy-batched): misses decrement along the ray, hit increments the endpoint cell; clamping; occupancy/free/unknown thresholds. Unit tests: synthetic room, ray through known cells, saturation behavior.
- [ ] **6.2 Scan-to-map correction.** Before integrating each keyframe scan, match it against the *current map* using the Phase 5 correlative core (map rendered as likelihood field). Integrate at the corrected pose; feed the correction back to the fusion filter as a weak update. This bounds drift without a full graph.
- [ ] **6.3 Mapper node.** `hwr_slam/mapper_node.py`: subscribes `scan` + fused odom TF, keyframes on distance/turn, publishes `map` (`nav_msgs/OccupancyGrid`, throttled ~0.5 Hz) + `map→odom` TF from accumulated corrections. Start/stop/save via services.
- [ ] **6.4 Map I/O.** Save/load PGM+YAML byte-compatible with the existing tooling (resolution/origin/thresholds like `map_04.yaml`); `hwr map save <name>`.
- [ ] **6.5 (contingency, only if 6.2 shears on the real flat) Keyframe graph lite.** Keyframe poses + odom edges + scan-match edges on revisit (detected via matcher score against older keyframes); relax with a few Gauss-Newton iterations (scipy). Decide by evidence from 6.6, not preemptively.
- [ ] **6.6 Offline mapping.** Build maps from the Phase 3–5 bags; compare against the reference `map_04` (existing, from the old stack — used as ground truth context, not code): wall alignment overlay plot.
- [ ] **6.7 Live mapping run.** Attended teleop lap of the upstairs room with mapper live; save `hwr_map_01`; RViz screencap during build.

## Acceptance & evidence

- **Test:** live-built map visually congruent with `map_04` (doorways, walls, furniture footprints in the right places); wall thickness ≤2 cells along straight runs; the teleop loop closes with no visible double-wall/shear; mapper CPU leaves teleop responsive.
- **Evidence:** `evidence/phase6/`: overlay plot (our map vs map_04), saved map files, build-time screencap.

## Safety / rollback

- Attended teleop only. Mapper is a consumer — no motion authority.

**Effort.** 2 days (+1 if 6.5 triggers).

**Open questions.** Grid update rate vs Jetson budget with matcher also running (profile; keyframing keeps this bounded). Glass windows ate lidar in the old map ("door" artifact) — note expected artifacts in the map README rather than fighting physics.
