# Phase 8 — Hand-rolled navigation

**Objective.** Goal-directed autonomous motion on our whole stack: costmap, A* global planner, a local controller that exploits the holonomic base, forward-only recoveries, and a goal API. Gate: the standing tour, then an unattended soak — the project's real graduation bar.

**Prerequisites.** Phases 5–7 (odom, map, localization). Sim/unit tests before robot time (8.5 is built early, not last).

## Subtasks

- [ ] **8.1 Costmap.** `hwr_nav/costmap.py`: static layer from saved map + live layer from `scan` and `depth_to_ranges` (decaying marks), inflation = robot radius (~0.20 m) + margin with cost ramp. Publishes for RViz. Unit tests: known obstacles → expected cost cells; decay behavior.
- [ ] **8.2 Global planner.** A* on the costmap (8-connected, cost-weighted, unknown = expensive not forbidden), gradient/path smoothing, plan validity re-check on costmap updates. Tests: synthetic mazes, no-path cases, cost-vs-distance tradeoffs.
- [ ] **8.3 Local controller.** Carrot follower with holonomic output: velocity vector toward the carrot (vx,vy) + heading control (face direction of travel by default — keeps the lidar's good sector forward; final goal yaw honored at the end). Live costmap collision check on the immediate velocity command; slow-down ramps near obstacles/goal. **Reverse (negative vx) structurally impossible** — clamp at the controller, not by convention. Tests in the simulator.
- [ ] **8.4 Recoveries + goal server.** On stuck (no progress / controller blocked / localization health low): ① stop + clear live layer; ② rotate-scan (refresh lock + costmap); ③ forward-arc escape (the ESCAPE lesson from the old stack — but never backward); ④ replan; ⑤ abort with reason. Goal API as a ROS action (`NavigateToPose`-shaped for familiarity), plus named places from a `places.yaml`.
- [ ] **8.5 Simulator (build FIRST, alongside 8.1).** `tools/sim2d.py`: kinematic mecanum robot + ray-cast lidar against a map image, publishing the same topics the real robot does — planner/controller develop entirely against it (also future regression harness). Deterministic seeds for CI-style runs on the PC.
- [ ] **8.6 Attended live runs.** Single goals across the room; doorway traversal; dynamic obstacle (walk into its path) → stops, re-plans or waits. Tune inflation/speeds.
- [ ] **8.7 The tour.** drums → door → window → start (`places.yaml` from the old stack's photo-verified coordinates as the starting point, re-verified on OUR map), photo at each station via camera snapshot, 3 consecutive full runs.
- [ ] **8.8 Soak.** 20 random valid goals, unattended-but-observed, auto-log every result + battery; abort cleanly on battery gate.

## Acceptance & evidence

- **Test:** tour 3/3 with per-station photos; soak ≥18/20 goals reached without human touch; dynamic-obstacle test passes; zero reverse commands in the entire log (grep the cmd log — structural guarantee audited).
- **Evidence:** `evidence/phase8/`: tour photo sets, soak results table (goal, outcome, time), cmd_vel audit note.

## Safety / rollback

- Sim-first for every planner/controller change; live runs attended until 8.8; soak observed with estop at hand; battery gate wired into the goal server (refuse new goals <11.2 V, abort + park <11.0 V).
- Known trap from the old stack: costmap inflation can imprison the robot → recovery ② and ③ exist for exactly this; test it deliberately (box the robot in, watch it escape forward).

**Effort.** 3–4 days including the simulator.

**Open questions.** Controller frequency vs everything else on the CPU (target 10 Hz; degrade gracefully); whether the tour's "door" station needs a new photo-verified pose on our map (the old map's east pocket was a glass artifact).
