# Phase 9 — Exploration + arm behaviors

**Objective.** The robot maps unknown space by itself, and the arm becomes a usable subsystem (poses, camera aiming, gripper, simple IK) — the physical skills the LLM agent will orchestrate.

**Prerequisites.** Phase 8 (nav) for exploration; Phase 1/3 servo path for the arm (arm work can start earlier in parallel if convenient — it has no nav dependency).

## Subtasks

### Exploration
- [ ] **9.1 Frontier detection.** On the live SLAM map: boundary cells between free and unknown, clustered; score = cluster size / travel cost / clearance; re-evaluate on map updates. Unit tests on synthetic half-explored maps; sim run in `tools/sim2d.py`.
- [ ] **9.2 Explore mode.** Mode manager runs SLAM (not MCL) + nav goals to best frontier; termination when no frontier ≥ threshold remains; auto map-save. Safety: same gates as Phase 8, plus per-goal timeout and total-session battery budget.
- [ ] **9.3 Live run.** From an empty map at the usual start pose, attended: explore the upstairs room to completion; compare coverage against `map_04` free area.

### Arm
- [ ] **9.4 Pose library + primitives.** `hwr_arm`: named poses YAML (forward-look, floor-look, ceiling-look, carry, tucked — seeded from the known-good pulses), smooth multi-servo interpolation (duration-based, using the protocol's native duration field), gripper open/close/grip-with-timeout.
- [ ] **9.5 Camera aiming.** pan/tilt(pulse) ↔ camera ray (from URDF chain): `aim_at(bearing, elevation)` and `sweep(n_views)` primitives; snapshot service returning a JPEG (foundation for look-and-describe in Phase 10).
- [ ] **9.6 Geometric FK/IK.** 5-DOF FK from URDF params; closed-form-ish IK for the practical subset (planar reach + base pan + wrist), `reach(x,y,z)` returning pulses or "unreachable"; validated against FK round-trip tests + 3 physical tape-measure checks.

## Acceptance & evidence

- **Test:** exploration reaches ≥90% of map_04's free area with zero collisions (attended); arm: `sweep(3)` produces 3 aimed photos; `reach` places the gripper within 3 cm of a marked point, 3/3; gripper picks up a light object (foam block).
- **Evidence:** `evidence/phase9/`: final explored map + coverage overlay, sweep photos, reach test photos/video.

**Safety.** Exploration inherits all Phase 8 gates; arm motions tested with the arm clear, torque-off on estop; IK clamped to joint limits ±2.09 rad and a keep-out around the chassis/camera cable.

**Effort.** 2 days (1 exploration, 1 arm).

**Open questions.** Whether SLAM quality during autonomous (jerkier) motion needs slower explore speeds; gripper force behavior (no torque feedback in protocol reads? verify what `read_torque_state` actually returns — empirical).
