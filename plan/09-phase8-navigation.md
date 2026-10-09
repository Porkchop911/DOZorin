# Phase 8 — Hand-rolled navigation

**Objective.** Goal-directed autonomous motion on our whole stack: costmap, A* global planner, a local controller that exploits the holonomic base, forward-only recoveries, and a goal API. Gate: the standing tour, then an unattended soak — the project's real graduation bar.

**Prerequisites.** Phases 5–7 (odom, map, localization). Sim/unit tests before robot time (8.5 is built early, not last).

## Subtasks

- [x] **8.1 Costmap.** Static plus decaying scan/depth live layers, 0.20 m hard inscribed chassis radius plus 0.10 m soft margin, ROS publication, and unit coverage are implemented. A*/simplification and swept command checks reject the hard footprint; live acceptance beside a 0.371 m pole correctly shortened the affected centerlines to 0.20 m.
- [x] **8.2 Global planner.** Cost-weighted 8-connected A*, path simplification, and no-path tests are implemented.
- [x] **8.3 Local controller.** Holonomic carrot controller, translation-free pure-yaw handling, yaw-aware progress watchdog, immediate swept collision check, structural no-reverse clamp, and a 30-degree rotate-first perception cone are implemented and tested. Autonomous yaw is capped at 0.30 rad/s. A degraded localization sample still commands immediate zero velocity; the action may resume only if fresh healthy localization returns within a bounded 1.5-second pause, otherwise it aborts.
- [x] **8.4 Recoveries + goal server.** Stop/clear/rotate/replan recovery and the HWR `NavigateToPose` action are implemented. Initial no-path goals abort at zero; a live-command collision stops, performs up to three bounded replans, and aborts at zero if no progress resumes. Forward-arc recovery is implemented but opt-in and remains disabled pending a dedicated open-space acceptance test. Named-place verification remains operational work.
- [x] **8.5 Simulator (build FIRST, alongside 8.1).** Deterministic `tools/sim2d.py` and planner/controller regression tests are implemented.
- [ ] **8.6 Attended live runs.** Current-pose, multiple 90/180-degree yaw goals, 0.15/0.25/0.50/0.70 m goals, a continuous 1.00 m across-room goal, obstacle-avoidance waypoints, and a stationary full-room semantic panorama passed across 2026-07-22/23 (`evidence/phase8/live_navigation_20260722.md`). Post-reboot takeover/camera restoration also passed. A stand-pole door approach correctly aborted at zero when its path closed. The panorama confirms there is no currently visible open doorway and the legacy door coordinate is a map/glass artifact; doorway traversal now requires the human to identify/open the real passage. The operator explicitly deferred the physical dynamic-obstacle trial on 2026-07-29; the synthetic stop/no-retry result remains accepted, but physical dynamic-obstacle behavior is not accepted.
- [ ] **8.7 The tour.** start → window → drums → door → start. The old stack's approach poses are only unverified starting guesses. Each object needs a physically measured map anchor and a collision-free approach pose; a station succeeds only when the final localized robot-center distance to the actual object is at most 0.50 m. Visibility and a station photo are required evidence but never count as arrival. Historical 2026-07-27 window/drums claims are not accepted as current proof: the 2026-07-29 drums run showed that the stored drums anchor can report 0.479 m while fresh grounding places the actual drum head roughly 1.0 m away. `object_approach_candidates` samples and ranks costmap-reachable poses around point or segment geometry. `tools/run_object_approach.py` now performs bounded re-ground/replan execution, independently audits each VLM pixel, tolerates one depthless specular point only when another audited point has metric depth, and requires live VLM+depth+TF geometry for terminal success. Stored place proximity alone returns `verification_required`, never arrival. Still required: complete and preserve a current drums arrival with direct depth proof, replace the invalid drums anchor, establish door identity/geometry, then run 3 consecutive full tours with a photo and measured distance at every station.
- [x] **8.8 Soak.** `tools/run_nav_soak.py` dry-plans or explicitly executes up to 20 deterministic reachable goals, journals every event/result, never resets estop itself, and cancels/re-latches stop on timeout, localization loss, estop, or any negative-forward command. Execution requires semantic preflight: it rotates toward costmap-reachable candidates, fuses a corridor-scoped local-VLM assessment with fresh metric clearance, rejects people/cables/poles/bases/overhangs, ranks provisionally safe routes, revalidates the winner, and falls back from 0.35-0.75 m legs to 0.20-0.34 m escape steps. Trapped-start recovery may cross an equal-cost non-obstacle inflation plateau only without increasing cost; a true 255 obstacle cell must descend immediately, and command prefixes remain constrained to the verified escape route. Directional blocked-prefix/free-corridor profiles remain subject to semantic and metric approval. Staged 2/2 and 5/5 runs passed on 2026-07-23. The detached acceptance run on 2026-07-27 passed 20/20 in one invocation, after 230 bounded candidates and 226 semantic assessments, with 160 semantic rejections, zero failures/refusals, zero reverse samples, healthy final localization, and stop re-latched. Evidence: `evidence/phase8/semantic_full_20goal_20260727.md` and its linked JSONL/stdout/frame.

## 2026-07-28 collision gate

The staged drums approach contacted a chair. Phase 8 live acceptance is
re-opened and all earlier soak/tour results are historical evidence only; they
do not approve further autonomous motion. The preserved incident bundle is
`evidence/phase8/collision_chair_20260728/incident.md`.

Local containment changes now fail closed on a person inside the proposed
swept corridor or any reported near-field hazard, preserve VLM hold decisions,
require fresh lidar and depth
for translation, reject rotation from a blocked footprint, remove
trapped-start command exceptions, abort immediately on translation
localization loss/stall, disable live-obstacle retries by default, cap staged
legs at 0.15 m, and terminate after the first physical-leg failure.

Before any new floor-motion acceptance run, all of the following remain open:

- [x] Review the chair incident patch and its stationary test evidence. The
  corrected suite passes 264 tests; live warmed traces and hashes are recorded
  in the incident bundle.
- [ ] Deploy only while stopped, then prove stale lidar/depth, blocked current
  footprint, person-visible, chair-visible, localization-loss, and leg-timeout
  cases all refuse motion with wheels clear. The corrected forward-camera
  chair-visible preflight now refuses translation; the remaining cases still
  need their combined live acceptance record.
- [x] Add recorded-command evidence that a detected obstacle causes one stop
  and terminal abort, never a route-around or automatic retry. The
  event-triggered 2026-07-28 depth test recorded exactly one applied-motion
  burst followed by `blocked after live-costmap replans`; hashes are in the
  chair-incident bundle. The matching physical-object trial was explicitly
  deferred by the operator on 2026-07-29 and is not acceptance evidence.
- [x] Re-run a 0.05 m open-floor canary with a measured exclusion zone and a
  human at the stop control. The 2026-07-28 corrected run advanced odometry by
  about 0.049 m, captured nonzero nav/applied/wheel streams, and re-latched
  estop; see `collision_chair_20260728/enabled_canary_trace.txt`.
- [x] Staged object approach resumed with bounded 0.10 m legs after the operator
  authorized continued attended development until input is needed. The window
  canary passed. Subsequent yaw work exposed and fixed scan-odometry x/y
  contamination during pure rotation; two live 28-degree turns then preserved
  exactly zero odometric translation. Direct drums work remains open because
  current visual/depth evidence disproved the stored anchor and the presently
  visible genuine drum components have no stable collision-free 0.50 m
  candidate from this side. Every rejected attempt stopped before translation.

## Acceptance & evidence

- **Test:** tour 3/3 with per-station photos and measured object distance ≤0.50 m for window, drums, and door; soak ≥18/20 goals reached without human touch; dynamic-obstacle test passes; zero reverse commands in the entire log (grep the cmd log — structural guarantee audited).
- **Evidence:** `evidence/phase8/`: tour photo sets, soak results table (goal, outcome, time), cmd_vel audit note.

## Safety / rollback

- Sim-first for every planner/controller change; live runs attended until 8.8; soak observed with estop at hand; localization freshness, collision checks, command leases, and human stop remain mandatory.
- Known trap from the old stack: costmap inflation can imprison the robot → recovery ② and ③ exist for exactly this; test it deliberately (box the robot in, watch it escape forward).

**Effort.** 3–4 days including the simulator.

**Open questions.** Controller frequency vs everything else on the CPU (target 10 Hz; degrade gracefully); whether the tour's "door" station needs a new photo-verified pose on our map (the old map's east pocket was a glass artifact).
