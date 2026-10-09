# Downstairs exploration development session

## Compact handoff — 2026-07-30

- Work only on the independent HWR stack. The factory stack is not part of the
  development path.
- Power decisions belong exclusively to the human operator. Software and tools
  must not query power telemetry or use it as a gate.
- Robot access was last known as `ubuntu@192.168.0.223`; it stopped responding
  on 2026-07-29 before the final `run_object_approach.py` update could be
  deployed.
- The local suite passes 264 tests. The latest local object-approach work fixes
  zero-motion reporting, requires direct VLM+depth+TF proof for arrival, retains
  individually valid audited depth points, improves stationary TF recovery, and
  adds fine marker-audit offsets.
- Pure-yaw scan odometry no longer injects x/y translation. Live paired turns
  confirmed zero translational drift after the fix.
- Semantic translation assessment is corridor-scoped: a person outside the
  swept envelope does not veto motion; a person in the corridor, a hazard,
  insufficient visibility, or metric failure does.
- Stored drums geometry is invalid. Seeing a named object is not arrival, and a
  station is reached only with current physical evidence at no more than 0.50 m.
- The physical dynamic-obstacle trial is deferred and is not part of this
  session.
- The robot will be placed by the human on the downstairs floor. It will not
  attempt to traverse stairs.

## Goal

Produce a repeatable, evidence-backed map of the larger downstairs area using
the independent perception, SLAM, navigation, exploration, and local-VLM stack.
The run should expand in bounded sectors, stop cleanly on uncertainty, save
recoverable map checkpoints, and identify useful semantic landmarks without
claiming physical arrival from visibility alone. The final required deliverable
is a human-readable description of the downstairs space grounded in the saved
metric map and captured observations.

## Work sequence

### 1. Restore and reconcile the runtime

1. Reconnect to the robot and confirm the independent runtime through process
   and navigation-context inspection only.
2. Deploy the pending object-approach file and verify local/remote hashes.
3. Build the affected ROS packages, run the full PC suite, then run stationary
   robot-side import and dry-run checks.
4. Preserve the old-room map and logs before starting a new downstairs map.

Exit: source, installed code, runtime processes, and evidence all identify the
same revision; the robot is stopped.

### 2. Harden frontier exploration before floor motion

1. Replace raw cluster-centroid goals with reachable free-cell observation
   poses set back from the unknown boundary.
2. Score frontiers using actual A* travel cost, target clearance, information
   gain, and heading change rather than straight-line distance alone.
3. Add failed-frontier cooldown/blacklisting so one unreachable frontier cannot
   be selected five times and abort the whole session.
4. Add per-goal timeout, total-session timeout, cancellation confirmation, map
   and pose freshness, localization-health gating, and stop-latched termination.
5. Journal every frontier snapshot, score, selected goal, preflight result,
   navigation result, and map-save result.
6. Put every translation leg through the same fresh metric corridor check and
   local-VLM semantic assessment used by the accepted navigation runner.

Exit: deterministic unit tests cover reachable goal selection, clearance/path
ranking, blacklist behavior, timeouts, stale inputs, cancellation, and journals.

### 3. Add negative-obstacle coverage

1. Derive a ground-support profile from aligned depth using the calibrated
   camera geometry; distinguish supported floor, positive obstacle, missing
   depth, and probable drop-off.
2. Reject a proposed swept corridor when required floor support disappears or
   depth uncertainty is too high. The local VLM may describe the scene but may
   not override geometric floor-support failure.
3. Validate offline with synthetic floor, threshold, and missing-depth cases,
   then stationary downstairs frames. Keep staircase regions out of reachable
   free space.

Exit: a staircase edge or unsupported floor prefix cannot produce a frontier
translation goal.

### 4. Establish the downstairs frame

Human input is needed once here: place the stopped robot on a level, open
downstairs staging patch with the camera forward, then say it is ready.

1. Capture a stationary color/depth/lidar panorama and local-VLM scene summary.
2. Start a fresh SLAM map with a recorded origin and preserve the initial frame.
3. Verify scan alignment, camera orientation, floor support, current footprint,
   and localization stability without translating.

Exit: the initial map and sensor bundle are internally consistent and saved.

### 5. Bounded exploration canaries

1. Dry-rank all current frontiers and record why the winner is reachable.
2. Execute one short frontier leg, capped at 0.10 m, then stop, reobserve, and
   compare expected versus fused physical motion.
3. Expand to three 0.20 m maximum legs with fresh replan and semantic/metric
   preflight after each leg.
4. Confirm that obstacle, person-in-corridor, unsupported floor, localization
   loss, or stale perception terminates at zero without selecting another goal.

Exit: three consecutive bounded legs succeed with no contact, no unsupported
floor exposure, consistent odometry/localization, and stop re-latched.

### 6. Sector-by-sector downstairs exploration

1. Explore one visibility sector at a time rather than issuing a long route to
   the globally highest-scoring frontier.
2. Save a versioned map after every successful frontier and on every clean
   termination.
3. Periodically rotate in bounded increments to refresh lidar overlap and use
   the local VLM to label rooms, doors, furniture, passages, and ambiguous
   regions.
4. Revisit loop-closure viewpoints when localization confidence degrades.
5. Stop when no reachable, supported, semantically acceptable frontier remains.

Exit: a final map, coverage report, trajectory, frontier journal, panorama set,
and termination reason exist under `evidence/phase9/downstairs_*`.

### 7. Acceptance and next capability

- Replay the journal offline and verify that each dispatched goal was the
  highest-ranked eligible frontier at that instant.
- Compare explored free area with the final manually reviewed downstairs map;
  target at least 90% of safely reachable free space, not unknown space behind
  stairs or closed barriers.
- Promote stable semantic landmarks into a separate topological layer. Do not
  create verified object-arrival anchors until a later direct approach proves
  the physical object is within 0.50 m.
- After one complete attended run, repeat from the saved downstairs map to test
  relocalization and directed navigation between two confirmed free-space
  waypoints.

### 8. Describe the explored space

1. Register every accepted panorama and semantic observation to the robot pose
   and metric map location where it was captured.
2. Segment the explored map into meaningful zones or rooms and describe their
   approximate dimensions, shape, and relative arrangement.
3. Describe traversable connections between zones: openings, corridors,
   doorways, narrow passages, and blocked or unexplored boundaries.
4. Inventory stable visible landmarks, furniture, fixtures, floor surfaces,
   and persistent obstacles. Keep recognition separate from verified physical
   arrival anchors.
5. Identify stair/drop-off regions, tight clearances, occluded regions, and any
   other navigation-relevant constraints supported by geometry.
6. Separate the report into observed facts, metric estimates, model
   interpretations, and unresolved uncertainty. Do not infer hidden rooms or
   object identities from a single ambiguous view.
7. Produce:
   - `evidence/phase9/downstairs_space_description.md`;
   - an annotated final map keyed to the described zones and landmarks;
   - a concise user-facing overview of what the space is like and how its
     traversable parts connect.

Exit: another person can understand the downstairs layout, contents, major
navigation constraints, and unexplored portions from the description and map
without watching the exploration run.

## Session progress — 2026-07-30

- [x] Runtime reconciled on the independent stack; factory runtime remained
  stopped. Power telemetry was not queried or used.
- [x] Frontier goals now use reachable set-back observation cells, one-pass
  Dijkstra travel costs, clearance/information/heading ranking, trapped-start
  descent, frontier cooldowns, and local failed-bearing cooldowns.
- [x] Exploration now has bounded staged yaw, 0.20 m maximum legs, fresh
  metric/floor/VLM preflight, persistent journaling, stop-latched goal
  boundaries, responsive stop during slow semantic calls, and committed
  frontiers across rotation stages.
- [x] Mapper/costmap resize races wait for a matching grid generation instead
  of terminating a valid session.
- [x] Saved mapping sessions now resume the exact occupancy geometry and saved
  physical map pose after a process restart.
- [x] Stationary downstairs panorama and initial semantic characterization are
  saved under `evidence/phase9/downstairs_panorama_*`.
- [x] One 0.10 m canary and three 0.20 m maximum canary translations completed
  with stop re-latched and fused physical motion evidence.
- [ ] Sector exploration is in progress. Two sectors reached the dining/bar
  aisle through the plant/furniture corridor. The latest sector completed three
  more confirmed 0.10 m translations and stopped; its remaining evidence is on
  the robot because network access dropped during transfer.
- [ ] Live-validate the new pure-yaw budgets: at most six 30° stages per
  committed frontier and 18 per session. This replaces the previously observed
  inefficient 25-stage sector scan; implementation passes locally but is not
  deployed while the robot is offline.
- [ ] Resume additional sectors, pull the final map/journal/panorama set,
  calculate safely reachable coverage, annotate zones, and finish
  `evidence/phase9/downstairs_space_description.md`.

### Shutdown checkpoint — 2026-07-30

- The last successful robot-side control command confirmed exploration stopped,
  estop latched, and `downstairs_20260730.{pgm,yaml,state.yaml}` saved.
- The robot network became unreachable before the final map/journal transfer.
  The checkpoint remains on the robot at `/home/ubuntu/hwr/maps/`.
- All three sector-2 image/context pairs are local under
  `evidence/phase9/downstairs_sector2_20260730/`; latest recorded pose is about
  `(1.036, 2.368, 1.095)` in the map frame.
- Local source includes the new six-stages-per-frontier / 18-stages-per-session
  yaw budgets and passes 276 tests. That final yaw-budget change is not yet
  deployed.
- Resume sequence: power on; pull the saved map/state and exploration journal;
  deploy/rebuild `hwr_explore`; verify map resume while stopped; run a bounded
  rotation-budget canary; then continue sector exploration.

## Session progress - 2026-07-31

- [x] Pulled the saved map, state, exploration journal, and missing sector
  evidence before resuming. The independent graph was restored; the factory
  service remained inactive throughout development.
- [x] Deployed and live-validated the six-rotations-per-frontier and
  18-rotations-per-session budgets. A driver-reset observation race and an
  active-goal grid-resize race were corrected; the full suite now passes 277
  tests.
- [x] Added an evidence-producing bounded-sector monitor. It counts translation
  attempts (including timeout/failure attempts), saves image/context pairs,
  stops exploration, and latches the stop on every exit. It only uses
  battery-free navigation endpoints.
- [x] Persisted the exploration leg cap at 0.10 m. A restart regression that
  briefly restored the old 0.20 m default was detected by a one-attempt canary,
  corrected, and revalidated with a 0.0996 m successful leg.
- [x] Extended the 0.10 m navigation deadline from four to six seconds and
  added a physical-odometry leash at requested distance plus 0.05 m. This
  removed false short-leg timeouts while bounding travel under map-pose drift.
- [x] Increased the translation clearance margin from 0.20 m to 0.25 m after a
  chair leg reduced the corridor to the exact old threshold. A one-leg escape
  canary then selected a clearance-increasing direction and moved away.
- [x] Completed sectors 3 through 11. The journal contains 20 bounded
  translation attempts (15 successes), 71 successful bounded rotations, and
  13 frontier-failure events. The 2026-07-31 map-frame route advanced about
  1.92 m net with no reported contact.
- [x] Saved and pulled `downstairs_session_final_20260731.*`, a gateway map
  rendering, a trajectory overlay, all pose-linked sector captures, and an
  updated grounded `downstairs_space_description.md`. Exploration is disabled
  and stop is latched at the terminal pose.
- [ ] Continue only from the terminal checkpoint: reduce yaw staging, explore
  the darker passage and remaining bright kitchen/bar sector from clear floor,
  replay frontier ranking, compute inflated safely reachable coverage, and
  validate loop/relocalization before claiming the 90% target.

## User-input checkpoints

Only two expected inputs interrupt autonomous development:

1. The robot is reachable again if its network address changed.
2. The robot has been physically placed on the level downstairs staging patch.

All other implementation, stationary validation, short canaries, mapping,
evidence capture, and analysis proceed without asking for per-leg approval.
