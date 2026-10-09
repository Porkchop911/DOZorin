# Live navigation acceptance — 2026-07-22

- Independent graph: HWR driver/base/sensors/odom/localizer/nav; factory stack
  remained inactive.
- Costmap: 200×200 cells at 5 cm resolution, loaded from the HWR-built map.
- Current-pose action goal: accepted and `SUCCEEDED`; applied chassis command
  remained exactly zero.
- Inflated-map preflight for the attended physical goal: start and target costs
  were both zero and A* returned a valid path.
- Physical goal: **0.15 m forward** in map coordinates; action returned
  `SUCCEEDED`.
- The applied command ended at exact zero and localization remained healthy
  (score 0.771).
- Post-goal image: `nav_015_20260722.jpg`; the observed lane remained clear.

This accepts the first attended single-goal run. Doorway/dynamic-obstacle runs,
the three-tour gate, and the 20-goal soak remain open.

## Low-obstacle integration and recovery finding

- Corrected the depth optical TF, which had mapped camera-forward to base `-X`.
- Added robust ground-plane subtraction; the false floor arc disappeared.
- Live result: 54/64 depth beams, nearest stand leg 1.17–1.19 m at −12.3°;
  all endpoints were lethal in the costmap and 51–52 raised the static baseline.
- A later 0.15 m goal did not progress as expected and invoked the legacy
  rotate-scan/forward-arc recovery. The action aborted; estop was applied and
  exact zero velocity verified. Final depth clearance was 0.31 m and the image
  `nav_abort_20260722.jpg` showed the robot facing close stand/table legs.
- Corrective policy: no-path and immediate live-costmap collision now abort
  without motion. Forward-arc recovery is opt-in and disabled by default.
- Post-fix blocked-goal acceptance used the nearest live depth endpoint
  `(−0.033, −0.674)` as the requested goal. The action returned
  `ABORTED: no collision-free path`, applied velocity remained exact zero, and
  `blocked_before_20260722.jpg` / `blocked_after_20260722.jpg` show no pose
  change.
- Immediate collision prediction now samples the full projected motion segment
  at half-cell spacing rather than checking only its endpoint.

## Repositioned open-space acceptance

- Global relocalization after the attended reposition seeded eight hypotheses;
  the best observed localization score was 0.957.
- A stationary yaw-only test exposed controller starvation: navigation generated
  the correct angular command, but repeated Python distance-field construction
  consumed about 75% CPU and allowed the command mux lease to expire.
- Replacing full-grid heap inflation with a bounded, vectorized inflation kernel
  reduced a representative 1,500-point live-grid update to 1.91 ms. A sustained
  estop-latched trace then observed 90 nonzero navigation samples, 214 applied
  command samples, and 213 wheel-command samples with navigation continuously
  selected by the mux.
- An attended 45-degree clockwise yaw goal and a subsequent 0.25 m forward goal
  both returned `SUCCEEDED`. Each action ended with an exactly zero applied
  command and healthy localization. Images: `yaw_accept_20260722.jpg` and
  `forward_accept_025_20260722.jpg`.

## Invalid-depth low-obstacle acceptance

- The post-goal image showed a very low black stand bar crossing the floor ahead.
  Its center ROI was only 41.8% depth-valid while adjacent floor ROIs were fully
  valid, identifying the bar primarily as a structured-light invalid-depth gap.
- The depth projector now converts supported, multi-row lower-image invalid gaps
  into conservative obstacle ranges while rejecting isolated holes, horizon
  gaps, and bottom-border gaps. Two synthetic regression tests cover both cases.
- With the robot stationary and estop latched, the live costmap contained 1,644
  raised cells and the forward centerline became lethal at 1.25 m. All 45 valid
  projected depth endpoints were lethal and raised the static map.
- The independent navigation graph remained online, relocalization succeeded
  with score 0.806 after restart, and `/cmd_vel_applied` was exactly zero.
- Full framework-free regression suite after depth-gap work: `160 passed in
  13.69s`.

## Bounded live-obstacle replanning

- An immediate live-costmap collision now publishes zero, discards the current
  path, and requests a fresh A* route instead of terminating the goal at the
  first blocked command.
- The retry budget allows three consecutive obstacle-triggered replans and is
  reset only after at least 3 cm of renewed progress. Exhaustion aborts the goal
  at zero; initial no-path goals still abort without motion.
- Robot deployment reported `max_live_replans=3`, relocalized with score 0.849,
  retained the low-bar forward lethal mark at 1.25 m, and published exactly zero
  applied velocity under estop.
- Full framework-free regression suite after the replan policy: `161 passed in
  13.18s`; Python compilation and `git diff --check` also passed.

## Attended route around the low bar

- Added `tools/send_relative_goal.py`, which snapshots the localized pose,
  transforms a bounded robot-relative offset into map coordinates, sends one HWR
  navigation action, reports feedback/result, and cancels on timeout. It never
  changes estop state itself.
- Extended costmap diagnostics with robot-relative forward, diagonal, and lateral
  lethal-ray clearances. The preflight showed more than 1.5 m on the front-right
  diagonal while the low stand bar occupied the direct-forward route.
- First front-right waypoint: 0.35 m forward plus 0.35 m right (0.495 m total).
  The action returned `SUCCEEDED`, remaining distance fell to about 0.02 m,
  localization stayed healthy (score 0.772, ESS 415), and applied velocity ended
  at exact zero. Evidence: `after_front_right_0495_20260722.jpg`.
- Second front-right waypoint: 0.30 m forward plus 0.30 m right (0.424 m total).
  The action returned `SUCCEEDED`, localization stayed healthy (score 0.742,
  ESS 481), and applied velocity again ended at exact zero. Evidence:
  `after_front_right_0424_20260722.jpg`.
- The second pose is a narrow pocket between stand legs/base on the left and
  cables/fan on the right. Post-goal inflated clearances were 0.40 m forward,
  0.35 m front-left, 0.20 m left, 0.95 m front-right, and 0.30 m right. Testing
  stopped with estop latched rather than attempting another tight translation or
  an unchecked in-place rotational sweep.

## Footprint enforcement and open-patch exit

- Directional diagnostics initially paired the first costmap message with a
  newer depth frame. The tool now settles for one second (configurable) while
  retaining the latest messages and reports costmap/depth timestamp skew.
- The settled view exposed a planning-model defect: inflation inside the 0.20 m
  chassis radius was a soft gradient, so global planning and swept command checks
  could accept a centerline inside the physical footprint. The costmap now marks
  the inner 0.20 m radius as inscribed cost 253, retains only the outer 0.10 m as
  soft margin, and A*/path simplification treat costs 200 and above as blocked.
  The URDF collision body is 0.26 by 0.20 m (0.164 m corner radius), so the hard
  radius remains conservative for in-place rotation.
- Live stationary acceptance beside the close pole changed the forward and
  front-right hard-footprint clearances from misleading long rays to 0.20 m,
  matching the 0.371 m depth endpoint at -19.4 degrees. Applied velocity remained
  exactly zero under estop.
- Full framework-free regression suite after the footprint correction:
  `162 passed in 14.15s`.
- Subsequent attended actions all used stop/recheck wrappers: 30 degrees clockwise
  (fan view), 20 degrees counterclockwise (wall/door alignment), 0.50 m forward,
  75 degrees counterclockwise, and 45 degrees counterclockwise. Every accepted
  action returned `SUCCEEDED`, remained localized, and ended at exact zero.
- One 0.495 m front-left request produced no motion because the short-lived client
  did not discover the action server within two seconds. The estop wrapper still
  verified zero; the client discovery allowance was raised to ten seconds.
- Retrying the same 0.35 m forward plus 0.35 m left goal succeeded and exited the
  pole pocket without reverse motion. Final hard-footprint clearances were 1.20 m
  forward/front-left, 0.75 m left, 1.25 m front-right, and 1.10 m right; localization
  score was 0.730 with ESS 420. Evidence: `open_patch_goal_0495_20260722.jpg`.

## Reboot recovery acceptance - 2026-07-23

- A persistent timestamp-only `.takeover` marker survived reboot while the factory
  service restarted. The old takeover script incorrectly treated the marker as
  authoritative and allowed a partial HWR graph to start alongside the factory
  graph. No navigation action or depth/costmap was available, and applied velocity
  remained exactly zero.
- Takeover markers now include the Linux boot ID. A marker is accepted only in the
  same boot while the factory service is inactive; otherwise it is logged as stale,
  removed, and the normal service-stop/serial-owner sequence runs. The live retry
  stopped the factory service, obtained exclusive hardware ownership, and a second
  invocation in the same boot correctly returned an idempotent no-op.
- Factory startup left the arm/camera aimed at the floor mat. Raw depth was healthy
  at about 14.6 Hz with 81% valid pixels around 0.27-0.48 m, but ground filtering
  correctly yielded 0/64 obstacle beams. Navigation now includes the HWR arm node;
  the live-verified named `forward` pose restored the camera over a two-second
  interpolation while the base stayed at exact zero. Afterward raw depth covered
  0.50-2.64 m and 59/64 projected beams were valid.
- Relocalization after clean takeover and camera restoration reached score 0.937.
  A 0.30 m forward plus 0.30 m right goal then succeeded, followed by a 0.70 m
  straight goal. Both ended at exact zero with healthy localization and active
  hard-footprint perception. Evidence: `reboot_goal_1_20260723.jpg` and
  `reboot_goal_070_20260723.jpg`.
- Same-boot takeover idempotence, shell syntax checks, Python compilation,
  `git diff --check`, and the complete framework-free suite passed (`162 passed in
  14.44s`).

## Across-room and doorway attempt - 2026-07-23

- A continuous 1.00 m goal across the available room lane returned `SUCCEEDED`.
  Feedback decreased monotonically from 1.000 m to 0.077 m, localization remained
  healthy (score 0.743, ESS 493), and applied velocity ended at exact zero.
  Evidence: `across_room_100_20260723.jpg`. This closes the attended across-room
  item in Phase 8.6.
- A subsequent 180-degree pure-yaw goal exposed a progress-watchdog defect: the
  watchdog counted only translation, so normal yaw progress was declared stuck
  and two rotate-scan recoveries ran before a zero-velocity abort. Evidence:
  `yaw_watchdog_abort_20260723.jpg`.
- Pure-yaw goals now ignore centimeter-scale localized-position drift and use a
  translation-free yaw command. The progress watchdog counts decreasing yaw
  error near the goal, and a genuine final-yaw stall aborts at zero rather than
  entering physical recovery. Live 90- and 180-degree retests both returned
  `SUCCEEDED` without recovery; evidence: `yaw_progress_accept_20260723.jpg`.
- The legacy door coordinate `(-1.55, -1.26)` is blocked on the accepted HWR map
  (live cost 252). A read-only bounded search found a provisional free approach
  cell at `(-1.225, -0.875)`, but the physical action aborted `no collision-free
  path` when a thin stand pole entered depth at 0.296 m. Output stopped exactly;
  evidence: `door_approach_abort_20260723.jpg`. Door coordinates remain unverified.
- A short diagonal escape then brought a table leg into the depth camera only at
  0.256 m because its 45-degree approach lay outside the camera's approximately
  +/-37-degree horizontal field. The controller now permits autonomous translation
  only within a 30-degree forward perception cone; larger path bearings rotate
  first so low side obstacles enter depth view before translation.
- The robot was left stationary with estop latched in the table-side pocket.
  Relocalization score was 0.836 and all forward rays were hard-blocked within
  0.15 m. Manual repositioning into an open patch is required before continuing
  doorway, tour, or soak tests.
- Complete regression suite after yaw-progress and perception-cone changes:
  `165 passed in 13.73s`; compilation and `git diff --check` passed.

## Local-VLM scene judgment - 2026-07-23

- Normal navigation mode now launches the established authenticated autonomy
  gateway alongside navigation and arm services. `deploy/env.zsh` auto-exports
  the robot's preserved uncommitted gateway configuration; port 8090 and the
  `/hwr_gateway` node were live while base output remained exactly zero.
- The Windows CLI fetched `/snapshot` through the authenticated gateway and sent
  it to the PC-local LM Studio vision endpoint. The VLM identified the immediate
  table-leg entrapment, found no clearly traversable forward corridor or visible
  doorway, saw no person in the current frame, and explicitly recommended human
  repositioning. This agrees with the hard-footprint rays blocked within 0.15 m.
- Semantic vision remains advisory. Depth, lidar, localization, costmap, and
  exact-zero checks continue to own motion acceptance.
- Added authenticated `GET /navigation/context`, which provides pose,
  localization, goal state, camera freshness, and hard-footprint rays without
  any power fields. Structured PC-local VLM assessment now consumes this endpoint
  plus `/snapshot` and returns `safe_to_translate`, doorway/person visibility,
  hazards, corridor description, and a bounded recommendation.
- A 30-degree left yaw-only view showed a chair, desk/computer area, cables, and
  open foreground. Metric forward clearance was 1.25 m, but the structured VLM
  conservatively returned `safe_to_translate: false`, no visible doorway, and
  `sensor_preflight`; no translation was attempted and base output remained zero.
- The autonomous agent no longer exposes a battery tool and no longer polls the
  general `/status` route. Goal polling, `where_am_i`, and the new
  `assess_scene` skill use only the battery-free navigation context. General
  telemetry remains a human-only surface.
- End-to-end `assess_scene` acceptance from the PC-local LM Studio endpoint
  returned `safe_to_translate: false`, no visible doorway, hazards `wires on
  floor` and `chair obstruction`, and `sensor_preflight`. The paired navigation
  context reported healthy tracking, no current goal, no active drive, and
  0.60-1.20 m hard directional clearances. No motion endpoint was called.
- The robot-side `hwr_agent` package rebuilt successfully. JSON validation,
  Python compilation, `git diff --check`, seven focused agent/CLI tests, and the
  full framework-free suite passed (`169 passed in 12.41s`).
- A stationary arm-camera sweep (base drive remained inactive) found no doorway.
  The left view reported a corridor blocked by furniture and floor clutter. The
  right view looked more open but exposed a VLM contradiction: it marked
  translation safe while also reporting wires/clothing and requesting sensor
  preflight. Both agent and CLI assessment paths now apply a deterministic,
  transparent consistency veto: reported floor hazards, a blocked/cluttered/
  uncertain corridor, or a stop/reposition recommendation force
  `safe_to_translate` false while preserving the model's original true value as
  `model_safe_to_translate`. The arm returned to the verified forward pose.

## Stationary semantic panorama - 2026-07-23

- After attended repositioning, the robot remained localized while a bounded
  yaw-only panorama covered the desk/table frame, fan and drum rack, boxes and
  tools, suitcase and office chair, PC/desk, window/curtains, and shelving. Each
  yaw goal succeeded, each wrapper re-latched estop immediately, and every
  battery-free navigation context reported no current goal and no active drive.
  No translation was commanded.
- Saved frames are `go_preflight_20260723.jpg` and
  `door_scan_{left25,left70,right90,right135,right180,right225,right270,right315}_20260723.jpg`.
  No frame contains an open passage into another space. The VLM initially
  hallucinated both a person in the fan/cable view and a doorway in the
  desk/blank-wall view; direct frame inspection rejected both. Prompts now
  require explicit visual evidence and define windows, curtains, blank walls,
  closed doors, and furniture gaps as non-doorways.
- An attempted arm-camera look while estop was latched produced essentially the
  same frame (2.25 mean absolute RGB levels of JPEG/noise difference). Root cause:
  the driver correctly discarded servo commands under estop, but the arm service
  acknowledged publication without knowing driver state. The driver now
  publishes transient-local `estop_state`; the arm node starts conservatively
  latched and rejects commands until it receives a reset state. Gateway arm and
  gripper routes translate service rejection into HTTP 409 instead of reporting
  success. The hardware driver now also boots with estop latched and immediately
  issues motor/servo stops before publishing its initial state.
- Full framework-free regression suite passed (`169 passed in 14.33s`), along
  with Python compilation and `git diff --check`.
- Deployed acceptance after navigation restart: `/estop_state` delivered
  `data: true`; an arm request while latched returned HTTP 409; battery-free
  context showed healthy localization, no goal, and inactive drive. After an
  explicit reset, the verified forward arm pose was accepted and estop was
  immediately re-latched. The stricter VLM prompt then correctly returned no
  person and no doorway with empty evidence fields.

## Soak-runner preflight and restart defects - 2026-07-23

- Added `tools/run_nav_soak.py` plus the framework-free
  `reachable_goal_candidates` sampler. Dry-run is the default; physical execution
  requires `--execute` and an already human-reset estop. It samples only genuinely
  free, known, A*-reachable cells on bounded 0.35-0.75 m legs, writes JSONL events,
  audits `/cmd_vel_applied`, and cancels/re-latches stop on timeout, localization
  loss, estop, or any negative-forward sample. Three deterministic sampler tests
  cover bounds, reachability, unknown-space rejection, and validation. Full suite:
  `172 passed in 12.85s`.
- The first live dry-run correctly refused stale pose. Investigation found that
  automatic global relocalization called coarse search on a transient empty scan
  and terminated the localizer with `ValueError: scan contains no hit points`.
  Empty/short scans now publish degraded health without updating or relocalizing;
  the automatic relocalization path also catches future degenerate-scan rejection.
  The node remained alive beyond the previous crash interval afterward.
- The same restart exposed malformed `/joint_states`: five names arrived with ten
  duplicated positions, flooding robot-state-publisher and preventing articulated
  TF updates. The driver created a valid 5/5 message immediately before publish,
  while a standalone publisher serialized correctly. Reusing one persistent,
  fixed-shape JointState and updating its five positions in place eliminated the
  transport anomaly; live subscribers then received exactly 5 names/5 positions.
- With those fixes deployed and estop still latched, the live soak preview produced
  all 20 deterministic free reachable candidates. No goal was sent. The new
  localization hypothesis differs substantially from the last physically verified
  pose, so physical soak execution remains blocked pending visual/map confirmation.

## Human-referenced localization and semantic canary - 2026-07-23

- The human identified the initial physical heading as facing the window. The
  restored forward camera view matched that reference, confirming the live
  localization hypothesis sufficiently for a bounded attended canary. A clean
  independent-stack takeover first stopped the remaining factory application;
  the live ROS graph and serial ownership were then HWR-only apart from the
  permitted button scanner.
- A yaw-only view toward the first metric candidate exposed a drying rack, table
  legs, and hanging fabric. Although the 2-D costmap offered a reachable goal,
  camera inspection correctly rejected translation because the overhang is not
  fully represented by planar lidar. A second room-facing view exposed mat-edge
  cables. Two additional yaw-only goals aligned the camera with a short open
  wood-floor segment; every wrapper immediately re-latched stop afterward.
- With healthy localization, no active goal, visual `safe_to_translate: true`,
  no reported hazards, `sensor_preflight`, and forward metric clearance beyond
  the 1.5 m reporting limit, a 0.40 m relative navigation goal ran from about
  `(1.185, -0.518)` to `(0.785, -0.517)`. It succeeded; the observed final pose
  was about `(0.788, -0.517)`, localization remained
  healthy, and stop was immediately re-latched.
- Post-canary inspection rejected further straight travel because the rack base
  and floor cables were closer. Another sampled candidate was rejected for the
  same rack base and hanging-fabric overhang. The attempted retrace direction
  was also rejected before translation because the camera showed a seated person
  and a loose cable across the lane. The robot remained stopped.
- Structured semantic normalization now handles both directions of VLM
  contradiction. A hazard-free, explicitly safe open-floor assessment that
  emits a generic `hold` is transparently corrected to `sensor_preflight`, while
  `blocked`, `clutter`, `uncertain`, or `obstruction` corridor language forces a
  visual veto even if the model's boolean is optimistic. The raw contradictory
  field is preserved for audit. Ten focused agent/CLI tests pass, along with
  Python compilation and `git diff --check`.
- Saved frames: `soak_canary_current_20260723.jpg`,
  `soak_alternate_view_20260723.jpg`, `soak_open_lane_20260723.jpg`,
  `soak_canary_success_20260723.jpg`, `soak_candidate2_view_20260723.jpg`, and
  `soak_retrace_view_20260723.jpg`.

## Semantic clutter handling - 2026-07-23

- The metric-only soak runner could propose routes through camera-visible drying
  rack bases, hanging fabric, cables, chair legs, and people. Execution now
  requires `--semantic-preflight`. For every candidate the robot performs a
  rotate-only alignment, captures fresh battery-free navigation context and a
  camera frame, asks the PC-local VLM through the established SSH reverse tunnel,
  applies deterministic contradiction checks, and translates only after both
  semantic and metric gates pass.
- Direct 0.35-0.75 m candidates remain preferred. When none pass, the runner
  searches bounded 0.20-0.34 m escape steps with a 0.10 m metric margin. Candidate
  targets are ranked by surrounding known-free costmap clearance. All
  provisionally allowed views in a band are scored using clearance surplus,
  person separation, and semantic confidence; the winner is rotated to and
  reassessed immediately before dispatch.
- The fusion layer distinguishes a visible person from a person intersecting the
  proposed robot-width corridor. It reclassifies objects only when the VLM's own
  rationale explicitly places them outside a sub-0.35 m path. Generic holds may
  be corrected only for hazard-free, person-free, positively clear micro-paths
  with fresh metric approval. Darkness, glare, shadows, low-light, occlusion,
  sensor-authority denial, and contradictory person-in-corridor evidence always
  veto that correction. Raw model fields and every correction remain in JSONL.
- Early acceptance runs exposed and then fixed three live contradictions: generic
  hold on a clear short lane, hazards listed despite rationale placing them
  outside the corridor, and `person_in_corridor: false` paired with rationale
  claiming a person in-path. The prompt now also identifies the robot's permanent
  bottom-edge gripper/base shapes so they are not mistaken for feet.
- Multiple single-leg trials succeeded after rejecting between six and twelve
  unsafe headings. The continuous acceptance log
  `semantic_continuous_retry_20260723.jsonl` passed 2/2 consecutive goals from
  about `(1.083, -0.873)` through `(1.262, -1.008)` to `(1.242, -0.778)`. Both
  were semantic-plus-metric escape legs, all final revalidations passed, reverse
  samples were zero, localization remained healthy, and stop was re-latched.
  A preceding continuous attempt completed goal 1 and then safely aborted goal 2
  on a local-VLM timeout; bounded inference retry was added afterward.
- Evidence includes `semantic_canary_20260723.jsonl`,
  `semantic_escape_fused_20260723.jsonl`,
  `semantic_best_safe_20260723.jsonl`,
  `semantic_continuous_retry_20260723.jsonl`, and paired final camera frames.

## Trapped-start escape - 2026-07-23

- A normal candidate passed preflight and finished beside the open suitcase, but
  the refreshed costmap then marked the robot-origin cell lethal. The previous
  gateway reduced every directional clearance to zero, A* rejected every start,
  and the soak sampler returned no candidates. The camera still showed open floor
  in directions away from the suitcase, so this was an inflated-start recovery
  defect rather than an absence of physical escape space.
- The gateway now reports `trapped_start` plus, per direction, the length of the
  blocked prefix and the next blocked distance after the ray exits that prefix.
  Live values exposed asymmetric escape geometry instead of five identical zeros:
  for example, the right ray exited after about 0.05-0.10 m while the left ray
  offered a longer corridor after a roughly 0.25-0.30 m prefix.
- A* and the live command segment checker may now leave a lethal start only while
  each traversed cost strictly decreases until below the lethal threshold. They
  cannot move laterally across an equal-cost plateau, increase cost toward the
  obstacle, or re-enter lethal space. Same-cell yaw goals remain valid and their
  zero-translation commands bypass the translation segment check.
- The semantic runner uses trapped-start candidates and approves an outward step
  only when the target lies beyond the blocked prefix and the free corridor covers
  the complete step plus margin. In `semantic_trapped_escape_20260723.jsonl`, the
  robot evaluated sixteen exits, provisionally accepted four, rejected the
  highest-scored one on final reassessment, and completed a 0.34 m escape from
  about `(1.310, -0.461)` to `(1.538, -0.638)`. The post-check reported
  `trapped_start: false`, healthy localization, no active goal, zero reverse
  samples, and stop re-latched. The paired frame is
  `semantic_trapped_escape_success_20260723.jpg`.

## Full semantic soak acceptance - 2026-07-27

- Reboot recovery first stopped the restored factory service, re-established the
  independent HWR navigation stack, and restored a usable level-forward camera
  view. No power telemetry was queried. A live trapped-start condition exposed
  two recovery gaps: equal-cost non-obstacle inflation plateaus and controller
  projections shorter than the plateau exit. Recovery now permits only
  non-increasing traversal across those non-obstacle plateaus, requires immediate
  descent from true 255 obstacle cells, and permits a partial command prefix only
  on an already verified A* escape route. The live canary exited the plateau,
  reached free space, and used zero reverse commands.
- The VLM occasionally emitted `safe_to_translate: false` together with
  `sensor_preflight`, a clear corridor, no hazards/person, and explicit no-hazard
  rationale. Deterministic normalization now corrects that false flag only when
  all positive evidence agrees and there is no darkness, glare, occlusion,
  uncertainty, corridor veto language, hazard, or in-corridor person. Raw model
  output remains preserved. Candidate-only localization interruptions now cancel
  that alignment, wait for healthy state, and continue; actual translation still
  stops the mission on localization loss.
- Long-lived SSH reverse tunnels repeatedly reset after roughly ten minutes. A
  process-scoped PC relay (`tools/lm_studio_relay.py`) exposed LM Studio to only
  the robot IP for the run, allowing the soak to execute detached on the robot.
  The relay was verified from the robot before launch and removed after evidence
  collection.
- The final seed-44 run completed 20/20 goals in one invocation over 1581.3 s.
  It evaluated 230 candidates and made 226 semantic assessments: 160 were
  rejected, 66 were provisionally allowed, and 27 underwent final revalidation.
  One transient candidate-alignment freshness lapse was recovered; there were no
  safe refusals, drive failures, or reverse samples (`max_reverse = 0.0`). The
  runner re-latched stop. Final localization was tracking and the robot was not
  trapped. Full table and artifact links are in
  `semantic_full_20goal_20260727.md`.
