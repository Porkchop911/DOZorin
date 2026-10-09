# Chair collision incident — 2026-07-28

## Containment

- The attended run was stopped immediately after the human reported contact
  with the chair.
- The emergency stop was latched and drive was inactive after containment.
- No further motion was authorized or sent during investigation.

## Preserved evidence

- `gateway_audit.jsonl` — SHA-256
  `4AD5F81BB9103F3DC8D2C2BC1C3BF091784AA051C55A791481349B27158F6716`
- `mode_session.log` — SHA-256
  `9FBF819787375AD8494330281A034A81E96222942AF3DAB117170ADEB00D4DF7`
- `post_collision_camera.jpg` — SHA-256
  `90360F866C8FC6A23B6ACA9711DB93D84AEC0BD644B17B27671AA9A2A54B9FBE`

The post-collision frame shows a chair base/pole and a person's leg/foot in
the immediate foreground and swept area.

## Timeline from the gateway audit

- 13:21:02.889 +02:00: a translation goal was accepted from approximately
  `(-0.416, -0.356)` toward `(-0.125, -0.375)`.
- 13:21:24.835 +02:00: that goal failed with `localization health lost` after
  21.945 seconds.
- The staged executor then selected another candidate instead of terminating.
- The following translation failed with `no collision-free path`.
- At 13:22:33 +02:00, the explicit incident stop was accepted and latched.

## Contributing software failures

1. The vision prompt inspected only a centered robot-width corridor and
   explicitly excluded nearby objects judged outside it. That does not cover
   chassis, arm, yaw, or lateral-correction sweep.
2. Semantic normalization could change a model `hold`/unsafe response into an
   allowed response from positive text fragments.
3. Any visible person did not veto motion when the model labelled them outside
   the centered corridor.
4. The live collision check allowed a partial escape from an inflated trapped
   cell. This exception cannot distinguish a stale map inflation artifact from
   a newly detected chair/person obstacle.
5. Translation could pause and resume through intermittent localization loss,
   and a short staged leg had no suitably short hard runtime bound.
6. A failed physical translation could be followed automatically by another
   candidate attempt.
7. Rotation-only goals bypassed the current-cell footprint collision check.

## Local corrective patch

- Any visible person now vetoes autonomous motion preflight.
- Every reported nearby hazard is retained; out-of-corridor hazard
  reclassification is disabled.
- Unsafe/hold model decisions are never promoted to safe by consistency code.
- The visual prompt now covers the full conservative swept envelope and names
  chairs explicitly.
- Translation requires both fresh lidar and fresh depth-range streams.
- Runtime command collision checks no longer allow trapped-start or partial
  escape exceptions.
- Rotation requires a fresh 360-degree lidar sample and a free current
  footprint cell.
- Localization loss aborts translation immediately; translation stalls abort
  without recovery motion.
- Live-obstacle replanning defaults to zero attempts.
- Object-approach stages default to 0.10 m and are limited to 0.05–0.15 m.
- A failed physical leg terminates the run and cannot choose another candidate.
- A stationary semantic/metric preflight is required before candidate
  rotation.

These changes are local and must pass stationary tests and review before a new
deployment or attended motion trial.

## Stationary regression evidence

- Focused navigation/semantic/object-approach tests: 59 passed.
- Complete core suite after the corrective patch: 247 passed.
- Python syntax compilation passed for the modified core, navigation node, and
  motion tools.
- The saved collision frame was re-evaluated with the revised full-envelope
  prompt using the local `qwen/qwen3-vl-4b` vision model. It returned
  `safe_to_translate: false`, `person_visible: true`, `recommendation: hold`,
  and hazards including the foot/person, cables, chair, and furniture legs.

## Post-patch deployment findings

- A network interruption was a robot reboot. The factory service auto-started
  on the new boot while the stale takeover marker referenced the prior boot.
  The HWR graph was stopped, takeover was re-established, and the factory
  service was verified inactive before restarting HWR.
- The reboot enumerated the stable `/dev/rrc` controller link as `ttyACM1`.
  `deploy/takeover.sh` incorrectly checked fixed `/dev/ttyACM0`; it now checks
  `/dev/rrc`.
- The first sensor-supervisor implementation reset `0bda:5489` and
  `0bda:0489`. Live `lsusb` proved these are shared Realtek hubs, so the resets
  disconnected the base serial controller and lidar and made the stop service
  unavailable. Camera recovery is now restricted to the actual Aurora 930
  device `3251:1930`, with a regression test forbidding known hub IDs.
- Localization-unhealthy and stale-depth translation refusals both terminated
  with zero commands in the initial trace. Those traces are not retained as
  acceptance proof because the trace subscriber lacked a discovery warmup.
- A synthetic blocked-footprint rotation probe was invalid: the injected scan
  and trace subscriber were still in DDS discovery when the goal was sent. The
  goal succeeded, and odometry/dead-reckoning changed by approximately
  0.17/0.29 rad before stop was re-latched. No further motion probe was sent.
  The trace tool now has an explicit discovery warmup and rejects captures with
  no applied-velocity or wheel samples. Costmap diagnostics now expose and can
  require the current-pose cell cost before a future refusal probe.

## Corrected stationary and canary acceptance

- Corrected warmed traces proved blocked-footprint rotation,
  localization-unhealthy translation, and stale-depth translation all
  terminate with zero applied velocity and zero wheel commands. The saved
  collision frame remains a semantic hard stop for its person, chair, cables,
  and furniture legs.
- Live depth diagnostics traced the intermittent 0.33 m marks to inferred
  invalid-depth holes at image columns 20--28 (the outer 5% of a 640 px
  frame), not to any measured 3-D obstacle point. The hole heuristic now
  excludes only that unreliable side band; direct measured obstacle points
  retain the full field of view. After deployment, 60/60 stationary frames
  contained no false inferred obstacle, while the forward costmap remained
  clear beyond 1.5 m.
- An estop-latched 0.05 m goal exposed another false-success path: nav/base
  published nonzero requested commands, `cmd_vel_applied` and odometry
  integrated them despite the driver refusing motor actuation, and localization
  drifted enough to report `goal reached`. Evidence before the fix is
  `pre_fix_false_motion_trace.txt` (SHA-256
  `692A16B5E8574EA33A024A263F831CD0BA9E777B3C35C390C1B54CBFB641E117`).
- Base output and model odometry are now hard-zero whenever the driver's
  transient-local estop state is latched. Navigation rejects a goal while
  latched and aborts immediately if estop trips in execution or recovery.
  `estop_gate_acceptance.txt` records 160 applied and 161 wheel samples, all
  zero, plus immediate goal rejection (SHA-256
  `7DC2495E2F011ED1EA51E539DB0BFEF1323C3072E03EB2A0F55100BAE03F693F`).
- After stationary global relocalization (best seed 0.802, tracked score 0.83),
  fresh semantic and metric preflight approved one 0.05 m forward canary. It
  reached the goal, advanced fused odometry by about 0.049 m, and re-latched
  estop. `enabled_canary_trace.txt` records 13 nonzero nav samples and 36
  nonzero applied/wheel samples (SHA-256
  `93D000D0294D9077CD24F7B799728F19E10E40284C6E6FC9394C4BBDE4D9C7BA`).
- A first odometry-confirmation threshold was too permissive: a nominal 0.05 m
  goal advanced fused odometry only about 0.017 m but still passed its 0.01 m
  minimum. That run is preserved as
  `permissive_odom_confirmation_trace.txt` (SHA-256
  `26E27012FC9C5F782839055A729A062BD498224899A627DB1FDD44FFC4689DF7`)
  and is not acceptance evidence.
- Goal completion now requires independent fused scan/IMU odometry equal to
  requested translation minus the controller's terminal tolerance (0.03 m for
  a 0.05 m goal), and requested yaw minus yaw tolerance. The strict live canary
  advanced fused odometry about 0.055 m, captured 14 nonzero nav and 43
  nonzero applied/wheel samples, succeeded, and re-latched estop. Evidence:
  `strict_odom_confirmation_trace.txt` (SHA-256
  `79C09E02307D6F134F4547F9708E70A1A721C5CB41DAA1D32ABFE42717A6F40B`).
- Field data from a 0.15 m leg measured 0.129 m fused displacement at the
  0.13 m confirmation boundary. A bounded 0.005 m odometry-noise allowance now
  accepts that within-tolerance result while the earlier 0.017 m false-success
  sample remains rejected.
- The event-triggered dynamic-obstacle test waited for actual nonzero applied
  velocity, injected a 0.25 m depth obstacle 0.25 seconds later, and terminated
  with `blocked after live-costmap replans`. The trace proves exactly one motion
  burst (1.65--2.30 s), 14 nonzero applied/wheel samples, and no retry. The
  synthetic publisher exited and `/depth_ranges` returned to one production
  publisher. Evidence SHA-256 values: `dynamic_triggered_trace.txt`
  `E6AFA13D78E6D6FB6D6B3CACEC648BFFD5B6DE0354671D7FD65EB283E4F34C4A` and
  `dynamic_triggered_publisher.txt`
  `758988762A6EBD14409BEAF20017768EB4D61F9A84C3C0223F3B5B7B4D78E92C`.
- Complete core suite after these corrections: 256 passed.

## Corrected camera geometry and semantic refusal

The first physical obstacle trial's image was aimed at the floor, so its visual
clear-corridor judgment was invalid. The arm was moved to the live-verified
`forward` pose while base command/applied/wheel telemetry stayed zero, then stop
was re-latched. In the corrected frame the chair is directly ahead at roughly
0.8 m (`camera_forward_gate_live.jpg`). A stationary 0.15 m semantic preflight
with a 0.20 m metric margin
reported the chair, furniture legs, and thin poles and returned `allowed=false`.
The metric ray alone reported 0.65 m and would have allowed the short leg, so
this specifically proves that the corrected visual hazard veto is active.

The gateway now publishes `camera_forward` from fresh joint-state readback and
rejects translation goals/exploration unless the measured arm pose matches the
verified forward pose. Metric semantic preflight independently requires the
same flag. The focused test module passes 31 tests and the full core suite
passes 258 tests.

## First resumed staged leg — 2026-07-29

After stationary global relocalization and a fresh semantic/metric preflight,
the operator authorized one bounded window-approach stage. The staged runner
completed two current-cell alignment goals and one translation goal, then its
configured one-stage limit stopped the run before another leg. Fused odometry
advanced about 0.068 m. `staged_window_leg_retry_trace_20260729.txt` recorded
three applied-motion bursts corresponding to those three accepted goals
(SHA-256 `4AD02A2DAB504D1262B9C285F17BEB0A4BF6896306D0B652016224361E84A58B`);
the gateway audit records a successful translation result followed by two
accepted stop calls. `staged_window_leg_after_20260729.jpg` is the final forward
frame. The window remains distant, so this is staged-execution evidence only
and not an object-arrival claim.
