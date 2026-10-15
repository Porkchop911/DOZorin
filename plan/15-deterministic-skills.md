# Phase 13 — Deterministic skills (move work out of Strata)

**Objective.** Strata decides *what* to do; deterministic code does *how*. Every loop the LLM
currently runs by hand (act → check pose → act again) becomes one tool call that closes the loop
on the robot.

**Evidence (Hermes `robot` state.db, 100 turns, 2026-10-13..15).**

| | |
|---|---|
| LLM time vs tool time | 5619 s vs 3182 s (64 % LLM) |
| `describe_view` (VLM, also Strata) | 1456 s, 107 calls |
| median turn | 74 s, 6 LLM calls |
| voice "turn right" | 34.5 s, 6 LLM calls (say, drive, reset_estop, drive, describe_view) |
| voice "Turn around" | 73 s, 15 LLM calls: drive ↔ where_am_i ×7, 17k chars reasoning |
| life tick | 5–16 calls: seen_objects → wander (→ relocalize → wander) → drive → describe_view → memory |
| "pick a cube" | 24–30 calls of drive/arm_aim ↔ describe_view |
| top bigrams | seen_objects→wander 30, drive→describe_view 29, wander→describe_view 24, relocalize→wander 12 |
| reset_estop | 9 calls, mostly right after arm_pose |

**Rule.** Closed loops run in the gateway (on the Jetson, next to the sensors), exposed as HTTP
endpoints; `skills.py` wraps each as one tool; `ears.py` may call the same endpoints. No loop that
needs a pose/vision check per step stays in the LLM.

## Steps

- [~] **13.1 `turn(degrees)`** (code + offline tests 2026-10-15; live tuning pending) — gateway `/motion/turn`: rotate in place until map yaw changed by
  `degrees` (±5°), wz ≤ 0.6 rad/s, timeout 3× nominal, stop on estop. Returns start/end pose and
  error. Closed loop on yaw also sidesteps the open "2× turn rate" question.
- [~] **13.2 `move(meters, direction)`** (code + offline tests 2026-10-15; live tuning pending; no `back` — base is no-reverse) — gateway `/motion/move`: forward/back/left/right by
  distance on pose, refuses if scan clearance < distance + margin. Returns pose and achieved distance.
- [ ] **13.3 `wander` self-heals** — on low localization score or plan failure: relocalize once, retry
  (removes relocalize→wander chains). Result includes pose and `seen_objects` at arrival.
- [ ] **13.4 `roam_step()`** — one life tick: seen_objects → wander → one describe_view → returns
  `{pose, objects, new_objects, view}`. Life-loop prompt: call `roam_step` once, then only
  memory/say.
- [ ] **13.5 `approach(target, stop_at_m)`** — visual servo on `/vision/objects` (class or track id,
  "person" for "come to me"): turn to bearing, translate until range ≤ `stop_at_m`, re-acquire on
  loss, timeout. No describe_view per step.
- [ ] **13.6 find what latches the estop** — logs show arm_pose/drive/arm_aim were *refused because
  the estop was already latched* (9×), not latched by them. Trip callers: gateway `/stop`, explore
  `_trip_stop` (every explore end), sensor supervisor (stale lidar/camera), driver invalid command.
  Needs robot logs (driver "estop tripped" times vs gateway audit / supervisor log / explore journal).
- [ ] **13.7 voice reflexes** — `ears.py` maps stop / turn left|right|around / forward / back /
  come here to 13.1/13.2/13.5 endpoints directly (canned spoken ack), then posts
  `[done by reflex] …` to Hermes asynchronously so memory stays consistent. Anything else → Hermes.
- [ ] **13.8 tool descriptions + SOUL.md** — tell Strata to prefer the composite tools; remove
  `drive` from the profile only if 13.1/13.2 cover every logged use.
- [ ] **13.9 `pick(target)`** — deferred: needs grasp calibration first; listed so the 30-call
  pick turns have an owner.

Order: 13.6 → 13.1 → 13.2 → 13.7 → 13.3 → 13.4 → 13.5 → 13.8.

## Live tuning (13.1/13.2)

Params on `/hwr_gateway`, settable with `ros2 param set`: `turn_max_wz` 0.5, `turn_min_wz` 0.15,
`turn_gain` 1.5, `turn_tolerance_deg` 4, `move_max_v` 0.15, `move_min_v` 0.05, `move_gain` 1.0,
`move_tolerance_m` 0.03, `move_clearance_m` 0.15. Yaw/distance come from `/odom`; `/stop` aborts a
running motion on the next 50 ms tick. Checks: `turn 90/180/-90/360` vs tape mark, `move 0.5` each
direction vs tape; if turns undershoot, raise `turn_min_wz` (friction) before touching gain.

## Acceptance (re-run the same state.db analysis)

- voice "turn around": ≤ 2 LLM calls, ≤ 10 s; physically ~180° (user checks).
- voice reflex: speech end → motion start ≤ 1.5 s, no Strata call.
- life tick: median ≤ 3 tool calls, median turn ≤ 25 s.
- reset_estop calls after arm_pose: 0.

Not in scope: turning Strata thinking off (separate decision; would speed up every remaining turn).
