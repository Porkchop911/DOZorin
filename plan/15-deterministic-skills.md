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

## Concept: four layers, each only calls the one below

| Layer | Lives | Latency | Owns | Never does |
|---|---|---|---|---|
| L0 safety | driver/base (exists) | ms | estop latch, no-reverse clamp, watchdog, mux | decide anything |
| L1 motions | gateway `/motion/*`, `/goto`, `/arm/*` | 50 ms loop | turn, move, approach, goto, arm poses; each refuses unsafe input and ends at zero velocity | call an LLM |
| L2 behaviours | `skills.py` (PC) | seconds | multi-step recipes with deterministic recovery: wander (arm-forward + relocalize + re-pick), roam_step (sense → wander → sense, VLM only on novelty) | ask Strata what to do next |
| L2' reflexes | `ears.py` (Jetson) | < 1.5 s | fixed voice grammar → L1 directly, canned ack | anything not in the grammar |
| L3 Strata (Hermes) | PC | 10–60 s | open speech, intent, memory, personality, choosing L2 behaviours | per-step motion, safety, retries |

Routing rules:
- **Voice:** grammar match → reflex (no Strata). Anything else → Hermes, with a note of reflexes
  done since its last turn prepended (no extra Strata call for bookkeeping).
- **Perception:** YOLO `seen_objects` is the default sense (22 ms). The VLM (`describe_view`) is
  called by L2 only on novelty, by L3 only when a person asks what it sees.
- **Recovery is code, not prompting:** camera-not-forward → arm_pose forward; localization lost →
  relocalize; unplannable pick → re-pick. Estop is never auto-cleared by L1/L2 (13.6 finds why it
  latches); L3 may still clear it per the 2026-10-13 decision.
- **Life tick:** one `roam_step` call; Strata only adds memory/say from its result.

## Steps

- [~] **13.1 `turn(degrees)`** (code + offline tests 2026-10-15; live tuning pending) — gateway `/motion/turn`: rotate in place until map yaw changed by
  `degrees` (±5°), wz ≤ 0.6 rad/s, timeout 3× nominal, stop on estop. Returns start/end pose and
  error. Closed loop on yaw also sidesteps the open "2× turn rate" question.
- [~] **13.2 `move(meters, direction)`** (code + offline tests 2026-10-15; live tuning pending; no `back` — base is no-reverse) — gateway `/motion/move`: forward/back/left/right by
  distance on pose, refuses if scan clearance < distance + margin. Returns pose and achieved distance.
- [~] **13.3 `wander` self-heals** (code + tests 2026-10-15) — on low localization score or plan failure: relocalize once, retry
  (removes relocalize→wander chains). Result includes pose and `seen_objects` at arrival.
- [~] **13.4 `roam_step()`** (code + tests 2026-10-15; novelty = class not seen in 30 min, state in `~/.hwr_roam_seen.json`) — one life tick: seen_objects → wander → one describe_view → returns
  `{pose, objects, new_objects, view}`. Life-loop prompt: call `roam_step` once, then only
  memory/say.
- [~] **13.5 `approach(target, stop_at_m)`** (gateway `/motion/approach` + tests 2026-10-15; needs camera forward) — visual servo on `/vision/objects` (class or track id,
  "person" for "come to me"): turn to bearing, translate until range ≤ `stop_at_m`, re-acquire on
  loss, timeout. No describe_view per step.
- [ ] **13.6 find what latches the estop** — logs show arm_pose/drive/arm_aim were *refused because
  the estop was already latched* (9×), not latched by them. Trip callers: gateway `/stop`, explore
  `_trip_stop` (every explore end), sensor supervisor (stale lidar/camera), driver invalid command.
  Needs robot logs (driver "estop tripped" times vs gateway audit / supervisor log / explore journal).
  2026-10-15 deploy: two sources confirmed — the driver latches at every stack start (by design),
  and the sensor supervisor latched every 15 s while the factory camera driver held the Aurora
  (takeover not redone after reboot). Every reboot needs `takeover.sh` before `mode.sh start`.
- [~] **13.7 voice reflexes** (ears.py grammar + self-check 2026-10-15; `back` → spoken refusal; notes prepended to next Hermes message, no extra Strata call) — `ears.py` maps stop / turn left|right|around / forward / back /
  come here to 13.1/13.2/13.5 endpoints directly (canned spoken ack), then posts
  `[done by reflex] …` to Hermes asynchronously so memory stays consistent. Anything else → Hermes.
- [~] **13.8 tool descriptions + SOUL.md** (2026-10-15: SOUL routing rules, "look before moving" rule removed, life_loop prompt = one roam_step) — tell Strata to prefer the composite tools; remove
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
`approach` reuses the turn/move params; checks: `approach person` from 3 m, from behind (search
spin), and with the person stepping out of view (expect `target lost` after 3 s).

Deploy: rebuild `hwr_core` + `hwr_gateway` on the Jetson, restart gateway and `ears.py`, restart
`hermes -p robot gateway` (new tools + SOUL.md).

## Acceptance (re-run the same state.db analysis)

- voice "turn around": ≤ 2 LLM calls, ≤ 10 s; physically ~180° (user checks).
- voice reflex: speech end → motion start ≤ 1.5 s, no Strata call.
- life tick: median ≤ 3 tool calls, median turn ≤ 25 s.
- reset_estop calls after arm_pose: 0.

Not in scope: turning Strata thinking off (separate decision; would speed up every remaining turn).
