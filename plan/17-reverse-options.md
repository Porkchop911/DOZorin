# 17 — Reverse / backing up: options (decision doc)

Status: **open, for discussion.** Not a plan to implement. Nothing here is approved.
Source of the open item: `plan/16-arm-dexterity.md` (Open, user 2026-10-16: "we need to talk about this in the future").

## 1. Why reverse is disabled today

| Fact | Source |
|---|---|
| Lidar mounted rpy (0,0,π), i.e. 180° backwards | `plan/90-clean-room-reference.md` l.58 |
| Vendor band-pass keeps 1.25–5.08 rad (≈219° usable); rear ~140° blocked by chassis | `plan/90-clean-room-reference.md` l.58 |
| Rear is blind, so "no reverse ever" is a project convention | `plan/91-conventions.md` rule 3 |
| Same rule baked into the controller as a structural clamp | `plan/09-phase8-navigation.md` 8.3 (`no-reverse clamp`) |
| `/drive` rejects vx outside 0–0.2 (422 "bounded no-reverse envelope") | `ros2_ws/src/hwr_gateway/hwr_gateway/gateway_node.py` ~l.671 |
| `/motion/move` has no "back" direction; base clamps vx ≥ 0 | `gateway_node.py` l.153 |
| Follow mode translates only forward, stops at hard costmap cells | `plan/16-arm-dexterity.md` Safety |
| Voice "back" / "reverse" → spoken refusal, no motion | `tools/ears.py` l.94–95 |
| Master plan lists rear-blind as a risk: "no reverse ever" rule | `plan/00-masterplan.md` l.85 |
| Acceptance: zero reverse commands in the cmd log | `plan/09-phase8-navigation.md` Test |
| Nav config (`ros2_ws/src/hwr_nav/config`): no reverse setting found | grep, unverified that no other param exists |

## 2. Where it hurts

| Situation | Evidence | Today's result |
|---|---|---|
| Cube closer than grasp reach (<0.17 m; bumper ≈0.125 m) | `plan/16` cycles: misses "x < 0.17" | Needs a human to back off |
| Cubes boxed in, base position chosen badly | `plan/16` cycles c1–c8 | Needs human reposition |
| Approach overshoot into cubes | `plan/16` incident: creep drove into nearer cubes; fix caps creep at ≥0.19 m | Refused, not recovered |
| Voice "go back" | `tools/ears.py` | Refused by design |
| Cornered by obstacle with no forward/side exit | Not in repo | Unverified: no recovery behaviour found for this case |

## 3. Options

| # | Option | What it needs | Risk | Effort |
|---|---|---|---|---|
| a | Keep no-reverse. Escapes via strafe (mecanum) and rotate-in-place | Nothing new. Confirm strafe clearance with the lidar sector (unverified) | Strafe may not clear a cube boxed in front; still needs human in worst case | S |
| b | Short blind reverse, capped at the distance just driven forward, slow | Odometry of own forward path; a cap (user's example ≤0.15 m); a rule that vx < 0 only while retracing | Rear is blind: objects behind the robot are not seen; lidar cannot confirm the path | M |
| c | Reverse only into cells seen free in the map/costmap recently | Map freshness check; costmap query for the rear cells (cells behind the robot are the blind sector, so freshness must come from past scans) | Map may be stale; moving objects; the rear sector was never scanned while driving forward | M |
| d | Rotate 180° first, then drive forward | Existing turn behaviour (`/motion/turn`); a clearance check after rotation | Turning costs time; rotation sweeps the cube/bumper area; does not help if the target is behind the base in a tight spot | S |
| e | Add a rear range sensor (ToF or ultrasonic) and gate reverse on it | Hardware on the rear; wiring; a driver; a clamp gated on the reading | Hardware change; sensor specs and cost unverified; mount location untested | L |

Notes:
- (a) and (d) need no new code paths for reverse. (b) and (c) are the only ones that create negative-vx motion, so they touch the structural clamp in `plan/09` and the `/drive` envelope.
- Any option that allows negative vx must change `plan/91` rule 3 and the acceptance check "zero reverse commands" in `plan/09`.

## 4. Open questions for the user

1. Is any reverse acceptable at all, or is "no reverse ever" a hard rule to keep?
2. If yes, what max reverse distance and speed? (Option b example: ≤0.15 m, slow. Unverified as a safe value.)
3. Is adding a rear sensor (option e) on the table, given hardware cost and wiring?
4. For cubes too close to grasp: is a human re-positioning acceptable for now, or must the robot back off on its own?
5. Should voice "go back" stay a refusal, or map to a capped reverse once (b) or (c) exists?
