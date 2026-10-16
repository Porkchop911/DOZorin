# Phase 14 — Arm mobility + dexterity (follow, pick, place, stack)

**Objective.** The arm becomes a manipulator Strata can use with one call per intent:
`follow(target)`, `find_cubes()`, `pick(color)`, `place(on=color | x,y)`, `stack_cubes(order)`.
Closed loops (servoing, grasp sequencing, grasp verification) run on the robot (L1 gateway);
recipes with recovery run in `skills.py` (L2); Strata only chooses (L3) — same layering as plan/15.

**Benchmarks (user, 2026-10-16).**

| | Setup | Pass |
|---|---|---|
| B1 object following | kit cube (or person/ball) moved by hand 0.5–2 m in front, incl. 1 m sideways and out past ±60° | 3 runs × 60 s: target centred (±10°) in ≥ 80 % of vision frames, lost ≤ 3 s, base turns/drives to keep it, no contact |
| B2 stack 3 cubes | red/green/blue kit cubes on the floor, scattered by the user within reach-or-short-drive in front | one `stack_cubes` call → 3-high stack standing 10 s after the arm retreats; ≥ 3/5 attempts, user-verified photo each |

Intermediate gates: pick 8/10, place-on-cube 8/10 (user counts).

## Findings that change the existing arm code (2026-10-16, URDF + vendor servo config)

- Vendor servo config: **every arm joint is flipped** — URDF angle = (500 − pulse) × 0.24°.
  Pan (joint1 axis −Z) therefore gives yaw = (pulse − 500) × 0.24° (pulse > 500 = left, matches memory).
- Zero pose = arm straight up; joints 2–4 rotate about +Y, links along +Z: 0.08513, 0.085147,
  then 0.05577 (joint5) + 0.08 (end effector) = **tool 0.1358 m**, not 0.08.
- Pan axis is at base_link (0.0466, 0, 0.1604); shoulder at z 0.1907. Old IK put the pan at x = 0.
- Camera sits on link4: 0.069 m along the link, 0.042 m on the "up" side, optical axis = link4
  direction. Forward pose (2=701, 3=120, 4=300) → link4 at 91° from vertical = horizontal ✓;
  floor tilt 88 → 142° = 52° below horizontal ✓. So when the tool points down, the camera does too
  (eye-in-hand).
- Old `ArmKinematics`/`arm_reach` was never physically validated (9.6 open) and is wrong on all
  three counts above; `arm_reach` tool description envelope is wrong too.
- Gripper (id 10) is not read back; readback is the cheapest grasp check (a held cube stalls the
  close short of its target).

## Steps

- [ ] **14.1 Kinematics rewrite** (`hwr_core/arm/kinematics.py`): URDF-convention planar chain
  with the dims above; `fk(pulses)` → tip xyz + pitch, `camera_pose(pulses)` → R, t in base_link,
  `ik(x, y, z, pitch, roll)` → pulses, `ik_any_pitch` (steepest feasible pitch from −90° up),
  `pixel_to_plane(u, v, K, pulses, z)` ray-floor intersection. Tests: FK of forward/floor pose
  match the observed camera directions; FK∘IK round trip; ray of the image centre in forward pose
  is horizontal.
- [ ] **14.2 Plumbing:** `ArmReach.srv` + `roll`; arm_node uses the new IK; driver reads back
  servo 10 as `gripper` joint; `poses.yaml` + `look_down` pose and `grip` gripper level.
- [ ] **14.3 Cube detection** in `vision_node`: LAB colour blobs (red/green/blue, ranges in
  `hwr_gateway/config/cubes.yaml`, seeded live) published in `vision/objects` as class
  `red_cube` etc. with `center` px and frame `K` — so `approach`/`follow` take cubes for free.
- [ ] **14.4 L1 manipulation** (gateway): `GET /arm/cubes` (floor xy per cube from FK + ray),
  `POST /arm/pick {color}` (look_down → detect → open → hover → descend → close → lift →
  verify by gripper readback → carry), `POST /arm/place {on | x,y, level}`, all ending in a known
  pose, refusing on estop, never moving the base.
- [ ] **14.5 L1 follow** (gateway `/arm/follow {target, seconds, distance_m}`): 10 Hz pan/tilt
  servo on the target's pixel error; base wz ∝ arm pan; vx ∝ range error only when pan < 15°
  and the costmap is clear (hard cost ≥ 79); hold on loss, end "target lost" after 3 s; returns
  arm to forward.
- [ ] **14.6 L2 skills + tools**: `find_cubes`, `pick`, `place`, `follow`, `stack_cubes(order)`
  (pick → place-on-top, re-detect between steps; if a cube is out of reach: turn/move toward it
  once, re-detect), fixed `arm_reach` description. Tools in `config/tools.json`; SOUL routing:
  "manipulation = these tools, never arm_reach loops".
- [ ] **14.7 Live calibration** (user present): (a) forward/floor/look_down camera directions vs
  FK; (b) reach to 3 tape marks ≤ 1 cm (gravity trims per joint if not); (c) LAB ranges for the
  3 cubes in room light; (d) gripper `grip` level + readback threshold.
- [~] **14.8 Benchmarks B1, B2** (B2 series 1: 1/5, not accepted; B1 not run) with rates and photos in `evidence/phase14/`.

## Safety

Arm commands refused while estop latched (existing). Base never moves inside pick/place. Follow
translates only forward (no-reverse envelope) and stops at hard costmap cells. Every L1 call ends
at zero velocity and a named arm pose.

## Session log

### 2026-10-16 (live, user present)

| Item | Result |
|---|---|
| look_down / find_cubes | cubes found 3/3 after retune; detector = floor-normalised LAB + convex-hull filter (auto white balance moved red b by 54 between two lamps) |
| FK vs depth at 0.6 m | FK ray 0.68 m vs depth 0.57-0.60 m (~10 % long); close range consistent to ~1 cm |
| grip check | `close 500` holds a cube at readback 486 = indistinguishable from empty -> `grip: 600` (empty 598, held ~490) |
| wrist roll | sign was wrong (`roll_sign: -1`); yaw from min-area box was garbage on oblique cubes -> outline edges + overhead re-locate at 8 cm hover |
| place | red on blue placed (level 1); green on 2-stack released too high -> `place_clearance: -0.005` |
| pick | 4 holding / 7 tries (red 1st try was a good grasp dropped by the bad grip check) |
| B2 attempt 1 | FAIL - software (move right after arm_pose, 409) |
| B2 attempt 2 | FAIL - green on red OK, blue missed at x 0.136 (against the bumper) |
| B2 attempt 3 | **PASS** (user-verified 3-high standing) - one `stack_cubes` call, 41 s, 2 picks + 2 places all first-try, 1 reposition (turn + capped creep); `b2_3_before/after.jpg` |
| B2 attempt 4 | FAIL - tool said "stacked", all 3 on the floor: places at -60/-55 deg (edge of reach) fell. Fix: place only at <= -70 deg + after-release check from look_down |
| B2 attempt 5 | FAIL - green pick at -40 deg missed (overhead re-locate saw nothing at that angle). Fix: pick only at <= -55 deg, else reposition |
| **B2 series 1** | **1/5 (#3)** - not accepted (needs >= 3/5). Every failure was a different bug, each fixed; rerun a fresh 5 |
| S2 #1 | FAIL - placed "not on target" (check worked): picked -55, placed -75 -> cube 20 deg tilted, slid off (user). Fix: place within 15 deg of the pick pitch |
| S2 #2 | FAIL - exact pick pitch (-90) unreachable for the place. Fix: +/-15 deg window |
| S2 #3 | FAIL - green pick missed; green 4.5 cm from red (layout left by #2's drop), jaw hit red |
| Setup rule (user) | cubes >= 8 cm apart. Jaws close left-right at roll 0 (user) - input for a later neighbour-clearance roll choice |
| incident | into-reach creep (drive, costmap bypassed because it inflates the cubes) **drove into the nearer cubes**. Fix: creep capped so every cube in view stays >= 0.19 m ahead, else refuse. Bumper ~0.125 m. |

Open (user, 2026-10-16): **reverse / backing up** — "we need to talk about this in the future".
Base is structurally no-reverse (rear-blind); a cube too close to grasp currently needs a human.
