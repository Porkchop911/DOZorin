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
- [~] **14.8 Benchmarks B1, B2** (B2: series 1 1/5, series 2 0/5, series 3 1/2 open; B1 not run) with rates and photos in `evidence/phase14/`.

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
| S2 #4 | FAIL - place out of reach; creep refused because the HELD cube looked like an obstacle. Fix: creep ignores the held colour. (Also: sensor supervisor latched estop on a lidar dropout mid-session; recovered, reset) |
| S2 #5 | FAIL - creep worked; pick -55, place -70: green hit red's edge and tumbled (user). Fix: picks <= -70 (else creep), place within +/-10 deg of pick |
| **B2 series 2** | **0/5** - total 1/10 |
| User 2026-10-16 | any stacking order; robot may reposition blocks and its body itself |
| S3 #1 | FAIL - green at 0.275 judged "close" (+0.04 slack from pre-steep days), no creep, same pick retried 8x. Fix: creep to 0.22 (+0.02), corridor +/-0.13 m, stop on repeated failure |
| S3 #2 | **PASS** (user-verified) - auto order green/red/blue, 44 s, both places verified by the after-release check |
| Self-reset (unstack) | `pick(level=n)` added; blue floor spot unreachable at its pick pitch -> place turned the body and gave up holding blue; ad-hoc script then picked red (gripper open) and dropped blue out of view. TODO: floor placement without the pitch window; unstack as a skill that stops on a failed place |
| Session end | battery low (user). B2 overall: 1/5, 0/5, S3 1/2 -> **not accepted**. B1 follow not run. Arm parked forward, gripper open |
| Setup rule (user) | cubes >= 8 cm apart. Jaws close left-right at roll 0 (user) - input for a later neighbour-clearance roll choice |
| incident | into-reach creep (drive, costmap bypassed because it inflates the cubes) **drove into the nearer cubes**. Fix: creep capped so every cube in view stays >= 0.19 m ahead, else refuse. Bumper ~0.125 m. |

| 2026-10-17 (offline) | `unstack_cubes(stack)` skill + tool: top-down to the floor 0.10 m beside the tower, stops at first failure, no body moves. Floor place: no pitch window, flattest -45 deg. Untested live |

### 2026-10-17 (live, sunlight, user present part-time)

| Item | Result |
|---|---|
| detector | sunlight washed colours out -> wider LAB ranges with L caps, jaw band (bottom 65 px ignored: camera rides on the gripper), shaded-face suppression |
| map pose | degraded localization froze map x/y over ~1 m driving -> stack tracked in odometry |
| moves | into-reach strafes away from too-close cubes and sidesteps around a blocker; stack_cubes searches (floor view, 60 deg turns) |
| S4 #1-#5 | #3 **PASS** (user-verified). #2 tool lied "stacked" + opened gripper mid-air on a failed find (user: "dropping cubes for no good reason") -> fixed: never open mid-air, final tower check. #5 started from a standing tower (my error) |
| centring analysis (user: "mostly about centering") | (1) place re-located the target at hover where the HELD cube hides it -> 3.5 cm wrong spot; now look_down spot. (2) side bias -0.73 cm on 6 places -> place_side_trim. (3) floor calibration 14 places: radial overshoot 0.76 mm/deg tilt from vertical -> place_radial_per_deg. (4) pick error along the fingers rides into the place -> 90 deg re-grip (turn at hover: turning down there dragged the cube) |
| stack picks | finger tips caught the cube below -> grip stacked cubes 8 mm high |
| final check | per-level planes (floor plane read a standing tower as apart) |
| cycles c1-c8 | verified place errors after trims mostly <= 1 cm; remaining failures: misses near the bumper (x < 0.17), cubes boxed in (base choice), unstack floor spots too close |
| last change (untested) | base = farthest cube <= 0.4 m; unstack spots x >= 0.19 |
| end | battery 9.5 V (user: needs charging). /stop latched estop; arm left in last pose |

### 2026-10-17 (afternoon, live, user present)

| Item | Result |
|---|---|
| search | cubes 0.6-0.8 m out: look_down saw none, the search turned before ever using the floor view, and the floor pose (servo4 88) cut them at the image top. Fix: floor view first, floor pose servo4 130 -> all 3 at 0.63-0.78 m |
| S5 #1 | FAIL (sensor estop): camera cable trapped under a wheel -> lidar stale x3 + camera disconnect mid-carry; supervisor released its estop after each lidar respawn, but vision_node hung after the camera reconnect (hand-restarted). Finished by hand: blue on red (-5/+5 mm), green on blue (-6/0 mm), user-verified standing |
| unstack | `unstack_cubes` from a tower at 0.155 m: 2 picks + 2 floor places first try, verified <= 9 mm |
| S5 #2 | FAIL: red 0.17 m out beside the front wheel; the jaw hit the robot's own wheel (user). Fix: gateway reports floor cubes < 0.19 m from the arm base "out of reach" -> skill strafes away first |
| S5 #3 | FAIL: red on blue OK; then green placed "not on target"/"out of reach" 7x over 368 s. Cause (user): the body turned while the arm was in look_down holding green at stack height -> swept the 2-stack over. Fix: `_into_reach` lifts the arm to forward before every turn/creep |
| S5 #4 | FAIL: green held; strafe + 36 deg turn toward the blue base lost it ("target not seen"), then set down / re-pick / lost 3x for 90 s; camera disconnected twice -> lidar stale -> estop. Fix: stop when a place fails the same way twice |
| S5 #5 | FAIL (USB hub 1-2.1 reset, error -71: camera + lidar drop together, 15 resets since boot incl. idle) - hardware. Same loop as #4 before it: turn undershot, the base seen 6 cm left of the odometry estimate was treated as a blocker -> strafed away from it -> "target not seen" -> set down in front -> re-picked (nearest cube). Fix: `_into_reach(target=)` re-spots on the live detection and never counts its own target as a blocker |
| hub | user re-seated; 3 min arm cycling + S6 #1: 0 hub resets |
| S6 #1 | FAIL: green on blue placed 9 mm off (passed the 1.5 cm check); red placed centred on green (user) -> tower toppled. Fix: a place > 6 mm off that will carry another cube is re-picked from the stack and re-placed once |
| S6 #2 | FAIL: blue on red placed 9 mm off again; the redo re-pick's jaws knocked blue off red (user) -> redo reverted. Level-1 side errors today -8.3/-8.6/-9.1 mm vs floor +3.6..+5.7 mm: one side trim fits neither -> per-level calibration |
| calibration | not started: evening light lost red (a 154-157 < 160 -> min 150) and made a phantom blue from floor glare (b 88-98 -> blue max b 86); cycle-1 pick crept blue under the jaws (short strafe 0.01/0.04 m, side-reach creep with the target still ahead) -> fixed `31b897b`. `/tmp/cal_cycle.sh` on the robot (log `state/side_cal.jsonl`) - now `tools/side_cal_cycle.zsh` |
| end | user: shut down. Arm forward, gripper open, mode stopped |

| 2026-10-17 night | boot path: `hwr.service` (new unit) started by hand -> takeover + navigation stack up in 12 s, gateway OK; not enabled, cold boot not tried, ears unit left uninstalled (mic). `HWR_MAP_YAML` -> room_20261015 |
| look_around | `dc7b5b9`: arm pans the camera 0/+50/-50 deg (gateway `find_cubes pan_deg`), body still. Live 9.4 s, blue only visible at -50; same cube across pans within 2-3 cm. Stack search sweeps the arm, then turns 120. Wall corner at the image top read blue (5000 px) -> blobs touching the top dropped (`evidence/phase14/wall_blue1.jpg`) |
| S6 #3 | FAIL 0 stacked: blue picked twice at 0.238/-75, jaws short, pushed it. Then a detour side-reach crept blue to 0.165 m, inside the 0.19 m wheel guard -> creep now stops on the reach circle (`56e1e6e`) |
| pick cal | floor picks: -75/-80 at 0.228-0.238 m missed 3/3 (short, cube shoved) -> floor picks within 5 deg of vertical, creep to 0.205 m (`ca454b8`). Steep picks 4/6 held. User at hover: roll correct (parallel), jaws ~short. Servo readback: shoulder 4-14 pulses short of command (FK ~9 mm short); a readback trim barely moved it (deadband) and the next place overshot 10.5 mm -> reverted. Refine (overhead) vs look_down: radial -5..-28 mm always; the 2 steep misses had a sideways shift of -6/-7 mm, the 4 holds +2..+13 (n small) |

| pick loop 2 | 8/8 steep picks held (7x -90, 1x -85, 0.200-0.213 m, y -0.037..+0.037), 8/8 floor places verified (max 7.3 mm). Refine sideways shift -3..+8 mm, no link to misses -> earlier lead not confirmed. USB hub 1-2.1 reset once more (8x error -71; camera on port 2 is the same hub); vision recovered by itself. Run ended: battery low (user) |

Open: offline 2026-10-18, untested live: place_side_trim_on 0.016 for places on a cube, stack_cubes sweeps the arm for a lost tower (top cube, own level) before the 2nd place try. Steep picks 12/14 - rerun stack series 6; USB hub 1-2.1 still resets (move camera off that hub); per-level place side calibration (level-1 places land ~9 mm right, floor ~+5 mm), then series 6 (0/1 counted); stack re-locate when the tower is not seen; vision_node hangs after a camera reconnect; USB hub 1-2.1 resets (camera moved to hub port 2 - watch dmesg); re-run cycles with re-grip + farthest base; vision_node does not recover from a camera reconnect; supervisor does not release its estop after a lidar respawn (manual reset twice); B2 still not accepted.

Open (user, 2026-10-16): **reverse / backing up** — "we need to talk about this in the future".
Base is structurally no-reverse (rear-blind); a cube too close to grasp currently needs a human.
