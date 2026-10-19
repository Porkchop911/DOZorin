# 19 — Sockmark: socks into the toy truck (2026-10-18)

User: "Find the socks in the room put them in the metal truck near the drum chair." Full loop.

## Facts (user + live probe 2026-10-18)

| Item | Fact |
|---|---|
| Socks | single, flat on the floor; dark grey/black, one grey on the rug |
| Truck | small metal toy truck, open bed, rim < 10 cm; sits under the drum throne between its legs |
| Detection | YOLO: no sock class (a sock read "tie" 0.41). VLM `qwen/qwen3-vl-4b` grounding (bbox 0-1000) found the sock and the truck, ~1 s warm (272 s cold load) |
| VLM false positive | the gripper fingertips at the image bottom edge read "sock" -> drop boxes touching the bottom edge |

## Build

Pure logic in `hwr_core` with tests; gateway does geometry and arm; the agent (PC) asks the VLM.

1. **Grounding parse** — `hwr_core.navigation.vision_geometry.parse_grounding(text, width, height)`:
   VLM JSON (with or without ``` fences) -> pixel boxes + centre; drops boxes touching the bottom edge.
2. **`POST /vision/locate {pixels, plane_z}`** — pixel -> floor point through the live camera pose
   (`pixel_to_plane`), returned in base and map frames. Same joint conversion as `/places/anchor`.
3. **`POST /arm/pick_at {x, y, z}`** — soft object at a base-frame point: steep grasp (within 5 deg of
   vertical, like floor cube picks), any yaw (cloth gathers between the jaws), grip, lift. Reports the
   gripper readback; the skill judges the pick by looking (sock gone from the spot).
4. **`/arm/place` gets `z`** — release height for the x,y case (truck bed: rim + margin).
5. **Agent skill `sockmark(max_socks=6)`**, loop:
   - search: arm sweep (floor + forward views, pans 0/+50/-50), VLM for socks, locate -> map points;
     none -> turn body 120 deg, up to 3 times; still none -> done;
   - nearest sock: nav to a standoff 0.25 m short facing it; re-find from the floor view; creep so the
     sock is at reach (0.20 m, as cube picks); `pick_at`; look again: sock still there -> retry once;
   - truck: VLM finds it (located at its bed height), standoff + re-find + creep, `place` at
     z = 0.14 m, open;
   - report socks moved / missed with denominators.

## Risks

- The truck is between the throne legs: the costmap may refuse the standoff, and the arm must reach
  over the truck without touching the legs. Fallback: ask the user to move the truck clear.
- Cloth picks are new: the jaws may push the sock instead of gathering it. Test 3 picks before looping.
- The held sock hangs in front of the camera in the forward pose; nav semantic checks may refuse.

## Acceptance

Sock picks: >= 3/5 held. Full run: all socks in view of the room centre end up in the truck, with
user-confirmed counts.

## Live 2026-10-18 (first run, interrupted by a reboot)

| Step | Result |
|---|---|
| Find socks (VLM point_2d, ~1 s warm) | works; box centres missed diagonal socks -> `point_2d` |
| Approach | -40 deg view is blind < 0.2 m: overshot onto a sock. Two-stage creep + steep (-60) re-find + `back()` |
| Pick | 1 held / 3. z 0.01 closed above; `grip` 600 pinched then slipped; held with `squeeze` 680 at -75 deg |
| Jaw travel (empty readback) | 550->551, 600->596, 650->643, 700->664: jaws shut ~660 (not 598) |
| Arm IK at z = 0 | pulse rounding dips below the floor -> refused; grasp z clamped >= 3 mm |
| Carry | user: the low forward arm dragged the sock ("asking for trouble"). `carry` = raised {2: 500, 3: 500, 4: 85}; translation allows it |
| Truck while holding | the held sock covers the bottom-centre of the image, where the truck is. Panned further: found, landmarked `toy_truck` (`/places/anchor {cls, point}`) |
| Truck distance | locate said 0.65 m from centre, user saw ~0.35-0.5: camera model is off in the carry pose (calibrate). Costmap blocked at 0.43 m (throne legs); last 0.1 m crept via `/drive` |
| Localization | heading flips between two solutions (~-18 vs ~-143 deg) on relocalize: map drops not trusted |

Next: re-pick, carry, and drop with the truck 10-15 cm off the front edge (user estimate) -> place at
x ~0.27 m, z 0.14. Fix the bottom-strip filter (drop only hits under the gripper, not the whole strip).

### First sock delivered (2026-10-18, user-confirmed "in the bed")

Last metre by hand, then built into `_deliver`:
- carry pose + map drive stopped at 0.82 m (costmap inflation between throne legs and rack); mid-move
  "blocked" came back as an outcome, so the creep fallback never ran (fixed);
- held sock hides the truck; tilting the wrist barely helps (at its limit); panning the camera 34 deg
  showed it; carry-pose locate read ~0.18 m long against the user's estimate;
- first drop at 0.28 m landed on the rug in front: the bed centre was 0.39 m (top view), beyond the
  0.305 m drop reach at -45 deg;
- worked: crept 9.5 cm, re-picked the sock from the rug (held, readback 606), dropped at (0.32, 0.02)
  from z 0.12 at -50 deg -> in the bed. Drops now allow -30 deg (reach 0.32 m at z 0.12).
- Picks this session: 5 held / 7 (2 early misses before `squeeze` and z = 0).

### Shoulder went limp (2026-10-18)

After a held pick the shoulder servo (2) lost torque: readback 322 vs `carry` 500, the robot turned
-142 deg with the arm low and stood on a sock; the carry gate then refused the move. User found the arm
limp and lifted it; torque came back by itself. Servo 2 at 51 C (others 38-48), 11.1 V: likely stall
protection from the jaws pressing the floor at z ~3 mm. Fixes: grasp z >= 8 mm; `GET /arm/state?pose=`
and `_lift_to("carry")` must confirm the readback (<= 30 pulses) before any turn or drive (user: "lift
into driving height BEFORE turning").

### Root cause of the limp arm (2026-10-18, user photo)

The robot stood with a front wheel on one end of the sock it had grasped: lifting fought the robot's
weight, the arm stalled and the servos gave up (shoulder 296, elbow 131, wrist 304 vs carry). The
lift check (`_lift_to`) caught it this time and nothing drove. Guards: `pick_at` checks the arm reached
hover after the grasp, else opens at once and returns `pinned`; `_approach` backs off 0.10 m when any
part of the target is within 0.17 m (under the bumper) before picking.
Picks this session: 9 held / 11 attempts. Deliveries: 1 (hand-guided last metre).

## Status at end of 2026-10-18 and next session

| Item | State |
|---|---|
| Socks in the truck | 1 (last metre hand-guided); 0 by the loop end to end |
| Sock picks | 9 held / 11 (all holds since `squeeze` 680 + z >= 8 mm) |
| Truck sightings (4B) | real ones locate to ~7-19 cm; but it invents trucks (sock pile, dark objects at the image edge) ~1 in 4 searches. A crop yes/no check rejected 2/3 real trucks: not usable |
| Localization | relocalize sometimes flips/shifts (heading -18 vs -143 deg; 0.9 m jumps after restarts): map points from before a restart are not trusted |
| Truck place | deleted (stale frame); truck now in front of the window, open floor |

Next, in order:
1. **Truck sighting you can trust.** Options: user confirms the photo once per session (works today);
   or require 2 sightings from different views within 0.15 m before anchoring.
2. **Relocalize once at start, never mid-run;** re-find the truck after every stack restart.
3. **One end-to-end delivery by the loop** with the truck on open floor: carry -> drive near (0.36 m)
   -> set down beside (0.23, 0.15) -> steep look -> creep (<= budget) -> re-pick -> drop (z 0.12).
4. Then the acceptance run: all socks in view of the room centre into the truck, counted by the user.
5. Calibrate the carry-pose camera (read ~0.18 m long) if the drop needs it.

## 2026-10-19: live patching did not converge -> redesign (user decision)

Session: 0 socks delivered by the loop; picks 15 held / 17; ~30 commits. The single steps work (find,
pick, carry, set down, see the truck, drop). What failed every time was chaining them at close range
around the truck:

| Failure | Cause |
|---|---|
| Robot drove into the truck | blind `/drive` creeps toward a truck estimate off by decimetres; the dark truck gives no depth |
| Re-pick impossible | set-down sock ended under the bumper after creeps/turns; reverse refused once it had turned |
| "Truck not seen" (5x) | each time a different filter or view gap: depth FOV at 0.55 m, label hijack, bottom strip, < 0.20 m filter, robot facing the sock |
| Arm went limp (3x) | sock pinned under a wheel (1); jaws pushing the floor (2, likely) |
| Socks skipped | in-truck radius 0.15 caught the set-down sock beside the truck |
| Costmap blocked every gap | inflation 0.30 (user: now 0.18) |

### New delivery (one look, one drive, one drop)

States, each a small function with its own test; any failure stops with the sock set down in the open:

1. **Pick far from the truck.** Only socks >= 0.5 m from the truck point are picked (no crowding).
   Socks closer than that are reported, not picked.
2. **Carry to a standoff** 0.6 m from the truck, facing it (costmap-checked moves only, no creeps).
3. **Set down beside, look once** from 0.6 m: the whole truck is in the -20/-40 views with depth below
   its near side. Truck centre = near side + 4.5 cm along the ray, stored in the **odometry** frame.
4. **Re-pick** the set-down sock (it has not moved, the robot has not moved): `pick_at`, lift, verify.
5. **Final approach by odometry:** turn to the centre, one costmap-checked move to put it 0.27 m ahead
   (a block at the truck itself is fine: stop there). Odometry over ~0.33 m is good to ~1 cm.
6. **Drop** at the odometry centre converted to the base frame (z 0.12, -30 deg allowed). Arm forward.

Removed: leapfrog hops, blind creeps toward the truck, re-looks at close range.

## Live 2026-10-19 afternoon: redesign first runs (0 delivered, every stage reached once)

Setup each session: relocalize (0.78-0.88), stale truck places deleted, VLM qwen3-vl-4b from the PC
(robot `hwr.env` model reset to the 4B), `GatewaySkills(max_actions=400)`. Truck found only with the
user's help ("at the window") + a depth pixel; the 4B named the robot's own gripper (2x) and a sock (1x)
"toy truck" and never named the real truck at 1.1-1.7 m.

| Run | Got to | Stopped by | Fix |
|---|---|---|---|
| 1 | pick held | straight carry blocked by a sock on the line | `633b867` nav goal to the standoff |
| 2 | standoff, set-down, truck seen | re-pick out of reach (set at 0.275, landed 0.29-0.31) | `a2c3324` |
| 3 | - | "sock" = the tether cable in view; missed | user moved the cable |
| 4 | re-pick held (635) | bed 1.13 m: inflated truck blocks costmap moves at 0.7 m | `fe5fed5` costmap move to the block + <= 0.25 m odometry drive |
| 5 | set-down | re-pick point inside the 0.19 m wheel guard | `2bbfb40` set down at 0.225, point nearest 0.225 m |
| 6 | pick held, standoff | truck not seen from 1.1 m (VLM) | open: see below |

- Truck anchor from 1.7 m away (depth on its top) was ~0.4 m short: true ~(-0.50, 1.28) map, which matches
  the pre-reboot place (-0.47, 1.24). The standoff is computed from the anchor, so it looked from 1.1 m.
- `truck` hits inside the 0.19 m wheel guard are dropped (`c322aec`).
- Failed deliveries leave set-down socks around the standoff.

Next: re-anchor the truck from <= 0.8 m, rerun `sockmark(max_socks=1)`; then check the odometry drive and
the drop land in the bed (user confirms).
