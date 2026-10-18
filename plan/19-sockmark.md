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
