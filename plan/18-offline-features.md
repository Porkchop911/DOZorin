# 18 — Offline features: landmarks, follow v2, mapping, reverse (2026-10-18)

Built while the robot was off. Nothing in this file has run on the robot yet.
The rule for every item: the logic is a pure function in `hwr_core` with unit tests, and the ROS
nodes and gateway only wire it in. Each item ends with the live acceptance test to run next session.

## 18.1 Landmark auto-anchoring

- **Problem.** Places exist only when a person calls `mark_place`. YOLO sees chairs, couches and
  tables every roam tick, but forgets where they were.
- **Build.**
  - `hwr_core.navigation.places.anchor_object(places, cls, object_xy, robot_xy)` turns a sighting
    into an object place:
    - the object point;
    - an approach pose 0.40 m from the object toward where the robot stood, facing it;
    - `auto: true`, `verified: false`, and a `sightings` count.
  - Merging: a sighting within 0.75 m of an auto place of the same class updates it (running
    mean of the object point). A sighting farther away creates `<class>_2`, `<class>_3`, and so on.
  - Places a person marked (`verified: true`) are never touched.
  - Movers are not anchored: person, animals, cubes, and anything without a depth range.
- **Gateway.** `POST /places/anchor` anchors every object currently seen with a range. It uses the
  map pose plus the arm's pan and refuses when localisation is unhealthy.
  - Object position = pan axis + range along (pan + image bearing).
- **Agent.** `roam_step` calls it each tick and reports what it anchored. `goto_place("chair")`
  then works through the existing object-place path (0.50 m reach check).
- **Live test.** 2 roam ticks in the room. Check that `list_places` shows the couch/chair within
  ~0.3 m of where they are, and that `goto_place("<class>")` reaches it.

## 18.2 Follow v2

- **Problem.** `/arm/follow` (14.5) never ran live (B1). When the target leaves the view it stands
  still for 3 s and quits. Its range input is raw per-frame depth, so the forward speed jitters.
- **Build.**
  - `hwr_core.arm.manipulation.lost_search_step(last_bearing, lost_s)`: the base turns toward the
    side the target left on, at 0.35 rad/s, with the camera centred.
  - The search gives up after `search_s` (8 s) and reports "target lost".
  - A target never found at start: one arm sweep (0/+60/-60) before giving up.
  - Range smoothing: an exponential moving average (alpha 0.4), reset when the target is
    re-acquired.
  - The result reports `searches` and `reacquired`.
- **Live test.** B1 as written in plan/16. A person walks out of view to the left, and the robot
  turns and re-acquires within 8 s.

## 18.3 Mapping

- **Problem.** The map_room run of 2026-10-16 ended after 0.1 m: the VLM preflight rejected all 3
  frontier legs for clutter. Every reject cools down the whole direction. There is also no
  coverage number, so "explore more" can't be judged.
- **Build.**
  - `hwr_core.exploration.map_coverage(grid, resolution)` returns `{known_m2, free_m2,
    occupied_m2}`. The explore status and `map_room` report the area before, the area after, and
    the gain.
  - Shorter-leg retry: when the semantic preflight rejects a translation leg (not a glass
    boundary) longer than 0.40 m, the explorer re-asks once for half the leg (minimum 0.30 m)
    before cooling the frontier down. The pure helper is
    `hwr_core.exploration.shorter_leg(pose_xy, waypoint, minimum)`.
- **Live test.** `map_room` from the same spot. Report `known_m2` gained and the number of
  half-leg retries that passed.

## 18.4 Reverse: retrace plus a rear look (user decision 2026-10-18)

- **User.** "Retrace is good but also rotating camera left and right in a position that doesn't
  protrude over the robot chassis, maximum turn angle sees behind."
- **Policy** (plan/17 option b, plus a camera check):
  1. **Retrace only.** Reverse at most `min(0.15 m, straight path just driven forward)`.
     - `hwr_core.navigation.retrace.retrace_budget(trail, pose)` walks the odometry trail
       backwards while it stays within 3 cm of the line behind the robot and within 10° of the
       current heading.
     - A turn or strafe since then ends the budget.
  2. **Rear look.** Arm poses `rear_left` and `rear_right` keep the forward-pose arm geometry,
     pan ±120° (servo 1 at 1000/0), and tilt down 41°.
     - FK: the arm tip sits at (−0.02, ±0.11) m, inside the 0.20 m footprint, so nothing protrudes.
     - The depth node publishes a full-circle fan `depth_ranges_full` (base frame, 5° beams).
     - `hwr_core.navigation.retrace.rear_blocked(angles, ranges, meters)` reports any return
       inside the rear corridor (|y| ≤ 0.18 m, from 0.12 m behind centre to meters + 0.30 m).
     - Both sides must be clear. The camera stays on the left-rear while backing and aborts on
       a hit.
  3. **Known gap.** With about 75° FOV, the two looks cover about 83–157° on each side; directly
     behind (about ±23°) is not seen. The retrace rule covers that strip: the robot just came
     from there.
  4. **Structural clamp kept.** `base_node`'s mux and `allow_reverse=false` are unchanged.
     - Reverse arrives only on `cmd_vel_reverse`, which the base accepts only as pure −x, at most
       0.06 m/s, and only while no other source is active.
     - Nav, teleop and `/drive` still cannot reverse.
- **Gateway.** `POST /motion/back {meters}`: retrace budget, rear look both sides, back at
  0.05 m/s by odometry, then arm forward. The result reports `budget_m`, `travelled_m`, and the
  rear hits.
- **Agent.** `back(meters)` skill and tool. Voice "go back" stays a refusal: plan/17 Q5 is still
  open.
- **plan/91 rule 3** changes from "no reverse ever" to "reverse only via `/motion/back`".
- **Live test.**
  - Drive 0.3 m forward, then `back(0.15)`: travelled ≥ 0.13 m.
  - Turn 30°, then `back`: refused (budget 0).
  - A box at 0.25 m behind-left: refused with the hit.
