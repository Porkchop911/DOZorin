# highwonder_rework

Ground-up rework of the control software for the **Hiwonder ROSOrin Mecanum Pro** robot (Jetson Orin Nano, `ubuntu@192.168.0.223`). Goal: our own code for every robot-specific layer — serial protocol, kinematics, safety, odometry, SLAM, localization, navigation, arm control — ending in an autonomous robot commanded by an agentic LLM through a clean Python API/CLI/tool-server.

The factory stack stays intact and remains the boot default; this stack **takes over and hands back** the hardware cleanly (see `plan/01-phase0-devloop.md`).

## Current state

Implementation is active and progress is evidence-gated:

- **Phases 0–1 accepted:** takeover/handback workflow, clean-room RRC protocol,
  golden traces, and live motor/servo verification.
- **Phase 2 partially accepted:** kinematics, bounded safety primitives, Robot
  facade, and bench validation are complete; floor calibration and square runs remain.
- **Phase 3 static live graph passed:** ROS messages, priority command mux, driver,
  base pipeline, URDF, teleop, CLI, and bag tools exist; floor runs/bags/RViz remain.
- **Phase 4 static live sensors passed:** lidar filtering, RGB-D wrapper and depth
  bumper, and IMU filter publish at target rates; empirical drift/FOV gates remain.
- **Phase 4 semantic perception passed live:** the articulated camera TF defect
  was fixed, a three-view RGB sweep was captured, and Windows `hwr describe`
  correctly ran authenticated snapshots through localhost LM Studio.
- **Phase 5 static live odometry passed:** scan matching, fusion, ROS wrapper,
  exclusive TF handoff, and offline replay exist; moving-loop acceptance remains.
- **Phase 6 stationary live mapping passed:** keyframed scan-to-map correction,
  log-odds mapping, map/pose/health publication, and PGM+YAML save exist; an attended
  teleop loop and wall-overlay acceptance remain.
- **2026-07-22 partial moving map captured:** one attended quarter-turn expanded
  the map and stopped correctly. Subsequent actuation was refused internally, so
  the partial map was saved as evidence and localization/navigation were not run.
  The refusal was traced to power gates removed by operator decision on 2026-07-22.
- **Phases 7–10 are built, not physically accepted:** coarse global relocalization
  + MCL, live/static costmaps, A*, forward-only recovery, frontier
  exploration, arm/aim/gripper services, authenticated gateway, full CLI, agent
  skills/tool manifest/MCP shim, and LM Studio mission loop are present. Their live
  gates remain deliberately open.
- **Phase 11 foundation exists:** disabled-by-default systemd service, reversible
  boot switch, dependency installer, and runbook. Do not enable it before an HWR map,
  gateway token, and Phase 7–10 live gates are accepted.

The 2026-07-21 live session handed back cleanly. The subsequent no-takeover
deployment built all 14 packages and passed the new Jetson tests plus gateway
authentication smoke test. Power telemetry is observational only; the human
operator is solely responsible for power decisions.

The detailed checklist and acceptance criteria live in [`plan/`](plan/):

| File | Content |
|---|---|
| [`plan/00-masterplan.md`](plan/00-masterplan.md) | Requirements, architecture, target repo layout, phase overview, risks |
| `plan/01…12-phase*.md` | One detailed subplan per phase: objective, granular subtasks, acceptance test + evidence spec, safety/rollback |
| [`plan/90-clean-room-reference.md`](plan/90-clean-room-reference.md) | Protocol/kinematics/URDF facts extracted from the vendor stack (the clean-room spec) |
| [`plan/91-conventions.md`](plan/91-conventions.md) | Working rules: bench-first, operator power responsibility, evidence-gated progress, Jazzy-ready code |
| [`plan/92-decisions.md`](plan/92-decisions.md) | Architecture decision records |

Work proceeds phase by phase; a phase's checkboxes flip only when its acceptance test has produced the specified evidence artifact.

## Sockmark (socks into a container)

`GatewaySkills.sockmark(max_socks, truck="laundry basket", bed_past=0.15, final=0.35, reach_drop=(0.25, 0.30))`
runs: find the container, then repeat find sock, drive to it, grip and lift, carry to the container, drop, back off 0.15 m,
turn 150 deg. The sock is never set down and re-picked. Live logs and the failure table are in `plan/19-sockmark.md`.
Delivered 2026-10-20 (user-checked): the toy truck once, then the basket repeatedly.

### Pitfalls and engineering challenges

| Problem | What happened | What we do |
|---|---|---|
| The VLM names things wrongly | The 4B model calls a sweatshirt, pants and a drying rack a "sock" or "laundry basket". | A second yes/no ask on a crop (`_is_sock`). It is too strict at range, so far socks get rejected. Containers: depth-measure the rim and anchor it; don't trust a sighting. |
| The VLM is not repeatable | The same view returned a sock once and `[]` the next time. | Sweep many views (all pans per tilt, stop at the first hit); retry the whole search, not one view. |
| The held sock blinds the lidar | A sock 13-30 cm ahead looked like a wall: costmap "blocked", clearance 0 everywhere. | `nav_node` masks returns under 0.30 m within +-50 deg of the front while the gripper is closed. |
| The arm can't drop into a tall container | Carry pose puts the tip only 0.17 m ahead, which is the bumper. Releasing there lands on the rim, or the robot shoves the container. | Stop with the front rim 0.20 m ahead, extend with `/arm/reach` over the rim (z 0.30 over a 0.25 m rim), release. `/arm/place` caps z at 0.25. |
| The sock sticks to the jaws | It stayed on the fingertips after opening. | After opening: tip the jaws about 30 deg further down, wiggle the gripper, return to carry. |
| Anchor errors | An anchor at the container's edge missed; an old entry averaged into the new one; the container moved when bumped. | Anchor the centre of the front rim; delete the old `places.yaml` entry first; re-measure after any bump. |
| Localization drifts mid-run | A carry stopped 1 m short because the map pose was wrong. | `_deliver` relocalizes (about 30 s) when localization is unhealthy. |
| Reverse is the only unprotected move | `/drive` refuses reverse; the arm sweep and retrace budget refused or moved the arm with a limp servo. | `/motion/back` is arm-free and capped at 0.15 m per call. |
| Standing on a sock | The shoulder servo stalls when the lift is blocked by the sock under the robot. | A pinned lift opens the gripper, backs off and tries the next sock. |

Known gaps: a gateway read timeout aborts the whole run (it should retry once); the not-a-sock check needs tuning for far socks;
the 0.15 m basket centre offset is a guess.

## Development quickstart

Run the framework-free suite on the PC:

```powershell
cd hwr_core
python -m pip install -e ".[dev]"
python -m pytest -q
```

Deploy and build on the robot:

```powershell
.\deploy\sync.ps1
.\deploy\remote.ps1 "cd ~/hwr && ./deploy/build.sh"
```

Before starting the gateway, run `deploy/install_deps.sh`, copy
`config/hwr.env.example` to `config/hwr.env`, and replace the placeholder with a
random token of at least 24 characters. From the PC, set the same
`HWR_GATEWAY_TOKEN`, then use `python cli/hwr.py status`; `--help` lists drive,
navigation, exploration, arm, snapshot, map, and telemetry commands (`stop` halts motion).

Power telemetry never controls takeover or actuation; power decisions belong to
the human operator. Keep the robot attended with the `stop` command in reach. Take over with
`deploy/takeover.sh` and always restore the factory stack with `deploy/handback.sh`.

## From-scratch policy

**We write:** RRC serial protocol, mecanum kinematics, odometry (velocity model + IMU + our own laser scan matching), safety layer, arm control + IK, URDF, teleop/CLI/gateway, occupancy-grid SLAM, particle-filter localization, A* planner + holonomic local controller, frontier exploration, LLM agent tool server.

**We reuse (justified exceptions, each recorded in `plan/92-decisions.md`):** the OS and ROS 2 middleware itself (rclpy, DDS, TF2, RViz, rosbag2), the closed Deptrum Aurora 930 camera driver, initially the prebuilt `sclidar_ros2` lidar node (own UART driver is a stretch milestone, task 4.4), and the CH341 kernel driver. Platform ≠ robot logic.

**Clean-room rule:** vendor code in `D:\robot\Hiwonder_software_stack\vendor\` is *read as a specification*, never copied. Our implementations cite the reference facts in `plan/90-clean-room-reference.md`.

## Related repos

- `D:\robot\Hiwonder_software_stack` — snapshot + documentation of the existing stack (hardware specs in `docs/HARDWARE.md`, provenance, operating guides).
- `D:\robot\src_ros2` — vendor workspace mirror with the original git history.
