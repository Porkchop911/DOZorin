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
navigation, exploration, arm, snapshot, map, telemetry, and estop commands.

Power telemetry never controls takeover or actuation; power decisions belong to
the human operator. Keep the robot attended with the estop path in reach. Take over with
`deploy/takeover.sh` and always restore the factory stack with `deploy/handback.sh`.

## From-scratch policy

**We write:** RRC serial protocol, mecanum kinematics, odometry (velocity model + IMU + our own laser scan matching), safety layer, arm control + IK, URDF, teleop/CLI/gateway, occupancy-grid SLAM, particle-filter localization, A* planner + holonomic local controller, frontier exploration, LLM agent tool server.

**We reuse (justified exceptions, each recorded in `plan/92-decisions.md`):** the OS and ROS 2 middleware itself (rclpy, DDS, TF2, RViz, rosbag2), the closed Deptrum Aurora 930 camera driver, initially the prebuilt `sclidar_ros2` lidar node (own UART driver is a stretch milestone, task 4.4), and the CH341 kernel driver. Platform ≠ robot logic.

**Clean-room rule:** vendor code in `D:\robot\Hiwonder_software_stack\vendor\` is *read as a specification*, never copied. Our implementations cite the reference facts in `plan/90-clean-room-reference.md`.

## Related repos

- `D:\robot\Hiwonder_software_stack` — snapshot + documentation of the existing stack (hardware specs in `docs/HARDWARE.md`, provenance, operating guides).
- `D:\robot\src_ros2` — vendor workspace mirror with the original git history.
