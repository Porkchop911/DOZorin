# highwonder_rework

Ground-up rework of the control software for the **Hiwonder ROSOrin Mecanum Pro** robot (Jetson Orin Nano, `ubuntu@192.168.0.223`). Goal: our own code for every robot-specific layer — serial protocol, kinematics, safety, odometry, SLAM, localization, navigation, arm control — ending in an autonomous robot commanded by an agentic LLM through a clean Python API/CLI/tool-server.

The factory stack stays intact and remains the boot default; this stack **takes over and hands back** the hardware cleanly (see `plan/01-phase0-devloop.md`).

## Current state

**Planning.** No implementation exists yet. Everything is specified in [`plan/`](plan/):

| File | Content |
|---|---|
| [`plan/00-masterplan.md`](plan/00-masterplan.md) | Requirements, architecture, target repo layout, phase overview, risks |
| `plan/01…12-phase*.md` | One detailed subplan per phase: objective, granular subtasks, acceptance test + evidence spec, safety/rollback |
| [`plan/90-clean-room-reference.md`](plan/90-clean-room-reference.md) | Protocol/kinematics/URDF facts extracted from the vendor stack (the clean-room spec) |
| [`plan/91-conventions.md`](plan/91-conventions.md) | Working rules: bench-first, battery discipline, evidence-gated progress, Jazzy-ready code |
| [`plan/92-decisions.md`](plan/92-decisions.md) | Architecture decision records |

Work proceeds phase by phase; a phase's checkboxes flip only when its acceptance test has produced the specified evidence artifact.

## From-scratch policy

**We write:** RRC serial protocol, mecanum kinematics, odometry (velocity model + IMU + our own laser scan matching), safety layer, arm control + IK, URDF, teleop/CLI/gateway, occupancy-grid SLAM, particle-filter localization, A* planner + holonomic local controller, frontier exploration, LLM agent tool server.

**We reuse (justified exceptions, each recorded in `plan/92-decisions.md`):** the OS and ROS 2 middleware itself (rclpy, DDS, TF2, RViz, rosbag2), the closed Deptrum Aurora 930 camera driver, initially the prebuilt `sclidar_ros2` lidar node (own UART driver is a stretch milestone, task 4.4), and the CH341 kernel driver. Platform ≠ robot logic.

**Clean-room rule:** vendor code in `D:\robot\Hiwonder_software_stack\vendor\` is *read as a specification*, never copied. Our implementations cite the reference facts in `plan/90-clean-room-reference.md`.

## Related repos

- `D:\robot\Hiwonder_software_stack` — snapshot + documentation of the existing stack (hardware specs in `docs/HARDWARE.md`, provenance, operating guides).
- `D:\robot\src_ros2` — vendor workspace mirror with the original git history.
