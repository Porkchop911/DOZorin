# Masterplan — highwonder_rework

Rebuild the ROSOrin Mecanum Pro's control software from scratch: full hardware control **and hand-rolled autonomy** (own SLAM, localization, planning), ending with the robot autonomously driven by an agentic LLM. This file is the top-level map; each phase has a detailed subplan (`01…12-phase*.md`).

## Requirements (agreed 2026-07-07)

1. **Scope**: full hardware control + hand-rolled autonomy — not Nav2/slam_toolbox reconfigured; our own mapping, localization, planner, and local controller.
2. **Stack**: ROS 2 + Python. **Humble native now, Jazzy-ready**: pure `rclpy`, no Humble-only APIs, portable configs (Jetson = Ubuntu 22.04 / JetPack 6.2, only Humble has binaries there).
3. **Coexistence**: switchable. Factory stack stays intact and autostarting; we take over `/dev/rrc` and hand it back cleanly.
4. **Interface**: Python API + CLI first; end state = agentic LLM control (LM Studio at `192.168.0.148:1234`, later Hermes).

## Architecture

```
                       PC (Windows)                         Jetson (robot)
  LLM agent ──tools──▶ cli/hwr.py ──HTTP/WS──▶ hwr_gateway ──ROS 2──▶ hwr_* nodes
                                                              │
   hwr_core (framework-free python lib)  ◀── imported by ────┘
   rrc protocol · kinematics · safety                 /dev/rrc @1M baud → STM32
```

- **hwr_core**: pip-installable, zero ROS imports — protocol, kinematics, safety. Testable on the PC with plain pytest. The ROS layer stays thin.
- **Control plane PC↔robot = HTTP/WS gateway**, not Windows-side DDS (no ROS on Windows; the gateway doubles as the LLM tool surface). ADR-003.
- **Autonomy runs on our odometry**: the RRC protocol exposes **no encoder reads** — the vendor integrates commanded cmd_vel open-loop. We fuse velocity model + IMU + our own 2D laser scan matching (ADR-004). This is where the rework genuinely beats the vendor.
- Dev loop: edit on PC → `deploy/sync` → colcon build/run on the Jetson over SSH. SLAM/planner development happens offline on the PC against recorded rosbags first.

## Target repo layout (created in Phase 0, not now)

```
hwr_core/           rrc/ (framing, crc8, codecs, transport, Board) · kinematics/ · safety/ · tests/
ros2_ws/src/
  hwr_msgs          our interfaces (minimal; prefer std msgs)
  hwr_driver        ROS node over hwr_core: motors, servos, IMU, battery, buzzer/LED
  hwr_base          cmd_vel mux → kinematics → driver; smoothing; model odometry
  hwr_description   our URDF/xacro + robot_state_publisher launch
  hwr_teleop        keyboard teleop
  hwr_sensors       lidar bring-up (reused binary) + our scan band-pass + depth→ranges
  hwr_odom          scan-matching odometry + our fusion filter
  hwr_slam          hand-rolled occupancy-grid SLAM
  hwr_loc           hand-rolled MCL + global relocalization
  hwr_nav           costmap, A* planner, holonomic local controller, recoveries
  hwr_explore       frontier exploration
  hwr_arm           arm poses, FK/IK, camera-aim, gripper
  hwr_gateway       FastAPI REST + WS telemetry (auth token)
  hwr_agent         LLM tool server (OpenAI-style tools + MCP)
cli/hwr.py          PC CLI → gateway
deploy/             sync, takeover.sh / handback.sh, optional hwr.service
tools/              bag helpers, 2D kinematic simulator, plot scripts
evidence/           acceptance-test artifacts, per phase (photos, plots, logs)
```

## Phase overview

| Phase | Subplan | Delivers | Gate (evidence-backed) |
|---|---|---|---|
| 0 | `01-phase0-devloop.md` | repo scaffold, sync, takeover/handback | 3× clean takeover→handback cycles |
| 1 | `02-phase1-rrc-protocol.md` | clean-room RRC protocol lib | battery/IMU/motors/servos via our bytes; golden traces match |
| 2 | `03-phase2-kinematics-safety.md` | mecanum kinematics, safety layer, `Robot` API | 1 m square ≤10 cm/15°; watchdog + estop proven |
| 3 | `04-phase3-ros2-teleop.md` | ROS nodes, URDF, teleop, bags | correct TF/lidar orientation in RViz; figure-8; ≥3 bags |
| 4 | `05-phase4-sensors.md` | lidar/camera under our launch, band-pass, depth ranges | walls ±3 cm; 219° coverage; chair leg detected |
| 5 | `06-phase5-odometry.md` | scan-matching odometry + fusion | 2×2 m loop closes ≤5 cm/5° |
| 6 | `07-phase6-slam.md` | occupancy-grid SLAM, PGM+YAML maps | room map congruent with map_04, no shear |
| 7 | `08-phase7-localization.md` | MCL + global relocalization | kidnap recovery <30 s, 3/3 |
| 8 | `09-phase8-navigation.md` | costmap, A*, holonomic controller, recoveries | tour 3/3 with photos; 20-goal soak ≥90% |
| 9 | `10-phase9-exploration-arm.md` | frontier exploration; arm behaviors | auto-map ≥90% of map_04 free area |
| 10 | `11-phase10-agent.md` | gateway, CLI, LLM tool server | ≥4/5 natural-language missions unaided |
| 11 | `12-phase11-boot.md` | optional boot service, runbook, audit | cold boot→mission→handback, one command each |

Ordering is strict through Phase 5; Phases 6–8 iterate together on bags; 9–10 can interleave. Stretch: own lidar UART driver (task 4.4).

## Cross-cutting rules

See `91-conventions.md`. Non-negotiables: bench-first (wheels off ground) for any new actuator path; battery check at session start/end (<11000 mV = wrap up); every acceptance test leaves an artifact under `evidence/` before its checkbox flips; PC-side unit tests before robot time for all algorithmic code; no Humble-only APIs.

## Risks

| Risk | Impact | Mitigation |
|---|---|---|
| No encoder telemetry in RRC protocol | odometry rests on scan matching (Phase 5) | offline bag development; IMU fusion; acceptance gate before SLAM |
| `sclidar_ros2` + Deptrum drivers are prebuilt Humble/22.04 binaries | anchors robot to Humble | Jazzy-ready code everywhere else; stretch 4.4 replaces lidar; camera has no rewrite path (closed structured-light SDK) — accepted |
| Protocol corners unspecified (RGB, OLED quirks, gamepad) | minor features may need trial-and-error | empirical confirmation against firmware, documented in PROTOCOL.md as discovered |
| Hand-rolled SLAM/MCL reliability | autonomy quality | correlative-matcher core shared across odom/SLAM/loc; sim + bags first; explicit acceptance thresholds |
| PC-side ROS tooling (RViz, bag replay) needs Linux | dev friction | WSL2 or the existing VMware Ubuntu VM — decide at task 3.5 (ADR-005 placeholder) |
| Rear-blind robot (lidar ~219° usable) | collision risk when reversing | **no reverse ever** rule baked into controller + recoveries |
