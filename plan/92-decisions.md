# Architecture Decision Records

## ADR-001 — ROS 2 Humble native now, Jazzy-ready code (2026-07-07)

**Context.** Jetson runs Ubuntu 22.04 (JetPack 6.2); only Humble has apt binaries there. Jazzy requires 24.04 → container or reflash. The prebuilt lidar/camera drivers on the robot are Humble/22.04 artifacts.
**Decision.** Develop and run on native Humble. Write all our code Jazzy-ready (pure rclpy, no Humble-only APIs, portable launch/params) so migration is a rebuild, not a rewrite.
**Consequences.** Zero container friction now; vendor sensor binaries stay usable. Revisit when: JetPack ships 24.04, or task 4.4 (own lidar driver) removes the biggest binary anchor. Camera SDK remains the hard anchor either way.

## ADR-002 — Reuse the closed Deptrum Aurora 930 camera driver (2026-07-07)

**Context.** The RGB-D camera is a structured-light device with a proprietary SDK (`deptrum-ros-driver-aurora930`); no source exists on disk or publicly. A clean-room reimplementation would mean reverse-engineering a USB protocol + depth reconstruction — months, off-mission.
**Decision.** Permanently reuse the vendor camera node, wrapped in our own launch/remaps. Everything downstream consumes standard `sensor_msgs`.
**Consequences.** One closed component in an otherwise from-scratch stack (documented exception); camera anchors us to platforms Deptrum supports.

## ADR-003 — PC↔robot control plane is an HTTP/WS gateway, not Windows DDS (2026-07-07)

**Context.** The dev PC is Windows; ROS 2 on Windows is heavy and fragile, and the end state is LLM-agent control, which wants a tool-shaped API anyway.
**Decision.** A `hwr_gateway` node (FastAPI REST + websocket telemetry, token auth) is the single off-robot control surface. CLI, dashboards, and the LLM agent all speak to it. On-robot, everything is ROS 2.
**Consequences.** No ROS install on Windows; one auditable choke point for commands (logging, guardrails); LLM tools map 1:1 to endpoints. RViz/bag work still needs a Linux env (ADR-005, open).

## ADR-004 — Odometry = velocity model + IMU + our own laser scan matching (2026-07-07)

**Context.** The RRC protocol exposes **no encoder reads** — the STM32 closes the wheel loop internally. The vendor "odometry" integrates commanded cmd_vel (ignoring vy!) fused with IMU yaw; it drifts badly and cannot see slip.
**Decision.** Hand-roll a 2D correlative scan matcher as the primary displacement source, fused with the commanded-velocity model (including vy) and IMU yaw rate in our own small filter. The same correlative core is reused for SLAM scan-to-map matching and MCL relocalization.
**Consequences.** Odometry quality depends on scan-match robustness (mitigate: offline bag tuning, Phase 5 acceptance gate before SLAM). CPU budget must be watched (~5 Hz matcher, numpy).

## ADR-005 — PC-side Linux env: WSL2 Ubuntu 24.04 + ROS Jazzy (decided 2026-07-08)

**Context.** Bags recorded on the robot need replay + RViz on the PC for offline SLAM/planner development.
**Options.** WSL2 + ROS desktop; the VMware Ubuntu 22.04 ROS 2 VM (VMware not found installed on the PC); X-forwarding from the robot (too slow).
**Decision.** The PC already runs WSL2 Ubuntu **24.04** — Jazzy's native platform. Install `ros-jazzy-desktop` there for RViz/replay (WSLg GUI, shared filesystem with D:\). This doubles as the standing **Jazzy-readiness proof** for ADR-001: our packages must build and our bags must replay on Jazzy in WSL, continuously.
**Consequences.** Cross-distro (Humble robot ↔ Jazzy PC) bag replay: record bags as `mcap` where possible and keep message deps to std interfaces + hwr_msgs (built in both places). If a cross-version incompatibility ever bites, fall back to a 22.04/Humble WSL distro alongside.

## ADR-006 — Lidar driver: reuse binary now, own UART decoder as stretch (2026-07-07)

**Context.** `sclidar_ros2` exists only as a prebuilt binary; the COIN-D6 speaks plain UART at 230400 with a likely-simple point-packet format, but no protocol doc is on disk.
**Decision.** Phase 4 reuses the binary behind our launch/frame names so autonomy is never blocked. Task 4.4 (stretch): sniff `/dev/lidar`, decode the packet format (header, angle/distance/intensity fields, checksum), publish `LaserScan` from pure Python, swap behind the same launch arg.
**Consequences.** From-scratch purity has one temporary exception with a defined exit; if 4.4 stalls, the binary remains acceptable (it's a sensor source, not robot logic).
