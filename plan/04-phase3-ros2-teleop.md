# Phase 3 — ROS 2 layer, URDF, teleop, bags

**Objective.** Lift hwr_core into a minimal, Jazzy-ready ROS 2 graph: driver node, TF/URDF, cmd_vel pipeline, keyboard teleop — plus the rosbag workflow that all autonomy phases depend on.

**Prerequisites.** Phase 2. Frames: `90-clean-room-reference.md` §Frames.

## Subtasks

- [ ] **3.1 hwr_msgs + hwr_driver.** Package `hwr_msgs` only if std msgs don't fit (target: `WheelSpeeds`, `ServoCommand`; battery = `sensor_msgs/BatteryState`, IMU = `sensor_msgs/Imu`). `hwr_driver` node: owns one `Robot`; pubs `imu` (50 Hz), `battery` (1 Hz), `button`; subs `wheel_speeds`, `servo_cmd`, `beep`; exposes estop service. Parameters from YAML. Lifecycle-style clean shutdown (stop + handback-safe).
- [ ] **3.2 hwr_description.** Our URDF/xacro from reference dims: base_footprint→base_link z 0.025; lidar xyz (0.048156, 0, 0.106748) **rpy (0,0,π)**; arm chain (joint1 origin/axis, ±2.09 limits); camera on link4 with optical frame. `display.launch.py` for robot_state_publisher (+ joint states from servo read-back at 5 Hz).
- [ ] **3.3 hwr_base.** `cmd_vel` mux (priority: estop > teleop > agent > nav; timeout-based), smoothing to Limits, → kinematics → `wheel_speeds`. Model odometry publisher: integrate commanded vx **and vy** + IMU yaw (simple exact integration per tick), publish `odom_model` + TF `odom→base_footprint` (flagged so Phase 5 fusion can take over TF later), covariances honest (large).
- [ ] **3.4 Teleop + CLI stub.** `hwr_teleop` keyboard node (hold-to-drive, release=stop, speed steps, estop key) runnable over SSH. `cli/hwr.py` v0 talking straight to the robot over SSH for `status|beep|arm pose` (gateway comes in Phase 10; keep this thin).
- [ ] **3.5 Bag workflow (ADR-005 decision point).** Pick WSL2 vs the existing VM for PC-side RViz/replay; document in ADR-005. `tools/record.sh` (scan_raw, imu, odom_model, wheel_speeds, tf, camera off by default) and `tools/replay.sh`. Record ≥3 teleop bags in the flat: slow tour, figure-8, in-place rotation.
- [ ] **3.6 RViz check.** With lidar running under the factory... **no** — lidar isn't ours until Phase 4; for this phase RViz check = TF tree + URDF visual + teleop-driven odom trace only. (Scan overlay validation moves to Phase 4.)

## Acceptance & evidence

- **Test:** `ros2 doctor`-clean graph; RViz shows correct URDF + TF (arm moves when posed); teleop figure-8 driven comfortably; ≥3 bags recorded and replayable on the PC env; driver survives SIGINT with motors stopped.
- **Evidence:** `evidence/phase3/`: RViz screenshot (TF tree + model), bag file list + `ros2 bag info` output, teleop session log.

## Safety / rollback

- Teleop release-to-stop + mux timeout are the operative guards; watchdog still underneath.
- Bags contain no secrets; camera topics excluded by default to keep sizes sane.

**Effort.** 1.5–2 days including the env decision.

**Open questions.** Joint-state read-back rate that doesn't disturb servo bus (probe 2–5 Hz); WSL2 GUI performance for RViz.
