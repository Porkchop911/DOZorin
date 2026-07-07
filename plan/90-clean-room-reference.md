# Clean-room reference — hardware & protocol facts

Extracted 2026-07-07 by *reading* the vendor stack (`D:\robot\Hiwonder_software_stack\vendor\src_ros2\src\`) and the live robot. This file is the specification our implementations are written against; vendor code is never copied. Line references are into the vendor tree for re-verification.

## RRC serial protocol (STM32 ↔ Jetson)

Reference: `driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` (732 lines). No protocol PDF exists anywhere on disk; this SDK **is** the spec. STM32 firmware is a binary blob (`RosRobotControllerM4-20260414.hex`) — corners not exercised by the SDK need empirical confirmation.

### Transport
- Device `/dev/rrc` (udev symlink → `ttyACM0`), **1,000,000 baud**, 8N1, timeout 5 s, **RTS and DTR forced low before open**.
- Single RX thread, byte-wise state machine: STARTBYTE1 → STARTBYTE2 → FUNCTION → LENGTH → DATA → CHECKSUM. Reports go to per-function latest-value mailboxes (queue maxsize 1). Servo reads are write-then-wait under a lock.

### Framing
```
0xAA 0x55 | function(1) | length(1) | data[length] | crc8
```
- CRC-8: table-driven, init 0, no final XOR, computed over `[function, length, data...]` (both TX and RX). Extract the 256-entry table from SDK lines 48–65 by hand-copying values (data table, not creative code).
- RX accepts function only if `< 11`.

### Function IDs and payloads (data[0] = sub-command where noted)
| ID | Function | TX payloads | RX reports |
|---|---|---|---|
| 0 | SYS | `[0x01, low, high]` battery-level calibration | `[0x04, mV_u16_LE]` battery, ~1 Hz. **Unit is millivolts** (SDK comment claims otherwise; verified live ≈12000) |
| 1 | LED | `[id, on_ms u16, off_ms u16, repeat u16]` | — |
| 2 | BUZZER | `[freq u16, on_ms u16, off_ms u16, repeat u16]` | — |
| 3 | MOTOR | `[0x01, count, {id-1 u8, rps float32 LE}×count]` set speed; `[0x05, type]` motor type (JGB520=0, **JGB37=1**, JGB27=2, JGB528=3) | — |
| 4 | PWM_SERVO | `[0x03, dur_L, dur_H, {id, pos u16}...]` set; `[0x07, id, offset i8]`; reads `0x05` pos / `0x09` offset | read replies |
| 5 | BUS_SERVO | `[0x01, dur_L, dur_H, count, {id, pos u16}×count]` set-pos (dur in ms, pos 0–1000); `0x03` stop; `0x0B/0x0C` torque on/off; reads: pos `0x05`, vin `0x07`, temp `0x09`, torque state `0x0D`; id set/read `0x10/0x12`; offset `0x20/0x22/0x24`; limits angle `0x30/0x32`, vin `0x34/0x36`, temp `0x38/0x3A`; HX extensions: scan-ids `0x40`, PID read/set `0x42/0x43`, baud `0x44` | read replies |
| 6 | KEY | — | event byte: PRESSED 0x01, LONGPRESS 0x02, LP_REPEAT 0x04, RELEASE_FROM_LP 0x08, RELEASE_FROM_SP 0x10, CLICK 0x20, DOUBLE 0x40, TRIPLE 0x80 |
| 7 | IMU | — | 6× float32 LE: ax,ay,az (**g**), gx,gy,gz (**deg/s**). Convert ×9.80665 and →rad. Orientation NOT provided |
| 8 | GAMEPAD | — | `<HB4b`: buttons bitmask, hat, lx,ly,rx,ry |
| 9 | SBUS | — | 16× i16 channels + ch17/ch18/lost/failsafe; normalize `2*(v-192)/1600-1` |
| 10 | OLED | `[line, len, utf8...]` | — |

### Known gaps (confirm empirically, log findings in docs/PROTOCOL.md when it exists)
- `set_rgb` is *called* by the vendor node but **not defined** in the SDK — RGB wire format unknown (try LED-like payloads; low priority).
- **No encoder/odometry read function exists.** The STM32 closes the wheel velocity loop internally; the host never sees encoder counts. See ADR-004.

## Mecanum kinematics

Reference: `driver/controller/controller/mecanum.py` (60 lines) + real constants from `odom_publisher_node.py:105`.

- **wheelbase = 0.17706 m, track = 0.17165 m, wheel Ø = 0.08 m** (the class defaults, incl. Ø 0.0065, are wrong/unused — always use the instantiation values).
- Mixing, with `k = (wheelbase + track)/2`:
  ```
  m1 = vx − vy − wz·k      m2 = vx + vy − wz·k
  m3 = vx + vy + wz·k      m4 = vx − vy + wz·k
  wire = [m1, m2, −m3, −m4]        # motors 3,4 sign-inverted
  rps  = v / (π · wheel_diameter)  # wire unit is rev/s float32
  ```
- Vendor safe clamps: ±0.2 m/s linear, ±0.5 rad/s angular (their app path). Treat as starting envelope; characterize real limits on the stand (task 1.5).

## Frames / URDF ground truth

Reference: `simulations/rosorin_description/urdf/pro/*.xacro`.

- `base_footprint → base_link`: z = 0.025 m.
- **Lidar**: `base_link → lidar_frame` xyz (0.048156, ≈0, 0.106748), **rpy (0, 0, π)** — mounted 180° rotated; real. Vendor band-passes the scan to angles 1.25–5.08 rad (≈219° usable; rear ~140° blocked by chassis). The robot is rear-blind → no reverse maneuvers, ever.
- **Camera on arm link4** (eye-in-hand): `camera_connect_joint` xyz (−0.036369, −0.0181, 0.046638), rpy ≈ (−1.5621, −1.5621, 1.5708); optical frame rotated (−π/2, 0, −π/2). Aiming the camera = posing the arm.
- **Arm**: 5 revolute joints, all limits ±2.09 rad; joint1 at (0.04662, ≈0, 0.16044) axis −Z; end_effector_link at link5 + 0.08 m. Gripper = bus servo **id 10** (fixed joint in URDF).
- Servo ID map (live-verified): 1 = base pan (500 center, >500 left), 2 shoulder, 3 elbow, 4 camera tilt (88 floor / 300 forward / 500 ceiling), 5 wrist roll, 10 gripper. Known-good forward-look pose: `(1,500)(2,701)(3,120)(4,300)(5,500)`. Pulses 0–1000 ↔ 0–240° (0.24°/unit).

## Sensors & external drivers

| Device | Interface | Driver status |
|---|---|---|
| COIN-D6 lidar | `/dev/lidar` (CH340, port-pinned udev) @ 230400, 10 Hz, 0.9°, ≤12 m | `sclidar_ros2` prebuilt binary on robot (params: port, baud, `version=4`; publishes `scan_raw`). **No source anywhere.** Reuse initially; own UART decoder = stretch task 4.4 |
| Aurora 930 RGB-D | USB 2.0, 640×400@12fps, FOV 74°×51°, range 0.15–3 m | `deptrum-ros-driver-aurora930` closed SDK. Reuse permanently (ADR-002) |
| MPU6050 IMU | via RRC function 7 | ours (Phase 1) |
| WonderEcho Pro audio | ALSA card 0 `plughw:0,0` 48 kHz | ours later (voice not in critical path) |

Also on robot but being **replaced by our code**: `robot_localization` (EKF), `rf2o_laser_odometry`, `laser_filters`, vendor `imu_filter`.

## Vendor behavior worth knowing (not copying)

- Vendor odometry (`odom_publisher_node.py`) integrates **commanded** cmd_vel at 50 Hz — and ignores vy entirely (mecanum strafe invisible to their odom!); EKF fuses that with IMU yaw. This is the quality bar to beat in Phase 5.
- Vendor bus-servo "action groups" are SQLite `.d6a` files (columns → servo ids 1–5, col 6 → id 10) — we don't adopt the format; noted for possible import tooling later.
- `wifi_manager/button_scan.py` (vendor systemd unit) also opens `/dev/ttyACM0` alongside the ROS driver and must be left running during takeover (it coexists; killing it breaks the app-pairing button).

## Ops facts

- Robot: `ssh ubuntu@192.168.0.223` (key auth, passwordless sudo). PowerShell quoting mangles SSH args — use the base64 pattern: build script → `[Convert]::ToBase64String` → `ssh … "echo <b64> | base64 -d | zsh"` (zsh needed for ROS env: bash drops `ROS_LOCALHOST_ONLY=0` and silently kills DDS discovery). Never retry-storm SSH (sshd MaxStartups self-DoS).
- Factory autostart: `start_app_node.service` → `bringup.launch.py`. `ROS_DOMAIN_ID=0`, FastDDS.
- Battery: `/ros_robot_controller/battery` (mV). **<11000 wrap up, <10500 stop immediately.** Check at session start/end. Pack: 11.1 V 6000 mAh; charge only powered off.
- Full hardware datasheet: `D:\robot\Hiwonder_software_stack\docs\HARDWARE.md`; User Manual PDF §1.10 for component specs.
