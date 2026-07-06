# Hardware Specifications — Hiwonder ROSOrin Pro (Mecanum, Jetson Orin Nano)

Sources: **[UM]** = `1. ROSOrin Pro User Manual.pdf` (94 pp, §1.10 Hardware Introduction), at
`D:\robot\Jetson Orin Nano & Orin NX-20260623T120657Z-3-001\Jetson Orin Nano & Orin NX\1. Tutorials\1. ROSOrin Pro User Manual\`.
**[LIVE]** = measured on the running robot over SSH, 2026-07-06. **[FIELD]** = discovered during operation (see `MODIFICATIONS.md` / `added/guides/ROBOT_HANDOFF_GUIDE.md`).

ROSOrin Pro is an integrated AI education/research robot: mecanum chassis, LiDAR, 6-DOF robotic arm with gripper, 3D depth camera (arm-mounted), and an AI voice box, on a Jetson Orin Nano running Ubuntu 22.04 + ROS 2 Humble. [UM §1.1]

## System architecture

Two-controller design [UM §1.11]:
- **STM32 low-level controller** — motors, encoders, IMU, servos, battery monitoring, LED/buzzer/OLED. Talks to the Jetson over USB serial (`/dev/ttyACM0`, udev symlink `/dev/rrc`) using Hiwonder's framed hex "RRC" protocol with CRC8, 1 Mbaud.
- **Jetson Orin Nano (ROS main controller)** — runs Ubuntu 22.04 / ROS 2 Humble and everything else (SLAM, Nav2, vision, voice, LLM clients).

## ROS main controller — Jetson Orin Nano [LIVE]

| Item | Value |
|---|---|
| Module | NVIDIA Jetson Orin Nano (Developer Kit, "Super"), P-number p3767-0005 |
| CPU | 6× Arm Cortex-A78AE (aarch64) |
| RAM | 7.4 GiB |
| Storage | 228 GB NVMe SSD (`/dev/nvme0n1p1`, 61 GB used) |
| OS | Ubuntu 22.04.5 LTS, kernel 5.15.148-tegra |
| JetPack / L4T | JetPack 6.2, L4T R36.4.3 |
| CUDA / cuDNN / TensorRT | 12.6.68 / 9.3.0.75 / 10.7.0.23 (OpenCV 4.11.0 with CUDA) |
| Power mode | MAXN_SUPER |
| ROS | ROS 2 Humble (native, no Docker), 392 `ros-humble-*` debs, Python 3.10.12 |

The Jetson sits on a Hiwonder expansion board exposing LED network-status indicators, mode buttons, GPIO and I2C. [UM §1.10.3.2]

## STM32 robot controller [UM §1.10.2.1]

| Item | Value |
|---|---|
| MCU | STM32F407VET6 (Arm Cortex-M4, 168 MHz, 512 KB flash, 192 KB SRAM, FPU+DSP) |
| Motor channels | up to 4 DC encoder motors |
| Power outputs | 2× 5 V, max 5 A each |
| IMU | MPU6050 6-axis (3-axis accel + 3-axis gyro), I2C |
| Debug / flashing | SWD port (PA13=DIO, PA14=CLK); USB-serial firmware update |
| Firmware | `vendor/firmware/RosRobotControllerM4-20260414.hex` (as shipped) |
| Bus | USB serial to Jetson, 1 Mbaud, framed + CRC8 ("RRC" protocol) [UM §1.11.2] |

One board supports mecanum / Ackermann / differential / tank chassis; this unit is the **mecanum** variant (`MACHINE_TYPE=ROSOrin_Mecanum_Pro`). [LIVE]

## Power [UM §1.10.2.2]

| Item | Value |
|---|---|
| Battery | dedicated 11.1 V 6000 mAh lithium pack |
| Charging voltage | 12.6 V (dedicated charger; red = charging, green = full. Charge only while powered OFF) |
| Discharge | ≤ 10 A, SM connector with reverse-polarity protection |
| Protections | overcharge, overcurrent, overdischarge, short circuit |
| Monitoring | `/ros_robot_controller/battery` (UInt16, millivolts). [FIELD] Treat < 11000 mV as "wrap up"; pack ran flat once at neglect. Reading 2026-07-06: 11933–12082 mV |

## Drive motors — Hall-encoder DC geared [UM §1.10.2.3]

| Parameter | Value | Parameter | Value |
|---|---|---|---|
| Rated voltage | 12 V | Gear ratio | 1:90 |
| No-load speed | 110 rpm | Rated speed | 85 rpm |
| Stall torque | 15 kg·cm | Rated torque | 2.6 kg·cm |
| Stall current | 3.2 A | Rated current | 0.36 A |
| Rated power | ≈ 8.3 W | Encoder | AB two-phase Hall, 11-line magnetic ring |
| Motor type | brushed PM | Shaft | 6 mm D-shaped eccentric |
| Connector | PH2.0-6P | Encoder voltage | 3.3–5 V |

[FIELD] Software clamps in the vendor controller: ±0.2 m/s linear, ±0.5 rad/s angular; drive topics `/controller/cmd_vel` (raw, watchdog needs ≥10 Hz), `/cmd_vel` (safe).

## Robotic arm — HX-12H bus servos [UM §1.10.2.4]

| Parameter | Value |
|---|---|
| Servo model | HX-12H digital bus servo (metal gears) |
| Operating voltage | DC 9–12.6 V |
| Control | UART serial bus commands, connector 5264-3P |
| Range | 0–1000 units ↔ 0–240°, accuracy 0.3° |
| Speed / torque | 0.2 s/60° @ 11.1 V; stall 12 kg·cm @ 11 V |
| Size / weight | 35.19 × 26.19 × 27.20 mm, 36.3 g |

[FIELD] Servo ID map (topic `/ros_robot_controller/bus_servo/set_position`, `ServosPosition` msg):

| ID | Joint | Notes |
|---|---|---|
| 1 | base pan | 500 = center, >500 = left |
| 2, 3 | shoulder, elbow | — |
| 4 | camera tilt | 88 = floor, 300 = forward, 500 = ceiling |
| 5 | wrist roll | 500 = level |
| 10 | gripper | — |

Forward-look arm pose used by all scripts: `(1,500)(2,701)(3,120)(4,300)(5,500)`. Servo offsets: `/home/ubuntu/software/Servo_upper_computer/servo_config.yaml`. The manual calls the arm 6-DOF (5 joints + gripper). The depth camera is **mounted on the arm** — aiming the camera means posing the arm.

## Depth camera — Deptrum Aurora 930 [UM §1.10.4]

| Item | Specification |
|---|---|
| Technology | 3D structured light, 40 mm baseline, Class 1 laser |
| Depth accuracy / precision | 8 mm @ 1 m / 3 mm @ 0.5 m, 7 mm @ 1 m |
| Working distance | 15–300 cm |
| Depth, color, IR streams | each 640 × 400 @ 12 fps, FOV 74° × 51° |
| Interface / power | USB 2.0 (shows as `Aurora 930`, id 3251:1930 [LIVE]); 5 V, avg < 1.6 W |
| Ambient light | 3–80,000 lux; −10…55 °C |

[LIVE] Topics: `/depth_cam/rgb0/image_raw` (bgr8, ~11 Hz), `/depth_cam/depth0/image_raw` (~13 Hz), `/depth_cam/ir0/image_raw`, `/depth_cam/depth0/points` (pointcloud disabled in normal operation — it starved the CPU; replaced by `depth_guard.py`, see `ADDED_SOFTWARE.md`).

## LiDAR — COIN-D6 [UM §1.10.5]

| Item | Specification |
|---|---|
| Model / principle | COIN-D6, TOF |
| Scanning range | 360° (datasheet — but see below) |
| Measurement radius | 12 m (black object) |
| Sampling / scanning | 4000 samples/s, 10 Hz (9.5–10.5), 0.9° angular resolution |
| Accuracy | ≤ 4 mm (0.1–2 m), ≤ 15 mm (2–12 m) |
| Interface | UART 230400 bps, 5 V / 240 mA typ (`/dev/lidar` → `ttyCH341USB0` [LIVE]) |
| Size | 52.7 × 45.2 × 34.4 mm |

**[FIELD] Reality on this robot — differs from datasheet:** the lidar is chassis-recessed and mounted **rotated 180°** (static TF `base_footprint→lidar_frame` yaw = 180°; base-forward ≈ lidar 181°). Only ~**219° of usable FOV** (valid returns lidar-frame 72°–291°); the rear ~140° is blocked by the chassis — the robot is **blind to the rear**, which is why reverse/backup recoveries are disabled in Nav2 (see `MODIFICATIONS.md`). Identified as `LIDAR_TYPE=SCLIDAR` in the vendor stack.

## Voice box — WonderEcho Pro

AI voice interaction box (USB). [LIVE] Enumerates as `JMTek USB PnP Audio Device` = ALSA card 0 (`plughw:0,0`), 48 kHz native, full-duplex (mic + speaker); a separate CH340 serial (`/dev/ring_mic` → `ttyCH341USB1`) carries the vendor wake-word protocol. `MIC_TYPE=WonderEchoPro`. Vendor offline ASR is iFlytek (`xf_mic_asr_offline`); the added assistant uses sherpa-onnx instead (see `ADDED_SOFTWARE.md`).

## Other components

- **OLED module** [UM §1.10.2.5]: 0.91″ blue OLED, 128 × 32, SSD1306, I2C @ 5 V, 30.0 × 11.5 mm outline. Shows IP/status; ROS topic `/ros_robot_controller/set_oled`.
- **PS2 wireless controller** [UM §1.10.2.6]: USB receiver on the STM32; START = stop/reset + wake.
- **Buzzer / RGB LEDs / button** on the STM32 board: topics `/ros_robot_controller/set_buzzer`, `/set_led`, `/set_rgb`, `/button`.
- **Packing list / assembly**: [UM §1.1.1] (graphical); assembly video (Jetson Orin version): https://youtu.be/b7xMWE_v10Y

## USB topology [LIVE]

Realtek 4-port USB 3.0 + 2.0 hubs → Aurora 930 (camera), QinHeng CH340 ×2 (lidar, ring-mic serial), QinHeng USB Single Serial 1a86:55d4 (= STM32 `/dev/ttyACM0`, ModemManager-ignored via `99-ttyACM0.rules`), JMTek audio (WonderEcho), Realtek Bluetooth.

## Safety envelope [FIELD]

No rear sensing (lidar blind ~140° aft, camera faces forward), no cliff/stair sensing, no bumpers. The added Nav2 behavior trees remove reverse maneuvers; scripts lock the arm forward before driving. Battery must be checked before/after every session.
