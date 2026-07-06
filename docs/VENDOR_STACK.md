# Vendor ROS 2 stack — package tour

The factory workspace lives at `~/ros2_ws` on the robot (`src/` mirrored here as `vendor/src_ros2/src/`). All packages ship with placeholder descriptions; the purposes below are inferred from code, the User Manual (§1.12), and operation. Vendor example commands for nearly everything are collected in `vendor/src_ros2/command` (sections separated by 🌸) — build, calibration, SLAM, nav, vision demos, voice, LLM demos.

## Drivers & control

| Package | Purpose |
|---|---|
| `driver/ros_robot_controller` (+`_msgs`) | The core serial bridge ROS↔STM32 over `/dev/rrc` (framed + CRC8, 1 Mbaud). Exposes motors, bus/PWM servos, IMU raw, battery, buzzer, LEDs, OLED, button, SBUS. SDK: `ros_robot_controller_sdk.py` |
| `driver/controller` | `cmd_vel` → wheel mixing (mecanum & ackermann), odometry publisher, `rf2o_laser_odometry`, EKF configs |
| `driver/kinematics` (+`kinematics_msgs`) | Arm inverse kinematics (`kinematics_control.py`, `transform.py`) |
| `driver/servo_controller` (+`_msgs`) | Bus + PWM servo management, action groups |
| `driver/sdk` | Low-level helpers (`common.py`, `led.py`) |
| `controller_manager`/`servo_manager` nodes | spawned by bringup from the above |

## Bringup & system

| Package | Purpose |
|---|---|
| `bringup` | `bringup.launch.py` — starts the whole robot at boot (see `SYSTEM.md`); `startup_check.py` |
| `interfaces` | Custom msgs/srvs (`SetInt64`, `SetString`, …) |
| `calibration` | Linear/angular velocity + IMU calibration (`imu_calib.yaml`) |
| `multi` | Multi-robot / master-slave (formation, TF follower) |
| `simulations/rosorin_description` | URDF/xacro robot model (`pro/rosorinpro.xacro`, `pro/lidar.urdf.xacro`), RViz display launch |

## Perception & navigation

| Package | Purpose |
|---|---|
| `peripherals` | LiDAR (sclidar/sllidar/stl06n/ydlidar variants), depth camera, IMU filter, joystick & keyboard teleop, usb_cam |
| `slam` | gmapping / cartographer / slam_toolbox / RTAB-Map mapping launches, `config/slam.yaml`, `maps/` (saved maps live here on the robot) |
| `navigation` | Nav2 + TEB local planner; `config/nav2_params.yaml`, 2D and RTAB-Map navigation launches |

## Apps & examples

| Package | Purpose |
|---|---|
| `app` | Phone-app-facing features: line following, object tracking, self-driving, lidar apps (avoid/follow/guard), AR object loader |
| `example` | Teaching demos: color detect/track/sorting, MediaPipe body/hand/face, AR, QR, garbage classification, RGB-D functions (track-and-grab, plane fit, prevent-falling), YOLO detect (TensorRT `.engine` models on-robot), self-driving lane detect |
| `openclaw_controller` | Gripper-arm scene apps (AprilTag, track-and-grab, smart-scene nav, voice), local gateway `http://127.0.0.1:18789`. **Note:** the on-robot version is a newer vendor refactor (claw_* modules) than the published source zip — see `MODIFICATIONS.md` §"vendor drift" |
| `large_models` (+`_msgs`) | LLM/VLM/ASR/TTS agent framework; provider config in `large_models/config.py` (**modified** → LM Studio) |
| `large_models_examples` | LLM/VLM demos: voice move, color track, visual patrol, VLM camera/track/navigation, function calling, road network planning |
| `xf_mic_asr_offline` (+`_msgs`) | iFlytek offline mic ASR / wake word ("HELLO HIWONDER"), voice-control demos (move/color/navigation) |

## Where things run from

Packages are colcon **symlink-installed** with `need_compile=False` — Python and config edits under `~/ros2_ws/src/...` take effect on restart without rebuilding. The vendor git history for this tree (7 commits by Hiwonder, Feb–Mar 2026, plus our baseline import) is preserved in `D:\robot\src_ros2\.git`; the shipped image (2026-05-09) is slightly newer than the published zip — the differences are catalogued in `MODIFICATIONS.md`.
