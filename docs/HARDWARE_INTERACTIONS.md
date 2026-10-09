# Hardware Interactions

This document maps physical robot components to Linux devices, ROS interfaces, responsible software, and practical failure modes. Hardware specifications live in `docs/HARDWARE.md`; OS/device facts live in `docs/SYSTEM.md`.

## Interaction Map

| Hardware | Linux/device path | Main ROS/software interface | Responsible area | Notes |
|---|---|---|---|---|
| STM32F407 low-level controller | `/dev/rrc` -> `ttyACM0` | `ros_robot_controller_node`; `ros_robot_controller/imu_raw`, `joy`, `sbus`, `button`, `battery`; `set_motor`, `set_led`, `set_rgb`, `set_buzzer`, `set_oled`, bus/PWM servo topics/services | `vendor/src_ros2/src/driver/ros_robot_controller` | Central bridge between ROS and low-level hardware; duplicate stacks can fight for this serial device |
| Four mecanum drive motors | Via STM32 on `/dev/rrc` | `cmd_vel`, `controller/cmd_vel`, `app/cmd_vel` -> `ros_robot_controller/set_motor` | `vendor/src_ros2/src/driver/controller` and `ros_robot_controller` | Topic choice matters; `cmd_vel` is clamped in controller source, while `controller/cmd_vel` is raw |
| Wheel/IMU odometry path | Via STM32 and ROS controller stack | `odom_raw`, EKF-remapped `odom`; `ros_robot_controller/imu_raw` -> `imu_corrected` -> `imu` | `driver/controller`, `peripherals/launch/imu_filter.launch.py`, `robot_localization` | Sideways mecanum motion has known odometry limitations |
| COIN-D6 lidar | `/dev/lidar` -> `ttyCH341USB0` | SCLIDAR driver remaps `/scan` to `scan_raw`; Pro laser filter outputs `scan` | `vendor/src_ros2/src/peripherals`, `slam`, `navigation` | This unit is rear-blind and physically rotated; no-backup Nav2 behavior is a safety response |
| Deptrum Aurora 930 depth camera | USB camera device | `depth_cam/rgb0/image_raw`, `depth_cam/depth0/image_raw`, `depth_cam/ir0/image_raw`, `depth_cam/depth0/points`, camera info topics | `peripherals/launch/include/aurora930.launch.py`; consumers in vision scripts | Camera is mounted on the arm, so the arm pose controls view direction |
| Depth-derived obstacle hint | Camera stream, no separate device | `/depth_cam/depth0/image_raw` -> `/depth_scan` synthetic `LaserScan` | `added/robot_scripts/depth_guard.py`; modified Nav2 local costmap config | Added to avoid CPU-heavy full pointcloud use |
| WonderEcho Pro audio | ALSA card, `plughw:0,0` in live notes | Audio capture/playback for vendor and added voice flows | `xf_mic_asr_offline`, added assistant scripts | Do not start always-on mic flows without explicit operator intent |
| WonderEcho Pro serial wake/control | `/dev/ring_mic` -> `ttyCH341USB1` | vendor wake/ASR control protocol | `vendor/src_ros2/src/xf_mic_asr_offline` | udev rule is pinned to physical USB port because adapters lack stable serials |
| HX-12H bus servos and arm | Via STM32 on `/dev/rrc` | `ros_robot_controller/bus_servo/set_position`, `bus_servo/set_state`, `bus_servo/get_state`; higher layers in servo/kinematics/openclaw packages | `ros_robot_controller`, `servo_controller`, `kinematics`, `openclaw_controller` | Safe forward pose is used before driving; camera is arm-mounted |
| Battery monitor | Via STM32 on `/dev/rrc` | `ros_robot_controller/battery` (`UInt16`) | `ros_robot_controller` | Field guidance treats low voltage as an operational stop condition |
| OLED, LEDs, buzzer, button | Via STM32 on `/dev/rrc` | `ros_robot_controller/set_oled`, `set_led`, `set_rgb`, `set_buzzer`, `button` | `ros_robot_controller` | Useful for status and debugging |
| PS2 wireless controller | Receiver through STM32 path | joystick/control stack | vendor controller/peripherals stack | Button map is noted in `added/guides/OPERATING.md` |

## Critical Safety Constraints

- The robot has no rear sensing coverage sufficient for safe reverse motion. Do not re-enable Nav2 backup behavior without adding verified rear obstacle sensing.
- The robot has no documented cliff/stair sensing or bumper safety layer. Treat autonomous operation near stairs, drop-offs, glass, thin obstacles, cables, or people as unsafe without supervision.
- The web video server and rosbridge are documented as unauthenticated in `docs/SYSTEM.md`. Keep the robot on a trusted network.
- The arm-mounted camera can point away from the driving direction. Scripts that rely on depth/camera data must set or verify the arm pose.
- Battery voltage should be checked before and after sessions. `docs/HARDWARE.md` records field guidance to wrap up below roughly 11000 mV.

## udev And Device Naming

The robot depends on stable device names:

| Stable name | Backing device | Rule source | Risk |
|---|---|---|---|
| `/dev/rrc` | STM32 `ttyACM0` | `vendor/system/etc/udev/rules.d/99-ttyACM0.rules` | Serial contention if two stacks run |
| `/dev/lidar` | CH340 `ttyCH341USB0` | `vendor-modified/etc/udev/rules.d/lidar.rules` | Physical-port pinning breaks if replugged |
| `/dev/ring_mic` | CH340 `ttyCH341USB1` | `vendor-modified/etc/udev/rules.d/xf_mic.rules` | Physical-port pinning breaks if replugged |

The lidar and mic rules were modified because factory port paths were wrong for this physical unit. See `docs/MODIFICATIONS.md` for proof and backups.

## Hardware-To-Software Debug Pattern

When a hardware feature fails, debug from the bottom upward:

1. Confirm the Linux device exists, for example `/dev/rrc`, `/dev/lidar`, or `/dev/ring_mic`.
2. Confirm only one process owns serial devices, especially `/dev/rrc`.
3. Confirm the ROS node responsible for the device is running.
4. Confirm the expected topic/service exists.
5. Confirm data is flowing at a plausible rate.
6. Only then debug higher-level planners, scripts, or model clients.

This ordering prevents treating device or launch failures as navigation, ASR, or AI behavior bugs.

## Source-Backed Interface Notes

### STM32 bridge

`ros_robot_controller/ros_robot_controller_node.py` creates the low-level ROS surface. The node starts `Board()`, calls `enable_reception()`, stops motors at startup, loads servo offsets from `/home/ubuntu/software/Servo_upper_computer/servo_config.yaml`, and polls IMU/joy/SBUS/button/battery in a background thread.

The serial protocol is implemented in `ros_robot_controller_sdk.py`: `/dev/rrc` is opened at 1 Mbaud, frames start with `0xAA 0x55`, then carry function id, data length, payload, and CRC8. Function ids map to system, LED, buzzer, motor, PWM servo, bus servo, key, IMU, gamepad, SBUS, and OLED. If logs show `Validation failed`, the CRC check rejected a frame.

The default private-topic names are resolved under the node name:

| Direction | Interfaces |
|---|---|
| Hardware to ROS | `ros_robot_controller/imu_raw`, `ros_robot_controller/joy`, `ros_robot_controller/sbus`, `ros_robot_controller/button`, `ros_robot_controller/battery` |
| ROS to hardware | `ros_robot_controller/set_motor`, `set_led`, `set_rgb`, `set_buzzer`, `set_oled`, `enable_reception`, `bus_servo/set_position`, `bus_servo/set_state`, `pwm_servo/set_state` |
| Services | `ros_robot_controller/bus_servo/get_state`, `ros_robot_controller/pwm_servo/get_state`, `ros_robot_controller/init_finish` |

### Drive and odometry

`controller/odom_publisher_node.py` subscribes to `controller/cmd_vel`, `cmd_vel`, and `app/cmd_vel`. The `cmd_vel` callback clamps x/y to +/-0.2 m/s and yaw to +/-0.5 rad/s before forwarding to the raw path. The raw path chooses mecanum behavior from `MACHINE_TYPE` and publishes `ros_robot_controller/set_motor`.

The same node publishes `odom_raw`; `controller/launch/controller.launch.py` starts `robot_localization/ekf_node`, remapping `odometry/filtered` to `odom`.

For the mecanum variant, the wheel mixer converts linear x, linear y, and yaw rate into four motor rps commands. Motors 3 and 4 are negated after the kinematic calculation, matching the physical motor orientation. The EKF fuses x/y/yaw-rate velocity from `odom_raw` with yaw/yaw-rate from filtered `imu`; it does not fix the source odometry issue where sideways strafe does not integrate into pose.

### Lidar

`peripherals/launch/lidar.launch.py` chooses the concrete lidar driver from `LIDAR_TYPE`. For this robot, `LIDAR_TYPE=SCLIDAR`, so `launch/include/sclidar.launch.py` starts `sclidar_ros2/sclidar` on `/dev/lidar` at 230400 baud. The driver remaps `/scan` to `scan_raw`; on Pro machines, `lidar.launch.py` adds a `laser_filters` scan-to-scan chain from `scan_raw` to `scan`.

### Depth camera and depth guard

`peripherals/launch/depth_camera.launch.py` chooses the Aurora launch when `DEPTH_CAMERA_TYPE=aurora`. `launch/include/aurora930.launch.py` remaps vendor driver topics to `depth_cam/rgb0/image_raw`, `depth_cam/depth0/image_raw`, `depth_cam/ir0/image_raw`, `depth_cam/depth0/points`, and camera-info topics.

`added/robot_scripts/depth_guard.py` subscribes to `/depth_cam/depth0/image_raw`, filters by approximate height/range, and publishes `/depth_scan` as a sparse 64-beam `LaserScan` in `base_footprint` at up to 3 Hz. The modified `nav2_params.yaml` includes `depth_scan` as a local-costmap observation source.

### IMU

The STM32 bridge publishes `ros_robot_controller/imu_raw`. `peripherals/launch/imu_filter.launch.py` runs `imu_calib/apply_calib`, remapping `raw` to `ros_robot_controller/imu_raw` and `corrected` to `imu_corrected`, then runs `imu_filter_madgwick`, remapping `imu/data_raw` to `imu_corrected` and `imu/data` to `imu`.

### Arm and servo layers

There are two practical levels for arm control:

| Level | Interface | Use |
|---|---|---|
| Low-level bus servo | `ros_robot_controller/bus_servo/set_position`, `bus_servo/set_state`, `bus_servo/get_state` | Direct pulse-level servo control through STM32 |
| Higher-level servo controller | `servo_controller`, `joint_controller`, `controller_manager/joint_states`, `controller_manager/servo_states`, trajectory actions | Joint-oriented control, action groups, and kinematics/openclaw integration |

The direct bus-servo path is what many added scripts use for safe forward arm poses. The higher-level `servo_controller` path converts pulse/radian/degree commands, clamps pulses to 0-1000, and republishes to the low-level bus-servo topic. The conversion assumes 1000 ticks over 240 degrees.

The kinematics node runs as `/kinematics` and provides services such as `/kinematics/set_pose_target`, `/kinematics/get_current_pose`, and `/kinematics/set_joint_value_target`. It watches `/controller_manager/servo_states`, solves arm poses for servo IDs 1-5, and leaves the gripper outside the IK solve. `openclaw_controller` builds on this layer for track-and-grab workflows, while many added scripts still use direct pulse-level commands for predictable camera poses.

Source review found likely bugs in the trajectory action controller path, so for debugging prefer confirming the simpler `servo_controller` topic and low-level bus-servo topics before relying on `follow_joint_trajectory`.

### Gamepad and keyboard

The PS-style gamepad is read by the STM32 and published by the bridge as `ros_robot_controller/joy`. `peripherals/joystick_control.py` consumes that topic and publishes `controller/cmd_vel`. On the mecanum robot:

| Control | Effect |
|---|---|
| Left stick x | Strafe via `linear.y` |
| Left stick y | Forward/back via `linear.x` |
| Right stick x | Turn via `angular.z` |
| START | Short buzzer beep |

Keyboard teleop uses `w/a/s/d` and also publishes raw `controller/cmd_vel`. Because both manual input paths use the raw topic, keep the robot clear and supervised during testing.

### Voice and scene demos

The vendor offline voice stack has two hardware paths: `/dev/ring_mic` serial wake/control for `awake_node.py`, and audio capture/playback through the WonderEcho/iFlytek libraries. Demo launches combine `mic_init.launch.py` with task nodes such as `voice_control_move.py`, navigation, color tracking, arm control, sorting, and garbage classification. Movement demos can publish raw `/controller/cmd_vel`, so treat them like manual/raw-drive tools.

`openclaw_controller` scene launches combine the same arm/camera/base hardware in larger workflows. `robot_base_control.launch.py` starts controller, depth camera, lidar, joystick control, `track_and_grab`, kinematics, and init pose. `start.launch.py` adds large-model vocal detect, TTS, `voice_openclaw`, and optional YOLO. `track_and_grab` uses `/servo_controller`, `/kinematics/get_current_pose`, and `/kinematics/set_pose_target`.

## Open Work

`docs/TASKS.md` task DOC-014 should still collect live validation notes and lower-priority edge cases. The core controller, lidar, camera, depth-guard, IMU, joystick, kinematics, first-pass servo, voice, and openclaw paths above are source-backed.
