# Code Walkthrough

This document is the landing page for package and script-level explanations. It is intentionally started as a structured index: detailed package cards should be filled by the local-model tasks in `docs/TASKS.md` and reviewed before being treated as authoritative.

Use this format for each package or script group:

```md
## Name

**Purpose:** What behavior this code owns.
**Where:** Source paths.
**Entry points:** Launch files, scripts, nodes, or commands.
**Main inputs:** Topics, services, actions, files, environment variables.
**Main outputs:** Topics, services, actions, files, hardware effects.
**Hardware touched:** Physical devices affected, if any.
**Important implementation notes:** Non-obvious choices and constraints.
**Failure modes / gotchas:** How it breaks and what symptoms look like.
**Source paths:** Paths used to verify the section.
```

## Current Coverage

| Area | Status | Task |
|---|---|---|
| `driver/ros_robot_controller` | Started | DOC-001 |
| `driver/controller` | Started | DOC-002 |
| `navigation` | Started | DOC-003 |
| `slam` | Started | DOC-004 |
| `peripherals` | Started | DOC-005 |
| `large_models` and `large_models_examples` | Started | DOC-006 |
| `xf_mic_asr_offline` | Started | DOC-007 |
| `openclaw_controller` | Started | DOC-008 |
| Added navigation scripts | Started | DOC-009 |
| Added exploration scripts | Started | DOC-010 |
| Added perception/arm scripts | Started | DOC-011 |
| Added voice/LLM scripts | Started | DOC-012 |
| Added tour/orchestration scripts | Started | DOC-013 |
| Nav2/SLAM config explanations | Queued | DOC-015 |

## Initial Package Map

This table is copied at the behavior level from `docs/VENDOR_STACK.md` and should be replaced or expanded by reviewed package cards.

| Package area | High-level purpose |
|---|---|
| `driver/ros_robot_controller` | ROS-to-STM32 serial bridge for motors, servos, IMU, battery, LEDs, buzzer, OLED, buttons |
| `driver/controller` | Velocity handling, mecanum/Ackermann paths, odometry, EKF-related launch/config |
| `driver/kinematics` and `driver/servo_controller` | Arm inverse kinematics, bus/PWM servo management, action groups |
| `bringup` | Factory top-level launch for the running robot stack |
| `peripherals` | Lidar, camera, IMU filtering, joystick, keyboard teleop, camera launch support |
| `slam` | Mapping launches and map storage |
| `navigation` | Nav2 and TEB navigation launch/config |
| `app` and `example` | Vendor phone-app features and teaching demos |
| `large_models` and `large_models_examples` | Vendor LLM/VLM/ASR/TTS framework and demos |
| `xf_mic_asr_offline` | Vendor iFlytek offline ASR and voice control demos |
| `openclaw_controller` | Gripper/arm scene applications and local gateway behavior |

## `driver/ros_robot_controller`

**Purpose:** This is the central ROS-to-STM32 bridge. It converts ROS messages into Hiwonder board SDK calls and publishes low-level hardware state back into ROS.

**Where:** `vendor/src_ros2/src/driver/ros_robot_controller` and `vendor/src_ros2/src/driver/ros_robot_controller_msgs`.

**Entry points:** `setup.py` registers `ros_robot_controller = ros_robot_controller.ros_robot_controller_node:main`. `launch/ros_robot_controller.launch.py` starts that executable with the `imu_frame` parameter. The node name is `ros_robot_controller`.

**Main inputs:** Private node subscriptions expand to the public `ros_robot_controller/...` namespace in the default launch:

| Input | Effect |
|---|---|
| `ros_robot_controller/set_motor` | Sets motor speeds on the STM32 board |
| `ros_robot_controller/set_led` | Controls LED state |
| `ros_robot_controller/set_rgb` | Controls RGB pixel states |
| `ros_robot_controller/set_buzzer` | Controls buzzer frequency/timing |
| `ros_robot_controller/set_oled` | Writes OLED text |
| `ros_robot_controller/enable_reception` | Enables/disables board reception and sensor polling |
| `ros_robot_controller/bus_servo/set_position` | Sends bus-servo position commands |
| `ros_robot_controller/bus_servo/set_state` | Changes bus-servo ID, position, offset, limits, torque, saved offset, or stop state |
| `ros_robot_controller/pwm_servo/set_state` | Sets PWM servo position or offset |

**Main outputs:** `ros_robot_controller/imu_raw`, `ros_robot_controller/joy`, `ros_robot_controller/sbus`, `ros_robot_controller/button`, and `ros_robot_controller/battery`. The polling thread publishes these only while reception is enabled.

**Services:** `ros_robot_controller/bus_servo/get_state`, `ros_robot_controller/pwm_servo/get_state`, and `ros_robot_controller/init_finish`.

**Hardware touched:** STM32 controller over `/dev/rrc`; drive motors; IMU; battery monitor; bus servos; PWM servos; LEDs; RGB LEDs; buzzer; OLED; buttons; SBUS/gamepad state.

**Important implementation notes:** The node constructs `Board()` from `ros_robot_controller_sdk.py`, calls `enable_reception()` at startup, loads PWM servo offsets from `/home/ubuntu/software/Servo_upper_computer/servo_config.yaml`, stops all four motors during initialization, and publishes sensor state in a daemon thread at roughly 50 Hz. IMU acceleration is converted to m/s^2 using 9.80665 and angular velocity is converted from degrees to radians.

**Failure modes / gotchas:** If `/dev/rrc` is missing or already owned by another process, most of the robot appears dead above this layer. If `enable_reception` is false, commands can still be sent but sensor/battery/button publications stop. `docs/SYSTEM.md` notes duplicate stacks can contend for `/dev/rrc`.

**Source paths:** `ros_robot_controller/ros_robot_controller_node.py`, `launch/ros_robot_controller.launch.py`, `setup.py`, `docs/SYSTEM.md`, `docs/HARDWARE.md`.

**Message surface notes:** `ros_robot_controller_msgs` defines the wire-facing ROS message shapes used here: `MotorState`/`MotorsState`, `ServosPosition`/`ServoPosition`, bus/PWM servo state and command messages, `ButtonState`, `BuzzerState`, `LedState`, `OLEDState`, `RGBState(s)`, `Sbus`, plus `GetBusServoState` and `GetPWMServoState` services. The bridge mostly translates these messages one-for-one into `Board` SDK calls.

**Serial protocol notes:** `ros_robot_controller_sdk.py` opens `/dev/rrc` at 1,000,000 baud. Frames use start bytes `0xAA 0x55`, then function id, length, data, and CRC8 over the function/length/data bytes. Function ids cover system, LED, buzzer, motor, PWM servo, bus servo, key, IMU, gamepad, SBUS, and OLED. The receive thread parses frames into one-item queues by function; higher-level getters (`get_imu`, `get_gamepad`, `get_sbus`, `get_battery`) drain those queues only when reception is enabled. Motor commands use the motor function with subcommand `0x01`, motor count, then `(id-1, rps)` pairs. Bus servo position commands use subcommand `0x01`, duration in milliseconds, count, then `(id, pulse)` pairs. This is the actual low-level boundary below ROS.

## `driver/controller`

**Purpose:** This package converts velocity commands into motor commands, publishes raw odometry, launches IMU filtering/EKF integration, and starts the servo controller stack.

**Where:** `vendor/src_ros2/src/driver/controller`.

**Entry points:** `setup.py` registers `odom_publisher = controller.odom_publisher_node:main` and `init_pose = controller.init_pose:main`. `launch/odom_publisher.launch.py` starts robot description, the `ros_robot_controller` launch, and the `odom_publisher` node. `launch/controller.launch.py` includes odom publisher, `peripherals/launch/imu_filter.launch.py`, `robot_localization` EKF, and `servo_controller.launch.py`.

**Main inputs:**

| Input | Behavior |
|---|---|
| `controller/cmd_vel` | Raw velocity path into `cmd_vel_callback` |
| `cmd_vel` | Clamped path: x/y limited to +/-0.2 m/s and yaw to +/-0.5 rad/s before forwarding |
| `app/cmd_vel` | Ackermann/app path; can command PWM steering behavior before forwarding |
| `set_odom` | Attempts to reset odometry pose |

**Main outputs:** `ros_robot_controller/set_motor`, `ros_robot_controller/pwm_servo/set_state`, `odom_raw`, and `set_pose`. The EKF launch remaps `odometry/filtered` to `odom` and remaps EKF `cmd_vel` to `controller/cmd_vel`.

**Hardware touched:** Drive motors through `ros_robot_controller/set_motor`; PWM steering/servo path for Ackermann-style behavior.

**Important implementation notes:** `odom_publisher_node.py` chooses mecanum, differential, tank, or Ackermann behavior from `MACHINE_TYPE`. On this robot, `MACHINE_TYPE=ROSOrin_Mecanum_Pro`, so linear x, linear y, and angular z are passed to `mecanum.MecanumChassis.set_velocity()`. Odometry integration publishes at about 50 Hz.

**Alternate chassis branches:** The same node contains vendor support for other `MACHINE_TYPE` values. If the type contains `Differential`, `linear.y` is forced to zero and the mecanum helper is reused as the wheel-speed mapper. If the type is `ROSOrin_Acker`, `linear.y` is discarded, `ackermann.py` computes rear wheel speeds plus a steering PWM target, and `acker_cmd_vel_callback()` can steer PWM servo ID 1 in place when `linear.x == 0`. The Ackermann helper clamps steering to about +/-34 degrees and maps steering around a 1500 us-style neutral PWM position.

**Failure modes / gotchas:** The odometry integrator updates x/y position from `linear_x` and yaw only; `linear_y` is published in the twist but does not contribute to pose integration. This explains the known strafe drift note in `added/guides/OPERATING.md`. The `set_odom` callback calls `self.clock().now()` even though `self.clock` is assigned as an object, so the operating guide warns that publishing `set_odom` crashes the node. `load_calibrate_param` reads ROS1-style `~linear_correction_factor` names after declaring non-tilde parameters, so the operating guide marks it broken.

**Source paths:** `controller/odom_publisher_node.py`, `controller/mecanum.py`, `controller/ackermann.py`, `launch/controller.launch.py`, `launch/odom_publisher.launch.py`, `setup.py`, `added/guides/OPERATING.md`.

**Mecanum math:** `mecanum.py` computes wheel linear speeds as:

| Motor | Linear speed before rps conversion |
|---|---|
| 1 | `linear_x - linear_y - angular_z * (wheelbase + track_width) / 2` |
| 2 | `linear_x + linear_y - angular_z * (wheelbase + track_width) / 2` |
| 3 | negated after `linear_x + linear_y + angular_z * (wheelbase + track_width) / 2` |
| 4 | negated after `linear_x - linear_y + angular_z * (wheelbase + track_width) / 2` |

The node instantiates the mecanum model with wheelbase `0.17706`, track width `0.17165`, and wheel diameter `0.08`. `calibrate_params.yaml` sets `linear_correction_factor: 0.94` and `angular_correction_factor: 1.00`. The EKF config fuses velocity x/y/yaw-rate from `odom_raw` and yaw/yaw-rate from filtered `imu`, with `two_d_mode: true`, output frame `odom`, and base frame `base_footprint`.

## `navigation`

**Purpose:** This package launches Nav2 localization/navigation around saved maps and selects the Nav2 controller configuration.

**Where:** Factory source is `vendor/src_ros2/src/navigation`; live user-modified config copies are in `vendor-modified/ros2_ws/src/navigation/config`.

**Entry points:** `launch/navigation.launch.py` is the high-level launch. It resolves paths through `need_compile`, includes `slam/launch/include/robot.launch.py` as the base robot launch, waits 10 seconds, then includes `navigation/launch/include/bringup.launch.py` with the selected map and `config/nav2_params.yaml`. `launch/include/navigation_base.launch.py` loads Nav2 composable nodes for controller, smoother, planner, behavior server, BT navigator, waypoint follower, velocity smoother, and lifecycle manager.

**Main inputs:** Saved map YAML selected by `map:=...`; lidar scan `/scan`; localization transforms; Nav2 action goals; modified `nav2_params.yaml`; TEB or DWB controller config selected by `use_teb`.

**Main outputs:** Nav2 command stream through `cmd_vel_nav` and smoothed `cmd_vel`; plans, costmaps, lifecycle-managed navigation services/actions.

**Hardware touched:** Indirectly controls the drive base through the controller stack. Uses lidar and, in the modified local costmap, `/depth_scan` from `depth_guard.py`.

**Important implementation notes:** The factory `navigation_base.launch.py` rewrites `default_nav_to_pose_bt_xml` to `navigate_to_pose_minimal.xml`. The live modified config in `vendor-modified` explicitly sets both `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` to no-backup behavior trees. `docs/MODIFICATIONS.md` records this as intentional because the robot is rear-blind.

**Failure modes / gotchas:** If only one Nav2 behavior-tree path is changed, the other navigation pipeline can still use backup behavior. The modified local costmap expects `/depth_scan`; if `depth_guard.py` is not running, lidar still exists but thin forward obstacles from depth may not enter that observation source. Navigation should be treated as supervised and not complete; see `docs/ADDED_SOFTWARE.md`.

**Source paths:** `launch/navigation.launch.py`, `launch/include/navigation_base.launch.py`, `vendor-modified/ros2_ws/src/navigation/config/nav2_params.yaml`, `vendor-modified/ros2_ws/src/navigation/config/navigate_*_no_backup.xml`, `docs/MODIFICATIONS.md`.

**Modified Nav2 config notes:** The modified `nav2_params.yaml` keeps AMCL on `base_footprint`, `/scan`, and `nav2_amcl::DifferentialMotionModel`. Local costmap is a 3 m rolling window at 5 Hz update / 2 Hz publish, with `robot_radius: 0.08`, inflation radius `0.15`, and voxel observations from both `/scan` and `/depth_scan`. The `/depth_scan` source is a `LaserScan` with 0.25-2.0 m obstacle range, 0.25-2.5 m raytrace range, and expected 3 Hz updates. Global costmap uses `/scan` only, map frame, 0.05 m resolution, `track_unknown_space: true`, and inflation radius `0.2`. The planner is `nav2_navfn_planner/NavfnPlanner` with `allow_unknown: true`.

**No-backup behavior trees:** `navigate_to_pose_no_backup.xml` and `navigate_through_poses_no_backup.xml` retain replanning, costmap clears, spin, and wait recoveries. They remove the standard Nav2 `BackUp` recovery action. The through-poses tree replans at 0.333 Hz and removes passed goals within radius `0.7`; the single-pose tree replans at 1 Hz.

**TEB/rotation-shim notes:** `nav2_controller_teb.yaml` wraps TEB in `nav2_rotation_shim_controller::RotationShimController`, using TEB as `primary_controller`. It sets `controller_frequency: 6.0`, goal tolerance `0.15 m` / `0.1 rad`, `max_vel_x: 0.25`, `max_vel_x_backwards: 0.05`, `max_vel_theta: 2.0`, and strong forward-drive/non-holonomic weights (`weight_kinematics_nh: 1000.0`, `weight_kinematics_forward_drive: 1000.0`). The reverse velocity is not zero, but it is capped to 5 cm/s and heavily discouraged; the behavior tree removes explicit backup recovery.

## `slam`

**Purpose:** This package launches mapping flows and stores map assets used by navigation.

**Where:** Factory source is `vendor/src_ros2/src/slam`; live user-modified `slam.yaml` is in `vendor-modified/ros2_ws/src/slam/config`; added map copies are in `added/robot_maps`.

**Entry points:** `setup.py` registers `map_save = slam.map_save:main`. `launch/slam.launch.py` is the high-level mapping launch. It includes the robot base launch, then after 10 seconds starts one mapping mode selected by `slam_method`: `slam_toolbox`, `cartographer`, `gmapping`, or `rrt_explorer`. `launch/rtabmap_slam.launch.py` handles the RTAB-Map mapping path.

**Main inputs:** `/scan`, odometry/TF, optional camera inputs for RTAB-Map flows, launch arguments such as `slam_method`, `enable_save`, `sim`, and namespace/robot names.

**Main outputs:** Map data, map save service support, and saved map YAML/PGM files under the robot's `slam/maps` path.

**Hardware touched:** Indirectly consumes lidar, odometry, and camera data. It does not command actuators by itself except when paired with exploration flows.

**Important implementation notes:** The top-level launch computes frame names from `robot_name`, pushing a namespace when needed. `docs/MODIFICATIONS.md` records user retuning of `slam.yaml` to reduce CPU load on the Jetson.

**Failure modes / gotchas:** Mapping quality depends on localization, scan coverage, odometry quality, CPU load, and careful driving. Existing map files are snapshots of particular sessions; do not assume every map is current or equally reliable. `map_04` is documented as the current upstairs nav map in `docs/MODIFICATIONS.md`.

**Source paths:** `launch/slam.launch.py`, `launch/rtabmap_slam.launch.py`, `slam/map_save.py`, `vendor-modified/ros2_ws/src/slam/config/slam.yaml`, `added/robot_maps`, `docs/MODIFICATIONS.md`.

**Modified SLAM config notes:** The user-modified `slam.yaml` keeps `slam_toolbox` in mapping mode with `scan_topic: scan`, `odom_frame: odom`, `map_frame: map`, `base_frame: base_footprint`, `inverted_laser: true`, and `use_map_saver: true`. Compared to the factory copy, it reduces CPU pressure by changing `throttle_scans` from `1` to `3`, `transform_publish_period` from `0.01` to `0.05`, `map_update_interval` from `0.1` to `2.0`, and `minimum_travel_distance` from `0.0` to `0.1`.

**Mapping modes:** `slam.launch.py` always includes `launch/include/robot.launch.py` first, then waits 10 seconds before starting the selected mapping mode. `slam_toolbox` uses `include/slam_base.launch.py`, rewrites frame/scan parameters into `config/slam.yaml`, starts `sync_slam_toolbox_node`, and remaps `/slam_toolbox/save_map` to `/save_map` when saving is enabled. `cartographer` starts `cartographer_node` plus `cartographer_occupancy_grid_node` with `cartographer_2d.lua`. `gmapping` rewrites frame parameters into `config/gmapping.yaml` and starts `slam_gmapping`. `rrt_explorer` starts a Nav2-style exploration/navigation group with `use_teb=true`.

**RTAB-Map flow:** `rtabmap_slam.launch.py` is separate from the `slam_method` switch. It includes the same robot base launch with action pose `horizontal`, then starts `include/rtabmap.launch.py` after 10 seconds. The RTAB include starts `rtabmap_sync/rgbd_sync` and `rtabmap_slam/rtabmap`, subscribes to scan input, remaps `scan` to `/scan`, and uses `base_footprint` as `frame_id`. The top-level file computes namespaced camera/odom/scan variables but only passes `use_sim_time` into the include, so the actual RTAB include currently uses its own fixed remaps.

## `peripherals`

**Purpose:** This package launches and wraps sensors and operator input devices: lidar, depth camera, IMU filtering, joystick, keyboard teleop, and camera support.

**Where:** `vendor/src_ros2/src/peripherals`.

**Entry points:** `setup.py` registers `joystick_control`, `tf_broadcaster_imu`, and `teleop_key_control`. Important launches are `lidar.launch.py`, `depth_camera.launch.py`, `imu_filter.launch.py`, `joystick_control.launch.py`, `teleop_key_control.launch.py`, and `usb_cam.launch.py`.

**Main inputs:** Environment variables `LIDAR_TYPE`, `DEPTH_CAMERA_TYPE`, `MACHINE_TYPE`, and `need_compile`; raw hardware devices such as `/dev/lidar`; IMU raw data from `ros_robot_controller/imu_raw`; keyboard/joystick input.

**Main outputs:**

| Area | Outputs |
|---|---|
| Lidar | `scan_raw` from the selected lidar driver, filtered `scan` through `laser_filters` on Pro machines |
| Aurora camera | `depth_cam/rgb0/image_raw`, `depth_cam/depth0/image_raw`, `depth_cam/ir0/image_raw`, `depth_cam/depth0/points`, camera info topics |
| IMU | `imu_corrected` then filtered `imu` |
| Keyboard teleop | `controller/cmd_vel`; PWM servo state commands |

**Hardware touched:** Lidar, depth camera, IMU data path, joystick/keyboard input path.

**Important implementation notes:** `lidar.launch.py` selects an include launch by `LIDAR_TYPE`; for `SCLIDAR`, `include/sclidar.launch.py` runs `sclidar_ros2/sclidar` on `/dev/lidar` at 230400 baud and remaps `/scan` to `scan_raw`. On Pro machines, `lidar.launch.py` adds a `laser_filters` `scan_to_scan_filter_chain` from `scan_raw` to `scan`. Other supported include paths cover `LD19`, `MS200`, `A1`, `G4`, and `STL06N` style drivers; most use `/dev/lidar`, while the MS200 include uses `/dev/ldlidar`. Non-Pro machines skip the filter node and map the driver directly to the public scan topic. `depth_camera.launch.py` selects `include/aurora930.launch.py` when `DEPTH_CAMERA_TYPE=aurora`; that launch remaps the vendor driver outputs into the `depth_cam/...` topic names. If `DEPTH_CAMERA_TYPE=ascamera`, it starts `ascamera/ascamera_node` at 640x480/15 FPS and remaps to the same `depth_cam/...` topic family. Any other value falls back to `usb_cam.launch.py`, which only provides RGB topics under `/depth_cam/rgb0/...` plus a static transform. `imu_filter.launch.py` applies calibration from `calibration/config/imu_calib.yaml` and then runs `imu_filter_madgwick`.

**Failure modes / gotchas:** If the `need_compile` or device environment is wrong, launch path selection can point at the wrong package tree or driver. If `/dev/lidar` is missing, the SCLIDAR node cannot produce scans. The Aurora launch defaults `point_cloud_enable` to true, but added documentation records that full pointcloud use can starve CPU in navigation workflows.

**Source paths:** `launch/lidar.launch.py`, `launch/include/*.launch.py`, `config/lidar_filters_config_*.yaml`, `launch/depth_camera.launch.py`, `launch/usb_cam.launch.py`, `launch/imu_filter.launch.py`, `peripherals/teleop_key_control.py`, `setup.py`.

## `driver/servo_controller` and `driver/kinematics`

**Purpose:** These packages form the higher-level arm/servo layer above `ros_robot_controller`. `servo_controller` accepts pulse/radian/degree servo targets and exposes joint state/trajectory surfaces. `kinematics` provides arm pose services and demos around inverse/forward kinematics.

**Where:** `vendor/src_ros2/src/driver/servo_controller`, `vendor/src_ros2/src/driver/kinematics`, and related message packages.

**Entry points:** `driver/controller/launch/controller.launch.py` includes `servo_controller/launch/servo_controller.launch.py`. `servo_controller` code starts `controller_manager`, which creates a `ServoManager`, joint position controllers for configured joints, arm/gripper trajectory action controllers, and publishers for joint/servo state.

**Main inputs:** `servo_controller` topic with `servo_controller_msgs/ServosPosition`; `joint_controller` with `sensor_msgs/JointState`; trajectory actions under `arm_controller/follow_joint_trajectory` and `gripper_controller/follow_joint_trajectory`; lower-level bus-servo services from `ros_robot_controller`.

**Main outputs:** `controller_manager/joint_states`, `controller_manager/servo_states`, and lower-level `ros_robot_controller/bus_servo/set_position`.

**Hardware touched:** HX-12H bus servos in the arm and gripper. The code maps joint names such as `joint1` through `joint5` and `r_joint` onto configured servo IDs; action groups map the sixth action column to servo ID `10` for the gripper.

**Important implementation notes:** `JointPositionController` treats the servo pulse range as 0-1000 over 240 degrees and converts between pulses and radians relative to each joint's configured init/min/max values. `ServoManager.set_position()` clamps duration to 0.02-30 s and servo pulse to 0-1000 before publishing to the low-level bridge.

**Failure modes / gotchas:** `joint_trajectory_action_controller.py` appears to contain source bugs in the trajectory path (`self.self.joint_names`, `pos_rad_to_raw`, and `FollowJointTrajectoryResult` names that do not match imported symbols/functions in the shown file). Treat direct trajectory action use as needing live verification. The simpler `servo_controller` topic path is used by many scripts and examples.

**Source paths:** `driver/servo_controller/servo_controller/controller_manager.py`, `servo_manager.py`, `joint_position_controller.py`, `joint_trajectory_action_controller.py`, `action_group_controller.py`, `driver/controller/launch/controller.launch.py`, `docs/HARDWARE.md`.

**Kinematics service surface:** `kinematics/launch/kinematics_node.launch.py` starts `search_kinematics_solutions`. The node name is `kinematics`; private services therefore appear as `/kinematics/...`.

| Service | Purpose |
|---|---|
| `/kinematics/set_link` / `get_link` | Set/read link lengths used by IK/FK |
| `/kinematics/set_joint_range` / `get_joint_range` | Set/read allowed joint ranges in degrees |
| `/kinematics/set_pose_target` | Given target `[x, y, z]`, pitch, pitch range, and resolution, search IK and return target pulses, current pulses, RPY, and minimum variation |
| `/kinematics/get_current_pose` | Convert current servo pulses from `/controller_manager/servo_states` to a FK pose |
| `/kinematics/set_joint_value_target` | Given joint pulse values, compute FK pose |

`search_kinematics_solutions_node.py` chooses the IK solution with minimum total pulse movement from current servo positions. It only tracks servo IDs 1-5 from `/controller_manager/servo_states`; the gripper servo is outside that arm-pose solve.

`kinematics_control.py` is a request-building helper for those services, not a complete standalone CLI. Its sample `__main__` block references undefined names, so use the helper functions as examples for message shape rather than as a directly runnable program.

## `peripherals` joystick and keyboard control

**Purpose:** Operator input paths for manual driving.

**Where:** `vendor/src_ros2/src/peripherals/peripherals/joystick_control.py` and `teleop_key_control.py`.

**Joystick path:** `joystick_control.py` subscribes to `ros_robot_controller/joy`, maps axes into `controller/cmd_vel`, and publishes a short buzzer beep on START. For mecanum/differential machines, left stick x maps to `linear.y`, left stick y maps to `linear.x`, and right stick x maps to `angular.z`. Defaults are `max_linear: 0.2`, `max_angular: 2.0`, and a deadband of `0.1`. The underlying SDK maps PS-style buttons into a `sensor_msgs/Joy` array: cross index 0, circle 1, square 3, triangle 4, L1/R1 6/7, L2/R2 axes 5/4, select/start 10/11, and D-pad into axes 6/7.

**Keyboard path:** `teleop_key_control.py` publishes directly to `controller/cmd_vel`: `w` forward, `s` backward, `a` left turn, `d` right turn. On non-Ackermann machines it uses `LIN_VEL=0.2` and `ANG_VEL=0.5`. It publishes a stop command on exit.

**Failure modes / gotchas:** Both manual paths publish to raw `controller/cmd_vel`, not the clamped `cmd_vel` topic. Joystick max angular speed is higher than the keyboard path by default.

## `large_models` and `large_models_examples`

**Purpose:** `large_models` is the vendor LLM/VLM/ASR/TTS framework. `large_models_examples` contains demo behaviors built on that framework.

**Where:** `vendor/src_ros2/src/large_models`, `vendor/src_ros2/src/large_models_examples`, and user-modified copies under `vendor-modified/ros2_ws/src/large_models`.

**Entry points:** `large_models/large_models/setup.py` registers `vocal_detect`, `agent_process`, and `tts_node`. `launch/start.launch.py` starts `agent_process` first, then after 10 seconds starts `vocal_detect` and `tts_node`. `launch/agent_process.launch.py`, `vocal_detect.launch.py`, and `tts_node.launch.py` can also be used separately.

**Main inputs:** ASR text from `vocal_detect/asr_result`; optional camera topic passed to `agent_process`; service calls to set model, prompt, LLM/VLLM content, tools, and chat recording state; environment variables `ASR_MODE` and `ASR_LANGUAGE`; model/provider values in `large_models/config.py`.

**Main outputs:** `agent_process/result`, `agent_process/tools`, `vocal_detect/asr_result`, `vocal_detect/wakeup`, `vocal_detect/angle`, `tts_node/play_finish`, and audio playback through the TTS node.

**Important implementation notes:** The modified `config.py` points English/international LLM calls at LM Studio on `http://192.168.0.148:1234/v1` with `llm_api_key='lm-studio'`. `agent_process.py` has `enable_think = False` in the English LLM path to avoid slow qwen reasoning mode. If `ASR_MODE=offline`, `agent_process.py` selects `offline_llm` and an Ollama client instead of OpenAI-compatible HTTP.

**Failure modes / gotchas:** `docs/MODIFICATIONS.md` notes that the LM Studio config path is only used when the vendor flow is in online ASR mode; offline mode routes to local Ollama. Source review also found `launch/agent_process.launch.py` appending `ollama_server` to `z`, but no `z` is defined in that file. Treat the offline launch path as needing verification before relying on it.

**Source paths:** `large_models/large_models/setup.py`, `large_models/large_models/launch/start.launch.py`, `launch/agent_process.launch.py`, `large_models/large_models/large_models/config.py`, `vendor-modified/.../config.py`, `vendor-modified/.../agent_process.py`, `docs/MODIFICATIONS.md`.

**Examples package taxonomy:** `large_models_examples/setup.py` registers console scripts for direct movement (`llm_control_move`, `llm_control_move_offline`), color tracking (`llm_color_track`), visual patrol (`llm_visual_patrol`), VLM camera/track/navigation flows (`vllm_with_camera`, `vllm_track`, `vllm_navigation`), function calling (`llm_control`, `road_network_tool`), road-network navigation (`road_network_navigator`, `nav2_execution_node`), arm tracking (`vllm_track_arm`), transport/pick workflows (`automatic_pick`, `automatic_transport`, `vllm_navigation_transport`, `vllm_transport_dietitianl`), sorting/classification (`object_sorting`, `llm_object_sorting`, `waste_classification`, `llm_waste_classification`), and object transport (`object_transport`, `vllm_object_transport`).

Most example launches compose three layers: a base hardware/application launch such as controller, depth camera, lidar, navigation, or kinematics; `large_models/launch/start.launch.py` or its component vocal/agent/TTS launches; then one scenario node from `large_models_examples`. Examples that use `camera_topic` usually default to `/depth_cam/rgb0/image_raw`. Sorting, transport, tracking, and navigation examples can move the robot or arm, so treat them as full robot demos rather than passive model tests.

## `xf_mic_asr_offline`

**Purpose:** Vendor offline voice stack for the WonderEcho/iFlytek microphone path. It handles wake detection, offline ASR, and demo-specific voice commands.

**Where:** `vendor/src_ros2/src/xf_mic_asr_offline` and `vendor/src_ros2/src/xf_mic_asr_offline_msgs`.

**Entry points:** This package is CMake/script-based. `CMakeLists.txt` builds the C++ `voice_control` executable, installs Python scripts such as `awake_node.py`, `asr_node.py`, `voice_control_move.py`, and installs launch/config/feedback assets. `launch/mic_init.launch.py` starts the mic core. Demo launches such as `voice_control_move.launch.py`, `voice_control_navigation.launch.py`, and color/arm/garbage variants include mic setup plus task-specific nodes.

**Main inputs:** `/dev/ring_mic` serial wake/control path, ALSA audio through the vendor libraries, `/awake_node/awake_flag`, `/voice_control/get_offline_result`, and demo-specific perception/navigation topics.

**Main outputs:** `/awake_node/awake_flag`, `/awake_node/angle`, `/asr_node/voice_words`, `/voice_control/voice_words`, `/ros_robot_controller/set_buzzer`, and demo-specific commands such as `/controller/cmd_vel` in `voice_control_move.py`.

**Important implementation notes:** `awake_node.py` opens the mic serial port at 115200 and publishes wake flags/angles. `asr_node.py` waits for `/voice_control/get_offline_result`, listens for `/awake_node/awake_flag`, and publishes recognized words on its private `voice_words` topic. `voice_control_move.launch.py` brings up controller, lidar, mic initialization, `voice_control_move.py`, and `init_pose`.

**Demo launch behavior:**

| Launch | Includes / starts | Notes |
|---|---|---|
| `voice_control_move.launch.py` | Controller, lidar, mic init, `voice_control_move.py`, controller `init_pose` action `init` | Voice movement demo; commands raw `/controller/cmd_vel` through the script |
| `voice_control_navigation.launch.py` | Navigation stack, mic init, `voice_control_navigation.py`, RViz | Builds namespaced `cmd_vel`, `goal`, and `local_costmap/costmap` parameters from `robot_name`/`master_name` |
| `voice_control_navigation_transport.launch.py` | Example navigation-transport launch, mic init, `voice_control_navigation_transport.py` | Passes `map_name`, broadcast mode, a place pose, and a fixed `pick_position` |
| `voice_control_arm.launch.py` | Controller, mic init, `voice_control_arm.py` | Arm voice demo |
| `voice_control_color_detect.launch.py` | Controller, color-detect example, mic init, `voice_control_color_detect.py`, horizontal init pose | Starts color detection with display enabled |
| `voice_control_color_track.launch.py` | Color-track example with `start=false`, mic init, `voice_control_color_track.py` | Voice controls the task start/selection |
| `voice_control_color_sorting.launch.py` | Color-sorting example with `start=false`, `broadcast=true`, mic init, voice script | Sorting demo bridge |
| `voice_control_garbage_classification.launch.py` | Garbage-classification example with `start=false`, `broadcast=true`, mic init, voice script | Classification demo bridge |

**Command surface:** `config/call.bnf` contains the offline grammar. English aliases include `go forward`, `go backward`, `turn left`, `turn right`, `stop`, `drift`, `go to A/B/C point`, `go back to the start`, `come here`, `start/stop color recognition`, `track red/green/blue object`, `stop tracking`, `start/stop color sorting`, `sort waste`, `stop sort waste`, `pick a carrot`, `pass me please`, `navigate and transport`, `gripping the sphere/cylinder/cuboid`, and `stop gripping`.

| Script | ROS effects |
|---|---|
| `voice_control_move.py` | Publishes `/controller/cmd_vel`; forward/back commands run about 2 s at +/-0.2 m/s, turns run about 2 s, and `come here` uses wake angle then lidar follow on `/scan_raw` |
| `voice_control_navigation.py` | Publishes goal poses for A/B/C/start and can also publish to a configured raw `cmd_vel` topic |
| `voice_control_navigation_transport.py` | Calls `/navigation_transport/pick` with configured `pick_position` for `navigate and transport` |
| `voice_control_color_detect.py` | Calls `/color_detect/set_param` to start/stop red/green/blue circle detection and plays feedback for detected colors |
| `voice_control_color_track.py` | Calls `/color_track/start` and `/color_track/set_color` for red/green/blue tracking or stop |
| `voice_control_color_sorting.py` | Calls `/color_sorting/start` and `/color_sorting/stop` |
| `voice_control_garbage_classification.py` | Calls `/garbage_classification/start` and `/garbage_classification/stop` |
| `voice_control_arm.py` | Runs action groups `voice_pick` and `voice_give` through the servo action-group controller |

**Failure modes / gotchas:** If `/dev/ring_mic` is missing, wake/control behavior fails before the higher-level voice demos can work. `docs/MODIFICATIONS.md` records that the factory udev rule was wrong for this unit and was repinned. `voice_control_move.py` publishes `/controller/cmd_vel`, so movement demos use the raw velocity path.

**Source paths:** `CMakeLists.txt`, `config/call.bnf`, `launch/mic_init.launch.py`, `launch/voice_control_*.launch.py`, `scripts/awake_node.py`, `scripts/asr_node.py`, `scripts/voice_control_*.py`, `docs/MODIFICATIONS.md`, `docs/HARDWARE.md`.

## `openclaw_controller`

**Purpose:** Vendor gripper/arm scene package for voice-openclaw and track-and-grab style workflows.

**Where:** `vendor/src_ros2/src/openclaw_controller`.

**Entry points:** `setup.py` registers `voice_openclaw` and `track_and_grab`. Launch files include `start.launch.py`, `navigation_manager.launch.py`, `smart_factory.launch.py`, `robot_base_control.launch.py`, and `yolo_node.launch.py`.

**Main inputs:** `voice_openclaw.py` subscribes to `/vocal_detect/wakeup`, `/vocal_detect/asr_result`, `/tts_node/play_finish`, and `/voice_openclaw/feishu_enable`; `track_and_grab_node.py` uses kinematics services and exposes start/stop/color services.

**Main outputs:** Voice responses through `/tts_node/tts_text`; YOLO start service use; arm/gripper commands through `/servo_controller`; track-and-grab result image and pick-finish topics.

**Important implementation notes:** `track_and_grab_node.py` creates clients for `/kinematics/get_current_pose` and `/kinematics/set_pose_target`, publishes servo positions on `/servo_controller`, and exposes private `start`, `stop`, `set_color`, and `init_finish` services. `voice_openclaw.py` bridges voice/ASR events to TTS and scene behavior.

**Launch behavior:**

| Launch | Includes / starts | Notes |
|---|---|---|
| `start.launch.py` | `large_models` vocal detect, TTS node, `robot_base_control.launch.py`, `voice_openclaw`; optionally `yolo_node.launch.py` | Arguments `use_yolo`, `stream`, and `feishu_enable`; YOLO is off by default |
| `robot_base_control.launch.py` | Controller, depth camera, lidar, joystick control, `track_and_grab`, kinematics node, controller init pose | This is a broad hardware bringup for openclaw workflows; it starts arm, camera, lidar, and manual input paths |
| `navigation_manager.launch.py` | `track_and_grab`, navigation stack with `use_teb=true`, `large_models_examples` `navigation_controller`, RViz | Runs under `robot_name` namespace and delays main nodes by 3 s |
| `smart_factory.launch.py` | `track_and_grab`, kinematics node, `large_models_examples` road-network launch | Chooses installed package paths or `/home/ubuntu/ros2_ws/src/...` paths based on `need_compile` |
| `yolo_node.launch.py` | `example/yolo_node` | Defaults to camera topic `depth_cam/rgb0/image_raw`, confidence `0.60`, model `yolo26n`; switches to `yolov11n` if model arg contains `11` |

**Failure modes / gotchas:** `docs/MODIFICATIONS.md` records vendor drift in this area: the live image is a newer vendor refactor than the published source zip. Treat this repo snapshot as the source for the captured stack, but verify against the live robot before assuming behavior from older vendor material.

**Source paths:** `setup.py`, `openclaw_controller/voice_openclaw.py`, `openclaw_controller/track_and_grab_node.py`, `launch/start.launch.py`, `launch/navigation_manager.launch.py`, `docs/MODIFICATIONS.md`.

## Added Script Groups

The added scripts are documented at a summary level in `docs/ADDED_SOFTWARE.md`. Detailed cards should be added here after source review.

| Group | Scripts |
|---|---|
| Navigation/localization | `global_loc.py`, `scan_overlay.py`, `initpose.py`, `goto.py`, `goto_place.py`, `goal_pick.py` |
| Exploration/mapping | `explore.py`, `explore2.py`, `rtab_drive.py`, `rtab_drive04.py`, `rtabmap_map.launch.py` |
| Perception/arm | `vlook.py`, `look.py`, `capframe.py`, `depth_guard.py`, `arm_pose.py` |
| Voice/LLM | `assistant.py`, `llm_drive.py`, `voice_drive.py`, `calibrate_wake.py`, `debug_listen.py` |
| Tours/assignments | `tour*.sh`, `assignment*.sh`, `phase_a.sh`, `boot_tour.sh`, `run_tour.sh`, `prep_down.sh` |
| PC tools | `added/pc_tools/*.py` |

## Added navigation/localization scripts

**Purpose:** These scripts sit above the vendor Nav2/SLAM stack and provide headless localization, goal sending, named-place navigation, costmap-aware approach selection, and scan/map visual verification.

**Where:** `added/robot_scripts/global_loc.py`, `scan_overlay.py`, `initpose.py`, `goto.py`, `goto_place.py`, `goal_pick.py`.

| Script | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `global_loc.py` | `MAPY` env var or `/home/ubuntu/ros2_ws/src/slam/maps/map_04.yaml`; `/scan`; TF `base_footprint` to scan frame; map image | Brute-force/refined scan-match estimate; publishes `/initialpose` six times | Fails if map files, scan, or TF are unavailable; expensive over many free cells |
| `scan_overlay.py` | `MAPY`; `/scan`; TF `map` to scan frame; map image | Saves `/tmp/overlay.png`; prints robot map pose and scan point count | Prints `NO SCAN` or `NO TF` if ROS graph/localization is not ready |
| `goto.py` | CLI args `x y [yaw_deg]`; Nav2 `navigate_to_pose` action server | Sends `NavigateToPose` goal; prints feedback and result; cancels after 90 s timeout | Goal rejected, Nav2 unavailable, or timeout |
| `goto_place.py` | `~/places.yaml`; place name or `--list`; `~/goto.py` | Lists known places or shells out to `goto.py` with selected pose | Unknown place; ambiguous alias; missing/malformed places file |
| `goal_pick.py` | Station name in hard-coded `NOMINAL`; `/global_costmap/costmap` with transient-local QoS | Prints a free `x y yaw_deg` approach pose facing the station | Unknown station raises `KeyError`; no costmap; no free approach within search ring |
| `initpose.py` | Optional CLI args `x y yaw_deg`; defaults to `0 0 0`; ROS graph | Publishes `/initialpose` six times in frame `map`, with yaw converted from degrees | Uses fixed covariance (`x/y: 0.25`, yaw about `0.068`); only seeds localization and does not verify convergence |

**Important implementation notes:** `global_loc.py` scores candidate base poses by projecting current lidar points into the saved occupancy map and comparing them to a wall-distance transform. It accounts for the static base-to-lidar transform. `goal_pick.py` deliberately queries the live global costmap rather than trusting nominal station coordinates, so it avoids goals that are occupied in the current Nav2 map.

**Hardware touched:** Indirectly consumes lidar/TF/map data and commands Nav2 goals; `goto.py` can make the robot move through Nav2.

**Source paths:** `added/robot_scripts/global_loc.py`, `scan_overlay.py`, `goto.py`, `goto_place.py`, `goal_pick.py`, `initpose.py`, `docs/ADDED_SOFTWARE.md`.

## Added exploration/mapping scripts

**Purpose:** These scripts experiment with autonomous exploration and RTAB-Map mapping on top of the running base stack.

**Where:** `added/robot_scripts/explore.py`, `explore2.py`, `rtab_drive.py`, `rtab_drive04.py`, `rtabmap_map.launch.py`.

| Script | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `explore2.py` | `/scan`, `/map`, `/depth_cam/rgb0/image_raw`, `/depth_cam/depth0/image_raw`, TF map/odom/base, LM Studio VLM, STOP file at `/home/ubuntu/scout_out/STOP` | Reactive frontier-ish driving on `/controller/cmd_vel`, arm pan scans, VLM descriptions, `scout.json`, logs, periodic map saves | Raw velocity driving; depends on lidar/camera/TF/map/model; rear is blind so it avoids reverse translation |
| `rtab_drive.py` | Same core sensor/control set as `explore2.py`; RTAB-Map-oriented map output | Same reactive driving/VLM behavior as `explore2.py`, but saves `map_rtab` | Same raw-drive and sensor/model risks as `explore2.py` |
| `rtab_drive04.py` | Same core sensor/control set as `explore2.py`; map_04-oriented output | Same reactive driving/VLM behavior, saves `map_04`, and sets `SPIN_EVERY=1e12` so periodic spin-scans are effectively disabled after startup | Same raw-drive and sensor/model risks; fewer periodic vision sweeps means less observation refresh |
| `rtabmap_map.launch.py` | Existing base stack, `/odom`, `/scan`, RGB/depth camera topics | Starts `rtabmap_sync/rgbd_sync` and `rtabmap_slam/rtabmap` with Jetson-conscious parameters | Assumes base stack already provides odom, lidar, camera, and TF; not a full bringup launch |
| `explore.py` | Earlier scan-frontier script using `/scan`, `/map`, RGB image, and VLM | Spin-scans every cycle, picks nearest unvisited frontier, drives toward it, saves `map_02` and `scout.json` | No depth-bumper input and less conservative rear-blind handling than `explore2.py`; superseded for practical use |

**Important implementation notes:** `explore2.py` accounts for the 180-degree lidar mounting by mapping base heading to lidar beam angle plus pi. It writes outputs under `/home/ubuntu/scout_out`, honors a STOP file, uses depth image regions as a front bumper, and uses spin-scans with arm pan positions for VLM object lists. The RTAB drive variants are copies of the same reactive core with map-save target changes: `rtab_drive.py` saves `map_rtab`, while `rtab_drive04.py` saves `map_04` and nearly disables periodic spin-scans. `rtabmap_map.launch.py` uses external wheel/EKF odom, lidar for the occupancy grid, and camera for loop closure; it decimates images and limits features for Jetson budget.

**Hardware touched:** Drive base, arm servos, lidar, depth/RGB camera.

**Safety notes:** These are experimental autonomy scripts. They publish raw `/controller/cmd_vel`, can run for long periods, and should be supervised in a cleared area.

**Source paths:** `added/robot_scripts/explore.py`, `explore2.py`, `rtab_drive.py`, `rtab_drive04.py`, `rtabmap_map.launch.py`, `docs/ADDED_SOFTWARE.md`.

## Added perception/arm scripts

**Purpose:** These scripts use the arm-mounted depth camera for inspection or as an obstacle source. They are added workflows on top of the vendor camera and servo interfaces.

**Where:** `added/robot_scripts/vlook.py`, `look.py`, `capframe.py`, `depth_guard.py`, plus `added/robot_home/arm_pose.py`.

| Script | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `vlook.py` | Optional pan, output jpg, question; `/depth_cam/rgb0/image_raw`; LM Studio at `http://192.168.0.148:1234/v1/chat/completions` | Moves arm to a forward camera pose, saves a frame, sends it to `qwen/qwen3-vl-4b`, prints response | Fails if camera topic, bus servo topic, OpenCV bridge, network, or LM Studio model is unavailable |
| `depth_guard.py` | `/depth_cam/depth0/image_raw`; calibrated constants in the script | Publishes `/depth_scan` as a 64-beam `LaserScan` in `base_footprint` at up to 3 Hz | Assumes arm locked forward and fixed camera height/tilt; wrong arm pose makes obstacle projection unreliable |
| `capframe.py` | Optional output path, default `/tmp/cap.jpg`; `/depth_cam/rgb0/image_raw` | Waits up to 5 s, saves one BGR image, and prints shape/encoding | Exits with `NO FRAME` if no RGB image arrives |
| `look.py` | Direction or pulse target (`left`, `right`, `center`, `around`, or `0-1000`); optional question; MJPEG snapshot endpoint; LM Studio at `http://192.168.0.148:1234/v1/chat/completions` | Sets an older forward camera pose through `servo_controller`, captures `/tmp/look.jpg`, asks `qwen3.5-9b` for a description; `around` scans right/ahead/left | Superseded by `vlook.py`; depends on web video server, LM Studio, and older `servo_controller.bus_servo_control` helper path |
| `arm_pose.py` | CLI `dur id pos [id pos...]`; `servo_controller` topic | Publishes servo ID/pulse pairs and waits for movement duration plus margin | No semantic pose validation; wrong pulse pairs can move the arm into unsafe positions |

**Important implementation notes:** `vlook.py` uses the low-level bus-servo topic `/ros_robot_controller/bus_servo/set_position` with servo IDs 1, 2, 3, 4, and 5, then waits for the arm to settle before reading an RGB frame. `look.py` is an older path that sets pan positions `left=900`, `right=100`, and `center=500`, then uses a local video-server snapshot rather than subscribing to the ROS image topic. `depth_guard.py` converts depth pixels into approximate obstacle ranges after filtering by height band, range band, camera tilt, and camera offset. It exists because the full 12 Hz pointcloud path was too CPU-heavy for the navigation stack.

**Hardware touched:** Bus servos, arm-mounted Aurora camera, and indirectly Nav2 local costmap when `/depth_scan` is configured as an observation source.

**Source paths:** `added/robot_scripts/vlook.py`, `added/robot_scripts/look.py`, `added/robot_scripts/capframe.py`, `added/robot_scripts/depth_guard.py`, `added/robot_home/arm_pose.py`, `docs/ADDED_SOFTWARE.md`, `docs/HARDWARE.md`.

## Added voice/LLM scripts

**Purpose:** These scripts convert speech or text into either VLM inspection or direct velocity streaming. They are experimental added workflows and should be treated as operator-supervised.

**Where:** `added/robot_scripts/assistant.py`, `llm_drive.py`, `voice_drive.py`, `calibrate_wake.py`, plus `added/robot_home/debug_listen.py`.

| Script | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `assistant.py` | WonderEcho audio via `arecord -D plughw:0,0`; sherpa-onnx streaming model; sherpa Whisper model; optional `--dry` and `--seconds`; `allzeit.wav` | Always-on wake-word loop; after wake, transcribes a short command and routes to `look.py` or `llm_drive.py` | False wakes, missed commands, model path missing, audio device unavailable, unsafe if started without operator consent |
| `voice_drive.py` | Duration arg; WonderEcho audio; sherpa streaming model | Records `/tmp/voice.wav`, transcribes, shells out to `llm_drive.py` | Poor recognition, missing models/audio device, inherits `llm_drive.py` motion risks |
| `llm_drive.py` | Text command; LM Studio at `192.168.0.148:1234`; `qwen3.5-9b`; ROS graph | Requests JSON motion plan, streams `/controller/cmd_vel` at 20 Hz per segment, then publishes stop messages | LM Studio unreachable/bad JSON; direct raw velocity topic use; generated motion may be wrong despite prompt constraints |
| `calibrate_wake.py` | Optional seconds argument, default `40`; `arecord -D plughw:0,0`; sherpa streaming zipformer model under `~/third_party/sherpa-onnx` | Streams live endpoint transcriptions and prints `HEARD:` lines for wake-word tuning | Missing model/audio device; no persistent calibration is written by this helper |
| `debug_listen.py` | Fixed 20 s audio listen on `plughw:0,0`; same sherpa streaming model family | Prints audio amplitude, endpoint state, partial text, and final phrases for microphone/debug checks | Reports `NO DATA` if `arecord` exits or no audio is captured; does not command robot motion |

**Important implementation notes:** `assistant.py` separates wake detection from command recognition: a streaming sherpa model detects approximate wake tokens, then a Whisper recognizer transcribes the command window. It plays `allzeit.wav` with `aplay` and deliberately skips capture while the response sound plays. `llm_drive.py` asks the model for JSON actions shaped as `[x, y, z, t]`, parses the first JSON object from the model reply, and streams raw `/controller/cmd_vel` commands until each segment duration expires.

**Hardware touched:** WonderEcho Pro audio, speakers, and the drive base through `/controller/cmd_vel`.

**Safety notes:** Do not start `assistant.py` casually; it opens the mic continuously and can route recognized speech into robot motion. Prefer `--dry` for testing recognition/routing. `llm_drive.py` uses the raw velocity topic, so it bypasses the safer `/cmd_vel` clamp documented in the controller card.

**Source paths:** `added/robot_scripts/assistant.py`, `llm_drive.py`, `voice_drive.py`, `calibrate_wake.py`, `added/robot_home/debug_listen.py`, `docs/ADDED_SOFTWARE.md`, `docs/HARDWARE.md`.

## Added tour/orchestration scripts

**Purpose:** These shell scripts compose bringup hygiene, localization, Nav2 goals, costmap clears, photo proof, and recovery attempts into assignment/tour workflows.

**Where:** `added/robot_scripts/phase_a.sh`, `tour.sh`, `tour_down1.sh`, `boot_tour.sh`, `run_tour.sh`, `prep_down.sh`, `assignment*.sh`.

| Script / group | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `phase_a.sh` | zsh ROS env, battery topic, arm servo topic, Nav2 bringup, `global_loc.py`, `scan_overlay.py`, video server | Kills CPU-heavy app/demo nodes, locks arm forward, launches Nav2 if needed, starts `depth_guard.py`, localizes, saves overlay and camera snapshot | Nav2 half-load, localization score below threshold, missing battery/camera/depth topics |
| `tour.sh` | `global_loc.py`, `goal_pick.py`, `goto.py`, Nav2 costmap clear services, camera snapshot endpoint | Runs strict drums -> door -> window -> start assignment order with localization gate and photo per station | Aborts on low localization score or repeated failed leg; may hang/timeout on Nav2 issues |
| `assignment4.sh` | Hard-coded door/home coordinates, `goto.py`, costmap clear services, direct `/controller/cmd_vel` escape | Attempts remaining legs with rotate/forward escape if arrival pose traps the robot in inflated cost | Escape bypasses Nav2 and uses raw velocity; incomplete if stations keep failing |
| `boot_tour.sh` | Cold-session chain using map `map_04.yaml`, Nav2 bringup, `global_loc.py`, `goto_place.py`, `goto.py`, and camera snapshots | Logs to `~/scout_out/boot_tour.log`, locks arm forward, starts Nav2 and `depth_guard.py`, gates driving on localization score >=90%, visits drums/door/window/start, saves `tour2_*.jpg` | Starts background processes with `nohup`; aborts before driving if localization is below threshold |
| `run_tour.sh` | Existing localized Nav2 stack, `goto_place.py`, `goto.py`, camera snapshot endpoint | Simple detached tour over drums/door/window/start; writes `~/scout_out/tour.log` and `tour2_*.jpg` | Does not bring up or relocalize the stack; assumes Nav2 is already healthy |
| `prep_down.sh` | Downstairs map `map_rtab.yaml`, Nav2, `global_loc.py`, `scan_overlay.py`, camera snapshot endpoint | Kills heavy demos, locks arm forward, restarts Nav2 on downstairs map, starts `depth_guard.py`, localizes up to five times, saves overlay and camera evidence | No driving, but it kills/restarts navigation components and switches maps |
| `tour_down1.sh` | Downstairs-localized Nav2 stack, fixed poses for plant and kitchen, `goto.py`, camera snapshots | Clears costmaps, attempts each leg up to three times, saves `asgn_d_*.jpg` on success | Stops after failed plant/kitchen leg; relies on vision-discovered fixed poses from the script |
| `assignment.sh` / `assignment2.sh` / `assignment3.sh` | Older assignment/resume variants using fixed station coordinates, `goto.py`, costmap clears, camera snapshots | Retry loops for drums/door/window/start or resume subsets; photos under `~/scout_out` | Kept for history/resume use; later scripts document hard timeouts, corrected free-cell coordinates, and escape behavior |

**Important implementation notes:** These scripts are zsh and source `~/.zshrc`, because the robot environment is selected through shell startup files. `phase_a.sh` kills known CPU-heavy vendor demo nodes after boot, then uses an inline Python snippet to set the arm to the shared forward pose. The localization gates are score percentages parsed from `global_loc.py` output. `tour.sh` uses live `goal_pick.py` output before each leg rather than trusting fixed poses. The older assignment variants show the evolution from fixed goals, to hard timeouts, to corrected free-cell coordinates, and finally to `assignment4.sh`, which documents a specific failure mode: a station arrival can leave the robot in inflated cost so the next TEB goal is not feasible; its `escape()` rotates and creeps forward through `/controller/cmd_vel`, then clears costmaps.

**Hardware touched:** Drive base, arm servos, depth camera/video server, battery monitor. Most movement is through Nav2, except explicit escape/direct topic sections.

**Safety notes:** Treat these as supervised assignment scripts, not general autonomy. They can move the robot for minutes, clear costmaps, kill processes, and in some variants directly publish raw velocity. Check battery and physical surroundings first.

**Source paths:** `added/robot_scripts/phase_a.sh`, `tour.sh`, `boot_tour.sh`, `run_tour.sh`, `prep_down.sh`, `tour_down1.sh`, `assignment*.sh`, `docs/ADDED_SOFTWARE.md`, `added/guides/ROBOT_HANDOFF_GUIDE.md`.

## PC report tools

**Purpose:** These are Windows/PC-side post-processing scripts for scout/patrol outputs.

**Where:** `added/pc_tools/synth.py`, `gen_report.py`, `gen_report2.py`.

| Script | Inputs | Outputs / effects | Failure modes |
|---|---|---|---|
| `synth.py` | Hardcoded scout scratchpad directory containing `scout.json`; local LM Studio at `127.0.0.1:1234`; model `gemma-4-26b-a4b-it` | Writes `synthesis.md`; prints synthesized room report | Hardcoded path; LM Studio/model unavailable; assumes scout JSON schema |
| `gen_report2.py` | Hardcoded scout scratchpad directory containing `synthesis.md`, `scout.json`, images, and `map02f.png` | Writes self-contained `scout_report.html` with base64 images | Hardcoded path/assets; simple Markdown-to-HTML converter only handles narrow syntax |
| `gen_report.py` | Hardcoded patrol scratchpad directory containing `report*.json`, images, and `tourmap.png` | Writes `room_report.html` with embedded images and map | Hardcoded path/assets; report text includes run-specific observations |

**Important implementation notes:** These are not general-purpose command-line tools yet. They preserve specific run reports from scratchpad directories and should be parameterized before reuse.

**Source paths:** `added/pc_tools/synth.py`, `gen_report.py`, `gen_report2.py`, `docs/ADDED_SOFTWARE.md`.

## Review Rule

Do not paste local-model drafts into this file without checking them against the cited source paths. This stack mixes vendor code, user-modified vendor code, and added scripts, so inaccurate attribution is a real documentation bug.
