# Offline Reference Matrix

This is a compact lookup table for source-backed constants, environment variables, topics, and external assumptions. It complements the narrative docs in `GUIDE.md`, `ARCHITECTURE.md`, `CODE_WALKTHROUGH.md`, `HARDWARE_INTERACTIONS.md`, and `LAUNCH_MAP.md`.

## Environment Variables

| Variable | Used by | Meaning / known value |
|---|---|---|
| `need_compile` | Many launch files | Chooses installed package share paths when `True`, otherwise hardcoded `/home/ubuntu/ros2_ws/src/...` source paths |
| `MACHINE_TYPE` | Controller, peripherals, app/example demos, servo controller | Selects mecanum/differential/Ackermann branches; documented unit is `ROSOrin_Mecanum_Pro` |
| `LIDAR_TYPE` | `peripherals/launch/lidar.launch.py`, app demos | Selects concrete lidar include; documented unit uses `SCLIDAR` |
| `DEPTH_CAMERA_TYPE` | `peripherals/launch/depth_camera.launch.py`, vision demos | Selects `aurora`, `ascamera`, or USB fallback; documented unit uses Aurora 930 |
| `ASR_MODE` | `large_models`, voice/model launches | Selects online provider path versus offline/Ollama path |
| `ASR_LANGUAGE` | `large_models`, `xf_mic_asr_offline`, examples | Selects Chinese/English prompts and feedback behavior |
| `HOST` / `MASTER` | Multi-robot/nav/SLAM launches | Namespaces robot topics and frames |
| `MAPY` | Added localization/overlay scripts | Overrides map YAML path for `global_loc.py` and `scan_overlay.py` |

## Devices And External Endpoints

| Name | Value | Owner / consumers |
|---|---|---|
| Low-level controller | `/dev/rrc` at 1 Mbaud | `ros_robot_controller_sdk.py` |
| Lidar | `/dev/lidar` at 230400 baud for SCLIDAR/LD19-like paths | `peripherals/launch/include/*lidar*.launch.py` |
| MS200 lidar path | `/dev/ldlidar` | `ms200_scan.launch.py` |
| Ring mic serial | `/dev/ring_mic` at 115200 baud | `xf_mic_asr_offline/scripts/awake_node.py` |
| Added audio capture | `arecord -D plughw:0,0` | `assistant.py`, `voice_drive.py`, `calibrate_wake.py`, `debug_listen.py` |
| Robot video server | `http://127.0.0.1:8080/snapshot?...` locally; robot LAN server is documented in `docs/SYSTEM.md` | `look.py`, tour scripts, report evidence |
| LM Studio | `http://192.168.0.148:1234/v1/chat/completions` | Modified `large_models` config and added VLM/LLM scripts |
| OpenClaw gateway | `http://127.0.0.1:18789` | OpenClaw-related vendor notes |

## Motion Topic Semantics

| Topic | Meaning |
|---|---|
| `/cmd_vel` | Safer public path in `odom_publisher_node.py`; x/y are clamped to +/-0.2 m/s and yaw to +/-0.5 rad/s before forwarding |
| `/controller/cmd_vel` | Raw controller path used by teleop, added scripts, many vendor demos, and some LLM/voice flows |
| `/app/cmd_vel` | App/Ackermann path that can set PWM steering behavior before forwarding |
| `ros_robot_controller/set_motor` | Final motor command topic from controller to STM32 bridge |

## Arm And Servo Interfaces

| Layer | Interface | Notes |
|---|---|---|
| Low-level bus servo | `ros_robot_controller/bus_servo/set_position`, `set_state`, `get_state` | Direct pulse-level bridge to STM32 |
| Servo controller topic | `servo_controller` | Higher-level pulse/radian/degree path used by many scripts |
| State feedback | `controller_manager/servo_states`, `controller_manager/joint_states` | Kinematics consumes servo states for IDs 1-5 |
| Kinematics services | `/kinematics/set_pose_target`, `/kinematics/get_current_pose`, `/kinematics/set_joint_value_target` | Arm pose solve; gripper is outside the IK solve |
| Action groups | `/home/ubuntu/software/arm_pc/ActionGroups` | Used by init pose and voice arm demos |

## Known Added Script Outputs

| Script/group | Output paths |
|---|---|
| Exploration | `/home/ubuntu/scout_out/log.txt`, `scout.json`, VLM JPGs, STOP file at `/home/ubuntu/scout_out/STOP` |
| Tour/assignment scripts | `~/scout_out/assignment.log`, `tour.log`, `boot_tour.log`, `downstairs.log`, proof JPGs |
| `scan_overlay.py` | `/tmp/overlay.png` |
| `capframe.py` | Default `/tmp/cap.jpg` |
| `look.py` | `/tmp/look.jpg` |
| PC report tools | Hardcoded scratchpad paths; see `docs/CODE_WALKTHROUGH.md` for exact reuse notes |

## Offline Validation Commands

| Command | Purpose |
|---|---|
| `powershell -ExecutionPolicy Bypass -File tools/check_markdown_links.ps1` | Validate local Markdown links in the main docs |
| `powershell -ExecutionPolicy Bypass -File tools/static_source_scan.ps1` | Re-run the static source scan categories used by `docs/OFFLINE_AUDIT.md` |
| `python -m compileall -q added vendor/src_ros2/src vendor-modified/ros2_ws/src` | Parse Python files without requiring robot hardware or ROS runtime imports |
