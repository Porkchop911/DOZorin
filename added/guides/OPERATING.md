# ROSOrin Mecanum Pro — Operator's Guide

For the broader explanation of how the stack is layered, see `../../docs/GUIDE.md`, `../../docs/ARCHITECTURE.md`, and `../../docs/HARDWARE_INTERACTIONS.md`.

## 1. Mental model

This is a layered ROS 2 Humble stack. Commands flow **down**, sensor data flows **up**:

```
 Web UI / app / LLM / joystick          ← high-level intent
        ↓ /cmd_vel, /app/cmd_vel
 odom_publisher  (cmd_vel → wheel mixing, dead-reckoning)
        ↓ /ros_robot_controller/set_motor
 ros_robot_controller_node  (ROS ↔ serial bridge)
        ↓ /dev/rrc @ 1 Mbaud, framed+CRC8 protocol
 STM32 MCU  → motors, servos, IMU, LED, buzzer, battery
```

Everything is one workspace at **`~/ros2_ws`**, built in-place (`need_compile=False`, so nodes run straight from `src/` — see §12). Third-party drivers (LiDAR, depth cam, EKF, planners) live in **`~/third_party/*_ws`**.

Robot identity is set entirely by **environment variables** (in the shell init): `MACHINE_TYPE=ROSOrin_Mecanum_Pro`, `LIDAR_TYPE=SCLIDAR`, `DEPTH_CAMERA_TYPE=aurora`. Code branches on these strings, so changing the variable changes behavior.

## 2. Sourcing your shell

Before any `ros2` command works in a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash    # use setup.zsh if you're in zsh (default shell)
```

The autostart service does this via `~/.zshrc`. Confirm you're talking to the right robot: `echo $ROS_DOMAIN_ID` (it's `0` here).

## 3. Starting and stopping the stack

**It starts automatically at boot** via systemd:

```bash
systemctl status start_app_node.service       # is it running?
sudo systemctl restart start_app_node.service # restart the whole robot
sudo systemctl stop  start_app_node.service   # stop autostart (Restart=always, so disable to keep it down)
sudo systemctl disable start_app_node.service # stop it coming back on reboot
```

The service runs `ros2 launch bringup bringup.launch.py`, and `Restart=always` revives it after 30 s if it dies.

**To run the stack manually** (after `stop`ping the service, so you don't get two copies fighting over `/dev/rrc`):

```bash
ros2 launch bringup bringup.launch.py
```

**Emergency / quick kill** — there's a pre-made script:

```bash
~/.stop_ros.sh     # kill -9 every ros2 / --ros-args process
```

`bringup.launch.py` brings up: `controller` (motors+odom), depth camera, LiDAR, rosbridge websocket, web_video_server, the `app` nodes, joystick control, and `init_pose`. SLAM and navigation are **not** in bringup — you launch those yourself (§8–9).

## 4. Where things live

| Path | What |
|---|---|
| `~/ros2_ws/src/driver/ros_robot_controller/` | Serial bridge + low-level SDK (the heart) |
| `~/ros2_ws/src/driver/controller/` | cmd_vel→wheels, odometry, EKF configs |
| `~/ros2_ws/src/driver/kinematics/` | Arm inverse kinematics |
| `~/ros2_ws/src/driver/servo_controller/` | Bus + PWM servos, action groups |
| `~/ros2_ws/src/peripherals/` | LiDAR, depth cam, IMU filter, joystick, **teleop** |
| `~/ros2_ws/src/slam/` | gmapping/rtabmap SLAM + saved maps |
| `~/ros2_ws/src/navigation/` | Nav2 + TEB planner + nav2_params.yaml |
| `~/ros2_ws/src/app/` | line-following, object-tracking, self-driving, lidar apps |
| `~/ros2_ws/src/example/` | YOLO detection, self_driving demos |
| `~/ros2_ws/src/large_models*/` | LLM / VLM control (voice, color-track, navigation) |
| `~/ros2_ws/src/openclaw_controller/` | Gripper arm: object track, track-and-grab, AprilTag, voice |
| `~/ros2_ws/src/simulations/rosorin_description/` | URDF/xacro (robot model) |

## 5. Driving the robot

The base accepts `geometry_msgs/Twist`. There are **three** velocity topics with different behavior (note the naming is confusing — see §14):

| Topic | Handler | Behavior |
|---|---|---|
| `/controller/cmd_vel` | raw | No clamping — full speed. This is what EKF/Nav2 ultimately feed. |
| `/cmd_vel` | `app_cmd_vel_callback` | **Clamped** to ±0.2 m/s, ±0.5 rad/s (safe manual driving) |
| `/app/cmd_vel` | `acker_cmd_vel_callback` | Ackermann steering-servo path |

Quick test — nudge forward, then **always stop**:

```bash
ros2 topic pub -1 /controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
ros2 topic pub -1 /controller/cmd_vel geometry_msgs/msg/Twist '{}'   # stop
```

Mecanum can strafe: set `linear.y` for sideways, `angular.z` to spin.

**Keyboard teleop:**
```bash
ros2 launch peripherals teleop_key_control.launch.py
```

**Gamepad/joystick** is already running from bringup (`joystick_control`). The gamepad button map is in `ros_robot_controller_sdk.py` (PS-style: cross/circle/square/triangle, L1/R1/L2/R2).

## 6. Servos and the arm

Low-level servo topics (published to the MCU bridge):
- `/ros_robot_controller/bus_servo/set_position` — bus servos (arm joints)
- `/ros_robot_controller/pwm_servo/set_state` — PWM servos (e.g. Ackermann steering / gripper)
- Services `/ros_robot_controller/bus_servo/get_state` and `.../pwm_servo/get_state` to read back position/voltage/temp.

Higher-level arm control: the `kinematics` package (IK) and `servo_controller` (joint trajectories, action groups). For the gripper specifically, use the `openclaw_controller` launches (§11).

⚠️ Servo offsets are loaded at startup from `/home/ubuntu/software/Servo_upper_computer/servo_config.yaml` — edit there, not in code.

## 7. Inspecting a live system

```bash
ros2 node list                 # all running nodes
ros2 topic list                # ~50 topics live
ros2 topic echo /odom          # fused pose (EKF output)
ros2 topic echo /imu           # filtered IMU
ros2 topic echo /scan          # filtered laser scan (raw is /scan_raw)
ros2 topic hz /controller/joint_states
ros2 topic echo /ros_robot_controller/battery   # battery voltage (UInt16)
ros2 run tf2_tools view_frames # dump the TF tree to a PDF
```

**Web tools (already running from bringup):**
- **rosbridge websocket** on port **9090** — the web/app UI talks to the robot over this.
- **web_video_server** on port **8080** — open `http://<robot-ip>:8080` in a browser to see camera streams (`/depth_cam/rgb0/image_raw`, `/yolo/object_image`, `/line_following/image_result`, etc.).

## 8. Building a map (SLAM)

Stop or coexist carefully, then:

```bash
ros2 launch slam slam.launch.py                      # default gmapping
ros2 launch slam slam.launch.py slam_method:=rtabmap # 3D/visual SLAM
ros2 launch slam rviz_slam.launch.py                 # visualize (needs a display)
```

Drive the robot around (§5) to fill the map. Maps save into `~/ros2_ws/src/slam/maps/` (existing: `map_01`, `map_02`, `map_03`, `test`). The launch has an `enable_save` argument.

## 9. Autonomous navigation (Nav2 + TEB)

```bash
ros2 launch navigation navigation.launch.py map:=map_01      # load a saved map
ros2 launch navigation navigation.launch.py map:=map_01 use_teb:=true
ros2 launch navigation rviz_navigation.launch.py             # set goals via RViz
```

- `map:=` picks a file from `slam/maps/` (no `.yaml` extension — default `map_01`).
- Tuning lives in `navigation/config/nav2_params.yaml`.
- Send goals from RViz ("2D Goal Pose"), or publish to the Nav2 action/`/goal_pose`.

## 10. Built-in app behaviors

These run from bringup but sit idle until enabled (toggled via the app/rosbridge UI or by publishing to their enable services). The nodes:

- `line_following` → result image on `/line_following/image_result`
- `object_tracking` → `/object_tracking/image_result`
- `self_driving` (in `example`) → `/self_driving/image_result`
- `yolo_node` → detections on `/yolo/object_detect`, annotated image `/yolo/object_image`
- `lidar_controller` → obstacle-avoid / follow behaviors

Inspect what each exposes: `ros2 node info /line_following` etc.

## 11. LLM / voice / gripper ("openclaw")

Large-model demos (voice-driven movement, VLM tracking, color sorting, transport):

```bash
ros2 launch large_models_examples llm_control_move.launch.py
ros2 launch large_models_examples vllm_navigation.launch.py
```
API keys/config for these: `~/ros2_ws/src/large_models/large_models/large_models/config.py`.

Gripper-arm scene apps:
```bash
ros2 launch openclaw_controller claw_track_and_grab.launch.py
ros2 launch openclaw_controller smart_scene_navigation.launch.py
ros2 launch openclaw_controller voice_control.launch.py
```

These talk to a **local gateway at `http://127.0.0.1:18789`** (OpenAI-compatible `/v1/chat/completions`). Auth token is read from `~/.openclaw/openclaw.json` (`gateway.auth.token`). `openclaw_controller/scripts/request_demo.py` is a minimal example client.

## 12. Editing code and rebuilding

Because `need_compile=False`, **Python nodes run directly from `~/ros2_ws/src`** — but they're launched from the installed copy. Safe workflow:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select controller   # one package
source install/setup.bash
sudo systemctl restart start_app_node.service                 # reload
```

`--symlink-install` means future Python edits take effect on node restart without rebuilding. C++ packages (and `interfaces`/msg changes) always need a rebuild.

## 13. Logs and troubleshooting

- ROS logs: `~/.ros/log/` and `~/log/`.
- Service logs: `journalctl -u start_app_node.service -f`.
- **"Nothing moves":** check the serial link `/dev/rrc` exists and only one process owns it (kill duplicates with `~/.stop_ros.sh`). Confirm reception is enabled — the bridge needs `/ros_robot_controller/enable_reception` true to read sensors.
- **"校验失败" in logs** = serial CRC mismatch (noise / contention on `/dev/rrc`).
- **Robot drifts on the map when strafing:** known odometry limitation (§14).

## 14. Gotchas you must know (this build, as-is)

1. **Odometry ignores sideways motion.** Pure `linear.y` (strafe) produces no pose change in dead-reckoning, only in the twist. Expect localization drift if you strafe a lot. (`odom_publisher_node.py:266`)
2. **Confusing cmd_vel naming:** `/cmd_vel` is the *clamped/safe* topic; `/controller/cmd_vel` is *raw/full-speed*. Don't assume from the names.
3. **`/set_odom` crashes the node** if you publish to it (`self.clock()` bug). Don't use it.
4. **`controller/load_calibrate_param` service is broken** (ROS1-style `~` param names). Set calibration via params at launch instead.
5. **`get_bus_servo_state` voltage/torque fields throw** (SDK method-name mismatch) — position/temperature read fine.
6. **Two copies of the stack will fight over the serial port.** Always stop the systemd service before launching `bringup` by hand.
7. **Security:** rosbridge (9090) and web_video_server (8080) are unauthenticated. Anyone on the LAN can drive the robot or view cameras. Keep it on a trusted network.
