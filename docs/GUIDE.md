# Guide

This is the guided reading path for the Hiwonder ROSOrin Mecanum Pro software stack captured in this repo. It explains how the pieces fit together and points to the deeper reference docs when a detail becomes operational or source-level.

For ownership and trust boundaries, start with `PROVENANCE.md`: this repo deliberately separates factory-original files in `vendor/`, live user-modified factory files in `vendor-modified/`, and added scripts/maps/guides in `added/`.

## The System In One Mental Model

The robot is a Jetson Orin Nano running Ubuntu 22.04 and ROS 2 Humble. The Jetson does the high-level work: launch orchestration, ROS graph, perception, SLAM, Nav2, voice/LLM clients, web services, and added task scripts.

The Jetson does not directly drive motors or servos. It talks over USB serial to an STM32F407 low-level controller. The STM32 owns the drive motors, encoders, IMU, arm servos, battery monitor, LEDs, buzzer, OLED, and buttons. In ROS terms, the central bridge is the vendor `ros_robot_controller` package, using the stable device name `/dev/rrc`.

Sensors flow upward into ROS topics. Commands flow downward into actuator topics:

```mermaid
flowchart TD
  operator[Operator, app, joystick, Nav2, added scripts, LLM] --> cmd[Velocity or servo command topics]
  cmd --> controller[ROS controller and servo nodes]
  controller --> bridge[ros_robot_controller_node]
  bridge --> serial[/dev/rrc serial link]
  serial --> stm32[STM32 low-level controller]
  stm32 --> actuators[Motors, servos, LEDs, buzzer, OLED]
  sensors[Lidar, camera, IMU, mic, battery] --> ros[ROS topics]
  ros --> operator
  ros --> nav[SLAM, localization, Nav2, vision, voice]
```

The most important practical consequence is that the robot is a layered system. Bugs often appear far from their cause: a missing udev symlink can look like a ROS launch failure, a CPU-heavy camera stream can look like bad navigation, and a physical lidar blind spot can become a Nav2 behavior-tree policy.

## Boot To Running ROS Graph

The factory stack starts automatically through `start_app_node.service`. That service launches `bringup.launch.py`, which starts the low-level controller, odometry/EKF stack, lidar, depth camera, rosbridge, web video server, joystick control, and vendor demo/app nodes. Details and live observations are in `docs/SYSTEM.md`.

Important boot facts:

- the robot user is `ubuntu`;
- the default shell is zsh, and the ROS environment chain is loaded through shell startup files;
- the live robot uses `ROS_DOMAIN_ID=0`;
- Python/config packages are symlink-installed, so many source/config edits take effect after restart without rebuild;
- the autostart service has `Restart=always`, so manual bringup work must avoid duplicate stacks fighting for devices.

For operation commands, use `added/guides/OPERATING.md`. This guide intentionally explains the shape of the system rather than duplicating every command.

## How Commands Become Motion

There are multiple velocity topics with different meanings. The important distinction from `added/guides/OPERATING.md` is:

- `/cmd_vel` is the clamped safer manual path;
- `/controller/cmd_vel` is the raw path used deeper in the stack;
- `/app/cmd_vel` is the Ackermann-style app path.

Those topics feed the vendor controller logic, which mixes desired motion into wheel commands for the mecanum base. The low-level bridge sends motor state to the STM32 over `/dev/rrc`.

The robot has a known odometry limitation: sideways mecanum motion is not represented as cleanly as forward motion. Navigation and localization work best when strafing is limited and the map/localization state is checked.

## How Sensors Feed The Stack

The lidar, depth camera, IMU, mic, and battery all enter the software stack differently:

- the COIN-D6 lidar uses `/dev/lidar` and publishes laser scans, but this unit has only about 219 degrees of usable field of view and is rear-blind;
- the Aurora depth camera publishes RGB, depth, IR, and optionally pointcloud data, but pointcloud use can starve CPU;
- the IMU originates on the STM32 path and is filtered/fused into odometry;
- the WonderEcho Pro appears both as USB audio and a CH340 serial wake-word/control device;
- battery voltage is published by the STM32 bridge.

Use `docs/HARDWARE.md` for specifications and `docs/HARDWARE_INTERACTIONS.md` for the software-facing map of devices to topics, nodes, and failure modes.

## Mapping And Navigation

SLAM and navigation are separate from base bringup. The vendor `slam` package provides mapping launches and map storage. The vendor `navigation` package runs Nav2 with TEB tuning. User changes in `vendor-modified/` retuned SLAM/Nav2 and added no-backup behavior trees.

The no-backup behavior is not cosmetic. The robot has no rear sensing coverage from the lidar and no bumper/cliff system. Reverse recovery behavior is unsafe on this physical unit, so the Nav2 behavior trees were changed to avoid backup maneuvers. See `docs/MODIFICATIONS.md` for exact changed files and `docs/ARCHITECTURE.md` for the design rationale.

The added navigation scripts sit above the vendor stack. They localize, choose goals, drive to named places, verify views, and orchestrate assignment/tour flows. Their current status is honest rather than polished: `docs/ADDED_SOFTWARE.md` says navigation is not "done" and documents known limits.

## Voice, Vision, And Local Models

The vendor stack includes `large_models`, `large_models_examples`, and `xf_mic_asr_offline`. This repo also captures user modifications that point large-model config at LM Studio on the PC and added scripts using sherpa-onnx, local wake words, VLM image description, and LLM-generated drive commands.

Conceptually:

- vendor voice/ASR/LLM demos are ROS package features;
- added scripts in `added/robot_scripts` are task-oriented robot workflows;
- PC-side LM Studio provides model serving at `192.168.0.148:1234`;
- the assistant and drive scripts should be treated as experimental unless a guide says otherwise.

## Where To Go Next

| Need | Read |
|---|---|
| Ownership and proof method | `PROVENANCE.md` |
| OS, boot, devices, network | `docs/SYSTEM.md` |
| Hardware specs | `docs/HARDWARE.md` |
| Software-to-hardware mapping | `docs/HARDWARE_INTERACTIONS.md` |
| Layered architecture and flows | `docs/ARCHITECTURE.md` |
| Package and script explanations | `docs/CODE_WALKTHROUGH.md` |
| Factory package overview | `docs/VENDOR_STACK.md` |
| User modifications | `docs/MODIFICATIONS.md` |
| Added scripts and status | `docs/ADDED_SOFTWARE.md` |
| Operating commands | `added/guides/OPERATING.md` |
| Remaining doc work | `docs/TASKS.md` |

## Glossary

| Term | Meaning in this repo |
|---|---|
| ROS 2 | Middleware used for robot nodes, topics, services, actions, launch files, and parameters |
| Node | A running ROS process/component such as controller, lidar driver, or ASR node |
| Topic | ROS publish/subscribe data stream, for example `/scan` or `/cmd_vel` |
| Service | ROS request/response interface, often used for configuration or one-shot commands |
| Action | ROS long-running goal interface, used by Nav2 navigation goals |
| TF | ROS transform tree describing coordinate frames such as map, odom, base, lidar, and camera |
| Nav2 | ROS 2 navigation stack for localization, planning, costmaps, controllers, and recovery behavior |
| Costmap | Grid map used by Nav2 to decide where the robot can drive |
| Behavior tree | Nav2 decision policy for navigation and recovery actions |
| udev | Linux device naming/rules system used here for `/dev/rrc`, `/dev/lidar`, and `/dev/ring_mic` |
| Symlink install | Colcon install mode where installed Python/config files point back to source files |
