# Launch Flow Map

This map summarizes the launch paths that matter most for understanding or changing the robot. It is source-derived and can be reviewed without a live ROS graph.

## Path Resolution Rule

Many launch files read `need_compile`:

- `need_compile=True`: resolve package paths through `get_package_share_directory(...)`;
- otherwise: use hardcoded source paths under `/home/ubuntu/ros2_ws/src/...`.

Most operator scripts source `~/.zshrc` first, because that shell startup chain supplies the robot environment.

## Base Robot Bringup

```mermaid
flowchart TD
  controller["driver/controller/launch/controller.launch.py"]
  odom["odom_publisher.launch.py"]
  rrc["ros_robot_controller.launch.py"]
  imu["peripherals/launch/imu_filter.launch.py"]
  ekf["robot_localization ekf_node"]
  servo["servo_controller.launch.py"]

  controller --> odom
  odom --> rrc
  controller --> imu
  controller --> ekf
  controller --> servo
```

Key outputs:

- raw and filtered odometry: `odom_raw`, `odom`;
- raw drive command input: `controller/cmd_vel`;
- clamped drive command input: `cmd_vel`;
- final motor command: `ros_robot_controller/set_motor`;
- servo/arm interfaces through `servo_controller` and `controller_manager/...`.

## SLAM Bringup

```mermaid
flowchart TD
  slam["slam/launch/slam.launch.py"]
  robot["slam/launch/include/robot.launch.py"]
  toolbox["include/slam_base.launch.py"]
  carto["include/cartographer.launch.py"]
  gmapping["include/gmapping.launch.py"]
  rrt["include/rrt_explorer.launch.py"]

  slam --> robot
  slam -->|"after 10s, slam_method=slam_toolbox"| toolbox
  slam -->|"slam_method=cartographer"| carto
  slam -->|"slam_method=gmapping"| gmapping
  slam -->|"slam_method=rrt_explorer"| rrt
```

`robot.launch.py` starts the controller stack, depth camera, lidar, joystick control, init pose, and optional laser odometry wiring. Mapping nodes are delayed so the base sensor stack has time to start.

## RTAB-Map SLAM

```mermaid
flowchart TD
  top["slam/launch/rtabmap_slam.launch.py"]
  robot["slam/launch/include/robot.launch.py"]
  rtab["slam/launch/include/rtabmap.launch.py"]
  sync["rtabmap_sync/rgbd_sync"]
  mapper["rtabmap_slam/rtabmap"]

  top --> robot
  top -->|"after 10s"| rtab
  rtab --> sync
  rtab --> mapper
```

The top-level RTAB launch computes namespaced camera/odom/scan variables but currently only passes `use_sim_time` into the RTAB include. The include itself uses fixed remaps such as `scan -> /scan`.

## Nav2 Bringup

```mermaid
flowchart TD
  nav["navigation/launch/navigation.launch.py"]
  base["slam/launch/include/robot.launch.py"]
  bringup["navigation/launch/include/bringup.launch.py"]
  navbase["navigation/launch/include/navigation_base.launch.py"]
  nav2["Nav2 composable nodes"]

  nav --> base
  nav -->|"after 10s"| bringup
  bringup --> navbase
  navbase --> nav2
```

Modified Nav2 config points both single-pose and through-poses navigation at no-backup behavior trees. Local costmap consumes `/scan` and the added `/depth_scan` source when `depth_guard.py` is running.

## Peripherals

```mermaid
flowchart TD
  lidar["peripherals/launch/lidar.launch.py"]
  camera["peripherals/launch/depth_camera.launch.py"]
  imu["peripherals/launch/imu_filter.launch.py"]
  joy["peripherals/launch/joystick_control.launch.py"]

  lidar -->|"LIDAR_TYPE=SCLIDAR"| sclidar["include/sclidar.launch.py"]
  lidar -->|"LD19/MS200/A1/G4/STL06N variants"| otherlidars["include/*lidar*.launch.py"]
  camera -->|"DEPTH_CAMERA_TYPE=aurora"| aurora["include/aurora930.launch.py"]
  camera -->|"DEPTH_CAMERA_TYPE=ascamera"| ascamera["include/ascamera.launch.py"]
  camera -->|"fallback"| usb["usb_cam.launch.py"]
  imu --> calib["imu_calib/apply_calib"]
  calib --> madgwick["imu_filter_madgwick"]
```

On Pro machines, `lidar.launch.py` adds a `laser_filters` chain from `scan_raw` to `scan`.

## OpenClaw

```mermaid
flowchart TD
  start["openclaw_controller/launch/start.launch.py"]
  base["robot_base_control.launch.py"]
  voice["voice_openclaw"]
  vocal["large_models vocal_detect"]
  tts["large_models tts_node"]
  yolo["yolo_node.launch.py optional"]

  start --> vocal
  start --> tts
  start --> base
  start --> voice
  start -. "use_yolo=true" .-> yolo
```

`robot_base_control.launch.py` starts controller, depth camera, lidar, joystick control, `track_and_grab`, kinematics, and init pose.

## Vendor Offline Voice

```mermaid
flowchart TD
  move["voice_control_move.launch.py"]
  nav["voice_control_navigation.launch.py"]
  arm["voice_control_arm.launch.py"]
  color["voice_control_color_*.launch.py"]
  garbage["voice_control_garbage_classification.launch.py"]
  mic["mic_init.launch.py"]

  move --> mic
  nav --> mic
  arm --> mic
  color --> mic
  garbage --> mic
```

All voice demo launches include mic initialization plus a task-specific node. Movement and navigation voice demos can publish raw `/controller/cmd_vel`.

## Added Operator Scripts

| Script | Launch/process behavior |
|---|---|
| `phase_a.sh` | Kills heavy demo nodes, locks arm forward, starts/restarts Nav2 on `map_04.yaml`, starts `depth_guard.py`, localizes, saves evidence |
| `boot_tour.sh` | Starts Nav2 and `depth_guard.py` if needed, gates on localization >=90%, then runs tour legs |
| `tour.sh` | Uses existing stack, relocalizes, chooses live costmap goals, runs drums -> door -> window -> start |
| `prep_down.sh` | Switches Nav2 to `map_rtab.yaml`, localizes, saves downstairs evidence without driving |
| `tour_down1.sh` | Runs downstairs plant/kitchen fixed-pose legs with costmap clears and retries |

