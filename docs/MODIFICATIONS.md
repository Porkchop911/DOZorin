# Modifications — everything that differs from factory

**Method.** The robot's live `~/ros2_ws/src` (1106 files) was md5-manifested over SSH (2026-07-06) and compared against *three* vendor references: the published git history's Chinese-comment and English-comment variants (`D:\robot\src_ros2` commits `b306a7c` / `422acec`) and the published source zip. Files differing from all three were then attributed by modification timestamp: **user-era = late June 2026 onward** (robot arrived ~2026-06-23); everything older shipped that way on the 2026-05-09 image. Live copies of every user-modified factory file are committed under `vendor-modified/` (paths mirror the robot).

## 1. User modifications to factory files

| File (robot path) | Changed | What / why |
|---|---|---|
| `~/ros2_ws/src/navigation/config/nav2_params.yaml` | 2026-07-05 | Retuned Nav2 params; wires `default_nav_to_pose_bt_xml` **and** `default_nav_through_poses_bt_xml` to the no-backup behavior trees (both must be set — fixing only one leaves reverse recoveries in the other pipeline); local-costmap observation source switched to the `depth_guard` synthetic scan after the 12 Hz pointcloud starved the CPU. On-robot backups: `.prebtxml`, `.prebehaviorbak`, `.predepthbak` |
| `~/ros2_ws/src/navigation/config/nav2_controller_teb.yaml` | 2026-07-05 | TEB local planner tune for the mecanum base (backup: `.prebak`) |
| `~/ros2_ws/src/slam/config/slam.yaml` | 2026-07-02 | slam_toolbox retune for Jetson CPU headroom: `map_update_interval 0.1→2.0`, `throttle_scans 1→3`, `minimum_travel_distance 0.0→0.1`, `transform_publish_period 0.01→0.05` (backup: `.heavybak`) |
| `~/ros2_ws/src/large_models/large_models/large_models/config.py` | 2026-06-29 | LLM provider pointed at **LM Studio on the PC**: `llm_base_url='http://192.168.0.148:1234/v1'`, `llm_api_key='lm-studio'`, local model instead of cloud (no real API keys present). Backup: `config.py.lmstudio.bak`. Gotcha: this path is only used with `ASR_MODE=online`; offline mode routes to local ollama |
| `~/ros2_ws/src/large_models/large_models/large_models/agent_process.py` | 2026-06-25 | `enable_think = False` — disable qwen reasoning mode for much faster replies on the Jetson |
| `/etc/udev/rules.d/lidar.rules` | 2026-06-29 | Lidar symlink repinned to the **actual** USB port `1-2.1.4` (factory rule pointed at `1-2.2.4` — wrong for this unit; `/dev/lidar` never appeared, sclidar driver crashed). Factory original preserved on-robot as `lidar.rules.bak`. Caveat: the two CH340 adapters are identical (no serial number), so the rule pins by physical port — replugging the lidar into another socket breaks it again |
| `/etc/udev/rules.d/xf_mic.rules` | 2026-06-29 | Same bug class for the WonderEcho Pro mic serial: repinned `ring_mic` to `1-2.4.1` (factory `1-2.3.1`); without it `vocal_detect` crashed and blocked the LLM demos. Factory original: `xf_mic.rules.bak` |
| System timezone | — | Asia/Shanghai → **Europe/Vienna** |

## 2. User files added *inside* the factory tree

| File (robot path) | What |
|---|---|
| `navigation/config/navigate_to_pose_no_backup.xml`, `navigate_through_poses_no_backup.xml` | Nav2 behavior trees with BackUp/reverse recovery removed — the robot is rear-blind (lidar ~140° aft blindspot), so backing up is unsafe. Copies: `vendor-modified/.../navigation/config/` |
| `slam/maps/map_01, map_02, map_04, map_rtab, map_remap` (.pgm/.yaml) | Maps built with the retuned SLAM (map_01 upstairs 2026-07-01, map_02/remap downstairs 2026-07-02, map_rtab RTAB-Map 2026-07-03, **map_04 = current upstairs nav map** 2026-07-05). Copies: `added/robot_maps/`. (`map_03`, `test` are factory) |
| `~/` scripts, `~/places.yaml`, `~/allzeit.wav`, `~/bin/robot_stack_power.py` | The whole bespoke script layer — see `ADDED_SOFTWARE.md` |

## 3. PC-side (not on the robot)

- **LM Studio** on `192.168.0.148:1234` serves the models (`qwen/qwen3-vl-4b` vision, qwen3.5 chat, `gemma-4-26b-a4b-it` synthesis). Models must be pinned (`lms load <id> -y`, no TTL) or JIT eviction breaks the robot mid-task.
- **Hermes agent** (`D:\hermes\automation`) gained a `skills/robotics/rosorin-robot` skill wrapping the robot — untested as of 2026-07-06.
- Report tooling `added/pc_tools/` (see `ADDED_SOFTWARE.md`).

## 4. Vendor drift — image newer than published source (NOT user changes)

The shipped image (2026-05-09) differs from Hiwonder's published zip/git (Feb–Mar 2026) in ~40 files, all with pre-June timestamps. For awareness only:

- `openclaw_controller` refactored (new `claw_*` modules + launches; old `voice_openclaw`/`feishu_sender`/`track_and_grab_node` removed).
- YOLO TensorRT engines added on-robot (`example/.../models/26/*.engine`, `v11/*.engine`), `app/models/*.obj` AR assets.
- `.typerc`: `VERSION` 2026-05-09, `ASR_LANGUAGE=English`, `ASR_MODE=offline`, `ROS_DOMAIN_ID=0` (zip has Chinese/online/56).
- Assorted vendor fixes (`ros_robot_controller` node/SDK, kinematics `transform.py`, URDF xacros, several example configs) and runtime-generated files (iFlytek ASR grammar cache, logs, calibration ROI/IMU yamls).

## 5. Change-management conventions used on the robot

Every risky edit left a sibling backup (`*.bak`, `.prebak`, `.pre<change>bak`, `.lmstudio.bak`, `.heavybak`) next to the file — the robot itself carries its own undo history. `need_compile=False` + symlink installs mean all of the above took effect without rebuilding.
