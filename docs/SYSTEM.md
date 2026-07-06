# System — OS image, boot chain, identity, network

Facts marked [LIVE] were captured over SSH on 2026-07-06 (sweep outputs archived in the session scratchpad; re-runnable any time).

## OS / image

- Shipped system image: **ROSOrinPro_Orin_20260509** (matches live `VERSION="|V1.0.0|App_ROS2_1.0|2026-05-09|"`). Raw image: `D:\robot\New folder\ROSOrinPro_Orin_20260509.img` (66.7 GB); flashing tools + tutorial: see `vendor/VENDOR_ASSETS.md`.
- [LIVE] Ubuntu 22.04.5, kernel 5.15.148-tegra, JetPack 6.2 (L4T R36.4.3), NVMe root 228 GB.
- Login user `ubuntu`, **login shell zsh** (leading `=` triggers zsh expansion — wrap remote work in `bash`, or `zsh` when ROS env is needed).
- Timezone: **Europe/Vienna** (changed by us; image shipped Asia/Shanghai).

## Boot chain / autostart

```
systemd: start_app_node.service (enabled, Restart=always, RestartSec=30)
  └─ /bin/zsh -c 'source home/ubuntu/.zshrc; ros2 launch bringup bringup.launch.py;'
       └─ bringup.launch.py  → whole robot stack (controller, servos, lidar, camera,
                                odometry+EKF, apps: line_following / object_tracking /
                                self_driving / yolo, rosbridge :9090, web_video_server :8080)
```

Captured unit file: `vendor/system/etc/systemd/system/start_app_node.service`. Stop the stack with `~/.stop_ros.sh` (captured at `vendor/system/home/.stop_ros.sh`). [FIELD] The always-on vision apps are CPU hogs; `phase_a.sh` kills yolo/self_driving/line_following/object_tracking after boot before nav work.

Other Hiwonder units [LIVE]: `button_scan.service`, `find_device.service`, `remote.service` (all `~/wifi_manager/` — Wi-Fi/app pairing; `button_scan.py` legitimately holds `/dev/ttyACM0` alongside the ROS driver), plus `jtop.service`, `nxserver.service` (NoMachine).

## Identity env (.typerc / .robotrc)

`~/.zshrc` → sources `~/ros2_ws/.zshrc` → sources `~/ros2_ws/.robotrc` → sources `~/ros2_ws/.typerc`. Live copies: `vendor/system/ros2_ws/`.

Key values [LIVE]:

| Var | Value | Note |
|---|---|---|
| `MACHINE_TYPE` | `ROSOrin_Mecanum_Pro` | product variant |
| `LIDAR_TYPE` | `SCLIDAR` | COIN-D6 |
| `DEPTH_CAMERA_TYPE` | `aurora` | Aurora 930 |
| `MIC_TYPE` | `WonderEchoPro` | |
| `ASR_LANGUAGE` / `ASR_MODE` | `English` / `offline` | image default (older published source: Chinese/online) |
| `ROS_DOMAIN_ID` | **0** | the `56` in the published source zip is historical — live robot runs domain 0 |
| `ROS_LOCALHOST_ONLY` | 0 | set via rc chain; **sourcing `.zshrc` under bash drops it and silently kills DDS discovery** — pipe remote ROS commands through `zsh` |
| `need_compile` | `False` | packages are used symlink-installed; config edits need no rebuild |
| RMW | FastDDS (default; `CYCLONEDDS_URI` in rc is a red herring — file doesn't exist) | |

`.robotrc` also sources the **third_party overlay workspaces** in `~/third_party/`: `third_party_ws`, `orbbec_ws`, `aurora_ws` (camera driver), `gmapping_ws`, and (in the published rc, not the live one) `rtabmap_ws`. Also present there: `sherpa-onnx`, `YDLidar-SDK`, `ch341ser_linux`, `yolo`, `opencv(_contrib)`, `rtw89` Wi-Fi driver. [LIVE]

## Devices (udev)

| Symlink | Target | Rule file | Status |
|---|---|---|---|
| `/dev/rrc` | `ttyACM0` (QinHeng 1a86:55d4 = STM32) | `99-ttyACM0.rules` (also sets `ID_MM_PORT_IGNORE`) | factory — `vendor/system/etc/udev/rules.d/` |
| `/dev/lidar` | `ttyCH341USB0` | `lidar.rules` | **modified** — repinned to USB port `1-2.1.4` (factory `1-2.2.4` was wrong for this unit); `vendor-modified/etc/udev/rules.d/lidar.rules`, factory original preserved as `lidar.rules.bak` |
| `/dev/ring_mic` | `ttyCH341USB1` | `xf_mic.rules` | **modified** — repinned to USB port `1-2.4.1` (factory `1-2.3.1` was wrong for this unit; `vocal_detect` crashed); `vendor-modified/etc/udev/rules.d/xf_mic.rules`, factory original preserved as `xf_mic.rules.bak` |

So the docs' `/dev/rrc` vs `/dev/ttyACM0` are the same device — `rrc` is the stable udev name.

## Network & access

| What | Where |
|---|---|
| SSH | `ssh ubuntu@192.168.0.223` (never retry-storm — sshd MaxStartups self-DoS) |
| Remote command pattern | base64-encode locally → `ssh … "echo <b64> \| base64 -d \| zsh"` (PowerShell strips quotes / injects CRLF otherwise) |
| Camera streams | `http://192.168.0.223:8080` (web_video_server, **no auth**) |
| rosbridge websocket | `:9090` (**no auth**) |
| NoMachine | nxserver (needs physical display / EDID dummy plug — virtual desktop ghosts) |
| OpenClaw local gateway | `http://127.0.0.1:18789` (on robot) |
| LM Studio (LLM backend) | `http://192.168.0.148:1234/v1` on the Windows PC — see `MODIFICATIONS.md` |
| App | WonderAi (Android APK / iOS App Store), AP-mode SSID prefix `HW` |

## Live stack snapshot [LIVE 2026-07-06]

28 nodes / 52 topics under the default bringup, including: `/ros_robot_controller` (STM32 bridge), `/controller_manager` + `/servo_manager`, `/odom_publisher` + `/ekf_filter_node` (wheel+IMU fusion → `odom`), `/sclidar_scan_publisher` + scan filter (`/scan_raw` → `/scan`), `/aurora` (depth cam), `/imu_calib` + `/imu_filter`, apps (`/line_following`, `/object_tracking`, `/self_driving`, `/yolo`, `/lidar_app`), `/asr_node`, `/voice_control_move`, `/joystick_control`, `/rosbridge_websocket`, `/web_video_server`. Full ROS interface conventions: `added/guides/OPERATING.md` §4–7.

## Software inventory [LIVE]

- 392 `ros-humble-*` debs; Python 3.10.12.
- Notable pip: `sherpa-onnx 1.12.14` (local ASR/wake word), `onnxruntime-gpu 1.22.0`, `torch 2.7.0`, `openai 2.36.0` (used to reach LM Studio), OpenCV 4.11 (CUDA).
- `~/software/` — vendor tools (incl. `Servo_upper_computer/servo_config.yaml` servo offsets).
- `~/large_models/` — vendor LLM demo scripts (ollama-era; superseded by LM Studio wiring).
