# Hiwonder ROSOrin Pro — complete software stack

Documentation + snapshot repo for the **Hiwonder ROSOrin Mecanum Pro** (Jetson Orin Nano, `ubuntu@192.168.0.223`): everything the robot runs, **clearly separated into factory-original, factory-modified, and added-by-us**, plus full hardware specifications. Snapshot date: **2026-07-06** (image `ROSOrinPro_Orin_20260509`, live-verified over SSH).

## Start here

| Question | Read |
|---|---|
| I want a guided explanation of the whole stack | [`docs/GUIDE.md`](docs/GUIDE.md) |
| How the system is layered and why it is designed this way | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| How code packages and scripts are being documented | [`docs/CODE_WALKTHROUGH.md`](docs/CODE_WALKTHROUGH.md) |
| How software talks to motors, sensors, servos, mic, and battery | [`docs/HARDWARE_INTERACTIONS.md`](docs/HARDWARE_INTERACTIONS.md) |
| Offline static risks and what can be checked without the robot | [`docs/OFFLINE_AUDIT.md`](docs/OFFLINE_AUDIT.md) |
| Quick lookup for env vars, devices, topics, and output paths | [`docs/REFERENCE.md`](docs/REFERENCE.md) |
| Which launch files start which parts of the robot | [`docs/LAUNCH_MAP.md`](docs/LAUNCH_MAP.md) |
| How every vendor demo package works and touches hardware | [`docs/VENDOR_DEMOS.md`](docs/VENDOR_DEMOS.md) |
| What documentation work remains / can be delegated locally | [`docs/TASKS.md`](docs/TASKS.md) |
| What's Hiwonder's and what's ours? | [`PROVENANCE.md`](PROVENANCE.md) — master table + proof method |
| Hardware specs (board, battery, motors, servos, camera, lidar…) | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| OS, boot chain, identity env, devices, network, live stack | [`docs/SYSTEM.md`](docs/SYSTEM.md) |
| What each factory ROS 2 package does | [`docs/VENDOR_STACK.md`](docs/VENDOR_STACK.md) |
| Every change we made, with reasons and backups | [`docs/MODIFICATIONS.md`](docs/MODIFICATIONS.md) |
| Every script we wrote (nav, vision, voice, tours, reports) | [`docs/ADDED_SOFTWARE.md`](docs/ADDED_SOFTWARE.md) |
| How to actually operate the robot | [`added/guides/`](added/guides/) — operator cheat-sheet, beginner intro, handoff guide |

## Repo layout

```
vendor/            ORIGINAL  factory ROS 2 workspace src (273 MB), STM32 firmware,
                             captured factory system files, pointers to big assets
vendor-modified/   MODIFIED  live copies of factory files we changed (robot paths preserved)
added/             ADDED     robot scripts, PC tools, our SLAM maps, robot-home strays, guides
docs/                        system, architecture, code, hardware, launch, audit, and demo docs
tools/                       local validation helpers such as Markdown link checking
PROVENANCE.md                master original-vs-added table
```

Git history mirrors provenance: commit 1 = factory, commit 2 = modifications+additions, commit 3 = docs.

## The robot in one paragraph

Mecanum-drive chassis with an STM32F407 low-level controller (motors, MPU6050 IMU, HX-12H bus-servo 6-DOF arm, 11.1 V 6 Ah pack) bridged over USB serial to a Jetson Orin Nano 8 GB (JetPack 6.2, Ubuntu 22.04, ROS 2 Humble). Sensors: COIN-D6 lidar (360° on paper — **~219° usable, rear-blind, mounted 180° rotated** on this unit), arm-mounted Deptrum Aurora 930 structured-light RGB-D camera (640×400@12fps), WonderEcho Pro voice box. The factory stack auto-starts via systemd (`bringup.launch.py`: drivers, EKF odometry, camera streams on :8080, rosbridge :9090, demo apps). On top of it we added LM-Studio-backed LLM/VLM control from the PC (192.168.0.148:1234), a sherpa-onnx wake-word voice assistant, retuned SLAM/Nav2 with no-reverse behavior trees, our own maps and localization/tour tooling.

## Big things deliberately NOT in this repo

System image (66.7 GB), factory source zip, course PDFs (incl. the 94-page User Manual the hardware doc cites), dev VM, installers — all indexed with paths and sizes in [`vendor/VENDOR_ASSETS.md`](vendor/VENDOR_ASSETS.md). The vendor git history remains at `D:\robot\src_ros2\.git`.
