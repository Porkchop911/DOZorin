# Provenance — what is Hiwonder's, what is ours

Legend: **ORIGINAL** = factory as shipped · **MODIFIED** = factory file changed by us · **ADDED** = created by user/Claude · *(vendor-drift)* = shipped image newer than Hiwonder's published source (still ORIGINAL).

| Component | Status | In this repo | Lives at (authoritative) |
|---|---|---|---|
| ROS 2 workspace `src` tree (18 packages) | **ORIGINAL** | `vendor/src_ros2/` | robot `~/ros2_ws/src` |
| `openclaw_controller` refactor, YOLO `.engine` models, AR `.obj` assets | **ORIGINAL** *(vendor-drift)* | — (listed in `docs/MODIFICATIONS.md` §4) | robot `~/ros2_ws/src` |
| STM32 firmware | **ORIGINAL** | `vendor/firmware/*.hex` | STM32 board flash |
| System image, flashing tools, course PDFs, WonderAi app, dev VM | **ORIGINAL** | pointers only: `vendor/VENDOR_ASSETS.md` | `D:\robot\...` |
| `.typerc` / `.robotrc`, `start_app_node.service`, `xf_mic.rules`, `99-ttyACM0.rules`, `.stop_ros.sh`, `wifi_manager/`, `~/software/`, `~/third_party/` | **ORIGINAL** | `vendor/system/` (captured live) | robot `/etc`, `~/` |
| `nav2_params.yaml`, `nav2_controller_teb.yaml` | **MODIFIED** (retune + BT wiring + depth-guard source) | `vendor-modified/ros2_ws/src/navigation/config/` | robot `~/ros2_ws/src/navigation/config/` |
| `slam.yaml` | **MODIFIED** (CPU retune) | `vendor-modified/ros2_ws/src/slam/config/` | robot |
| `large_models/config.py`, `agent_process.py` | **MODIFIED** (LM Studio; enable_think=False) | `vendor-modified/ros2_ws/src/large_models/...` | robot |
| `/etc/udev/rules.d/lidar.rules` | **MODIFIED** (USB port repin) | `vendor-modified/etc/udev/rules.d/` | robot |
| Timezone Europe/Vienna | **MODIFIED** | noted in `docs/MODIFICATIONS.md` | robot |
| No-backup Nav2 behavior trees (2 XML) | **ADDED** (installed into factory tree) | `vendor-modified/ros2_ws/src/navigation/config/` + mirror in `added/robot_scripts/` | robot |
| SLAM maps `map_01/02/04/rtab/remap` | **ADDED** (`map_03`, `test` are factory) | `added/robot_maps/` | robot `~/ros2_ws/src/slam/maps/` |
| ~35 robot scripts (nav, explore, vision, voice, tours) + nav configs mirror | **ADDED** | `added/robot_scripts/` | robot `~/` (PC mirror `D:\robot\robot_scripts`) |
| Robot-home strays (`arm_pose.py`, `debug_listen.py`, `find_places_down.py`, `places.yaml`, `allzeit.wav`, `bin/robot_stack_power.py`) | **ADDED** | `added/robot_home/` | robot `~/` |
| PC report tools | **ADDED** | `added/pc_tools/` | `D:\robot\pc_tools` |
| Operating guides (3 × .md) | **ADDED** | `added/guides/` | `D:\robot\` |
| LM Studio serving + model pinning, Hermes `rosorin-robot` skill | **ADDED** (PC-side) | described in `docs/MODIFICATIONS.md` §3 | PC `192.168.0.148`, `D:\hermes\automation` |

**Proof chain:** `D:\robot\src_ros2\.git` holds Hiwonder's own 7-commit history plus our import of the shipped source ("Baseline", `422acec`). The robot's live tree was md5-diffed against that baseline, the Chinese-comment vendor variant, and the published zip; survivors were attributed by mtime (user era begins ~2026-06-23). Method and full results: `docs/MODIFICATIONS.md`.

This repo's own git history mirrors the classification: commit 1 = ORIGINAL (`vendor/`), commit 2 = MODIFIED + ADDED (`vendor-modified/`, `added/`), commit 3 = documentation.
