# Documentation Plan

This plan turns `D:\robot\Hiwonder_software_stack` into a readable guide for the Hiwonder ROSOrin Mecanum Pro software stack. The target reader should be able to understand the system at two levels:

- high level: what runs where, why the architecture is shaped this way, and how robot behaviors flow through the stack;
- low level: what important packages, scripts, configs, ROS interfaces, and hardware links actually do.

The repo already has useful factual references:

| Existing document | Current role | Follow-up needed |
|---|---|---|
| `README.md` | Entry point, provenance-aware layout, one-paragraph system overview | Keep as start page; add links to future guide docs when they exist |
| `PROVENANCE.md` | Authoritative original / modified / added classification | Treat as source of truth for ownership and change attribution |
| `docs/SYSTEM.md` | OS, boot chain, environment, devices, network, live stack | Link from architecture and operations docs |
| `docs/HARDWARE.md` | Hardware specs and important field observations | Extend with software-to-hardware interaction guide |
| `docs/VENDOR_STACK.md` | Package-level vendor stack tour | Expand into detailed package cards |
| `docs/MODIFICATIONS.md` | Changed factory files and reasons | Keep authoritative for drift and local changes |
| `docs/ADDED_SOFTWARE.md` | Added scripts, maps, tools, and status | Expand into script-level behavior cards |
| `added/guides/*.md` | Operator-facing guides and handoff notes | Cross-link from the main guide instead of duplicating commands |

## Documentation Targets

Create these documents in addition to the existing files:

| Target | Purpose |
|---|---|
| `docs/GUIDE.md` | Main human-readable guide that explains the robot from first principles and points to deeper references |
| `docs/ARCHITECTURE.md` | Layered architecture, data/control flow, boot flow, and design tradeoffs |
| `docs/CODE_WALKTHROUGH.md` | Important package and script explanations with entry points, interfaces, and failure modes |
| `docs/HARDWARE_INTERACTIONS.md` | How software talks to the STM32, lidar, camera, mic, servos, motors, battery, LEDs, buzzer, and OLED |
| `docs/TASKS.md` | Local-model-ready task queue for drafting bounded documentation sections |

Do not move or delete the existing docs. The new docs should cross-link to them and reduce duplication.

## Reader Paths

Use these paths to keep the final documentation useful to different readers.

| Reader | Read in order |
|---|---|
| New operator | `README.md` -> `added/guides/OPERATING_FOR_BEGINNERS.md` -> `added/guides/OPERATING.md` -> `docs/HARDWARE_INTERACTIONS.md` |
| Software developer | `README.md` -> `PROVENANCE.md` -> `docs/ARCHITECTURE.md` -> `docs/CODE_WALKTHROUGH.md` -> relevant source files |
| ROS/navigation maintainer | `docs/SYSTEM.md` -> `docs/ARCHITECTURE.md` -> `docs/CODE_WALKTHROUGH.md` navigation and SLAM sections -> `docs/MODIFICATIONS.md` |
| Hardware/debug maintainer | `docs/HARDWARE.md` -> `docs/HARDWARE_INTERACTIONS.md` -> `docs/SYSTEM.md` devices section -> relevant udev/config/source files |
| Local model worker | `docs/TASKS.md` -> task inputs only -> produce bounded Markdown drafts with source paths |

## Step Plan

### 1. Build a coverage map

Create a table in `docs/GUIDE.md` or `docs/TASKS.md` that marks each major area as documented, partially documented, or missing.

Minimum areas:

- system boot, shell environment, network, and service management;
- ROS workspace layout and package responsibilities;
- low-level controller and STM32 serial bridge;
- mecanum drive and odometry;
- lidar, camera, mic, IMU, battery, servos, LEDs, buzzer, OLED;
- SLAM and Nav2;
- vendor demos and app nodes;
- added scripts, maps, tours, voice assistant, and PC tools;
- known limitations, safety constraints, and failure modes.

Acceptance criteria:

- every major package directory under `vendor/src_ros2/src` appears somewhere;
- every file under `added/robot_scripts` is either documented or explicitly queued;
- every hardware device in `docs/HARDWARE.md` has a software interaction entry or a queued task.

### 2. Write `docs/GUIDE.md`

This should be the narrative guide, not another inventory.

Recommended outline:

1. What this repo is and how provenance works.
2. The robot in one mental model.
3. Boot to running ROS graph.
4. How commands become motion.
5. How sensors become topics and planner inputs.
6. How mapping and navigation fit together.
7. How voice, VLM, and local scripts sit on top of the vendor stack.
8. Where to go for operations, debugging, and source details.

Acceptance criteria:

- readable without ROS expertise, while still naming real files/topics;
- links to existing docs instead of duplicating long tables;
- includes a small glossary for ROS, Nav2, TF, costmap, behavior tree, udev, and symlink install.

### 3. Write `docs/ARCHITECTURE.md`

Explain the architecture by layers and flows.

Required sections:

- physical architecture: Jetson, STM32, sensors, actuators, PC-side LM Studio;
- software layers: systemd, shell environment, ROS 2 workspace, third-party overlays, vendor packages, added scripts;
- boot flow from `start_app_node.service` to `bringup.launch.py`;
- command flow from UI/LLM/joystick/Nav2 to `/cmd_vel`, `/controller/cmd_vel`, serial bridge, STM32, and motors;
- sensor flow from hardware to ROS topics and consumers;
- navigation flow: map, localization, costmaps, planner/controller, behavior trees, depth guard;
- design choices and tradeoffs: no-backup Nav2 behavior, symlink installs, local LM Studio, physical-port udev rules, depth scan instead of pointcloud.

Use Mermaid diagrams where they add clarity. Keep diagrams text-based so local models can edit them safely.

Acceptance criteria:

- every diagram has a short paragraph explaining what matters;
- architecture choices include the reason and the operational consequence;
- safety constraints are visible, especially rear-blind lidar and no cliff/bump sensing.

### 4. Write `docs/CODE_WALKTHROUGH.md`

Document code behavior at package/script level. Do not paste large source blocks.

For each package or script group, use this structure:

```md
## Name

**Purpose:** ...
**Where:** ...
**Entry points:** launch files, scripts, nodes, or commands.
**Main inputs:** topics, services, actions, files, environment variables.
**Main outputs:** topics, services, actions, files, hardware effects.
**Hardware touched:** ...
**Important implementation notes:** ...
**Failure modes / gotchas:** ...
**Source paths:** ...
```

Highest-priority vendor package sections:

- `driver/ros_robot_controller` and `driver/ros_robot_controller_msgs`;
- `driver/controller`;
- `driver/servo_controller` and `driver/kinematics`;
- `peripherals`;
- `slam`;
- `navigation`;
- `bringup`;
- `app` and `example`;
- `large_models` and `large_models_examples`;
- `xf_mic_asr_offline`;
- `openclaw_controller`.

Highest-priority added script groups:

- navigation/localization: `global_loc.py`, `scan_overlay.py`, `initpose.py`, `goto.py`, `goto_place.py`, `goal_pick.py`;
- mapping/exploration: `explore2.py`, `rtab_drive*.py`, `rtabmap_map.launch.py`;
- perception/arm: `vlook.py`, `look.py`, `capframe.py`, `depth_guard.py`;
- voice/LLM: `assistant.py`, `llm_drive.py`, `voice_drive.py`, `calibrate_wake.py`;
- tours/assignments: `phase_a.sh`, `tour*.sh`, `assignment*.sh`, `boot_tour.sh`, `run_tour.sh`, `prep_down.sh`;
- PC tools: `added/pc_tools/*.py`.

Acceptance criteria:

- each section explains purpose, inputs, outputs, and failure modes;
- every factual claim points to a source path or existing doc;
- generated summaries say "unknown" or "not verified" instead of guessing.

### 5. Write `docs/HARDWARE_INTERACTIONS.md`

This is the bridge between `docs/HARDWARE.md` specs and actual software behavior.

Required hardware entries:

- STM32 controller: `/dev/rrc`, `ros_robot_controller_node`, serial protocol role, exposed topics/services;
- drive motors: cmd_vel topics, mecanum behavior, speed clamps, odometry limitations;
- IMU: raw/filtered topics and EKF role;
- lidar: `/dev/lidar`, scan topics, physical rotation, rear blind spot, Nav2 consequences;
- depth camera: RGB/depth/IR/pointcloud topics, CPU constraints, `depth_guard.py`;
- WonderEcho Pro: ALSA audio, `/dev/ring_mic`, vendor ASR, added sherpa assistant;
- arm and servos: bus servo topic, ID map, safe forward pose, camera-on-arm implication;
- battery: topic, voltage interpretation, operational threshold;
- LEDs, buzzer, OLED, button, PS2 controller: topics and operational notes;
- udev rules: stable names, physical-port pinning, replugging risk.

Acceptance criteria:

- every hardware interaction names the ROS topic/service/device path when known;
- dangerous behavior has an explicit warning;
- links back to `docs/HARDWARE.md`, `docs/SYSTEM.md`, and source/config files.

### 6. Update cross-links

After the new docs exist, update:

- `README.md` start table to include `docs/GUIDE.md`, `docs/ARCHITECTURE.md`, `docs/CODE_WALKTHROUGH.md`, and `docs/HARDWARE_INTERACTIONS.md`;
- `docs/VENDOR_STACK.md` to point readers to detailed package cards;
- `docs/ADDED_SOFTWARE.md` to point readers to detailed script cards;
- `added/guides/OPERATING.md` to point maintainers back to architecture and hardware interaction docs.

Acceptance criteria:

- no broken Markdown links;
- no duplicate source-of-truth tables unless clearly justified;
- existing provenance wording remains intact.

### 7. Add and maintain `docs/TASKS.md`

Use this file as the delegation queue for a local model. Keep tasks small enough that a local model can complete one task without scanning the entire repo.

Task card template:

```md
## DOC-XXX: Short title

**Goal:** One sentence.

**Inputs:**
- Exact files/directories to read.

**Output:**
- Exact Markdown section, table, or draft file to produce.

**Constraints:**
- Do not modify runtime code.
- Do not rewrite provenance claims.
- Do not invent facts; cite source paths or mark as unknown.

**Acceptance criteria:**
- Purpose, inputs, outputs, and failure modes are covered.
- Source paths are included for every factual claim.
- The result can be reviewed independently.
```

Initial local-model task queue:

| Task | Goal | Inputs |
|---|---|---|
| DOC-001 | Draft package card for `driver/ros_robot_controller` | `vendor/src_ros2/src/driver/ros_robot_controller`, `vendor/src_ros2/src/driver/ros_robot_controller_msgs`, `docs/HARDWARE.md`, `docs/SYSTEM.md` |
| DOC-002 | Draft package card for `driver/controller` | `vendor/src_ros2/src/driver/controller`, `added/guides/OPERATING.md` |
| DOC-003 | Draft package card for `navigation` | `vendor/src_ros2/src/navigation`, `vendor-modified/ros2_ws/src/navigation`, `docs/MODIFICATIONS.md` |
| DOC-004 | Draft package card for `slam` | `vendor/src_ros2/src/slam`, `vendor-modified/ros2_ws/src/slam`, `added/robot_maps`, `docs/MODIFICATIONS.md` |
| DOC-005 | Draft package card for `peripherals` | `vendor/src_ros2/src/peripherals`, `docs/SYSTEM.md`, `docs/HARDWARE.md` |
| DOC-006 | Draft package cards for `large_models` and `large_models_examples` | `vendor/src_ros2/src/large_models`, `vendor-modified/ros2_ws/src/large_models`, `vendor/src_ros2/src/large_models_examples`, `docs/MODIFICATIONS.md` |
| DOC-007 | Draft package card for `xf_mic_asr_offline` | `vendor/src_ros2/src/xf_mic_asr_offline`, `vendor/src_ros2/src/xf_mic_asr_offline_msgs`, `docs/HARDWARE.md` |
| DOC-008 | Draft package card for `openclaw_controller` | `vendor/src_ros2/src/openclaw_controller`, `docs/VENDOR_STACK.md`, `docs/MODIFICATIONS.md` |
| DOC-009 | Draft added navigation script cards | `added/robot_scripts/global_loc.py`, `scan_overlay.py`, `initpose.py`, `goto.py`, `goto_place.py`, `goal_pick.py` |
| DOC-010 | Draft added exploration script cards | `added/robot_scripts/explore.py`, `explore2.py`, `rtab_drive.py`, `rtab_drive04.py`, `rtabmap_map.launch.py` |
| DOC-011 | Draft added perception/arm script cards | `added/robot_scripts/vlook.py`, `look.py`, `capframe.py`, `depth_guard.py`, `added/robot_home/arm_pose.py` |
| DOC-012 | Draft added voice/LLM script cards | `added/robot_scripts/assistant.py`, `llm_drive.py`, `voice_drive.py`, `calibrate_wake.py`, `added/robot_home/debug_listen.py` |
| DOC-013 | Draft tour/orchestration script cards | `added/robot_scripts/*tour*.sh`, `assignment*.sh`, `phase_a.sh`, `boot_tour.sh`, `run_tour.sh`, `prep_down.sh` |
| DOC-014 | Build hardware-to-topic map | `docs/HARDWARE.md`, `docs/SYSTEM.md`, `vendor/src_ros2/src/driver`, `vendor/src_ros2/src/peripherals`, `added/robot_scripts/depth_guard.py` |
| DOC-015 | Extract Nav2 and SLAM config explanations | `vendor-modified/ros2_ws/src/navigation/config`, `vendor-modified/ros2_ws/src/slam/config`, `docs/MODIFICATIONS.md` |

### 8. Review and quality gate

Before considering the documentation set complete:

- check all Markdown links;
- compare package/script claims against source paths;
- verify hardware claims against `docs/HARDWARE.md`, `docs/SYSTEM.md`, and relevant source/config files;
- keep warnings near unsafe actions, especially movement, battery, unauthenticated network services, and reverse motion;
- remove duplicate explanations where a link is enough;
- keep provenance consistent with `PROVENANCE.md`;
- mark stale or unverified live observations with a date.

## Style Rules

- Prefer explanations over command dumps.
- Keep source paths close to claims.
- Use tables for inventories and diagrams for flows.
- Do not imply vendor code is ours.
- Do not imply added scripts are production-stable unless the docs prove it.
- Distinguish specification, live observation, and field experience.
- Use ASCII unless preserving existing names or copied strings requires otherwise.
- When a local model drafts text, review it before treating it as authoritative.

## Completion Criteria

The documentation effort is complete when:

- a new reader can follow `docs/GUIDE.md` and understand the system shape;
- a maintainer can find the package/script responsible for a behavior;
- hardware interactions are traceable from physical device to Linux device path to ROS interface;
- navigation and safety tradeoffs are explained, not just listed;
- local-model task cards exist for unfinished or lower-priority sections;
- all new docs link back to the existing provenance and operational references.
