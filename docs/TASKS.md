# Documentation Tasks

This is the delegation queue for producing detailed documentation in small, reviewable pieces. It is designed for local-model work: each task has bounded inputs, a clear output, and acceptance criteria.

Rules for every task:

- do not modify runtime code;
- do not rewrite provenance claims from `PROVENANCE.md`;
- cite source paths for factual claims;
- mark unknowns instead of guessing;
- keep output as Markdown that can be reviewed and merged into `docs/CODE_WALKTHROUGH.md`, `docs/HARDWARE_INTERACTIONS.md`, or `docs/ARCHITECTURE.md`.

## Task Template

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

## Coverage Matrix

| Area | Current coverage | Next task |
|---|---|---|
| Provenance and repo layout | Strong in `PROVENANCE.md` and `README.md` | Keep linked from all new docs |
| OS, boot, shell, network, live graph | Strong in `docs/SYSTEM.md` | Add architecture cross-links |
| Hardware specifications | Strong in `docs/HARDWARE.md` | DOC-014 for software interaction map |
| Vendor package overview | Strong in `docs/VENDOR_STACK.md`, `docs/CODE_WALKTHROUGH.md`, and `docs/VENDOR_DEMOS.md`; vendor demo packages now have a dedicated source-derived guide | Per-demo topic/service extraction can still be delegated for exhaustive tables |
| User modifications | Strong in `docs/MODIFICATIONS.md` | Link from architecture/code docs |
| Added scripts overview | Partial in `docs/ADDED_SOFTWARE.md`; all major script groups started and key variants source-backed in `docs/CODE_WALKTHROUGH.md` | Live validation and lower-priority edge cases remain open |
| Nav2/SLAM config explanation | Partial in `docs/MODIFICATIONS.md` | DOC-015 |
| Hardware-to-topic mapping | Started in `docs/HARDWARE_INTERACTIONS.md` with source-backed bridge/controller/lidar/camera/IMU/servo/kinematics/joystick/openclaw/voice notes | DOC-014 for live validation and lower-priority interface edges |
| Architecture diagrams | Started in `docs/ARCHITECTURE.md` | Review after package cards |
| Launch flow mapping | Started in `docs/LAUNCH_MAP.md` | Expand only if new launch families become important |
| Operator runbooks | Strong in `added/guides` | Keep cross-linked |
| Offline static audit | Started in `docs/OFFLINE_AUDIT.md` with raw-drive, hardcoded path/IP, and known bug leads | Keep current as source changes |
| Reference matrix | Started in `docs/REFERENCE.md` with env vars, devices, topics, and output paths | Expand only when new constants are discovered |

## Initial Queue

## DOC-001: Package card for `driver/ros_robot_controller`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md` and `docs/HARDWARE_INTERACTIONS.md`; message surface and SDK packet framing are covered. Remaining work is only a deeper edge-case review of all SDK subcommands if needed.

**Goal:** Explain the ROS-to-STM32 bridge and its message/service surface.

**Inputs:**
- `vendor/src_ros2/src/driver/ros_robot_controller`
- `vendor/src_ros2/src/driver/ros_robot_controller_msgs`
- `docs/HARDWARE.md`
- `docs/SYSTEM.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Covers `/dev/rrc`, serial bridge role, topics/services, STM32 hardware effects, and common failure modes.

## DOC-002: Package card for `driver/controller`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; mecanum math, EKF inputs, Ackermann branch, and alternate chassis behavior are covered. Remaining work is live validation on non-mecanum hardware only.

**Goal:** Explain velocity handling, mecanum control, odometry, and EKF configuration.

**Inputs:**
- `vendor/src_ros2/src/driver/controller`
- `added/guides/OPERATING.md`
- `docs/HARDWARE.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Distinguishes `/cmd_vel`, `/controller/cmd_vel`, and `/app/cmd_vel`; notes odometry limitations.

## DOC-003: Package card for `navigation`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md` and `docs/ARCHITECTURE.md`; behavior-tree XML, high-impact Nav2 params, TEB notes, and no-backup rationale are covered. Remaining work is full parameter-by-parameter review only if needed.

**Goal:** Explain the Nav2 stack, launch/config entry points, TEB use, and no-backup behavior.

**Inputs:**
- `vendor/src_ros2/src/navigation`
- `vendor-modified/ros2_ws/src/navigation`
- `docs/MODIFICATIONS.md`
- `docs/HARDWARE.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Identifies changed configs and behavior trees; explains why reverse recovery is disabled.

## DOC-004: Package card for `slam`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; modified `slam.yaml` deltas, include launches, mapping modes, and RTAB-Map flow are covered. Remaining work is live map/reliability annotation.

**Goal:** Explain mapping launches, map files, and user tuning.

**Inputs:**
- `vendor/src_ros2/src/slam`
- `vendor-modified/ros2_ws/src/slam`
- `added/robot_maps`
- `docs/MODIFICATIONS.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Lists supported mapping modes and explains current map status without overstating reliability.

## DOC-005: Package card for `peripherals`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; joystick, keyboard, SCLIDAR, alternate lidar include selection, Aurora, ascamera, and USB camera fallback paths are covered. Remaining work is live validation on hardware variants not present in this robot.

**Goal:** Explain lidar, depth camera, IMU filter, joystick, and teleop support.

**Inputs:**
- `vendor/src_ros2/src/peripherals`
- `docs/SYSTEM.md`
- `docs/HARDWARE.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Maps hardware devices to launch files/topics where source confirms them.

## DOC-006: Package cards for `large_models` and `large_models_examples`

**Status:** Guide-level source coverage in `docs/CODE_WALKTHROUGH.md`; `large_models` launch/config and `large_models_examples` entry-point taxonomy are covered. Remaining work is deep per-demo behavior and live verification of the offline launch caveat.

**Goal:** Explain the vendor large-model framework and demo package.

**Inputs:**
- `vendor/src_ros2/src/large_models`
- `vendor-modified/ros2_ws/src/large_models`
- `vendor/src_ros2/src/large_models_examples`
- `docs/MODIFICATIONS.md`

**Output:**
- Draft sections for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Notes LM Studio config changes and ASR mode caveats from `docs/MODIFICATIONS.md`.

## DOC-007: Package card for `xf_mic_asr_offline`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; command vocabulary, demo launches, and main demo script ROS effects are covered. Remaining work is live speech-recognition validation and any lower-priority phrase tuning.

**Goal:** Explain the vendor iFlytek offline ASR/wake-word stack.

**Inputs:**
- `vendor/src_ros2/src/xf_mic_asr_offline`
- `vendor/src_ros2/src/xf_mic_asr_offline_msgs`
- `docs/HARDWARE.md`
- `docs/SYSTEM.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Distinguishes audio device behavior from `/dev/ring_mic` serial behavior.

## DOC-008: Package card for `openclaw_controller`

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; launch-by-launch behavior is covered. Remaining work is live-image drift validation.

**Goal:** Explain gripper/arm scene apps and local gateway behavior.

**Inputs:**
- `vendor/src_ros2/src/openclaw_controller`
- `docs/VENDOR_STACK.md`
- `docs/MODIFICATIONS.md`
- `added/guides/OPERATING.md`

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Notes vendor drift and does not assume published source exactly matches live image behavior.

## DOC-009: Added navigation script cards

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; `initpose.py` behavior, covariance, and defaults are covered. Remaining work is only any missing edge cases discovered during live use.

**Goal:** Explain added scripts for localization and goal navigation.

**Inputs:**
- `added/robot_scripts/global_loc.py`
- `added/robot_scripts/scan_overlay.py`
- `added/robot_scripts/initpose.py`
- `added/robot_scripts/goto.py`
- `added/robot_scripts/goto_place.py`
- `added/robot_scripts/goal_pick.py`
- `docs/ADDED_SOFTWARE.md`

**Output:**
- Draft script-card group for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- For each script, covers purpose, inputs, outputs, required ROS state, and failure modes.

## DOC-010: Added exploration script cards

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; exact differences between `explore.py`, `explore2.py`, `rtab_drive.py`, and `rtab_drive04.py` are covered. Remaining work is live reliability notes.

**Goal:** Explain exploration, RTAB-Map mapping helpers, and current reliability.

**Inputs:**
- `added/robot_scripts/explore.py`
- `added/robot_scripts/explore2.py`
- `added/robot_scripts/rtab_drive.py`
- `added/robot_scripts/rtab_drive04.py`
- `added/robot_scripts/rtabmap_map.launch.py`
- `docs/ADDED_SOFTWARE.md`

**Output:**
- Draft script-card group for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Identifies which scripts are superseded and which are current best effort.

## DOC-011: Added perception and arm script cards

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; `look.py`, `capframe.py`, and `arm_pose.py` behavior is covered. Remaining work is live validation of camera/servo assumptions.

**Goal:** Explain camera, VLM, depth guard, and arm-pose helper scripts.

**Inputs:**
- `added/robot_scripts/vlook.py`
- `added/robot_scripts/look.py`
- `added/robot_scripts/capframe.py`
- `added/robot_scripts/depth_guard.py`
- `added/robot_home/arm_pose.py`
- `docs/HARDWARE.md`
- `docs/ADDED_SOFTWARE.md`

**Output:**
- Draft script-card group for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Explains camera-on-arm implications and why `depth_guard.py` exists.

## DOC-012: Added voice and LLM script cards

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; `calibrate_wake.py` and `debug_listen.py` behavior is covered. Remaining work is live microphone/model validation.

**Goal:** Explain added wake-word, speech transcription, and LLM drive scripts.

**Inputs:**
- `added/robot_scripts/assistant.py`
- `added/robot_scripts/llm_drive.py`
- `added/robot_scripts/voice_drive.py`
- `added/robot_scripts/calibrate_wake.py`
- `added/robot_home/debug_listen.py`
- `docs/ADDED_SOFTWARE.md`

**Output:**
- Draft script-card group for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Includes safety warning that the mic/assistant should not be started without user intent.

## DOC-013: Added tour and orchestration script cards

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; `boot_tour.sh`, `run_tour.sh`, `prep_down.sh`, `tour_down1.sh`, and older assignment variants are covered. Remaining work is live run-result annotation.

**Goal:** Explain shell orchestration scripts for assignments, tours, and prep flows.

**Inputs:**
- `added/robot_scripts/assignment*.sh`
- `added/robot_scripts/*tour*.sh`
- `added/robot_scripts/phase_a.sh`
- `added/robot_scripts/boot_tour.sh`
- `added/robot_scripts/run_tour.sh`
- `added/robot_scripts/prep_down.sh`
- `docs/ADDED_SOFTWARE.md`
- `added/guides/ROBOT_HANDOFF_GUIDE.md`

**Output:**
- Draft script-card group for `docs/CODE_WALKTHROUGH.md`.

**Acceptance criteria:**
- Covers ordering, timeouts, localization gates, photo proof, and known incomplete status.

## DOC-014: Hardware-to-topic map

**Status:** Source-backed in `docs/HARDWARE_INTERACTIONS.md`; bridge, drive, lidar, depth camera, IMU, servo-controller, kinematics, joystick, keyboard, depth-guard, openclaw, and voice package paths are covered. Remaining work is live validation and lower-priority device/interface edge cases.

**Goal:** Complete `docs/HARDWARE_INTERACTIONS.md` with source-backed device/topic mappings.

**Inputs:**
- `docs/HARDWARE.md`
- `docs/SYSTEM.md`
- `vendor/src_ros2/src/driver`
- `vendor/src_ros2/src/peripherals`
- `added/robot_scripts/depth_guard.py`

**Output:**
- Expanded tables for `docs/HARDWARE_INTERACTIONS.md`.

**Acceptance criteria:**
- Each row includes hardware, Linux device path if any, ROS interface, responsible package/node, and notes.

## DOC-015: Nav2 and SLAM config explanations

**Status:** Source-backed at high-impact level in `docs/CODE_WALKTHROUGH.md` and `docs/ARCHITECTURE.md`; remaining work is exhaustive TEB/Nav2 parameter annotation only if wanted.

**Goal:** Explain meaningful configuration changes in user-modified Nav2 and SLAM files.

**Inputs:**
- `vendor-modified/ros2_ws/src/navigation/config`
- `vendor-modified/ros2_ws/src/slam/config`
- `docs/MODIFICATIONS.md`
- `docs/HARDWARE.md`

**Output:**
- Draft config explanation section for `docs/CODE_WALKTHROUGH.md` or `docs/ARCHITECTURE.md`.

**Acceptance criteria:**
- Explains the reason and consequence of each high-impact config change.

## DOC-016: PC report tools reuse guide

**Status:** Source-backed in `docs/CODE_WALKTHROUGH.md`; tool inputs, outputs, hardcoded paths, and reuse caveats are covered. Remaining work is an implementation refactor only if requested.

**Goal:** Document or draft a refactor plan for making the PC report tools reusable instead of scratchpad-specific.

**Inputs:**
- `added/pc_tools/synth.py`
- `added/pc_tools/gen_report.py`
- `added/pc_tools/gen_report2.py`
- Any available sample `scout.json`, `report.json`, images, map image, or synthesis files.

**Output:**
- Draft section for `docs/CODE_WALKTHROUGH.md` or a short `docs/PC_REPORT_TOOLS.md` plan.

**Constraints:**
- Do not edit the scripts unless implementation is explicitly requested.
- Identify hardcoded paths and expected input schemas.
- Mark unavailable sample inputs as unknown.

**Acceptance criteria:**
- Lists required input files for each tool.
- Explains generated outputs.
- Recommends CLI parameters needed to make each tool reusable.

## DOC-017: Keep offline audit current

**Status:** Started in `docs/OFFLINE_AUDIT.md`; current high-value static findings are documented.

**Goal:** Keep source-only risk findings current as runtime code changes.

**Inputs:**
- `docs/OFFLINE_AUDIT.md`
- `docs/REFERENCE.md`
- `tools/check_markdown_links.ps1`
- Source scans over `vendor/`, `vendor-modified/`, and `added/`

**Output:**
- Patch to `docs/OFFLINE_AUDIT.md` and `docs/REFERENCE.md`.

**Constraints:**
- Do not claim live behavior unless verified on the robot.
- Keep findings source-backed and actionable.

**Acceptance criteria:**
- New raw motion paths, hardcoded deployment assumptions, and likely static bugs are captured.
- Residual live-validation gaps remain clearly marked.

## DOC-018: Exhaustive per-demo interface tables

**Status:** Started in `docs/VENDOR_DEMOS.md`; package-level and family-level demo behavior is covered, with stale entry points and major raw-motion/arm/voice risks identified.

**Goal:** Expand each vendor demo family into exact topic/service/parameter tables.

**Inputs:**
- `docs/VENDOR_DEMOS.md`
- One subdirectory at a time under `vendor/src_ros2/src/app`, `vendor/src_ros2/src/example`, `vendor/src_ros2/src/large_models_examples`, `vendor/src_ros2/src/openclaw_controller`, or `vendor/src_ros2/src/xf_mic_asr_offline`

**Output:**
- Patch or draft subsection for `docs/VENDOR_DEMOS.md`.

**Constraints:**
- Do not modify runtime code.
- Cite source paths for every interface claim.
- Keep source facts separate from inferred behavior.

**Acceptance criteria:**
- Each demo row includes launch/entry point, subscribed topics, published topics, services/actions used or exposed, parameters/env assumptions, hardware effects, and likely failure modes.
