# Offline Static Audit

This audit records issues and risk areas that can be found without a live robot. It is intentionally source-based: every item below should be treated as a lead for review or live validation, not proof of runtime failure.

## Highest-Value Findings

| Area | Source evidence | Risk | Suggested next step |
|---|---|---|---|
| Raw motion topic is widely used | `added/robot_scripts/*.py`, `added/robot_scripts/*.sh`, `vendor/src_ros2/src/app`, `vendor/src_ros2/src/example`, `vendor/src_ros2/src/large_models_examples` publish `/controller/cmd_vel` | This bypasses the safer `/cmd_vel` clamp in `controller/odom_publisher_node.py` | Keep a supervised-run list; prefer `/cmd_vel` for new user scripts unless raw behavior is intentional |
| Offline large-model launch bug | `vendor/src_ros2/src/large_models/large_models/launch/agent_process.launch.py` appends to `z`, but no `z` is defined in that file | `ASR_MODE=offline` launch path can fail before the model node starts | Patch or live-test before relying on offline Ollama mode |
| Servo trajectory action code has likely name errors | `driver/servo_controller/servo_controller/joint_trajectory_action_controller.py` references names such as `self.self.joint_names`, `pos_rad_to_raw`, and result symbols that do not match local imports | `follow_joint_trajectory` path may fail even if simpler pulse/joint topic paths work | Prefer `servo_controller` topic and direct bus-servo paths until trajectory action is fixed/tested |
| `set_odom` callback appears broken | `driver/controller/controller/odom_publisher_node.py` calls `self.clock().now()` even though `self.clock` is assigned as an object | Publishing `set_odom` can crash odom publisher | Avoid `set_odom`; use localization initial pose workflows instead |
| Hardcoded deployment paths are pervasive | `/home/ubuntu/ros2_ws/src/...`, `/home/ubuntu/software/...`, `~/scout_out`, and `~/third_party/...` appear throughout vendor and added scripts | Moving scripts to another user/workspace breaks unless paths are adapted | Parameterize new scripts; document required robot-home layout for legacy scripts |
| Hardcoded model backend is pervasive in added scripts | Added scripts use `http://192.168.0.148:1234/v1/chat/completions` and model IDs such as `qwen/qwen3-vl-4b`, `qwen3.5-9b`, and `gemma-4-26b-a4b-it` | PC IP/model mismatch breaks VLM/LLM workflows silently or at runtime | Add an env-var based config wrapper before making these scripts reusable |
| Shell scripts kill broad process patterns | `phase_a.sh`, `prep_down.sh`, assignment scripts, and factory `.stop_ros.sh` use `pkill`, `pkill -9`, or generated `kill -9` commands | Useful operationally, but can stop unrelated matching ROS processes | Keep these scripts operator-only; avoid calling them from automation without confirmation |
| Rear-blind robot still has many generic demos | Vendor app/example packages publish raw drive commands and were not designed around this unit's rear-blind lidar constraints | Generic demos may reverse/turn into unseen obstacles | Treat vendor demos as unsafe until reviewed for this physical unit |
| Python parse pass has one warning | `python -m compileall -q added vendor/src_ros2/src vendor-modified/ros2_ws/src` reports `driver/sdk/sdk/pid.py` invalid escape sequence `\i` in a docstring | Non-fatal today, but future Python versions may treat invalid escapes more strictly | Change the docstring to a raw string or escape the backslash if runtime code maintenance is allowed |

## Raw Motion Publishers

The scan found these notable raw-drive groups:

| Group | Examples | Notes |
|---|---|---|
| Added autonomy and LLM scripts | `llm_drive.py`, `explore.py`, `explore2.py`, `rtab_drive.py`, `rtab_drive04.py`, `assignment4.sh` | User-authored and already documented as supervised/raw-drive |
| Vendor voice demos | `xf_mic_asr_offline/scripts/voice_control_move.py`, `voice_control_navigation.py` | Voice commands can produce raw motion |
| Vendor app demos | `app/line_following.py`, `app/object_tracking.py`, `app/lidar_controller.py`, `app/self_driving.py` | These are often auto-start or demo workflows; `phase_a.sh` kills several CPU-heavy app/demo nodes |
| Vendor examples | `example/color_track`, `example/garbage_classification`, `example/navigation_transport`, RGB-D and body-control examples | Many are full robot demos, not passive perception nodes |
| Large-model examples | `large_models_examples/*llm*`, road-network tools, transport/pick demos | LLM/VLM examples can command base and arm hardware |

Use `docs/HARDWARE_INTERACTIONS.md` for the meaning of `/cmd_vel` versus `/controller/cmd_vel`.

## Hardcoded Runtime Assumptions

| Assumption | Source areas | Consequence |
|---|---|---|
| Robot user is `ubuntu` and workspace is `/home/ubuntu/ros2_ws` | Most launch files when `need_compile=False`, added shell scripts | Captured stack is not relocatable without edits |
| Stable devices are `/dev/rrc`, `/dev/lidar`, `/dev/ring_mic` | Controller, lidar, mic packages | udev rules and physical USB port mapping are part of the runtime contract |
| WonderEcho ALSA capture is `plughw:0,0` | Added assistant/debug scripts | Audio scripts are tied to this unit's ALSA ordering |
| LM Studio server is `192.168.0.148:1234` | Modified `large_models` config and added VLM/LLM scripts | PC-side backend must be up and model-loaded |
| ROS identity/env comes from shell startup | zsh scripts source `~/.zshrc`; launch files read `MACHINE_TYPE`, `need_compile`, `HOST`, `MASTER`, `ASR_MODE`, `ASR_LANGUAGE` | Launching from a clean shell can select wrong paths or crash on missing env vars |

## Checks Run Offline

- Markdown local-link validation over the new docs and touched entry docs.
- Static source scans with `rg` for hardcoded IPs/paths/devices, raw motion publishers, process-kill patterns, and known suspicious names.
- Repeatable scan helper: `tools/static_source_scan.ps1`.
- Python parse pass with `python -m compileall -q added vendor/src_ros2/src vendor-modified/ros2_ws/src`; no syntax failures, one non-fatal warning noted above.
- Source-reading passes over high-risk packages documented in `docs/CODE_WALKTHROUGH.md`.

## Not Verified Without Robot

- ROS graph names and remaps after launch expansion on the Jetson.
- Whether device nodes exist and are owned by the expected process.
- Camera, lidar, IMU, mic, battery, and servo telemetry.
- Nav2 localization quality, map correctness, and costmap behavior.
- Physical arm poses and collision margins.
- Voice recognition accuracy and LM Studio model availability.
