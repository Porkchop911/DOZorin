# Added software — everything user/Claude-built (June–July 2026)

Lives outside the factory packages: on the robot in `/home/ubuntu/` (authoritative), mirrored on the PC at `D:\robot\robot_scripts` → committed here under `added/robot_scripts/`. Strays that existed only on the robot were fetched live into `added/robot_home/`. PC-side tools: `added/pc_tools/`. Hand-written manuals: `added/guides/` (`OPERATING.md` cheat-sheet, `OPERATING_FOR_BEGINNERS.md` intro, `ROBOT_HANDOFF_GUIDE.md` — the richest operational reference).

Common conventions: shell scripts are `#!/bin/zsh` + `source ~/.zshrc`; a `batt()` helper reads `/ros_robot_controller/battery`; the arm is locked to the forward pose `(1,500)(2,701)(3,120)(4,300)(5,500)` before driving; runs log to `~/scout_out/`.

## Navigation / localization (`added/robot_scripts/`)

| Script | Purpose |
|---|---|
| `global_loc.py` | Headless global localization: scan-matches `/scan` over the saved map's free cells, publishes best `/initialpose` (map via `MAPY` env, default `map_04.yaml`) |
| `scan_overlay.py` | Renders scan-vs-map PNG to `/tmp/overlay.png` to verify localization off-board |
| `initpose.py` / `goto.py` / `goto_place.py` | Manual `/initialpose` seed; Nav2 `NavigateToPose` goal with feedback; drive to named waypoint from `~/places.yaml` |
| `goal_pick.py` | Prints an approach pose for a named station that is actually FREE in the live global costmap, facing the station |
| `find_places.py` (+ `robot_home/find_places_down.py`) | Discover named targets by arm-cam pan sweep + VLM + depth, propose standoff poses, optionally drive & photo-verify, write `~/places.yaml` |
| `depth_guard.py` | Condenses the arm depth image into a sparse synthetic `LaserScan` (~64 beams ≤3 Hz) for the local costmap — replaces the CPU-starving 12 Hz pointcloud |
| `nav2_params.yaml`, `nav2_controller_teb.yaml`, `navigate_*_no_backup.xml`, `map_04.*`, `places.yaml` | PC mirrors of the installed nav config / map / waypoints (live versions in `vendor-modified/` and `added/robot_maps/`) |

## Exploration / patrol

| Script | Purpose |
|---|---|
| `explore2.py` | Continuous reactive frontier explorer + vision scout (lidar-180° handling, depth bumper for thin obstacles, STOP kill-switch). Supersedes `explore.py` |
| `rtab_drive.py` / `rtab_drive04.py` | Frontier explorer variants tuned for RTAB-Map / map_04 |
| `rtabmap_map.launch.py` | Jetson-tuned RTAB-Map mapping on top of the running base stack (wheel/EKF odom, lidar grid, camera loop closure) |
| `patrol.py` / `patrol2.py` | Waypoint vision patrols (pan-sweep + VLM at each stop; needs Nav2) |

## Vision + arm

| Script | Purpose |
|---|---|
| `vlook.py` | Aim arm forward at a given pan, grab `/depth_cam/rgb0/image_raw` frame, describe via VLM (`qwen/qwen3-vl-4b`). Current best |
| `look.py` | Older variant (high-level servo path + `:8080` snapshot) |
| `capframe.py` | Save one RGB frame (cv_bridge reference) |
| `robot_home/arm_pose.py` | Set a raw arm pose |

## Voice / LLM

| Script | Purpose |
|---|---|
| `assistant.py` | Always-on wake-word assistant: `arecord` → sherpa-onnx streaming wake word (Czech "Soudruhu") → plays `allzeit.wav` ("Allzeit bereit!") → 3.5 s command window → sherpa Whisper base.en → routes to `look.py` / `llm_drive.py`. **Never start the mic without the user's go** |
| `llm_drive.py` | Plain English → LM Studio → mecanum action JSON → `/controller/cmd_vel` at 20 Hz per segment + stop |
| `voice_drive.py` | Push-to-talk: record N s, sherpa-transcribe locally, pipe to `llm_drive.py` |
| `calibrate_wake.py` | Tune how the English sherpa model hears a new wake word |
| `robot_home/debug_listen.py` | Mic debugging aid |

## Assignments / tours (zsh orchestration)

`tour.sh` (strict drums→door→window→start with ≥85% localization gate), `run_tour.sh` (detached, survives SSH drops), `boot_tour.sh` (full chain from cold boot), `phase_a.sh` (bring-up + localize only; kills CPU-hog apps), `assignment.sh`…`assignment4.sh` (the standing assignment with retries, hard timeouts, costmap clears, ESCAPE maneuver), `prep_down.sh` / `tour_down1.sh` (downstairs on `map_rtab`), `goal_pick.py` above feeds them. Photo proof per station via the `:8080` snapshot server.

## PC-side (`added/pc_tools/`)

| Script | Purpose |
|---|---|
| `synth.py` | Feeds per-view VLM object lists (`scout.json`) to LM Studio for a deduplicated room report (markdown) |
| `gen_report.py` / `gen_report2.py` | Self-contained HTML reports with base64-embedded patrol frames |

## Supporting bits

- `robot_home/allzeit.wav` — wake acknowledgement sound (espeak-ng).
- `robot_home/bin/robot_stack_power.py` — stack power helper.
- `robot_home/places.yaml` (+ `places_upstairs.bak.yaml`) — named waypoints on map_04, photo-verified 2026-07-05: `window`, `drums`, `desk` ("door" unrecorded — the map's east pocket is a lidar-through-glass artifact).
- On-robot only (not committed): sherpa-onnx models under `~/third_party/sherpa-onnx/`, `~/scout_out/` run logs/photos, `explore2.py.bak_*` iteration backups.

## Status honesty

Capabilities are graded by outcome, not code existence: navigation is **not** "done" (best supervised run 6/11 legs; stack dies on reboot without `phase_a.sh`-style care). The standing assignment (drums→door→window→start with photo proof) remains OPEN. See `added/guides/ROBOT_HANDOFF_GUIDE.md` §12 for known issues.
