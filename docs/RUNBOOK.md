# HWR operations runbook

The factory stack remains the recovery path. Keep the robot attended until the
Phase 8 soak gate is accepted. Power decisions belong to the human operator.

## Normal session

1. The human operator confirms the robot is ready.
2. Run `deploy/takeover.sh`, then use `deploy/mode.sh` to launch the required
   independent mode. The mode helper sources `deploy/env.zsh` itself and keeps
   the launch plus every child node in one stoppable process group.
   Navigation mode uses the autonomy launch, so the authenticated gateway and
   CLI/VLM snapshot path come up with navigation rather than as orphan processes.
3. Stop motion before changing mapping/localization modes; only one may own
   `map->odom`.
4. After a reboot, navigation starts the HWR arm service but does not move the
   arm automatically. If the camera is not forward-facing, with the base stopped
   call the live-verified `forward` named pose, wait for interpolation to finish,
   then re-latch estop before validating depth ranges.
5. Run `deploy/handback.sh` and verify scan and web UI health.

Typical mode commands on the robot:

```zsh
~/hwr/deploy/mode.sh start mapping
~/hwr/deploy/mode.sh restart localization /home/ubuntu/hwr/maps/hwr_map_01.yaml
~/hwr/deploy/mode.sh restart navigation /home/ubuntu/hwr/maps/hwr_map_01.yaml
~/hwr/deploy/mode.sh stop
```

Do not start long-lived nodes from only `ros2_ws/install/setup.zsh`;
`deploy/env.zsh` also exposes the framework-free `hwr_core` package.

## Boot at power-on (hwr.service) — first test

Not yet run on the robot (designed 2026-10-17). Sources: `deploy/hwr.service`,
`deploy/hwr-ears.service`, `deploy/boot.sh`, `deploy/pc_up.ps1`,
`config/hwr.env.example`, plan/14 §12.6, plan/15 "After a reboot".

### One-time setup

Robot:

1. In `~/hwr/config/hwr.env` (copy of `config/hwr.env.example`) fill:
   `HWR_MAP_YAML`, `HWR_HERMES_URL` (`http://192.168.0.148:12341`),
   `HWR_HERMES_KEY`, `HWR_HERMES_SESSION`. Keep `HWR_GATEWAY_TOKEN` set.
2. Sync and build.
3. `~/hwr/deploy/boot.sh install`: installs `hwr.service` and `hwr-ears.service`, disabled.
4. `~/hwr/deploy/boot.sh enable`: disables `start_app_node.service` (factory stack),
   enables and starts `hwr.service`. Ears follows via `Wants=`.
5. Revert: `~/hwr/deploy/boot.sh disable` stops `hwr.service` and restores the factory stack as boot default.

PC (repo root):

1. Start Strata on :8080 (user's job).
2. `deploy\pc_up.ps1` starts relay :12340→8080, relay :12341→8643, and
   `hermes -p robot gateway`. Warns if :8080 is down; does not start Strata.
3. Optional: `deploy\pc_up.ps1 -Register` adds logon task "HWR pc_up".

### Cold-boot check list

| Check | Command | Expect |
|---|---|---|
| Boot unit | `~/hwr/deploy/boot.sh status` | `hwr.service` enabled; `hwr-ears.service` active |
| Mode | `~/hwr/deploy/mode.sh status` | navigation running |
| Ears | `journalctl -u hwr-ears -f` | listening; no restart loop (restarts every 10 s on exit) |
| rosbridge | `ss -ltn \| grep 9090` | listening (skipped with a log line if the package is missing) |
| Gateway | `python3 ~/hwr/cli/hwr.py --url http://127.0.0.1:8090 nav-context` (on the robot) | answers; `camera_fresh: true` |

### Manual steps (deliberately not automated)

| Step | Why |
|---|---|
| 1. Estop reset (`reset_estop`, once) | Driver latches estop at every start, by design (boot into stopped state). |
| 2. Arm `forward` (`arm_pose('forward')`) | Arm never moves unasked. Goals are refused until `camera_forward: true`. |
| 3. Relocalize (`relocalize`), robot stationary | Boot does not know where the robot stands on the map. |

Hermes or the human performs these. Power decisions stay with the human.

### Acceptance

- Cold power-on, no SSH.
- After the human estop reset, "Soudruhu, turn left" moves the robot (speech-end → motion ≤ 1.5 s).
- Record: reboots worked / reboots tried = ___ / ___.

### Known risks to check on first boot

- `hwr-ears.service` runs `/usr/bin/python3`: confirm it has `sherpa_onnx`.
- Ears waits up to 180 s for `/run/user/1000/pulse/native`, then fails and retries every 10 s: confirm the robot user is uid 1000 and its desktop session (PulseAudio) starts at boot.
- `hwr.service` must win the camera from the factory stack (`takeover.sh`); a leftover factory node holds the camera.
- PC: `pc_up.ps1` does not start Strata; without it the relays and Hermes calls fail.

## First gateway setup

1. Run `~/hwr/deploy/install_deps.sh` once.
2. Copy `config/hwr.env.example` to `config/hwr.env`, generate a random token of
   at least 24 characters, and keep the real file uncommitted.
3. Set the same `HWR_GATEWAY_TOKEN` on the PC. The gateway listens on port 8090;
   every route requires the token, including telemetry.
4. Named places are per map: the gateway reads only `<map>.places.yaml` beside
   the loaded map YAML (`mark_place` writes the robot's localized pose there as a
   verified reference place; `list_places` shows them). The packaged
   `hwr_nav/config/places.yaml` (old map_04 frame) is no longer read; copy an
   object entry from it into a map's places file only after it has been
   physically rechecked on that map. For an
   object place, measure `object_x/object_y` on the physical object, then choose
   a collision-free `x/y` approach pose whose robot-center distance to that
   anchor is at most 0.50 m. Only then mark it verified. A camera view or photo
   proves recognition, not arrival; the gateway independently checks the final
   localized distance and changes an otherwise successful navigation result to
   failed when the object is farther than the configured limit.
   Place validation also rejects an approach `x/y` outside that same radius,
   before any goal can be dispatched.
   Extended surfaces may additionally define `object_end_x/object_end_y`; both
   fields are required together, and arrival is measured to the closest point
   on that finite physical segment.
5. Keep LM Studio bound to Windows localhost and run semantic vision from the
   PC with `python cli/hwr.py describe`; this avoids exposing the model server
   on the LAN while the CLI fetches the authenticated robot snapshot.
6. Automated perception and mission clients must read
   `GET /navigation/context`, never `GET /status`. The navigation endpoint is
   intentionally battery-free. `/status` is reserved for a human operator's
   explicit telemetry request and must not be polled by an agent.
7. Before any autonomous translation, verify that `/navigation/context` reports
   both `camera_fresh: true` and `camera_forward: true`. `camera_forward` is
   derived from fresh servo readback against the live-verified `forward` pose;
   a stale joint state or an arm aimed at the floor fails closed. The gateway
   rejects translation goals and exploration starts while this flag is false.

## Stationary object-approach planning

`tools/run_object_approach.py` defaults to a stationary dry run. Its explicit
`--execute --semantic-preflight` mode performs bounded staged execution: each
leg is at most 0.15 m, the target is re-grounded, localization must remain
stable, rotation receives a fresh metric preflight, translation receives both
metric and semantic corridor preflights, and the gateway stop is re-latched
after every action. A failed physical leg terminates the run without trying a
different candidate.

The tool ranks live-costmap approaches 0.30–0.48 m from a point or extended
surface, faces each pose toward the closest physical target point, and prints
its path length, clearance, cell cost, and exact object distance. Examples on
the robot:

```zsh
source ~/hwr/deploy/env.zsh
cd ~/hwr
python3 tools/run_object_approach.py --place window
python3 tools/run_object_approach.py --segment AX AY BX BY
```

`--ground DESCRIPTION` adds local-VLM grounding, live metric depth, and TF. It
requires the authenticated snapshot gateway and the existing process-scoped
workstation relay. Every proposed visual point is re-shown with a marker; an
unverified, ambiguous, low-confidence, depthless, or semantically mismatched
point fails closed. Thin/specular targets may retain another independently
audited point that does have depth, but all-depthless targets still fail. A
stored place anchor is navigation metadata only: it cannot set
`terminal_verified` or `object_reached`. Terminal success requires current
VLM identity grounding plus corresponding depth/transform geometry proving a
robot-center distance of at most 0.50 m. The output truthfully reports whether
any motion was dispatched, including zero-motion terminal checks.

## Robot moves unexpectedly

- Call `/estop` or run `python cli/hwr.py stop` immediately. If the gateway is
  unavailable, SSH in and call the ROS `/estop` service directly.
- Power off if ROS is unreachable. Do not reset estop until the command source is
  identified and `/wheel_speeds` is zero.
- Invalid/non-finite/over-limit wheel commands latch estop automatically.
- `cmd_vel_applied`, `wheel_speeds`, and model odometry are hard-zero while the
  driver estop state is latched. Navigation goals are rejected while latched and
  abort if the latch trips during execution or recovery. If a latched test ever
  reports nonzero applied/wheel values or goal success, stop the run and treat
  it as an instrumentation/odometry safety failure.

After contact with a person or object, keep estop latched, preserve the gateway
audit, mode-session log, and camera frame, and do not resume from the same run.
A collision is a failed acceptance result even if localization or the planner
reports a different terminal reason.

## Serial port busy or silent

1. Stop the HWR launch and inspect `sudo fuser -v /dev/ttyACM0`.
2. Run `deploy/handback.sh`; it restores the factory service and verifies health.
3. If still wedged, power-cycle with motors clear, then verify the factory stack
   before another takeover. Do not kill unknown port owners while moving.

## Power policy

- Power telemetry may be published and recorded, but it never controls takeover,
  drive, navigation, exploration, arm commands, or estop state.
- The human operator is solely responsible for deciding when to continue, stop,
  or charge the robot.
- The autonomous tool manifests expose no power or battery skill. Goal polling,
  localization reports, and VLM assessments use only `/navigation/context`.

## Localization degraded

- Motion stops when `localization_health.healthy` becomes false.
- Localization alone cannot complete a goal. Translation and yaw completion
  must also be confirmed by fresh fused scan/IMU odometry outside the
  controller's terminal tolerance; an unconfirmed pose aborts at zero.
- Automatic coarse global reseeding is disabled during normal tracking because
  it can jump between similar room hypotheses. Call `/global_relocalize`
  deliberately, leave the robot stationary while particles settle,
  and verify the scan overlay before sending another goal.
- If recovery fails, return to mapping mode only after stopping localization so
  there is exactly one `map->odom` publisher.

## Navigation reports no path or remains blocked

- A no-path or live-command collision now aborts at zero velocity; it does not
  trigger physical recovery motion. Inspect the inflated costmap and clear
  transient obstacles by waiting at least one second. Never command reverse
  manually.
- Re-localize if the scan/map overlay is wrong. Reduce inflation only after
  measuring chassis clearance; do not bypass the immediate collision gate.
- Recovery defaults to rotate-scan only. Forward-arc recovery exists behind the
  `allow_forward_recovery` parameter and remains disabled until a dedicated
  attended acceptance test proves it in measured open space.

## Restore factory boot

Run `~/hwr/deploy/boot.sh disable`. It stops/disables `hwr.service`, enables the
factory service, and starts it. Confirm `/scan` and the web UI before leaving the robot.
