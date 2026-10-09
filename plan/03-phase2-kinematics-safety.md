# Phase 2 — Kinematics, safety layer, first driving

**Objective.** From `drive(vx, vy, wz)` to four correct wheel speeds with a safety layer that makes unattended bugs survivable, and the `hwr_core.Robot` facade that everything above builds on. Ends with the robot driving measured shapes on the floor.

**Prerequisites.** Phase 1 (motors + servos work on stand). Constants: `90-clean-room-reference.md` §Mecanum.

## Subtasks

- [x] **2.1 Mecanum module.** `hwr_core/kinematics/mecanum.py`: `wheel_speeds(vx,vy,wz) -> [rps×4]` (wheelbase 0.17706, track 0.17165, Ø 0.08, signs [+,+,−,−]) and the pseudo-inverse `body_velocity(rps×4)`. Property tests: round-trip ≈ identity; pure vx → equal wheels; pure wz → symmetric; clamp scaling preserves direction.
- [x] **2.2 Safety layer.** `hwr_core/safety/`: `Limits` (vel + accel clamps, defaults ±0.2 m/s, ±0.5 rad/s, ramp ≤0.5 m/s²), `Watchdog` (auto-zero motors if no command for 0.4 s — runs in the transport thread so it fires even if the caller hangs), and `EStop` latch (any source → zero + require explicit reset). Power telemetry is observational only. Unit-tested with a fake board.
- [x] **2.3 Robot facade.** `hwr_core/robot.py`: `Robot.connect()`, `.drive(vx,vy,wz)`, `.stop()`, `.beep()`, `.battery_mv()`, `.imu()`, `.arm.set_pose(name|{id:pos}, duration)`, `.arm.read()`, context-manager (`__exit__` → stop + torque-safe). This *is* the Python API deliverable of requirement 4 (gateway/CLI later wrap it).
- [x] **2.4 Bench validation.** On stand: mixing sanity — command +vx, verify all wheels forward-rolling sense; +vy strafe pattern; +wz rotation pattern. Fix sign map here if reality disagrees with the reference.
- [ ] **2.5 Floor calibration (attended, open area).** Scripted runs: 1 m forward, 1 m strafe left, 360° rotate (IMU-integrated), each ×3; measure with tape/floor marks. Derive `linear_correction`, `strafe_correction`, `angular_correction` factors into a calibration YAML consumed by the kinematics module.
- [ ] **2.6 Acceptance run.** Scripted 1 m square with strafing legs (fwd, strafe-left, back **as strafe/rotate — no reverse translation**, strafe-right… i.e. square driven without negative vx), ×3.

## Acceptance & evidence

- **Test:** square returns to start within 10 cm and 15° (each of 3 runs); watchdog stops the robot ≤0.5 s after the command source is killed mid-drive (demonstrated); estop and command-validation unit tests green; bench sign map documented.
- **Evidence:** `evidence/phase2/`: tape-measure photos of end positions, calibration YAML, watchdog demo log, pytest output.

## Safety / rollback

- The watchdog and clamps land **before** the first floor test (convention #4). First floor runs at 0.1 m/s.
- Square test uses only forward/strafe/rotate — the no-reverse rule applies from day one.
- Rollback: none needed — hwr_core is additive; handback restores factory behavior.

**Effort.** 1 day.

**Open questions.** Do rps commands slew internally on the STM32 (affects our accel clamp defaults)? Measure during 2.5 rotation runs (gyro shows the true spin-up profile).

## Status (2026-07-08)

2.1–2.4 done: 71 unit tests green; bench 2.4 user-verified all three mixing patterns (+vx all-forward, +vy strafe diagonal, +wz left/right split); **live watchdog demo: fired 0.32 s after command source died** (bar ≤0.5 s), 0 CRC errors. Evidence: bench transcript in git history + `evidence/phase1/pytest.txt` superseded by later runs.

**2.5/2.6 (floor calibration + 1 m square) ON HOLD — user paused movement testing** after the buzzer incident (see `docs/PROTOCOL.md` §hard-learned). Calibration script `tools/live_phase2_floor_cal.py` is ready and now fully silent (printed countdowns). First attempt aborted mid-sequence with no data; rerun from scratch when movement is allowed.
