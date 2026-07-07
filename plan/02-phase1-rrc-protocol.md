# Phase 1 — Clean-room RRC protocol (`hwr_core.rrc`, no ROS)

**Objective.** Our own, fully tested implementation of the STM32 wire protocol: framing, CRC-8, every function codec we need, a robust serial transport, and a `Board` API. Pure Python, zero ROS imports, unit-testable on the PC.

**Prerequisites.** Phase 0 (takeover/handback). Spec: `90-clean-room-reference.md` §RRC.

## Subtasks

- [x] **1.1 Frame codec + CRC-8.** `hwr_core/rrc/crc8.py` (hand-transcribed 256-entry table + `crc8(bytes)->int`), `frames.py` (`encode(func, data)->bytes`, streaming `Decoder` class implementing the 6-state RX machine, tolerant of garbage/resync). Tests: CRC vectors computed independently (pure-python bitwise CRC as oracle), encode/decode round-trip, resync after noise, truncated frames.
- [x] **1.2 Serial transport.** `transport.py`: open `/dev/rrc` @1 M, RTS/DTR low pre-open, RX daemon thread feeding `Decoder`, per-function latest-value mailboxes with timestamps, TX lock, clean `close()`. Injectable fake serial for tests.
- [x] **1.3 Read-only codecs live.** `codecs.py` + `board.py`: battery (u16 LE mV), IMU (6×f32; convert g→m/s², deg/s→rad/s), button events. **Live test (takeover window):** print battery and 5 s of IMU at the wire rate; compare battery against the value the factory topic showed pre-takeover.
- [x] **1.4 Harmless actuators** *(RGB probe deferred — see outcome)*. Buzzer (`freq,on,off,repeat`), LED blink, OLED line write. Live: one beep, one blink, write "HWR" to OLED. Probe the RGB gap (LED-payload variants at low duty); whatever is learned → append to this file + `90` gap list. Timebox 30 min; RGB is cosmetic.
- [x] **1.5 Motors — bench only.** `set_motor_type` (confirm JGB37=1 needed or already persistent), `set_motor_speed([id,rps]...)`. **Robot on stand.** Per wheel: step 0.5 → 1 → 2 rps, visually confirm direction & spin-up; find max commanded rps that still tracks (listen/observe). Record sign map (expect 3,4 inverted at kinematics level, not here). Always end with explicit 0-rps + a `stop_all` helper.
- [x] **1.6 Bus servos.** set-position (dur ms, {id,pos}), read pos/vin/temp, torque enable/disable, `stop`. Live: read all 6 servo positions; command the known forward pose `(1,500)(2,701)(3,120)(4,300)(5,500)` with 1.5 s duration, arm clear.
- [x] **1.7 Golden traces** *(done on PC — equivalent oracle, vendor tree local)*. On the robot in a venv, drive the *vendor* SDK with a pyserial spy (subclass capturing TX bytes, no hardware needed) for: battery-cal, beep, motor set, servo set. Save captured frames as fixtures; pytest asserts our codecs emit byte-identical frames. (Vendor SDK is executed as an oracle, not copied.)
- [x] **1.8 Docs.** Start `docs/PROTOCOL.md` from `90-…md` §RRC + everything empirically confirmed (RGB result, motor-type persistence, battery rate).

## Acceptance & evidence

- **Test:** pytest green on PC (codec, CRC, decoder-resync, golden traces). Live: battery within ±50 mV of factory reading; IMU ≥40 Hz sustained 30 s; each wheel tracks commanded rps on stand; arm reaches forward pose smoothly.
- **Evidence:** `evidence/phase1/`: pytest output, battery comparison note, IMU rate log, stand video or photo sequence, servo read-back dump.

## Safety / rollback

- Motors only on stand; servo test with arm clear of the chassis and desk; every live script traps SIGINT → motors 0 + servo torque hold. Handback after each session.
- If the board wedges (no reports): power-cycle robot; factory stack re-verifies the hardware is fine — that isolation is exactly why golden traces come before wide actuator use.

**Effort.** 1–2 days.

**Open questions.** Does motor-type need setting per boot? ~~Battery report cadence?~~ **≈1 Hz** (measured 1.3).

## Status: ACCEPTED (2026-07-07)

All subtasks done (RGB probe deferred — cosmetic, gap list in docs/PROTOCOL.md). **46/46 pytest green** (12 golden traces byte-identical to vendor SDK — `evidence/phase1/pytest.txt`). Live acceptance all PASSED:
- battery +48 mV vs factory (bar ±50); IMU 112.7 Hz sustained 30 s (bar 40) — `evidence/phase1/live_1_3_1_4.md`
- motors: all 4 wheels stepped 0.5/1/2 rps on stand, user-verified; **wheel map confirmed m1=FL, m2=RL, m3=FR, m4=RR; right side needs sign inversion in kinematics** — `evidence/phase1/bench_1_5_1_6.md`
- servos: forward pose reached twice, worst error 13 pulses (3.1°; elbow gravity sag ~12 pulses is repeatable)

Findings for later phases: IMU accel scale ~1.07 g at rest (calibrate before Phase 5); 2 CRC errors/4535 frames under motor load (0 idle) — decoder resync worked, watch in Phase 2; battery reports ~1 Hz; servo 3 runs warm (52 °C) holding the arm.
