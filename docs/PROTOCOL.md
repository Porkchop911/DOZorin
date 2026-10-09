# RRC protocol — implementation notes (living doc)

Our clean-room implementation lives in `hwr_core/hwr_core/rrc/`. The full extracted
spec is `plan/90-clean-room-reference.md` §RRC; this file tracks what has been
**empirically confirmed on the wire** plus implementation decisions. Golden-trace
fixtures (vendor SDK TX bytes, `hwr_core/tests/fixtures/golden_traces.json`) pin our
codecs to byte-identical frames.

## Confirmed on hardware (2026-07-07, task 1.3/1.4)

| Fact | Value |
|---|---|
| Battery report (SYS sub 0x04) | u16 LE **millivolts**, cadence ≈ **1 Hz** (0.93 Hz measured) |
| Battery agreement with factory topic | +48 mV (within ±50 mV acceptance; unloaded upward drift after factory stop) |
| IMU report rate | ≈ **113 Hz** (112.7 Hz avg over 30 s, worst 1-s bucket 112) |
| IMU units | accel in g (az at rest ≈ 1.07 g — scale/bias uncalibrated); gyro wire values are deg/s and yaw requires a 2.0 hardware scale after SI conversion |
| Link quality @1 Mbaud | 4088 frames / 30 s: 0 CRC errors, 0 resyncs |
| Buzzer `<HHHH>`, LED `<BHHH>`, OLED `[line,len,utf8]` | accepted by firmware, no faults |
| Port sharing | our Transport + vendor `button_scan.py` coexist on ttyACM0 (same as factory driver does) |
| Wheel map (user-verified on stand) | **m1=front-left, m2=rear-left, m3=front-right, m4=rear-right**; positive rps = left side forward, right side backward → invert m3/m4 in kinematics |
| Motor speed steps 0.5/1/2 rps + 3 rps probe | all track visibly on stand; JGB37 type set explicitly at startup |
| Bus servo forward pose | reached in 1.5 s, worst read-back error 13 pulses (3.1°); elbow (id 3) sags ~12 pulses under gravity, repeatable |
| Link under motor load | 2 CRC errors / 4535 frames (0 when idle) — decoder resync recovers; monitor in Phase 2 |

## Implementation decisions

- **Decoder** resyncs on any violation (function > 10, CRC mismatch) and counts
  `frames_ok / crc_errors / resyncs` — exposed for link-health monitoring.
- **Transport**: single TX lock, one RX daemon thread, per-function latest-value
  mailboxes (`latest()` / `wait_report(predicate)`) + `subscribe()` for streams.
- **Bus-servo reads** are request/response on a shared report channel → serialized
  under a Board-level lock, replies matched by sub-command byte, `success != 0`
  (servo offline) returns None.
- Motor wire ids are 0-based (API takes 1-based); **no sign handling in Board** —
  the m3/m4 inversion belongs to the kinematics layer (Phase 2).
- RTS/DTR must be low before opening the port or the STM32 resets.
- **Yaw-rate calibration (2026-07-28):** a rotation-only scan pair correlated at
  80.75° while raw gyro integration reported 40.31° (ratio 2.003). The HWR
  driver applies configurable `gyro_z_scale: 2.0` before publishing `/imu`, so
  base odometry, filtered IMU, localization, and panorama aggregation share the
  calibrated rate.

## Hard-learned firmware behavior

- **BUZZER with `off_ms=0` latches the tone ON continuously** — no later frame we
  tried silenced it before the process moved on; required a power-off (2026-07-07).
  `Board.beep()` now clamps `off_ms >= 10`; `Board.buzzer_off()` exists for panic
  paths. Policy: scripts stay silent unless sound is explicitly wanted.

## Open gaps

- `set_rgb`: referenced by vendor node, undefined in SDK. Probe deferred (cosmetic);
  do it in an attended session — unattended firmware poking avoided by policy.
- Motor-type persistence across power cycles: unknown → task 1.5 sets JGB37
  explicitly at startup regardless (cheap, idempotent).
- PWM-servo functions (function 4) implemented as codecs only where needed later;
  the ROSOrin Pro arm uses bus servos (function 5).
- Encoder/odometry reads: **do not exist in this protocol** (ADR-004).
