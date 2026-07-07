# Phase 1 bench test — tasks 1.5 + 1.6 (2026-07-07)

Robot on stand, wheels off ground, user observing. Two takeover windows
(battery 11338→11344 mV, then 11041→11023 mV; session ended at wrap-up threshold).

## 1.5 Motors (`tools/live_phase1_bench.py`)

Per motor 1→4 (beep-count announced): +0.5 / +1.0 / +2.0 rps steps 1.5 s each,
then stop; +3.0 rps probe on motor 1; all-four +1.0 rps together. All commands
accepted; JGB37 motor type set explicitly at start.

**User-verified (visual):**
- All four wheels spun in sequence with three visible speed steps, fast probe and
  all-four run as scripted.
- **Motor 1 = front-left.** Groups 1&2 = left side, 3&4 = right side.
- With positive raw commands: **left wheels forward, right wheels backward** —
  confirms the m3/m4 inversion belongs in the kinematics layer, exactly as the
  vendor mixing formula implied.

**Confirmed wheel map:** m1=front-left, m2=rear-left, m3=front-right, m4=rear-right
(matches m1=vx−vy−wz·k / m2=vx+vy−wz·k / m3=vx+vy+wz·k / m4=vx−vy+wz·k).

## 1.6 Bus servos

Pre-move state dump (all six answered):

```
servo 1: pos=502 vin=11400mV temp=43C
servo 2: pos=723 vin=11400mV temp=41C
servo 3: pos=41  vin=11400mV temp=52C
servo 4: pos=150 vin=11400mV temp=33C
servo 5: pos=501 vin=11400mV temp=33C
servo 10: pos=498 vin=11404mV temp=35C
```

Forward pose (1,500)(2,701)(3,120)(4,300)(5,500) over 1.5 s — read-back:

```
run 1: deltas +2 / 0 / -12 / -4 / +1   worst 12 pulses (2.9 deg)  PASS
run 2 (demo with pan-left + camera-up intermediate): deltas +2 / 0 / -13 / -1 / 0
       worst 13 pulses (3.1 deg)  PASS
```

Servo 3 (elbow) consistently ~12 pulses shy — gravity load on the elbow joint;
consistent across runs, not drift. Note for Phase 9 IK (feed-forward or accept 3°).

## Link health under motor load

Bench run: 4535 frames, **2 CRC errors, 2 resyncs** (0 in the idle 30 s run earlier).
Decoder recovery worked as designed; rate 0.04% under motor electrical noise. Watch
in Phase 2 driving; if it grows, consider TX retry for critical commands.

## Battery

11338 → 11023 mV across both windows (motors + servos + factory restarts).
Session ended at the 11000 wrap-up threshold per discipline.
