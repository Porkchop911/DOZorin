# 20 — Colony picker make-or-break: can the arm put a needle tip within 1 mm? (2026-10-24)

User: pick bacterial/yeast colonies (Meta AI Developer Hackathon, build Oct 26 - Nov 4). A steel needle
held in the gripper, the UGREEN 1080p webcam overhead, the arm camera for close-ups, controllable lamps.
Colonies are 2-4 mm (yeast) and need a tip within ~1 mm. The arm misses by ~10 mm open-loop (plan/16:
shoulder 4-14 pulses short, places 8-9 mm off sideways, radial -5..-28 mm, "a readback trim barely
moved it (deadband)"). The only route is visual servoing on the overhead camera, which works only if
small pulse steps really move the tip. This test decides go / no-go before the application (Oct 25 23:59 ET).

## Facts

| Item | Fact |
|---|---|
| Servo step | 0.24 deg/pulse (`RAD_PER_PULSE`); at ~0.22 m reach 1 pulse on pan/shoulder ~ 0.9 mm at the tip |
| Known errors | elbow gravity sag ~12 pulses, repeatable (plan/02); shoulder short 4-14 pulses (plan/16) |
| Access | no raw-pulse gateway route; bench script uses `hwr_core.Robot.arm.set_pose({id: pulse})` and `arm.read()` directly, with the gateway/driver stopped |
| Camera | UGREEN 1080p, fixed focus, 90 deg FOV, UVC; auto exposure/white balance must be locked |

## Rig (~30 min)

- [ ] Robot on the floor, wheels chocked/taped; arm area clear; estop in reach (91-conventions §1, §5).
- [ ] Needle: thinnest available (2 mm), tape-wrapped where the jaws close, ~40 mm sticking out past the
      fingertips. A ~2 mm dot of bright nail polish 3-5 mm above the tip = the tracked marker.
- [ ] mm graph paper taped flat at plate height, at the reach spot (base x ~0.22 m, y 0), where the needle points straight down.
- [ ] UGREEN on a tripod, lens 10-20 cm straight above the paper; diffuse light, no direct glare.
- [ ] Lock the camera: `v4l2-ctl -d /dev/video0 -L`, then auto exposure off, white balance auto off, fixed values.

## S0 — camera check (10 min), gate for everything else

1. Take a 1920x1080 still of the paper. The 1 mm grid lines must be clearly separate at 100% zoom.
   Fail -> clip-on macro lens or phone camera (DroidCam), redo S0.
2. Scale: fit the grid -> mm/px at the paper (expect ~0.05-0.15). Record it.
3. Noise: 20 frames with nothing moving -> marker centroid scatter. Must be <= 0.1 mm (sigma),
   otherwise the steps below are not measurable (light flicker, focus, compression: average 3 frames).

## S1 — single-pulse step test (45 min), the real question

Start pose: `ArmKinematics.ik(0.22, 0.0, z_tip + 0.004, pitch=-pi/2)` -> `joints_to_pulses`, i.e. tip
~4 mm above the paper, needle vertical. Approach it from the same side every time (+20 pulses, then down to it).

For each servo 1 (pan), 2 (shoulder), 3 (elbow), 4 (wrist), one at a time:

| Block | Steps | Purpose |
|---|---|---|
| A | +1 x10 | does a single pulse move it? |
| B | -1 x10 | reverse: lost motion = backlash |
| C | +2 x5, -2 x5 | |
| D | +5 x3, -5 x3 | reference: expected mm/pulse |

Per step: command `duration_ms=300`, wait 1.0 s, read back the pulse (`arm.read()`), average 3 frames ->
marker centroid. Log one CSV row: `servo, block, commanded, readback, u, v, x_mm, y_mm, t`.
Abort and stop the servos if the tip touches the paper or the readback is > 30 pulses off.

Per servo, compute:
- **gain**: mm/pulse from block D (expected ~0.9 for pan/shoulder at 0.22 m)
- **single-step yield**: share of block A/B steps that moved >= 0.5 x gain (report as n/20)
- **deadband**: smallest step size that moves reliably (1, 2 or 5)
- **backlash**: steps lost after the A->B reversal
- **readback vs command**: does the readback follow +1 steps, or stick?

## S2 — hold and repeatability (20 min)

1. Hold at the start pose 60 s, a frame every 5 s -> drift (gravity sag, warm-up).
2. 10x: move 20 pulses away (same side), come back to the start pose -> return scatter (sigma, max).

## S3 — closed-loop trial (45 min), the demo in miniature

Five 2 mm dots drawn on the paper within ~15 mm of the start point.
Loop per dot: error (marker -> dot, mm; marker-to-tip offset measured once in S0) -> pulses with the S1 gains
(pan for sideways, shoulder for radial; elbow/wrist hold the needle vertical) -> round and command
(at least 1 pulse when |error| > 0.5 gain) -> wait, measure -> repeat, max 6 iterations.
Always end on a move from the same side (overshoot -> back off 5 pulses, approach again).

Record: iterations, final error in mm, and whether it went back and forth around the dot (oscillation).

## Decision

| Result | Verdict | Then |
|---|---|---|
| S1 pan + shoulder single-step yield >= 16/20, S3 final error <= 1.0 mm on >= 4/5 dots in <= 4 iterations | **GO** | Yeast colonies >= 2 mm, spaced >= 3 mm. Apply. |
| Single steps unreliable, but 2-pulse steps move; S3 <= 2.0 mm on >= 4/5 | **CONDITIONAL** | Only colonies >= 3-4 mm, sparse plates; always approach from the same side; try tuning the servo PID (HX `0x42/0x43`, 90-clean-room §BUS_SERVO) and rerun S1 for that servo. Apply, pitched as "large-colony picker". |
| Deadband >= 5 pulses or S3 > 2 mm | **NO-GO for the arm** | Fixed arm, plate moves instead (cheap XY stage / 3D-printer bed), or a camera-only demo that detects, ranks and guides picking by hand. Decide before applying. |

Lean extras, not needed for go/no-go: S4 electrical contact (needle on a GPIO input, a wire in the agar to
GND, lower 0.5 mm steps until it closes) and the shadow-meets-tip check with a low side lamp.

## Evidence

`evidence/phase20/`: S0 still + scale, `steps.csv`, S2/S3 CSVs, a plot of tip path per servo block, a one-line
README entry with the verdict and denominators (e.g. "pan 18/20 single steps, S3 4/5 <= 1.0 mm").

## Risks

- A long needle magnifies wrist play: if the wrist moves the tip more than pan does, hold the needle shorter.
- Warm-up drift: run S2 first if the arm has been idle; rerun S1 for pan if the drift is > 0.5 mm/min.
- The marker sits above the tip: the needle must stay vertical, or the tip offset changes with tilt.
  Check in S0 by tilting the wrist 5 pulses and watching the marker-to-tip distance.
