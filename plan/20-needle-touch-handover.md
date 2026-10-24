# 20 — Needle-touch proof of concept: handover (updated 2026-10-24 20:05, runs 17-18; next run will be 19)

Goal (user): the arm holds a needle and touches drawn dots (3 black first, then 5 red) on a sheet,
using two cameras (UGREEN on the PC + the arm camera on the robot). **The first contact with the paper
must be the dot**: no calibration touches, no touch-and-correct.

**Result: 0 drawn dots touched, 18 runs.** Run 18 is the first run under this rule that reached the paper
under its own arm-camera aim, and the user saw it: "it touched the paper slightly wrong an moved the
meedle" (m00238). It stopped because it hit the `FLOOR_Z` limit, not because it found the dot.
Before the user's rule, runs 1-9 touched blank paper while calibrating.

## What exists

| File | What |
|---|---|
| `tools/servo_shell.py` | Runs on the robot (`~/hwr/tools/`) over **one** SSH session: `{"set":{id:pulse},"ms":n}`, `{"read":true}`, `{"snap":true}` (arm camera JPEG from `camera/color/image_raw`). Publishes on `servo_cmd` beside the live stack, so no gateway token and no driver stop. Waits for the driver subscription before it reports ready (publishing earlier dropped moves). |
| `tools/needle_contact_selftest.py` | Offline check of the contact detector against `evidence/phase20/run18` frames plus a synthetic frame series. `python tools/needle_contact_selftest.py` -> 12 checks, all pass. No arm, no cameras, no SSH required.
| `tools/needle_touch.py` | PC side: `servo()` (persistent session), `Arm` (fixed-pitch IK, readback check plus one resend, pan preload against backlash), `Camera` (UGREEN, DSHOW, exposure locked, **drain thread**: buffered frames came back up to a move late), UGREEN tip/dot detection, arm-camera needle/dot detection, `run()`. |

Run: `python tools/needle_touch.py black <outdir>`. The arm must hold the needle (gripper 680, readback ~661).

## Arm camera: it works after a mode restart (was the blocker at 18:54)

At 18:54 nothing reached the arm camera: `"snaps": -1`, no JPEG from `{"snap":true}`, the supervisor kept
logging `stale camera: age 48665 ms > 3000 ms, latching estop`, and the node logged
`stream_mask_:0, rgb_stream_count_:0`. That is the ROS camera node, not `tools/` code — `v4l2-ctl` or
`ffmpeg` reporting `/dev/video0: Device or resource busy` on the robot is normal while `hwr.service` runs.
User chose "Restart stack without park" (m00121). Result:

| Step | Result |
|---|---|
| `cd ~/hwr && ./deploy/mode.sh restart navigation /home/ubuntu/hwr/maps/room_20261015.yaml` | group 3380 stopped, group 27988 started |
| `ros2 topic echo --once /estop_state` after the start | `data: true` — the driver **latches estop at every start by design** ("Driver latches estop at every start, by design (boot into stopped state)", `docs/RUNBOOK.md:74`); clear it with `ros2 service call /estop std_srvs/srv/SetBool "{data: false}"` -> `estop reset` |
| `/joint_states` immediately after the restart | {1:500, 2:310, 3:241, 4:270, 5:499, 10:662} — the arm **kept its hover pose and the needle through the power cycle**; no park needed, no droop observed |
| `ros2 topic hz --window 30 /camera/color/image_raw` | **average rate 10.286 Hz**; no `stale camera` in the new `~/hwr/state/mode_session.log`; node logged `rgb_stream_count_:1` then `recv first frame time is 1792861433405` |
| `tools/needle_touch.py` `snap()` | fresh 640x400 JPEG |

## Runs 17-18 (evidence on disk)

`evidence/phase20/run17/` and `evidence/phase20/run18/` hold `overview_arm.jpg`, `reference.jpg`,
`probe*.jpg`, `aim*.jpg`, `shot_*_touch.jpg`, `<color>_<n>_touch_arm.jpg` and `touches.csv`. Run 18's
console log: `D:/robot/.pi/tasks/01a16ebe-ad23-74b1-86b3-252d70b5ff4e-67116/bd06d6888.output`.

The instrumented probe stage (old ranked step 1) is in `tools/needle_touch.py`: probe snapshots
`probe{dot}_{axis}{plus,minus}.jpg`, per-probe candidate lists with distances from the reference dot, a
hover candidate dump, and separate messages for "no Jacobian" vs "degenerate Jacobian". Run 17 aborted on a
bug **in that logging** (`TypeError: unsupported operand type(s) for -: 'list' and 'list'` at
`tools/needle_touch.py:308` — the `cands()` log helper returned `[pixel, dist]` lists, so the difference for
`columns.append` got lists); fixed by sorting `(ndarray, dist)` pairs. Run 17 still produced usable data:

```
hover [0.25 0.  ] dots in 130 px of tip (340.0, 226.0): [[[313.8, 265.6], 47.5], [[421.4, 189.7], 89.1], [[247.9, 211.9], 93.2]]
  probe 0- xy [0.242 0.   ] dots 50 near ref [313.8 265.6]: [[[314.6, 246.5], 19.1]]
  probe 0+ xy [0.258 0.   ] dots 49 near ref [313.8 265.6]: [[[314.0, 282.4], 16.9], [[246.8, 227.5], 77.1]]
```

**The probes are no longer the weak link.** The dot is tracked through the approach-from-`xy - 2*PROBE`
sequence, and the 130 px hover window cleanly separates the target dots (34-97 px) from the sheet's printed
ruling (a band of ~55 blobs at v = 38-70, all farther out).

## Hard-won facts

| Fact | Evidence |
|---|---|
| E-stop was latched at session start (servos limp, readback 965/-64). Reset with `ros2 service call /estop std_srvs/srv/SetBool "{data: false}"`. **Explained**: the driver boots into the stopped state at every start, by design. | `docs/RUNBOOK.md:74`, `docs/RUNBOOK.md:170-172` |
| Folding to `forward` with a 15 cm needle hit the UGREEN. The reference frame is now taken with pan 250 (arm aside) at hover height. | user report |
| `ik_any_pitch` swaps posture between heights, so a "lower" pose was not lower. Use **one fixed pitch per spot**. | runs 3-4 |
| Pan backlash/deadband ~±7 mm at the tip; 3 mm one-sided Jacobian probes disagreed 10x. **±8 mm central differences** gave consistent arm-camera Jacobians (~2.0-2.7 px/mm). | runs 8-12 |
| Shoulder stops ~21 pulses short with the needle; the move check allows 35. | run 7 |
| Needle 6 cm past the fingertips is ~0.065 m past the model tool point (`NEEDLE_MODEL = 0.045` keeps the paper near model z +0.02, above the `ik` z>=0 guard). | run 12: model 0.085 ended ~2 cm above the paper |
| UGREEN: the tip end is dark, its shadow is as dark and ends *below* the tip; the tip is told apart by shape (1x13 vertical opening, tallest blob). Shadow-gap height extrapolation was ±5-7 mm off: not usable. | runs 2-10 |
| Arm camera: the tip sits at a **fixed pixel (341, 226)**; dots need black-hat detection (dot gray 35-97 vs paper ~190); the spiral binding/holes need a radius filter (<130 px from the tip). | `armcam_now_annotated.jpg` |
| When the tip nears a dot, the dot hides behind the tip in the arm image; re-acquiring then locked onto the needle's own shadow and steered the arm 6 cm off (run 14). Never re-acquire after hiding. Run 18 hit the same class of failure one stage earlier, in the alignment loop (`arm_dot(..., radius=60)` switched to a different blob mid-loop). | run 14 log, run 18 log |
| The arm camera's image axes are **crossed** with the model frame: an x move appears almost entirely as image v. Run 18: `px/mm [[-0.1, 2.5], [1.8, -0.1]]`, det -4.49 (px/mm)^2. Invertible, so `to_xy` is fine, but expect a small x-response in u. | run 18 |
| The UGREEN stop-motion contact rule has still never seen a contact. During run 18's real approach the tip kept moving 2.7 px/mm (u 1017 -> 1035, v 459 -> 591) with steps of 1-12 px, sign flips (-1, -10, -2, -3) and one 46 px jump (1030 -> 981 at 22.5 mm), so "2 quiet steps" cannot fire. | run 18 |
| Under load the arm camera's needle read changes while pose is unchanged: the frame at the bottom of run 18's descent reads tip **(342.0, 187.0)**, mask area 23.6k px, span_v 214 - versus 340-341, 225-226 / 13.5-20.5k / span_v 175-177 in all 23 hover, probe and aim frames at
other arm poses (14 aim + 8 probe + the hover overview, counted by `tools/needle_contact_selftest.py`). Caveat: that frame also has the paper at the lens, so mask merge is possible. | `evidence/phase20/run18/black_1_touch_arm.jpg` |

| The arm does **not** end a run where it started. `choose_pitch(x, y)` (`tools/needle_touch.py:243-249`) scans pitch from -85 to -30 degrees in 5-degree steps and returns the first one whose IK is reachable over `FLOOR_Z..START_Z + 0.01` with a +-2 cm margin; `run()` calls it once (`:258`) and the closing move calls it again (`:400`). Measured `/joint_states`: before run 17 (after the restart, no moves) {1:500, 2:310, 3:241, 4:270, 5:499, 10:662}; after run 17 crashed {1:500, 2:323, 3:215, 4:261, 5:500, 10:658} (expected - the crash skipped the recovery moves, the arm stayed at the last probe spot [0.258, 0.0]); after run 18 {1:498, 2:237, 3:270, 4:288, 5:500, 10:662} - a different elbow posture from the start, and **I have not verified where the tip actually is**. Read `/joint_states` and `arm_needle` before the next run. | runs 17-18 |
| Unit note on the pose numbers above: the pre-run-17 sets were converted from `/joint_states position` **radians**, and the rad-per-bus-unit factor is joint-dependent (measured ratios ~0.00409-0.00421 for joints 1-5, ~0.00406 for the gripper), while the 18:56 set {1:498, 2:311, 3:239, 4:270, 5:501, 10:662} came straight from `servo_shell.py`'s `{"read":true}` bus values. My conversions agree with the bus values to 1-2 units except joint3 (6 units), so **treat converted numbers as +-6 units and compare position radians directly**. `HWR_ARM_PULSE_RAD = 0.2062645` / "100 pulses = 0.3607527 rad" (`plan/04`, `plan/10`) does not match either observation. For any safety decision use the `{"read":true}` values. | runs 17-18 vs 18:56 read |

## Where it stopped

Design as run 16/17/18 (unchanged in shape): hover at model z 0.06 -> arm-camera Jacobian from ±8 mm
central-difference probes (`PROBE = 0.008`, `tools/needle_touch.py:229`) -> align the dot to
`AIM_BEYOND = 5` px past the tip in the arm image (~1.5 mm short of the dot centre, :232) with damped
corrections (`GAIN = 0.6`, :230) -> descend straight along the camera->tip ray 1.5 mm per step -> stop when
the UGREEN tip stops moving (2 quiet steps), floor model z 0.010 (`START_Z, FLOOR_Z, STEP = 0.06, 0.010,
0.0015`, :228), `NEEDLE_MODEL = 0.045` (:227).

**Run 17**: aborted at the probe stage by the logging bug above. Selection and tracking were healthy.

**Run 18** (`python tools/needle_touch.py black evidence/phase20/run18`, exit 0, ~2 min):
- dot 0, target `[315.2 247.8]`, 34.4 px from the tip: probes tracked it (4.3 / 24.5 / 27.3 / 14.1 px) and
  gave `px/mm [[-0.1, 2.5], [1.8, -0.1]]`. Alignment oscillated - offset `[29.3,-39.4]` -> `[11.4,5.1]` ->
  `[2.9,6.4]` -> `[3.8,-20.1]` -> `[1.0,4.2]` -> `[-0.7,4.1]` -> lost (
  `hover: lost the dot; dots near [338.3 217.3]: [[[274.6, 171.8], 78.2], ...]`), because the tracked blob
  jumped to a neighbour (`dot [326.2 216.3]`, `dot [333.7 241.5]`). Outcome: `dot 0: could not line up at
  hover; skipped (no touch)` - and it was appended to `touched`, so it is never retried.
- dot 1, target `[275.3 171.7]`: alignment **converged** - offsets `[58.6,44.5]` -> `[31.6,8.7]` ->
  `[17.2,-1.0]` -> `[8.0,-8.6]` -> `[3.3,-0.1]` -> `[3.6,-0.8]` -> `[0.4,0.2]` px at
  `xy [0.2533, 0.0373]`, then `ray [-0.01, -0.002, -1.0]`. The descent ran the full 49.5 mm
  (`tip [0.2528, 0.0372, 0.0105]`) and ended `dot 1 reached floor without seeing contact at model z 0.0105,
  ugreen tip (1035.0, 591.0)`. `touches.csv`: `1792861918,black,1,0,0.0105,0.2527,0.0372,1035.0,591.0`
  (contact flag 0).
- Physical outcome (user, m00238): it touched slightly off the aim and **moved the needle** - consistent with
  driving ~30-40 mm past the paper because nothing detected the contact.

So: reaching the paper via the arm camera works. Two things block a clean first dot touch - blob identity in
the alignment loop, and contact detection, whose absence turns every descent into a push.
Contact detection has since been replaced with an arm-camera rule (see "Done after the 19:25 handover" below);
it has passed its offline test but has never run on the real arm. Blob identity is still unfixed.

## Done after the 19:25 handover: arm-camera contact detection (old next step 1)

`tools/needle_touch.py` gained `arm_read()` (tip pixel + mask area + mask `span_v` from `arm_needle`) and
class `ArmContact`, and the descent now takes one arm frame per step and stops on it. The UGREEN
stop-motion is still measured and still printed, but it is **no longer a stop condition** (run 18 proved it
cannot fire).

`ArmContact` rule, exactly as tested:
- Baseline = median tip of the **first 3 frames of this descent**, not the hover frame. Reason: the hover
  view and the descent view differ by a fixed amount, and the paper creeping toward the lens during the
  descent must not fire the rule by itself.
- Fire on `dev` >= `CONTACT_DEV_PX = 6.0` px off that baseline held for `CONTACT_CONFIRM = 2` consecutive
  steps (a sustained bend); or on one step-to-step tip jump >= `FRAME_JUMP_PX = 15.0` px (the paper is at
  the lens, or the needle slipped); or when the needle blob is missing for 2 consecutive steps.
- Physical margin: the jump rule stops the descent on the frame where the change appears, i.e. **<= 1.5 mm
  (one `STEP`) after** the change was seen; the held rule needs `CONTACT_CONFIRM` steps, so **<= 3 mm after**
  it was first seen. In both cases the arm is still moving at 1.5 mm/step while the frame is being read, so
  the true overshoot past first contact is at least one step. The 6 px threshold is 2x the 3 px tip spread
  measured over 23 run-18 pose frames, so it is not reachable by pose noise alone.

Evidence per descent, new files in the run directory:
- `descent.csv`, one row per step: `dot,step,travel_mm,z_m,arm_tip_u,arm_tip_v,arm_dev_px,arm_area,span_v,ugreen_u,ugreen_v,ugreen_moved`
  — the arm read beside the UGREEN read, so a chosen trigger can be checked against the frames afterwards.
- `descend_{dot}_{step}.jpg` arm frames every `DESCEND_SHOT_EVERY = 4` steps, plus **every firing frame** and
  the final frame (`..._end.jpg`).
- `touches.csv` now has a header and a `stop_reason` column: `ts,color,dot,contact,z_m,x_m,y_m,
  ugreen_tip_u,ugreen_tip_v,stop_reason`, e.g. `stop_reason` = `tip jumped 39.1 px in one step` or `floor`.
- Fixed naming per old next-step 6: the closing UGREEN frame is `shot_..._touch.jpg` only when contact was
  detected, otherwise `shot_..._no_contact.jpg`.

Self-test results (`tools/needle_contact_selftest.py`, offline, on the run-18 frames):
```
ok   run18 pose frames read a needle: 23 frames, tips [[341.0, 225.0], [340.0, 226.0], [340.0, 226.0]]...
ok   tip is pose-invariant (<= 6.0 px spread): mean tip [340.4 225.3] spread [3.0, 2.0]
ok   no fire across pose changes: 23 frames, areas 13510-20471
ok   run18 load frame fires the rule: tip jumped 39.1 px in one step | tip [342. 187.] area 23601
     span_v 214 vs baseline [340. 226.] dev 39.05
ok   noise under 6.0 px never fires: 1px -> False; 2px -> False; 3px -> False; 4px -> False
ok   a held bend fires only after CONTACT_CONFIRM steps: step1 (False, 6.0) step2 (True, 6.0)
ok   a single big jump fires at once: tip jumped 16.0 px in one step
ok   a needle the camera lost stops the descent: after 2 missing frames: needle blob gone 2 steps
ok   missing frames before the baseline is built do not fire: [False, False, False]
ok   AIM_BEYOND still 5 px
ok   arm_needle still finds the documented fixed tip point: tip (341.0, 225.0)
--- results: ALL PASS
```

Uncertainty carried into the live test: the self-test proves the detector separates *the run-18 load frame*
from *pose frames*, but run 18 only has one load frame, and it also has the sheet at the lens. Only a live
run can show whether the trigger fires **on the dot** and not on the paper edge, and where in the descent it
fires. Watch `arm_dev_px` in `descent.csv`: a gradual climb means parallax/paper-creep (rule may fire too
late or for the wrong reason); a step change means a real load event.

**Not done, user chose detector first (2026-10-24):** the descent is still unbounded — from model z 0.06 it
can still drive 49.5 mm down to `FLOOR_Z` 0.010 if the trigger never fires, which is what moved the needle
in run 18. `FLOOR` is still nominal model z (`START_Z, FLOOR_Z, STEP = 0.06, 0.010, 0.0015`), unchanged.

## Next steps (ranked, after the contact detector)

1. **Run 19 with the arm-camera contact rule and read `descent.csv` before anything else.** What each outcome
   means, decided in advance: a row where `arm_dev_px` stays < 6 for the whole descent and `stop_reason =
   floor` -> the arm-camera read is not load-sensitive after all, report it and stop treating this detector as
   the answer; `stop_reason = tip jumped N px in one step` or `tip held ... 2 steps` at a model z the user
   confirms as a real dot touch -> the POC works, then aim for the 5 red dots; a fire at a z clearly above the
   paper (the user says the needle was still in the air) -> false trigger from the paper edge at the lens,
   which is then a threshold problem, not a concept problem. Write the prediction down before the run, not
   after it.
2. **Fix dot identity in the alignment loop**: predict where the dot must appear from the measured Jacobian
   and reject a candidate that disagrees with the prediction; or mask the printed ruling instead of relying on
   `radius=60`/`radius=90`. Do not add a skipped dot to `touched`.
3. **Get the geometry from the user rather than from guesses**: measured paper-top height above the arm base
   plane and the needle's protrusion past the fingertips. That sets `START_Z`/`FLOOR_Z`/`STEP`/
   `NEEDLE_MODEL` and shortens the unloaded part of the descent. Ask the user to re-check the needle first
   (bent vs slipped in the gripper vs re-clamp) and to say whether run 18 left a mark and where it is.
4. Before any new run, `python tools/needle_contact_selftest.py` (no robot needed, ~seconds) and confirm
   `arm_needle` still reads the documented fixed point at hover; note plan/20's fixed point (341, 226) vs
   observed (340-341, 225-226) via the self-test, i.e. ±1 px = sub-mm.
5. Replace the 130 px radius filter with a binding/hole mask (`shortcut:` comment in `run()`). Lower priority
   now: the radius separates dots from ruling correctly in runs 17-18.
6. Naming/record cleanliness: **done for the run files** (per-descent `descent.csv`, `stop_reason` column,
   `_touch` only on a detected contact). What remains is the count claim: `n` is 0-based in the file names
   while the console and the plan talk 1-based. Keep `touches.csv` as the record of truth.

## Run 19 preparation (prepared, not run)

User decision at the 20:2x update: commit and push the detector, **do not run run 19 yet**. Preparing it only.

- Command, as the code actually accepts it (`tools/needle_touch.py:508` = `run(sys.argv[1], sys.argv[2])`, no
  flags): `python tools/needle_touch.py black evidence/phase20/run19`.
- Descent bound in the code today: `START_Z, FLOOR_Z, STEP = 0.06, 0.010, 0.0015` (`tools/needle_touch.py:289`)
  and the stop test `if point[2] < FLOOR_Z` (`:451`). With no contact trigger that is 33 steps / 49.5 mm of
  blind travel. To make the first live test bounded, **raise `FLOOR_Z`** (e.g. 0.030 -> at most 20 mm of
  descent); there is no per-run flag for it, so this is an edit at `:289`. Waiting for the measured paper-top
  height before choosing the number (next step 3).
- Before starting: robot reachable over `ssh ubuntu@192.168.0.223`; `ros2 topic echo --once /estop_state` =
  false (`docs/RUNBOOK.md:74`, driver latches estop at every start); `servo_shell.py` `{"read":true}` pose
  recorded and compared with the post-run-18 pose {1:498, 2:237, 3:270, 4:288, 5:500, 10:662};
  arm camera publishing (`ros2 topic hz --window 30 /camera/color/image_raw`, expected ~10.3 Hz); `python
  tools/needle_contact_selftest.py` = ALL PASS; needle straight and protrusion measured.
- During the run I read `descent.csv` live: `arm_dev_px` climbing gradually = parallax/paper creep (the rule
  may fire late or for the wrong reason); a step change = a real load event. `stop_reason` is written into
  `touches.csv` for whichever path ended the descent.

State at the 19:25 update: robot in navigation mode (process group 27998? see below) with the run-15 map;
`/estop_state` false; the arm holds the needle (gripper readback 662) but sits in a posture that differs
from the pre-run hover — {1:498, 2:237, 3:270, 4:288, 5:500, 10:662} instead of {1:500, 2:310, 3:241,
4:270, 5:499, 10:662}; arm-camera topic publishing (measured 10.286 Hz at 19:03 after the restart; a short
10 s window just now reported frame gaps 0.051-0.319 s); PC UGREEN working; **no red dots visible in the
current frame**. I have not re-read the arm camera or the needle geometry since run 18 — do that before the
next run. `plan/20-needle-touch-handover.md`, `tools/needle_touch.py` and `tools/servo_shell.py` are
untracked in git (`git status` = `??`) — worth committing with the run17/run18 evidence.

State at the 20:05 update: **the Phase 20 work is committed locally** — `b8d2e7d
feat(needle-touch): aim from the arm camera so the first paper contact can be the dot` on branch `rework`,
35 files: `plan/20-needle-touch-handover.md`, `tools/needle_touch.py`, `tools/servo_shell.py` and
`evidence/phase20/run17`, `run18` (3.1 MB of frames plus both `touches.csv`). `git status -sb` shows
`## rework...hub/rework [ahead 1]` — the commit exists only on this PC, so it needs
`git push hub rework` to be safe (not done: push is a remote write, ask first).
Arm-camera contact detection added on top of it and **uncommitted**:
`tools/needle_touch.py` (`arm_read`, class `ArmContact`, per-step arm frame in the descent, `descent.csv`,
`touches.csv` header + `stop_reason`, honest `_touch` / `_no_contact` file names) and new
`tools/needle_contact_selftest.py`. Both compile clean and the self-test passes 12/12 offline.
Robot reachability at 20:05: `ssh ubuntu@192.168.0.223` **timed out on port 22** (6 s connect timeout, ~20 s
total), so estop / pose / `arm_needle` re-check and run 19 could not be done. Before run 19: robot reachable,
`/estop_state` false, `pos:` pose recorded, and the arm camera publishing. Still open and unchanged from the
19:25 update: whether the needle is still straight and how far it protrudes.

State at the 20:2x update: the detector is **committed and pushed** (see `git log -1` for the hash after this
section; the push closed the `ahead 1` gap that the 20:05 note described). Nothing was run on the robot. No
new SSH check was made in this update: the robot was last unreachable at 20:05.
