"""Offline check of the arm-camera contact detector added to tools/needle_touch.py (plan/20 step 1).

No arm, no cameras: it reads saved frames from evidence/phase20/run18 and feeds them to ArmContact.

Test 1 - pose is not contact. Run 18 produced 23 arm frames at different arm poses (hover, 8 probe
  poses, 14 aim poses). plan/20 records the tip at (340-341, 225-226) in all of them. The detector
  must not fire on any of them.
Test 2 - load is contact. The last frame of run 18's descent, black_1_touch_arm.jpg, reads tip
  (342.0, 187.0) with mask area 23.6k px against 13.5-20.5k. Feeding it after normal frames must fire.
Test 3 - the rule stays within noise. Small synthetic tip shifts below CONTACT_DEV_PX must not fire,
  and shifts above it must fire only after CONTACT_CONFIRM consecutive steps.
Test 4 - a needle the camera can no longer see (the sheet filling the lens) fires as a stop.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from needle_touch import (  # noqa: E402
    AIM_BEYOND,
    CONTACT_CONFIRM,
    CONTACT_DEV_PX,
    FRAME_JUMP_PX,
    ArmContact,
    arm_needle,
    arm_read,
)

EVIDENCE = HERE.parent / "evidence" / "phase20" / "run18"

failures: list[str] = []


def check(name: str, ok: bool, detail: str) -> None:
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        failures.append(name)


# ---------------------------------------------------------------- arm-frame tests (real images)
def arm_frames() -> list[np.ndarray]:
    """Every arm-camera frame run 18 saved, in stage order: overview, probes, aim."""
    names = ["overview_arm.jpg"]
    names += sorted(p.name for p in EVIDENCE.glob("probe*.jpg"))
    names += sorted(p.name for p in EVIDENCE.glob("aim*.jpg"))
    frames = []
    for name in names:
        image = cv2.imread(str(EVIDENCE / name))
        if image is None:
            raise SystemExit(f"missing {EVIDENCE / name}")
        frames.append(image)
    return frames


def test_pose_is_not_contact() -> None:
    frames = arm_frames()
    watch = ArmContact()
    tips, areas = [], []
    fired = None
    for index, image in enumerate(frames):
        read = arm_read(image)
        if read is None:
            check("pose frames readable", False, f"frame {index} has no needle blob")
            return
        tips.append(read["tip"].round(1).tolist())
        areas.append(read["area"])
        hit, why, _ = watch.update(read)
        if hit:
            fired = f"frame {index}: {why}"
            break
    tip_array = np.array(tips)
    spread = (tip_array.max(axis=0) - tip_array.min(axis=0)).round(1).tolist()
    check("run18 pose frames read a needle", len(tips) >= 20, f"{len(tips)} frames, tips {tips[:3]}...")
    check(f"tip is pose-invariant (<= {CONTACT_DEV_PX} px spread)",
          max(spread) < CONTACT_DEV_PX, f"mean tip {tip_array.mean(axis=0).round(1)} spread {spread}")
    check("no fire across pose changes", fired is None, fired or f"{len(tips)} frames, areas {min(areas)}-{max(areas)}")


def test_load_is_contact() -> None:
    frames = arm_frames()
    load = cv2.imread(str(EVIDENCE / "black_1_touch_arm.jpg"))
    if load is None:
        check("load frame present", False, "evidence/phase20/run18/black_1_touch_arm.jpg missing")
        return
    watch = ArmContact()
    base_tip = None
    for image in frames:
        read = arm_read(image)
        hit, why, dev = watch.update(read)
        if hit:
            check("baseline stable before the load frame", False, why)
            return
        base_tip = read["tip"]
    read = arm_read(load)
    hit, why, dev = watch.update(read)
    details = (
        "no read" if read is None
        else f"tip {read['tip'].round(1)} area {read['area']} span_v {read['span_v']} "
             f"vs baseline {base_tip.round(1)} dev {dev}"
    )
    check("run18 load frame fires the rule", hit and read is not None, f"{why} | {details}")
    if read is not None:
        check("load frame is a real change (>= 1 step jump threshold)",
              dev is not None and dev >= FRAME_JUMP_PX / 2, f"dev {dev}")


# ---------------------------------------------------------------- synthetic threshold tests
def synthetic(tip) -> dict:
    return {"tip": np.array(tip, dtype=float), "area": 15000, "span_v": 176}


def test_thresholds() -> None:
    base = [341.0, 226.0]
    watch = ArmContact()
    for _ in range(3):
        watch.update(synthetic(base))
    hits = []
    for offset in (1, 2, 3, 4):  # all below CONTACT_DEV_PX
        hit, why, dev = watch.update(synthetic([base[0] + offset, base[1]]))
        hits.append((offset, hit, dev))
    check(f"noise under {CONTACT_DEV_PX} px never fires", not any(h for _, h, _ in hits),
          "; ".join(f"{o}px -> {h} ({d})" for o, h, d in hits))

    watch = ArmContact()
    for _ in range(3):
        watch.update(synthetic(base))
    first = watch.update(synthetic([base[0], base[1] - CONTACT_DEV_PX]))
    second = watch.update(synthetic([base[0], base[1] - CONTACT_DEV_PX]))
    check("a held bend fires only after CONTACT_CONFIRM steps",
          (not first[0]) and second[0], f"step1 {first} step2 {second}")

    watch = ArmContact()
    for _ in range(3):
        watch.update(synthetic(base))
    hit, why, _ = watch.update(synthetic([base[0], base[1] - FRAME_JUMP_PX - 1]))
    check("a single big jump fires at once", hit, why)

    watch = ArmContact()
    for _ in range(3):
        watch.update(synthetic(base))
    misses = [watch.update(None) for _ in range(CONTACT_CONFIRM)]
    last_hit, last_why, _ = misses[-1]
    check("a needle the camera lost stops the descent", last_hit,
          f"after {CONTACT_CONFIRM} missing frames: {last_why}")

    watch = ArmContact()
    early = [watch.update(None) for _ in range(3)]
    check("missing frames before the baseline is built do not fire", not any(h for h, _, _ in early),
          f"{[h for h, _, _ in early]}")


def test_aim_geometry_unchanged() -> None:
    """The detector must not have disturbed the aim stage it sits after."""
    check("AIM_BEYOND still 5 px", AIM_BEYOND == 5, f"AIM_BEYOND = {AIM_BEYOND}")
    ok = True
    image = cv2.imread(str(EVIDENCE / "overview_arm.jpg"))
    found = arm_needle(image)
    if found is None:
        check("overview frame still yields a needle mask", False, "arm_needle returned None")
        return
    tip, mask = found
    ok = abs(tip[0] - 341) <= 2 and abs(tip[1] - 226) <= 2
    check("arm_needle still finds the documented fixed tip point", ok, f"tip {tip}")


for test in (test_pose_is_not_contact, test_load_is_contact, test_thresholds, test_aim_geometry_unchanged):
    print(f"\n--- {test.__name__}")
    test()

print("\n--- results:", "ALL PASS" if not failures else f"{len(failures)} FAILED {failures}")
sys.exit(1 if failures else 0)
