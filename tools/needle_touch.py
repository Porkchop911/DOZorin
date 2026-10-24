"""Needle-touch proof of concept (plan/20): the arm holds a needle, the UGREEN on the PC watches
the paper, and the tip is steered onto drawn dots in image space, then lowered until it touches.

PC side. Servos go through tools/servo_shell.py on the robot (one short SSH session per call).
"""
from __future__ import annotations

import base64
import json
import math
import subprocess
import threading
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hwr_core"))
from hwr_core.arm.kinematics import ArmKinematics, joints_to_pulses  # noqa: E402

ROBOT = "ubuntu@192.168.0.223"
SHELL = "source ~/hwr/deploy/env.zsh; python3 ~/hwr/tools/servo_shell.py"


_session: subprocess.Popen | None = None


def _reply() -> dict:
    while True:  # skip ROS log noise on stdout
        line = _session.stdout.readline()
        if not line:
            raise RuntimeError("servo shell session ended")
        if line.startswith("{"):
            return json.loads(line)


def servo(*commands: dict) -> list[dict]:
    """One long-lived SSH session to the servo shell (never reconnect in a loop: sshd MaxStartups)."""
    global _session
    if _session is None or _session.poll() is not None:
        _session = subprocess.Popen(["ssh", ROBOT, f'zsh -c "{SHELL}"'], stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        if not _reply().get("ready"):
            raise RuntimeError("servo shell not ready (no joint_states or no driver subscription)")
    replies = []
    for command in commands:
        _session.stdin.write(json.dumps(command) + "\n")
        _session.stdin.flush()
        replies.append(_reply())
    errors = [r["error"] for r in replies if "error" in r]
    if errors:
        raise RuntimeError(errors)
    return replies


def read() -> dict:
    return {int(k): v for k, v in servo({"read": True})[0]["pulses"].items()}


PAN_PRELOAD = 12  # pulses past the pan target before the final approach from below


class Arm:
    def __init__(self, needle: float):
        self.kin = ArmKinematics(tool=0.05577 + 0.08 + needle)  # needle = tip past the tool point

    def pitch_for(self, x: float, y: float, low: float, high: float) -> float:
        """Steepest tool pitch that reaches (x, y) at every height low..high. One pitch per spot:
        a pitch change mid-descent swaps the posture and the 'lower' pose need not be lower."""
        for degrees in range(-85, -24, 5):
            pitch = math.radians(degrees)
            if all(self.kin.ik(x, y, z, pitch) for z in np.linspace(low, high, 6)):
                return pitch
        raise ValueError(f"no pitch reaches {x:.3f} {y:.3f} over z {low:.3f}..{high:.3f}")

    def pulses(self, x: float, y: float, z: float, pitch: float) -> dict:
        solution = self.kin.ik(x, y, z, pitch)
        if solution is None:
            raise ValueError(f"unreachable {x:.3f} {y:.3f} {z:.3f} pitch {math.degrees(pitch):.0f}")
        return {k: v for k, v in joints_to_pulses(solution).items() if k != 5}

    def move(self, x: float, y: float, z: float, pitch: float, ms: int = 1200, preload: bool = True) -> dict:
        target = self.pulses(x, y, z, pitch)
        pan = read()[1]
        if preload and target[1] < pan:
            # pan backlash: ~+-7 mm at the tip (live). Always finish the pan from below; only the
            # pan overshoots, so the tip height is not affected near the paper.
            servo({"set": {"1": max(target[1] - PAN_PRELOAD, 0)}, "ms": 300})
            time.sleep(0.5)
        for _ in range(2):  # a dropped command is resent once
            servo({"set": {str(k): v for k, v in target.items()}, "ms": ms})
            time.sleep(ms / 1000 + 0.5)
            now = read()
            off = max(abs(now[k] - v) for k, v in target.items())
            if off <= 35:  # sag under load: elbow ~12 (plan/02), shoulder 21 with the needle (live)
                return target
        raise RuntimeError(f"arm did not reach {target}: readback {now}")


class Camera:
    def __init__(self, index: int = 0):
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        deadline = time.monotonic() + 3.0  # auto exposure settles; first frames come out dark
        while time.monotonic() < deadline:
            self.cap.read()
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)  # lock it: frame diffs need a steady exposure
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        # DSHOW ignores BUFFERSIZE and grabbing a few frames did not flush it (live: the image
        # showed the needle where it had been), so a thread drains the camera all the time.
        self.latest: list[tuple[float, np.ndarray]] = []
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        while True:
            ok, image = self.cap.read()
            if ok:
                self.latest = (self.latest + [(time.monotonic(), image)])[-8:]

    def frame(self, average: int = 3, settle: float = 0.3) -> np.ndarray:
        """Mean of `average` frames read at least `settle` s after this call."""
        after = time.monotonic() + settle
        deadline = after + 5.0
        while time.monotonic() < deadline:
            fresh = [image for t, image in self.latest if t > after]
            if len(fresh) >= average:
                return (sum(f.astype(np.float32) for f in fresh[:average]) / average).astype(np.uint8)
            time.sleep(0.02)
        raise RuntimeError("camera stopped delivering frames")


def dots(image: np.ndarray, color: str, min_area: int = 40) -> list[tuple[float, float]]:
    """Centroids of small black or red ink dots on white paper, sorted left to right."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == "black":
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 70))
    else:
        mask = cv2.inRange(hsv, (0, 90, 90), (12, 255, 255)) | cv2.inRange(hsv, (168, 90, 90), (180, 255, 255))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    found = [tuple(centers[i]) for i in range(1, count) if min_area <= stats[i, cv2.CC_STAT_AREA] <= 900]
    return sorted(found)


PAPER = (slice(120, 800), slice(300, 1500))  # v, u window: the sheet below the robot, above the holes


def arm_snap() -> np.ndarray:
    """Arm camera (eye-in-hand, looks along the needle) via the servo shell; no gateway token."""
    jpeg = base64.b64decode(servo({"snap": True})[0]["jpeg"])
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def arm_needle(image: np.ndarray) -> tuple[tuple[float, float], np.ndarray] | None:
    """(tip pixel, needle mask) in the arm camera: the dark blob rising from the bottom edge
    (fingers + needle); the tip is its topmost pixel."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    count, labels, stats, _ = cv2.connectedComponentsWithStats((gray < 70).astype(np.uint8))
    bottom = [i for i in range(1, count)
              if stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] >= gray.shape[0] - 2]
    if not bottom:
        return None
    needle = max(bottom, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    vs, us = np.nonzero(labels == needle)
    top = int(np.argmin(vs))
    mask = cv2.dilate((labels == needle).astype(np.uint8), np.ones((9, 9), np.uint8))
    mask[:max(vs[top] - 2, 0)] = 0  # keep the paper beyond the tip visible: the dot ends up there
    return (float(us[top]), float(vs[top])), mask


def arm_dots(image: np.ndarray, color: str, mask: np.ndarray) -> list[np.ndarray]:
    """Ink dots in the arm camera, ignoring the needle itself."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # local contrast: the dots are small and blurred here (black min 35-97, red ~125, paper ~190)
    spots = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    reddish = (hsv[..., 1] > 50) & ((hsv[..., 0] > 120) | (hsv[..., 0] < 15))
    hit = spots > 70 if color == "black" else (spots > 35) & (spots <= 70) & reddish
    hit &= mask == 0
    count, _, stats, centers = cv2.connectedComponentsWithStats(hit.astype(np.uint8))
    # shortcut: area cap drops punched holes/binding only while dots stay small in view
    return [np.array(centers[i]) for i in range(1, count) if 4 <= stats[i, cv2.CC_STAT_AREA] <= 400]


def arm_dot(image: np.ndarray, color: str, mask: np.ndarray, near, radius: float = 80) -> np.ndarray | None:
    """The dot nearest `near` in the arm camera."""
    found = [d for d in arm_dots(image, color, mask) if np.hypot(*(d - np.asarray(near))) < radius]
    return min(found, key=lambda d: np.hypot(*(d - np.asarray(near)))) if found else None


def tip(reference: np.ndarray, image: np.ndarray, near: tuple[float, float] | None = None,
        radius: int = 120) -> tuple[float, float] | None:
    """Needle tip = lowest (largest v) pixel that is near-black and changed vs a needle-free
    reference, searched around `near`. The tip end is dark (gray 10-50); its shadow is as dark and
    ends *below* the tip in this camera view, so the two are told apart by shape."""
    window = PAPER
    if near is not None:
        u, v = int(near[0]), int(near[1])
        window = (slice(max(v - radius, 0), v + radius), slice(max(u - radius, 0), u + radius))
    ref = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)[window].astype(np.float32)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)[window].astype(np.float32)
    diff = np.abs(img * (ref.mean() / img.mean()) - ref)
    dark = img < 0.5 * np.median(img)
    # the hanging needle is near-vertical in the image, its shadow diagonal (only ~7 px tall):
    # a 1x13 vertical opening keeps the tip's dark end and drops the shadow
    mask = cv2.morphologyEx(((diff > 35) & dark).astype(np.uint8), cv2.MORPH_OPEN, np.ones((13, 1), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    tallest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_HEIGHT]))  # shadow scraps are short
    vs, us = np.nonzero(labels == tallest)
    i = int(np.argmax(vs))
    return float(us[i] + window[1].start), float(vs[i] + window[0].start)


PAN_ASIDE = 250  # reference frame: arm swung ~60 deg right at low height (the folded pose points
# the needle up into the camera)
# Needle 6 cm past the fingertips ~ 0.065 past the tool point (live: model 0.085 ended ~2 cm above
# the paper at model z 0.009). The model needle is kept ~2 cm shorter, so the paper sits near
# model z +0.02 and poses stay above the z >= 0 guard in ik().
NEEDLE_MODEL = 0.045
START_Z, FLOOR_Z, STEP = 0.06, 0.010, 0.0015  # model heights; FLOOR ~1 cm past the estimated paper
PROBE = 0.008  # arm-camera Jacobian probe step (+-)
GAIN = 0.6  # damped correction; full gain oscillated (live 2026-10-24)
ONE_DOT = True  # user: one more try
AIM_BEYOND = 5  # px past the tip along the needle in the arm image (~1.5 mm at the dot)


def run(color: str, out: Path, x0: float = 0.25, y0: float = 0.0) -> None:
    """Arm camera steers (the tip sits at a fixed pixel in it), UGREEN detects contact.
    The only contact with the paper is the touch on each dot (user, 2026-10-24)."""
    out.mkdir(parents=True, exist_ok=True)
    arm, cam = Arm(NEEDLE_MODEL), Camera()
    log = (out / "touches.csv").open("a", encoding="utf-8")
    shots = iter(range(10_000))

    def choose_pitch(x, y):
        for degrees in range(-85, -24, 5):
            pitch = math.radians(degrees)
            # +-2 cm margin: Jacobian probes and corrections must stay reachable at this pitch
            if all(arm.kin.ik(x + dx, y + dy, z, pitch) for z in np.linspace(FLOOR_Z, START_Z + 0.01, 6)
                   for dx in (-0.02, 0.0, 0.02) for dy in (-0.02, 0.0, 0.02)):
                return pitch
        raise ValueError(f"no pitch reaches {x:.3f} {y:.3f}")

    def snap(name=None):
        image = arm_snap()
        if name:
            cv2.imwrite(str(out / name), image)
        return image

    pitch = choose_pitch(x0, y0)
    arm.move(x0, y0, START_Z, pitch, ms=2000)
    servo({"set": {"1": PAN_ASIDE}, "ms": 1500})
    time.sleep(2.3)
    reference = cam.frame()
    cv2.imwrite(str(out / "reference.jpg"), reference)
    arm.move(x0, y0, START_Z, pitch, ms=1500)
    overview = snap("overview_arm.jpg")
    found = arm_needle(overview)
    if found is None:
        raise RuntimeError("arm camera does not see the needle")
    tip_a, needle_mask = found
    print(f"arm cam tip {tip_a}; {color} dots {[d.round().tolist() for d in arm_dots(overview, color, needle_mask)]}",
          flush=True)

    def cands(found, near):
        """Log helper: [pixel, distance] for a candidate list, nearest first."""
        target = np.asarray(near)
        ordered = sorted(found, key=lambda d: np.hypot(*(d - target)))
        return [[d.round(1).tolist(), round(float(np.hypot(*(d - target))), 1)] for d in ordered[:6]]

    xy, touched = np.array([x0, y0]), []
    while True:
        dot_index = len(touched)
        # pick the untouched dot nearest the tip, from the current hover
        arm.move(*xy, START_Z, pitch)
        image = snap()
        jac = None
        # shortcut: the spiral binding and punched holes sit >= ~140 px from the tip at this hover;
        # a radius keeps them out until the binding gets its own mask
        candidates = [d for d in arm_dots(image, color, needle_mask) if np.hypot(*(d - np.array(tip_a))) < 130]
        # run 17 logging (plan/20 next step 1): what the hover sees before any probe move
        print(f"hover {xy.round(4)} dots in 130 px of tip {tuple(tip_a)}: {cands(candidates, tip_a)}", flush=True)
        # Jacobian of arm-image dot motion per metre of base xy, from two probes at this hover
        if candidates:
            ref_dot = min(candidates, key=lambda d: np.hypot(*(d - np.array(tip_a))))
            # central differences, each pair approached from the same side (deadband/backlash:
            # 3 mm one-sided probes gave Jacobians that disagreed 10x between dots, live)
            columns = []
            for axis, d in enumerate((np.array([PROBE, 0.0]), np.array([0.0, PROBE]))):
                seen = []
                for sign in (-1.0, 1.0):
                    arm.move(*(xy - 2 * d), START_Z, pitch, ms=500)
                    arm.move(*(xy + sign * d), START_Z, pitch, ms=500)
                    probe = snap(f"probe{dot_index}_{axis}{'plus' if sign > 0 else 'minus'}.jpg")
                    found = arm_dots(probe, color, needle_mask)
                    near = sorted([(f, float(np.hypot(*(f - ref_dot)))) for f in found
                                   if np.hypot(*(f - ref_dot)) < 90], key=lambda t: t[1])
                    print(f"  probe {axis}{'+' if sign > 0 else '-'} xy {(xy + sign * d).round(4)} "
                          f"dots {len(found)} near ref {ref_dot.round(1)}: "
                          f"{[[p.round(1).tolist(), round(dd, 1)] for p, dd in near[:6]]}", flush=True)
                    seen.append(near[0][0] if near else None)
                columns.append(None if any(s is None for s in seen) else (seen[1] - seen[0]) / (2 * PROBE))
            arm.move(*(xy - np.array([2 * PROBE, 0.0])), START_Z, pitch, ms=500)
            arm.move(*xy, START_Z, pitch, ms=500)
            if all(c is not None for c in columns):
                jac = np.column_stack(columns)
        if jac is None:
            print(f"dot {dot_index}: no Jacobian (probe stage lost the dot or none in 130 px); done", flush=True)
            break
        if abs(np.linalg.det(jac)) < 1e-3:
            print(f"dot {dot_index}: degenerate Jacobian {jac.round(2).tolist()} det {np.linalg.det(jac):.1e}; done",
                  flush=True)
            break
        to_xy = np.linalg.inv(jac)
        fresh = [d for d in candidates
                 if all(np.hypot(*(xy + to_xy @ (np.array(tip_a) - d) - t)) > 0.008 for t in touched)]
        if not fresh:
            print("all visible dots touched; done", flush=True)
            break
        target = min(fresh, key=lambda d: np.hypot(*(d - np.array(tip_a))))
        n = len(touched)
        print(f"dot {n}: arm-image {target.round(1)}, px/mm {(jac / 1000).round(1).tolist()}", flush=True)

        # align at hover, then descend 1.5 mm per step, re-aligning every step
        last, z, ugreen_prev, expected, aligned, still = target, START_Z, None, None, False, 0
        ys, xs = np.nonzero(needle_mask)
        along_img = np.array(tip_a) - np.array([xs.mean(), ys.mean()])
        aim = np.array(tip_a) + AIM_BEYOND * along_img / np.hypot(*along_img)
        for step in range(10):
            image = snap(f"aim{dot_index}_{step:02d}.jpg")
            seen = arm_dot(image, color, needle_mask, last, radius=60)
            if seen is None:
                print(f"  hover: lost the dot; dots near {last.round(1)}: "
                      f"{cands(arm_dots(image, color, needle_mask), last)}", flush=True)
                break
            offset = aim - seen
            last = seen
            print(f"  hover xy {xy.round(4)} dot {seen.round(1)} aim {aim.round(1)} offset {offset.round(1)}", flush=True)
            if np.hypot(*offset) < 3:
                aligned = True
                break
            candidate = xy + GAIN * (to_xy @ offset)
            if not all(arm.kin.ik(*candidate, zz, pitch) for zz in (FLOOR_Z, START_Z)):
                break
            xy = candidate
            arm.move(*xy, z, pitch, ms=600)
        if not aligned:
            print(f"dot {n}: could not line up at hover; skipped (no touch)", flush=True)
            touched.append(xy + to_xy @ (np.array(tip_a) - last))  # do not pick it again
            continue
        # descend along the camera->tip ray: camera and needle are rigid, so the dot stays at the
        # aim pixel and the tip runs into it; no paper height or parallax model needed
        joints = arm.kin.ik(*xy, z, pitch)
        _, camera = arm.kin.camera_pose(joints)
        start_tip = np.array(arm.kin.fk(joints)[:3])
        ray = (start_tip - camera) / np.linalg.norm(start_tip - camera)
        print(f"  ray {ray.round(3)} from tip {start_tip.round(4)}", flush=True)
        contact, travel, image = False, 0.0, cam.frame()
        while True:
            travel += STEP
            point = start_tip + travel * ray
            if point[2] < FLOOR_Z:
                break
            arm.move(*point, pitch, ms=400, preload=False)  # no sideways pan swing near the paper
            z = point[2]
            image = cam.frame()
            ugreen = tip(reference, image, None)
            moved = None
            if ugreen is not None and ugreen_prev is not None:
                moved = float(ugreen[1] - ugreen_prev[1])  # descending moves the tip down the image
                if moved >= 2:
                    expected, still = moved, 0
                elif expected is not None:
                    still = still + 1 if abs(moved) < 1 else 0  # a jump up is not "still"
                    contact = still >= 2  # ~5 px per step: one quiet frame can be noise
            print(f"  travel {travel * 1000:.1f} mm tip {point.round(4)} ugreen {ugreen} moved {moved}", flush=True)
            ugreen_prev = ugreen if ugreen is not None else ugreen_prev
            if contact:
                break
        xy = point[:2]
        cv2.imwrite(str(out / f"shot_{next(shots):03d}_{color}_{n}_touch.jpg"), image)
        snap(f"{color}_{n}_touch_arm.jpg")
        print(f"dot {n} {'TOUCH' if contact else 'reached floor without seeing contact'} at model z {z:.4f}, "
              f"ugreen tip {ugreen_prev}", flush=True)
        log.write(f"{time.time():.0f},{color},{n},{int(contact)},{z:.4f},{xy[0]:.4f},{xy[1]:.4f},"
                  f"{'' if ugreen_prev is None else f'{ugreen_prev[0]:.1f},{ugreen_prev[1]:.1f}'}\n")
        log.flush()
        touched.append(xy.copy())
        arm.move(*xy, START_Z, pitch, ms=1000)
        if ONE_DOT:
            break
    arm.move(x0, y0, START_Z, choose_pitch(x0, y0), ms=1500)


if __name__ == "__main__":
    run(sys.argv[1], Path(sys.argv[2]))
