"""Three attended, no-reverse 1 m square acceptance runs for Phase 2."""
from __future__ import annotations

import json
import math
import signal
import sys
import time
from pathlib import Path

import yaml

from hwr_core.kinematics import Calibration
from hwr_core.robot import Robot

SPEED = 0.1
ROT_SPEED = 0.35


def drive_for(robot: Robot, vx: float, vy: float, wz: float, amount: float) -> None:
    elapsed_amount = 0.0
    last = time.monotonic()
    while elapsed_amount < amount:
        applied = robot.drive(vx, vy, wz)
        now = time.monotonic()
        component = applied[0] if vx else applied[1] if vy else applied[2]
        elapsed_amount += abs(component) * (now - last)
        last = now
        time.sleep(0.05)
    robot.stop()
    time.sleep(0.5)


def load_calibration(path: Path) -> Calibration:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = raw["hwr_base"]["ros__parameters"]
    return Calibration(params["cal_linear"], params["cal_strafe"],
                       params["cal_angular"])


def main() -> int:
    root = Path.home() / "hwr"
    config = root / "ros2_ws/src/hwr_base/config/calibration.yaml"
    evidence = root / "evidence/phase2/square_acceptance.json"
    results = []
    with Robot.connect("/dev/rrc", calibration=load_calibration(config)) as robot:
        signal.signal(signal.SIGINT, lambda *_: (robot.stop(), sys.exit(130)))
        for run in range(1, 4):
            input(f"run {run}: place robot at start, clear the area, then press Enter")
            # World square without negative body-x: forward, left, turn around,
            # forward, left (world right), then turn back to starting yaw.
            drive_for(robot, SPEED, 0.0, 0.0, 1.0)
            drive_for(robot, 0.0, SPEED, 0.0, 1.0)
            drive_for(robot, 0.0, 0.0, ROT_SPEED, math.pi)
            drive_for(robot, SPEED, 0.0, 0.0, 1.0)
            drive_for(robot, 0.0, SPEED, 0.0, 1.0)
            drive_for(robot, 0.0, 0.0, ROT_SPEED, math.pi)
            error_cm = float(input("measured distance from start (cm): "))
            yaw_deg = float(input("measured absolute heading error (deg): "))
            results.append({"run": run, "position_error_cm": error_cm,
                            "heading_error_deg": yaw_deg,
                            "pass": error_cm <= 10.0 and yaw_deg <= 15.0})
        evidence.parent.mkdir(parents=True, exist_ok=True)
        evidence.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        print(f"evidence: {evidence}")
    return 0 if all(result["pass"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
