"""Staged, attended Phase-2 floor acceptance.

Each invocation performs at most one bounded motion.  Linear measurements and
square errors are recorded by separate non-hardware commands, making this safe
to drive through a non-interactive SSH session.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
from pathlib import Path

from hwr_core.kinematics import Calibration
from hwr_core.robot import Robot

from live_phase2_floor_cal import (GyroIntegrator, run_linear, run_rotation)
from live_phase2_square import drive_for, load_calibration


ROOT = Path.home() / "hwr"
RUNS_PATH = ROOT / "evidence/phase2/floor_calibration_runs.json"
SQUARE_PATH = ROOT / "evidence/phase2/square_acceptance.json"
CAL_PATH = ROOT / "ros2_ws/src/hwr_base/config/calibration.yaml"


def _load(path: Path, default):
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def _save(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _connect(calibration: Calibration | None = None) -> Robot:
    robot = Robot.connect("/dev/rrc", calibration=calibration)
    signal.signal(signal.SIGINT, lambda *_: (robot.stop(), robot.close(), sys.exit(130)))
    return robot


def calibration_run(axis: str, run: int) -> None:
    strafe_runs = (4, 5, 6, 10)
    expected = {"forward": range(1, 4), "strafe": strafe_runs,
                "strafe-right": strafe_runs, "rotate": range(7, 10)}
    if run not in expected[axis]:
        raise ValueError(f"run {run} is not valid for {axis}")
    robot = _connect()
    try:
        gyro = GyroIntegrator(robot)
        print("calibrating gyro bias; keep robot still")
        gyro.calibrate_bias()
        if axis == "rotate":
            result = run_rotation(robot, gyro, run)
        else:
            # The shared helper asks for an interactive tape measurement, so
            # reproduce only its bounded motion portion here.
            from live_phase2_floor_cal import SPEED, TARGET_M, countdown
            import time

            label = "STRAFE RIGHT" if axis == "strafe-right" else axis.upper()
            countdown(robot, f"run {run}: {label} 1.00 m")
            gyro.start()
            distance = 0.0
            last = time.monotonic()
            while distance < TARGET_M:
                strafe_speed = -SPEED if axis == "strafe-right" else SPEED
                applied = robot.drive(SPEED if axis == "forward" else 0.0,
                                      strafe_speed if axis.startswith("strafe") else 0.0,
                                      0.0)
                now = time.monotonic()
                component = applied[0] if axis == "forward" else applied[1]
                distance += abs(component) * (now - last)
                last = now
                time.sleep(0.05)
            robot.stop()
            result = {"commanded_m": distance,
                      "heading_drift_deg": math.degrees(gyro.stop()),
                      "actual_m": None}
    finally:
        robot.stop()
        robot.close()
    runs = _load(RUNS_PATH, {})
    canonical_axis = "strafe" if axis.startswith("strafe") else axis
    direction = "right" if axis == "strafe-right" else "left" if axis == "strafe" else axis
    runs[str(run)] = {"axis": canonical_axis, "direction": direction, **result}
    _save(RUNS_PATH, runs)
    print(f"run {run} complete; robot stopped")


def record_linear(run: int, actual_m: float) -> None:
    if actual_m <= 0.0:
        raise ValueError("actual distance must be positive")
    runs = _load(RUNS_PATH, {})
    item = runs.get(str(run))
    if item is None or item.get("axis") not in {"forward", "strafe"}:
        raise ValueError(f"linear run {run} has not completed")
    item["actual_m"] = actual_m
    _save(RUNS_PATH, runs)
    print(f"measurement recorded for run {run}")


def finalize_calibration() -> None:
    raw = _load(RUNS_PATH, {})
    groups = {name: [item for item in raw.values() if item.get("axis") == name]
              for name in ("forward", "strafe", "rotate")}
    if len(groups["forward"]) != 3 or len(groups["rotate"]) != 3:
        raise ValueError("three forward and three rotation runs are required")
    if len(groups["strafe"]) < 3:
        raise ValueError("at least three strafe runs are required")
    if any(item.get("actual_m") is None
           for name in ("forward", "strafe") for item in groups[name]):
        raise ValueError("all linear tape measurements are required")
    calibration = {
        "linear": 1.0 / statistics.median(item["actual_m"] for item in groups["forward"]),
        "strafe": 1.0 / statistics.median(item["actual_m"] for item in groups["strafe"]),
        "angular": 360.0 / statistics.median(abs(item["actual_deg"])
                                              for item in groups["rotate"]),
    }
    CAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    CAL_PATH.write_text(
        "hwr_base:\n  ros__parameters:\n"
        f"    cal_linear: {calibration['linear']:.6f}\n"
        f"    cal_strafe: {calibration['strafe']:.6f}\n"
        f"    cal_angular: {calibration['angular']:.6f}\n",
        encoding="utf-8")
    _save(ROOT / "evidence/phase2/floor_calibration.json",
          {"runs": raw, "calibration": calibration})
    print("calibration finalized")


def square_run(run: int) -> None:
    if run not in (1, 2, 3):
        raise ValueError("square run must be 1, 2, or 3")
    robot = _connect(load_calibration(CAL_PATH))
    try:
        print(f"square run {run}: starting in 8 seconds")
        import time
        time.sleep(8.0)
        drive_for(robot, 0.1, 0.0, 0.0, 1.0)
        drive_for(robot, 0.0, 0.1, 0.0, 1.0)
        drive_for(robot, 0.0, 0.0, 0.35, math.pi)
        drive_for(robot, 0.1, 0.0, 0.0, 1.0)
        drive_for(robot, 0.0, 0.1, 0.0, 1.0)
        drive_for(robot, 0.0, 0.0, 0.35, math.pi)
    finally:
        robot.stop()
        robot.close()
    results = _load(SQUARE_PATH, {})
    results[str(run)] = {"run": run, "position_error_cm": None,
                         "heading_error_deg": None, "pass": None}
    _save(SQUARE_PATH, results)
    print(f"square run {run} complete; robot stopped")


def record_square(run: int, position_cm: float, heading_deg: float) -> None:
    if min(position_cm, heading_deg) < 0.0:
        raise ValueError("errors must be non-negative")
    results = _load(SQUARE_PATH, {})
    if str(run) not in results:
        raise ValueError(f"square run {run} has not completed")
    results[str(run)] = {"run": run, "position_error_cm": position_cm,
                         "heading_error_deg": heading_deg,
                         "pass": position_cm <= 10.0 and heading_deg <= 15.0}
    _save(SQUARE_PATH, results)
    print(f"square result {run}: {'PASS' if results[str(run)]['pass'] else 'FAIL'}")


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("cal-run")
    run.add_argument("axis", choices=("forward", "strafe", "strafe-right", "rotate"))
    run.add_argument("run", type=int)
    measure = sub.add_parser("record-linear")
    measure.add_argument("run", type=int)
    measure.add_argument("actual_m", type=float)
    sub.add_parser("finalize-calibration")
    square = sub.add_parser("square-run")
    square.add_argument("run", type=int)
    square_result = sub.add_parser("record-square")
    square_result.add_argument("run", type=int)
    square_result.add_argument("position_cm", type=float)
    square_result.add_argument("heading_deg", type=float)
    args = parser.parse_args()
    try:
        if args.command == "cal-run":
            calibration_run(args.axis, args.run)
        elif args.command == "record-linear":
            record_linear(args.run, args.actual_m)
        elif args.command == "finalize-calibration":
            finalize_calibration()
        elif args.command == "square-run":
            square_run(args.run)
        else:
            record_square(args.run, args.position_cm, args.heading_deg)
    except (RuntimeError, ValueError) as exc:
        print(f"ABORT: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
