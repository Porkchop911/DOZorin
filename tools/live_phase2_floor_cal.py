"""Phase 2 floor calibration (task 2.5) — attended, open area.

Sequence (1 beep + 8 s placement pause before every run):
  runs 1-3: forward,  commanded exactly 1.00 m at 0.1 m/s
  runs 4-6: strafe left, commanded exactly 1.00 m at 0.1 m/s
  runs 7-9: rotate CCW, commanded exactly 360 deg at 0.4 rad/s
             (gyro-measured actual rotation printed - self-calibrating)

Measure each linear run with tape: straight-line distance start->end, and
note sideways drift. Ctrl-C stops safely at any time.
"""
import math
import signal
import sys
import time

from hwr_core.robot import Robot

SPEED = 0.1        # m/s
ROT_SPEED = 0.4    # rad/s
TARGET_M = 1.0
TARGET_RAD = 2.0 * math.pi
PLACE_PAUSE_S = 8.0


class GyroIntegrator:
    def __init__(self, robot):
        self.bias = 0.0
        self.total = 0.0
        self.active = False
        self._last = None
        self._samples = []
        robot.subscribe_imu(self._on_imu)

    def _on_imu(self, sample, stamp):
        if self._last is None:
            self._last = stamp
            return
        dt = stamp - self._last
        self._last = stamp
        if self.active:
            self.total += (sample.gz - self.bias) * dt
        else:
            self._samples.append(sample.gz)

    def calibrate_bias(self):
        self._samples = []
        time.sleep(2.0)
        if self._samples:
            self.bias = sum(self._samples) / len(self._samples)

    def start(self):
        self.total = 0.0
        self.active = True

    def stop(self) -> float:
        self.active = False
        return self.total


def countdown(robot, label):
    # silent by user request (and the off_ms=0 buzzer incident): printed countdown only
    print(f"\n>>> {label} - place robot at the start mark", flush=True)
    for s in range(int(PLACE_PAUSE_S), 0, -1):
        print(f"    starting in {s} s", flush=True)
        time.sleep(1.0)
    print("    GO", flush=True)


def run_linear(robot, gyro, axis, run_no):
    countdown(robot, f"run {run_no}: {'FORWARD' if axis == 'x' else 'STRAFE LEFT'} 1.00 m")
    gyro.start()
    dist = 0.0
    last = time.monotonic()
    while dist < TARGET_M:
        applied = robot.drive(SPEED if axis == "x" else 0.0,
                              SPEED if axis == "y" else 0.0, 0.0)
        now = time.monotonic()
        dist += (applied[0] if axis == "x" else applied[1]) * (now - last)
        last = now
        time.sleep(0.05)
    robot.stop()
    drift = math.degrees(gyro.stop())
    print(f"    commanded {dist:.3f} m, heading drift {drift:+.1f} deg -> measure actual distance")


def run_rotation(robot, gyro, run_no):
    countdown(robot, f"run {run_no}: ROTATE CCW 360 deg")
    gyro.start()
    cmd_int = 0.0
    last = time.monotonic()
    while cmd_int < TARGET_RAD:
        applied = robot.drive(0.0, 0.0, ROT_SPEED)
        now = time.monotonic()
        cmd_int += applied[2] * (now - last)
        last = now
        time.sleep(0.05)
    robot.stop()
    time.sleep(0.5)
    actual = math.degrees(gyro.stop())
    print(f"    commanded {math.degrees(cmd_int):.1f} deg, gyro-measured {actual:.1f} deg "
          f"(correction {math.degrees(cmd_int) / actual:.4f})" if actual else "    gyro dead?")


def main():
    with Robot.connect("/dev/rrc") as robot:
        signal.signal(signal.SIGINT, lambda *_: (robot.stop(), robot.close(), sys.exit(130)))
        mv = robot.battery_mv(timeout=3.0)
        print(f"battery: {mv} mV")
        if mv is None or mv < 11200:
            print("ABORT: battery")
            return 1

        gyro = GyroIntegrator(robot)
        print("calibrating gyro bias (keep robot still, 2 s) ...")
        gyro.calibrate_bias()
        print(f"gyro z bias: {math.degrees(gyro.bias):+.3f} deg/s")

        for i in (1, 2, 3):
            run_linear(robot, gyro, "x", i)
        for i in (4, 5, 6):
            run_linear(robot, gyro, "y", i)
        for i in (7, 8, 9):
            run_rotation(robot, gyro, i)

        print(f"\nbattery end: {robot.battery_mv()} mV")
        print("done - report the six tape measurements")
        return 0


if __name__ == "__main__":
    sys.exit(main())
