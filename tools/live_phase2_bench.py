"""Phase 2 bench validation (task 2.4) — mixing sanity + live watchdog demo.

PRECONDITION: robot on stand, wheels off the ground.
Run ON the robot inside a takeover window:
    PYTHONPATH=$HOME/hwr/hwr_core python3 $HOME/hwr/tools/live_phase2_bench.py

Sequence (each announced by beeps):
  1 beep : +vx 0.1 m/s   -> ALL wheels roll in robot-forward sense
  2 beeps: +vy 0.1 m/s   -> strafe-left pattern: FL back, RL fwd, FR fwd, RR back
  3 beeps: +wz 0.5 rad/s -> CCW: left wheels backward, right wheels forward
  4 beeps: watchdog demo -> drive starts, command source "dies", wheels must
           stop by themselves ~0.4 s later (timing printed)
"""
import sys
import time

from hwr_core.robot import Robot

HOLD_S = 2.5


def held_drive(robot, vx, vy, wz, seconds):
    """Drive while feeding the watchdog at 10 Hz (a live command source)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        robot.drive(vx, vy, wz)
        time.sleep(0.1)
    robot.stop()


def main():
    with Robot.connect("/dev/rrc") as robot:
        mv = robot.battery_mv(timeout=3.0)
        print(f"battery: {mv} mV")
        if mv is None or mv < 11000:
            print("ABORT: battery gate")
            return 1

        tests = [("+vx 0.1 (all wheels forward sense)", (0.1, 0.0, 0.0), 1),
                 ("+vy 0.1 strafe left (FL back, RL fwd, FR fwd, RR back)", (0.0, 0.1, 0.0), 2),
                 ("+wz 0.5 CCW (left back, right fwd)", (0.0, 0.0, 0.5), 3)]
        for label, (vx, vy, wz), beeps in tests:
            print(f"{beeps} beep(s): {label}")
            robot.beep(1200, 80, 120, beeps)
            time.sleep(0.6 + 0.2 * beeps)
            held_drive(robot, vx, vy, wz, HOLD_S)
            time.sleep(1.0)

        # ---- watchdog demo: command source dies mid-drive
        print("4 beeps: watchdog demo - drive, then command source dies")
        robot.beep(1200, 80, 120, 4)
        time.sleep(1.4)
        for _ in range(10):                    # 1 s of live driving
            robot.drive(0.1, 0.0, 0.0)
            time.sleep(0.1)
        t_dead = time.monotonic()
        print("  command source dead; waiting for watchdog ...")
        while robot.watchdog.fired_count == 0 and time.monotonic() - t_dead < 2.0:
            time.sleep(0.005)
        dt = time.monotonic() - t_dead
        if robot.watchdog.fired_count:
            print(f"  watchdog fired {dt:.3f} s after last command "
                  f"(timeout 0.4 s) -> {'PASS' if dt <= 0.5 else 'FAIL'}")
        else:
            print("  FAIL: watchdog never fired")
            return 1

        mv2 = robot.battery_mv()
        print(f"battery end: {mv2} mV")
        print(f"decoder: ok={robot._board._t.decoder.frames_ok} "
              f"crc_errors={robot._board._t.decoder.crc_errors} "
              f"resyncs={robot._board._t.decoder.resyncs}")
        return 0


if __name__ == "__main__":
    sys.exit(main())
