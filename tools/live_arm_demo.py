"""Arm re-demo for task 1.6: beep, pan left + camera up, then forward pose.

Run ON the robot inside a takeover window. Arm must be clear.
"""
import signal
import sys
import time

from hwr_core.rrc import Board

FORWARD_POSE = [(1, 500), (2, 701), (3, 120), (4, 300), (5, 500)]
INTERMEDIATE = [(1, 650), (4, 500)]  # pan left, camera tilt up - mild, known-safe values


def main():
    board = Board.open("/dev/rrc")
    signal.signal(signal.SIGINT, lambda *_: (board.stop_servos([1, 2, 3, 4, 5, 10]),
                                             board.close(), sys.exit(130)))
    try:
        mv = board.battery_mv(timeout=3.0)
        print(f"battery: {mv} mV")
        if mv is None or mv < 11000:
            print("ABORT: battery gate")
            return 1

        board.beep(1500, 150, 100, 2)
        time.sleep(0.8)
        print("pan left + camera up (1.2 s) ...")
        board.set_servo_positions(1200, INTERMEDIATE)
        time.sleep(1.8)
        print("back to forward pose (1.5 s) ...")
        board.set_servo_positions(1500, FORWARD_POSE)
        time.sleep(2.2)

        worst = 0
        for sid, target in FORWARD_POSE:
            pos = board.servo_position(sid)
            delta = None if pos is None else pos - target
            if delta is not None:
                worst = max(worst, abs(delta))
            print(f"  servo {sid}: target={target} actual={pos} delta={delta}")
        print(f"worst error: {worst} pulses ({worst * 0.24:.1f} deg)")
        print("PASS" if worst <= 20 else "CHECK")
        return 0
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())
