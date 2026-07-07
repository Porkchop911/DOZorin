"""Phase 1 bench test 1.5 + 1.6 — motors on stand, servo forward pose.

PRECONDITION: robot on a stand, all four wheels off the ground, arm clear.
Run ON the robot inside a takeover window:
    PYTHONPATH=$HOME/hwr/hwr_core python3 $HOME/hwr/tools/live_phase1_bench.py

Motor protocol (raw wire ids, NO kinematics sign handling):
  for each motor 1..4: N beeps announce motor N, then +0.5 / +1.0 / +2.0 rps
  steps 1.5 s each, then stop. Then a 3.0 rps probe on motor 1, then all-stop.
Servo protocol: read all positions/vin/temp, move to the known-good forward
pose (1,500)(2,701)(3,120)(4,300)(5,500) over 1.5 s, read back, report deltas.
Ctrl-C at any point stops all motors.
"""
import signal
import sys
import time

from hwr_core.rrc import Board, codecs

FORWARD_POSE = [(1, 500), (2, 701), (3, 120), (4, 300), (5, 500)]
SERVO_IDS = [1, 2, 3, 4, 5, 10]


def main():
    board = Board.open("/dev/rrc")

    def panic(*_):
        board.stop_all_motors()
        board.stop_servos(SERVO_IDS)
        board.close()
        sys.exit(130)

    signal.signal(signal.SIGINT, panic)
    try:
        mv = board.battery_mv(timeout=3.0)
        print(f"battery: {mv} mV")
        if mv is None or mv < 11000:
            print("ABORT: battery gate")
            return 1

        # ---------------- 1.5 motors (bench) ----------------
        print("motor type -> JGB37")
        board.set_motor_type(codecs.MOTOR_TYPE_JGB37)
        time.sleep(0.2)

        for motor in (1, 2, 3, 4):
            print(f"motor {motor}: announcing with {motor} beep(s)")
            board.beep(1200, 80, 120, motor)
            board.oled_text(1, f"M{motor} test")
            time.sleep(0.5 + 0.2 * motor)
            for rps in (0.5, 1.0, 2.0):
                print(f"  motor {motor} -> +{rps} rps (1.5 s)")
                board.set_motor_rps([(motor, rps)])
                time.sleep(1.5)
            board.set_motor_rps([(motor, 0.0)])
            print(f"  motor {motor} stop")
            time.sleep(1.0)

        print("motor 1 high-speed probe: +3.0 rps (1.5 s)")
        board.set_motor_rps([(1, 3.0)])
        time.sleep(1.5)
        board.stop_all_motors()
        time.sleep(0.5)

        print("all four: +1.0 rps together (2 s) - listen for balance")
        board.set_motor_rps([(1, 1.0), (2, 1.0), (3, 1.0), (4, 1.0)])
        time.sleep(2.0)
        board.stop_all_motors()
        print("motors done, all stopped")
        time.sleep(0.5)

        # ---------------- 1.6 bus servos ----------------
        print("servo state before move:")
        before = {}
        for sid in SERVO_IDS:
            pos = board.servo_position(sid)
            vin = board.servo_vin(sid)
            temp = board.servo_temp(sid)
            before[sid] = pos
            print(f"  servo {sid}: pos={pos} vin={vin}mV temp={temp}C")

        print("moving to forward pose over 1.5 s ...")
        board.set_servo_positions(1500, FORWARD_POSE)
        time.sleep(2.2)

        print("read-back after move:")
        worst = 0
        for sid, target in FORWARD_POSE:
            pos = board.servo_position(sid)
            delta = None if pos is None else pos - target
            if delta is not None:
                worst = max(worst, abs(delta))
            print(f"  servo {sid}: target={target} actual={pos} delta={delta}")
        print(f"worst position error: {worst} pulses ({worst * 0.24:.1f} deg)")

        mv2 = board.battery_mv(timeout=3.0)
        print(f"battery end: {mv2} mV")
        print(f"decoder: ok={board._t.decoder.frames_ok} "
              f"crc_errors={board._t.decoder.crc_errors} "
              f"resyncs={board._t.decoder.resyncs}")
        print("PASS" if worst <= 20 else "CHECK: servo error > 20 pulses")
        return 0
    finally:
        board.stop_all_motors()
        board.close()


if __name__ == "__main__":
    sys.exit(main())
