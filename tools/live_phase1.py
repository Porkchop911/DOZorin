"""Phase 1 live test 1.3/1.4 — read-only functions + harmless actuators.

Run ON the robot inside a takeover window:
    PYTHONPATH=$HOME/hwr/hwr_core python3 $HOME/hwr/tools/live_phase1.py

Prints battery, measures IMU + battery report rates over 30 s, then one
beep, one LED blink, one OLED write. No motion. Ctrl-C safe.
"""
import signal
import sys
import time

from hwr_core.rrc import Board

DURATION_S = 30.0


def main():
    board = Board.open("/dev/rrc")
    signal.signal(signal.SIGINT, lambda *_: (board.close(), sys.exit(130)))
    try:
        # ---- battery (1.3)
        mv = board.battery_mv(timeout=3.0)
        print(f"battery: {mv} mV")
        if mv is None:
            print("FAIL: no battery report")
            return 1

        # ---- IMU stream rate (1.3): acceptance >= 40 Hz sustained 30 s
        imu_stamps = []
        board.subscribe_imu(lambda sample, stamp: imu_stamps.append(stamp))
        sys_reports = []
        board._t.subscribe(0, lambda data, stamp: sys_reports.append(stamp))

        first = board.imu(timeout=2.0)
        print(f"imu first sample (SI): ax={first.ax:+.2f} ay={first.ay:+.2f} "
              f"az={first.az:+.2f} m/s^2  gx={first.gx:+.3f} gy={first.gy:+.3f} "
              f"gz={first.gz:+.3f} rad/s")

        print(f"sampling {DURATION_S:.0f} s ...")
        t0 = time.monotonic()
        time.sleep(DURATION_S)
        window = [t for t in imu_stamps if t >= t0]
        rate = len(window) / DURATION_S
        # worst 1 s bucket = "sustained"
        buckets = {}
        for t in window:
            buckets[int(t - t0)] = buckets.get(int(t - t0), 0) + 1
        worst = min(list(buckets.values())[:int(DURATION_S)]) if buckets else 0
        print(f"imu rate: {rate:.1f} Hz average, worst 1-s bucket {worst} Hz "
              f"({len(window)} samples/{DURATION_S:.0f}s)")
        batt_window = [t for t in sys_reports if t >= t0]
        print(f"battery report cadence: {len(batt_window)} reports in "
              f"{DURATION_S:.0f} s (~{len(batt_window)/DURATION_S:.2f} Hz)")
        print(f"decoder: ok={board._t.decoder.frames_ok} "
              f"crc_errors={board._t.decoder.crc_errors} "
              f"resyncs={board._t.decoder.resyncs}")

        # ---- harmless actuators (1.4)
        print("beep ...")
        board.beep(1500, 100, 100, 1)
        time.sleep(0.5)
        print("led blink ...")
        board.set_led(200, 200, 3)
        time.sleep(1.5)
        print("oled write 'HWR phase1' line 1 ...")
        board.oled_text(1, "HWR phase1")
        time.sleep(0.5)

        mv2 = board.battery_mv(timeout=3.0)
        print(f"battery end: {mv2} mV")
        verdict = "PASS" if (rate >= 40 and worst >= 40) else "FAIL"
        print(f"IMU-rate acceptance: {verdict}")
        return 0 if verdict == "PASS" else 1
    finally:
        board.stop_all_motors()  # belt and braces; nothing was driven
        board.close()


if __name__ == "__main__":
    sys.exit(main())
