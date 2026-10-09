"""hwr_teleop — keyboard teleop over SSH (raw terminal).

Keys (hold to drive; releasing = keys stop repeating = robot stops):
    w        forward            (NO reverse key: the robot is rear-blind)
    a / d    strafe left/right
    q / e    rotate CCW/CW
    [ / ]    speed down/up
    space    immediate stop
    x        ESTOP (latches via hwr_driver service; X again resets)
    ctrl-c   quit (stops first)

Publishes cmd_vel_teleop at 10 Hz; hwr_base's staleness timeout plus the driver
watchdog back this up if the terminal dies.
"""
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

KEY_HOLD_S = 0.35  # a key older than this counts as released
SPEEDS = [(0.05, 0.2), (0.1, 0.3), (0.15, 0.4), (0.2, 0.5)]  # (m/s, rad/s)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_teleop")
        self.pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)
        self.estop_client = self.create_client(SetBool, "estop")
        self.speed_idx = 1
        self.estopped = False
        self._pressed: dict[str, float] = {}
        self._lock = threading.Lock()
        self.create_timer(0.1, self._tick)

    def key_event(self, key: str) -> None:
        now = time.monotonic()
        with self._lock:
            if key == " ":
                self._pressed.clear()
            elif key == "[":
                self.speed_idx = max(0, self.speed_idx - 1)
                self._print_status()
            elif key == "]":
                self.speed_idx = min(len(SPEEDS) - 1, self.speed_idx + 1)
                self._print_status()
            elif key in ("x", "X"):
                self._pressed.clear()
                self.estopped = not self.estopped
                self._call_estop(self.estopped)
            elif key in "wadqe":
                self._pressed[key] = now

    def _call_estop(self, trip: bool) -> None:
        req = SetBool.Request()
        req.data = trip
        self.estop_client.call_async(req)
        print(f"\r{'ESTOP TRIPPED' if trip else 'estop reset':<40}", end="", flush=True)

    def _print_status(self) -> None:
        v, w = SPEEDS[self.speed_idx]
        print(f"\rspeed {self.speed_idx + 1}/{len(SPEEDS)}: {v} m/s, {w} rad/s"
              f"{' [ESTOP]' if self.estopped else '':<10}", end="", flush=True)

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            held = {k for k, t in self._pressed.items() if now - t < KEY_HOLD_S}
        v, w = SPEEDS[self.speed_idx]
        msg = Twist()
        if "w" in held:
            msg.linear.x = v
        if "a" in held:
            msg.linear.y = v
        if "d" in held:
            msg.linear.y = -v
        if "q" in held:
            msg.angular.z = w
        if "e" in held:
            msg.angular.z = -w
        self.pub.publish(msg)

    def send_stop(self) -> None:
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    print(__doc__)
    node._print_status()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1)
                if key == "\x03":  # ctrl-c
                    break
                node.key_event(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.send_stop()
        time.sleep(0.2)
        rclpy.try_shutdown()
        print("\nteleop ended, stop sent")


if __name__ == "__main__":
    main()
