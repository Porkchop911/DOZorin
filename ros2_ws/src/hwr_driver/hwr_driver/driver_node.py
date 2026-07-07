"""hwr_driver — the one process that owns the RRC board.

Pubs:  imu (sensor_msgs/Imu, param rate), battery (sensor_msgs/BatteryState ~1 Hz),
       joint_states (arm read-back, param rate), button (std_msgs/String)
Subs:  wheel_speeds (hwr_msgs/WheelSpeeds), servo_cmd (hwr_msgs/ServoCommand),
       beep (std_msgs/UInt16, value = freq Hz)
Srv:   estop (std_srvs/SetBool: data=true trip, false reset)

Safety (hwr_core primitives, unit-tested): watchdog auto-zeros motors 0.4 s
after the last nonzero wheel_speeds; battery gate refuses motion <11.0 V and
hard-stops + latches estop <10.5 V; SIGINT/shutdown always stops motors.
Jazzy-ready: plain rclpy, no distro-specific APIs.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from hwr_msgs.msg import ServoCommand, WheelSpeeds
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String, UInt16
from std_srvs.srv import SetBool

from hwr_core.rrc import Board, codecs
from hwr_core.safety import BatteryGate, EStop, Watchdog

ARM_JOINTS = {1: "joint1", 2: "joint2", 3: "joint3", 4: "joint4", 5: "joint5"}
PULSE_CENTER = 500
RAD_PER_PULSE = math.radians(240.0 / 1000.0)


class DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_driver")
        self.declare_parameter("device", "/dev/rrc")
        self.declare_parameter("imu_rate", 50.0)
        self.declare_parameter("joint_state_rate", 5.0)
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("watchdog_timeout", 0.4)

        device = self.get_parameter("device").value
        self.board = Board.open(device)
        self.board.set_motor_type(codecs.MOTOR_TYPE_JGB37)
        self.get_logger().info(f"board open on {device}")

        self.estop = EStop()
        self.gate = BatteryGate(on_hard_stop=self._battery_hard_stop)
        self.watchdog = Watchdog(
            self._watchdog_stop,
            timeout=float(self.get_parameter("watchdog_timeout").value))

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub_imu = self.create_publisher(Imu, "imu", sensor_qos)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 10)
        self.pub_joints = self.create_publisher(JointState, "joint_states", 10)
        self.pub_button = self.create_publisher(String, "button", 10)

        self.create_subscription(WheelSpeeds, "wheel_speeds", self._on_wheel_speeds, 10)
        self.create_subscription(ServoCommand, "servo_cmd", self._on_servo_cmd, 10)
        self.create_subscription(UInt16, "beep", self._on_beep, 10)
        self.create_service(SetBool, "estop", self._on_estop_srv)

        self._latest_imu = None  # (ImuSample, stamp) cached from the RX thread
        self.board.subscribe_imu(lambda s, t: setattr(self, "_latest_imu", (s, t)))
        self.board.subscribe_battery(self._on_battery)
        self.board.subscribe_key(self._on_key)

        imu_period = 1.0 / float(self.get_parameter("imu_rate").value)
        self.create_timer(imu_period, self._publish_imu)
        js_rate = float(self.get_parameter("joint_state_rate").value)
        if js_rate > 0:
            self.create_timer(1.0 / js_rate, self._publish_joint_states)

    # ------------------------------------------------------------- safety paths

    def _watchdog_stop(self) -> None:
        self.board.stop_all_motors()
        self.get_logger().warning("watchdog: wheel_speeds source went silent -> motors stopped")

    def _battery_hard_stop(self) -> None:
        self.estop.trip("battery hard stop")
        self.watchdog.disarm()
        self.board.stop_all_motors()
        self.board.stop_servos(list(ARM_JOINTS) + [10])
        self.get_logger().error("battery below hard-stop threshold: estop latched")

    # ------------------------------------------------------------- subscriptions

    def _on_wheel_speeds(self, msg: WheelSpeeds) -> None:
        if self.estop.tripped:
            self.board.stop_all_motors()
            return
        if not self.gate.allow_motion():
            self.board.stop_all_motors()
            self.get_logger().warning(
                f"battery gate: motion refused ({self.gate.latest_mv} mV)",
                throttle_duration_sec=5.0)
            return
        rps = list(msg.rps)
        self.board.set_motor_rps(list(zip((1, 2, 3, 4), rps)))
        if any(r != 0.0 for r in rps):
            self.watchdog.feed()
        else:
            self.watchdog.disarm()

    def _on_servo_cmd(self, msg: ServoCommand) -> None:
        if self.estop.tripped:
            return
        positions = list(zip(msg.ids, msg.positions))
        if positions:
            self.board.set_servo_positions(int(msg.duration_ms), positions)

    def _on_beep(self, msg: UInt16) -> None:
        self.board.beep(int(msg.data) or 1500, 100, 100, 1)

    def _on_estop_srv(self, request, response):
        if request.data:
            self.estop.trip("service")
            self.watchdog.disarm()
            self.board.stop_all_motors()
            self.board.stop_servos(list(ARM_JOINTS) + [10])
            response.message = "estop tripped"
        else:
            self.estop.reset()
            response.message = "estop reset"
        self.get_logger().warning(response.message)
        response.success = True
        return response

    # ------------------------------------------------------------- report relays

    def _on_battery(self, mv: int, stamp: float) -> None:
        self.gate.update(mv)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = mv / 1000.0
        msg.percentage = float("nan")
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.pub_batt.publish(msg)

    def _on_key(self, key_id: int, event: str, stamp: float) -> None:
        msg = String()
        msg.data = f"key{key_id}:{event}"
        self.pub_button.publish(msg)

    # ------------------------------------------------------------- timers

    def _publish_imu(self) -> None:
        if self._latest_imu is None:
            return
        sample, _ = self._latest_imu
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("imu_frame").value
        msg.linear_acceleration.x = sample.ax
        msg.linear_acceleration.y = sample.ay
        msg.linear_acceleration.z = sample.az
        msg.angular_velocity.x = sample.gx
        msg.angular_velocity.y = sample.gy
        msg.angular_velocity.z = sample.gz
        msg.orientation_covariance[0] = -1.0  # no orientation estimate
        self.pub_imu.publish(msg)

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for sid, name in ARM_JOINTS.items():
            pulse = self.board.servo_position(sid, timeout=0.2)
            if pulse is None:
                continue
            msg.name.append(name)
            msg.position.append((pulse - PULSE_CENTER) * RAD_PER_PULSE)
        if msg.name:
            self.pub_joints.publish(msg)

    # ------------------------------------------------------------- lifecycle

    def shutdown(self) -> None:
        self.watchdog.close()
        self.board.stop_all_motors()
        self.board.close()


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
