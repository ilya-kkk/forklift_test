import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"


KEY_BINDINGS = {
    "w": (1.0, 0.0),
    "W": (1.0, 0.0),
    "\x1b[A": (1.0, 0.0),
    "a": (1.0, 1.0),
    "A": (1.0, 1.0),
    "\x1b[D": (1.0, 1.0),
    "d": (1.0, -1.0),
    "D": (1.0, -1.0),
    "\x1b[C": (1.0, -1.0),
    "s": (-1.0, 0.0),
    "S": (-1.0, 0.0),
    "\x1b[B": (-1.0, 0.0),
    "z": (-1.0, 1.0),
    "Z": (-1.0, 1.0),
    "c": (-1.0, -1.0),
    "C": (-1.0, -1.0),
    "x": (0.0, 0.0),
    "X": (0.0, 0.0),
    " ": (0.0, 0.0),
    "k": (0.0, 0.0),
    "K": (0.0, 0.0),
}


HELP_TEXT = """Keyboard teleop

Controls:
  w / Up Arrow      forward
  a / Left Arrow    forward left
  d / Right Arrow   forward right
  s / Down Arrow    reverse
  z                 reverse left
  c                 reverse right
  r                 raise forks
  f                 lower forks
  x or Space or k   stop
  q                 quit
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("fork_lift_topic", "/forklift/fork_lift_cmd")
        self.declare_parameter("linear_speed", 0.6)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("motion_mode_switch_threshold", 0.05)
        self.declare_parameter("fork_lift_min", 0.05)
        self.declare_parameter("fork_lift_max", 0.26)
        self.declare_parameter("fork_lift_step", 0.05)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        motion_mode_topic = str(self.get_parameter("motion_mode_topic").value)
        fork_lift_topic = str(self.get_parameter("fork_lift_topic").value)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self._motion_mode_switch_threshold = float(
            self.get_parameter("motion_mode_switch_threshold").value
        )
        self._fork_lift_min = float(self.get_parameter("fork_lift_min").value)
        self._fork_lift_max = float(self.get_parameter("fork_lift_max").value)
        self._fork_lift_step = float(self.get_parameter("fork_lift_step").value)

        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._fork_lift_publisher = self.create_publisher(Float64, fork_lift_topic, 10)
        motion_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._motion_mode_publisher = self.create_publisher(
            String, motion_mode_topic, motion_mode_qos
        )

        self._current_linear = 0.0
        self._current_angular = 0.0
        self._current_motion_mode = BODY_FIRST
        self._current_fork_lift = self._fork_lift_min
        self.create_timer(1.0 / publish_rate, self._publish_current_command)

        self.get_logger().info(
            "Keyboard teleop ready: cmd_vel=%s motion_mode=%s fork_lift=%s"
            % (cmd_vel_topic, motion_mode_topic, fork_lift_topic)
        )

    def set_command(self, linear_scale: float, angular_scale: float) -> None:
        self._current_linear = linear_scale * self._linear_speed
        self._current_angular = angular_scale * self._angular_speed
        self._update_motion_mode(self._current_linear)

    def stop(self) -> None:
        self._current_linear = 0.0
        self._current_angular = 0.0

    def adjust_fork_lift(self, delta: float) -> None:
        self._current_fork_lift = min(
            self._fork_lift_max,
            max(self._fork_lift_min, self._current_fork_lift + delta),
        )

    def _update_motion_mode(self, linear_velocity: float) -> None:
        if abs(linear_velocity) < self._motion_mode_switch_threshold:
            return

        desired_mode = BODY_FIRST if linear_velocity >= 0.0 else FORKS_FIRST
        if desired_mode == self._current_motion_mode:
            return

        message = String()
        message.data = desired_mode
        self._motion_mode_publisher.publish(message)
        self._current_motion_mode = desired_mode

    def _publish_current_command(self) -> None:
        message = Twist()
        message.linear.x = self._current_linear
        message.angular.z = self._current_angular
        self._cmd_vel_publisher.publish(message)

        fork_lift_message = Float64()
        fork_lift_message.data = self._current_fork_lift
        self._fork_lift_publisher.publish(fork_lift_message)


def read_key(timeout_sec: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return ""

    key = sys.stdin.read(1)
    if key != "\x1b":
        return key

    sequence = key
    for _ in range(2):
        ready, _, _ = select.select([sys.stdin], [], [], 0.001)
        if not ready:
            break
        sequence += sys.stdin.read(1)
    return sequence


def main(args=None) -> None:
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(HELP_TEXT)
    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = read_key(0.1)
            if key in ("q", "Q", "\x03"):
                break

            if key in ("r", "R"):
                node.adjust_fork_lift(node._fork_lift_step)
            elif key in ("f", "F"):
                node.adjust_fork_lift(-node._fork_lift_step)

            if key in KEY_BINDINGS:
                linear_scale, angular_scale = KEY_BINDINGS[key]
                node.set_command(linear_scale, angular_scale)

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.0)
            node._publish_current_command()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
