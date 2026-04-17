import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class ForkPositionController(Node):
    def __init__(self) -> None:
        super().__init__("fork_position_controller")

        self.declare_parameter("joint_name", "fork_joint")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("position_cmd_topic", "/forklift/fork_cmd")
        self.declare_parameter("velocity_cmd_topic", "/forklift/fork_velocity_cmd")
        self.declare_parameter("position_lower_limit", 0.0)
        self.declare_parameter("position_upper_limit", 1.0)
        self.declare_parameter("invert_position_command", True)
        self.declare_parameter("max_velocity", 0.35)
        self.declare_parameter("p_gain", 2.5)
        self.declare_parameter("deadband", 0.002)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._joint_name = str(self.get_parameter("joint_name").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._position_cmd_topic = str(self.get_parameter("position_cmd_topic").value)
        self._velocity_cmd_topic = str(self.get_parameter("velocity_cmd_topic").value)
        self._position_lower_limit = float(
            self.get_parameter("position_lower_limit").value
        )
        self._position_upper_limit = float(
            self.get_parameter("position_upper_limit").value
        )
        self._invert_position_command = bool(
            self.get_parameter("invert_position_command").value
        )
        self._max_velocity = float(self.get_parameter("max_velocity").value)
        self._p_gain = float(self.get_parameter("p_gain").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )

        self._current_position: Optional[float] = None
        self._desired_position: Optional[float] = None
        self._last_velocity_command: Optional[float] = None
        self._missing_joint_warned = False

        self._velocity_publisher = self.create_publisher(
            Float64, self._velocity_cmd_topic, 10
        )
        self.create_subscription(
            JointState, self._joint_states_topic, self._joint_state_callback, 10
        )
        self.create_subscription(
            Float64, self._position_cmd_topic, self._position_cmd_callback, 10
        )
        self.create_timer(1.0 / self._publish_rate_hz, self._control_loop)

        self.get_logger().info(
            "fork_position_controller ready: joint=%s position_cmd=%s velocity_cmd=%s "
            "limits=[%.3f, %.3f] max_velocity=%.3f invert_command=%s"
            % (
                self._joint_name,
                self._position_cmd_topic,
                self._velocity_cmd_topic,
                self._position_lower_limit,
                self._position_upper_limit,
                self._max_velocity,
                self._invert_position_command,
            )
        )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            joint_index = message.name.index(self._joint_name)
        except ValueError:
            if not self._missing_joint_warned:
                self.get_logger().warn(
                    "Joint '%s' is not present in %s."
                    % (self._joint_name, self._joint_states_topic)
                )
                self._missing_joint_warned = True
            return

        if joint_index >= len(message.position):
            return

        self._missing_joint_warned = False
        self._current_position = float(message.position[joint_index])
        if self._desired_position is None:
            self._desired_position = self._clamp_position(self._current_position)

    def _position_cmd_callback(self, message: Float64) -> None:
        command_position = self._clamp_position(float(message.data))
        if self._invert_position_command:
            command_position = (
                self._position_upper_limit
                - command_position
                + self._position_lower_limit
            )
        self._desired_position = command_position

    def _control_loop(self) -> None:
        if self._current_position is None or self._desired_position is None:
            return

        error = self._desired_position - self._current_position
        if math.fabs(error) <= self._deadband:
            velocity_command = 0.0
        else:
            velocity_command = max(
                -self._max_velocity,
                min(self._max_velocity, self._p_gain * error),
            )

        if (
            self._last_velocity_command is not None
            and math.fabs(velocity_command - self._last_velocity_command) < 1e-4
        ):
            return

        velocity_message = Float64()
        velocity_message.data = velocity_command
        self._velocity_publisher.publish(velocity_message)
        self._last_velocity_command = velocity_command

    def _clamp_position(self, position: float) -> float:
        return max(self._position_lower_limit, min(self._position_upper_limit, position))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkPositionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
