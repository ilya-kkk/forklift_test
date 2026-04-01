import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


class CmdVelToTricycle(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_tricycle")

        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("steering_cmd_topic", "/forklift/rear_steering_cmd")
        self.declare_parameter("drive_cmd_topic", "/forklift/rear_wheel_cmd")
        self.declare_parameter("gz_cmd_vel_topic", "/model/forklift_demo/cmd_vel")
        self.declare_parameter("wheelbase", 1.2)
        self.declare_parameter("wheel_radius", 0.18)
        self.declare_parameter("max_steering_angle", 1.48353)
        self.declare_parameter("max_steering_rate", 1.4)
        self.declare_parameter("max_wheel_angular_velocity", 5.0)
        self.declare_parameter("deadband_linear_velocity", 0.02)
        self.declare_parameter("steering_hold_when_stopped", True)

        input_cmd_topic = self.get_parameter("input_cmd_topic").value
        steering_cmd_topic = self.get_parameter("steering_cmd_topic").value
        drive_cmd_topic = self.get_parameter("drive_cmd_topic").value
        gz_cmd_vel_topic = self.get_parameter("gz_cmd_vel_topic").value

        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._max_steering_angle = float(
            self.get_parameter("max_steering_angle").value
        )
        self._max_steering_rate = float(
            self.get_parameter("max_steering_rate").value
        )
        self._max_wheel_angular_velocity = float(
            self.get_parameter("max_wheel_angular_velocity").value
        )
        self._deadband_linear_velocity = float(
            self.get_parameter("deadband_linear_velocity").value
        )
        self._steering_hold_when_stopped = bool(
            self.get_parameter("steering_hold_when_stopped").value
        )

        self._current_steering = 0.0
        self._last_update_time = self.get_clock().now()

        self._steering_publisher = self.create_publisher(Float64, steering_cmd_topic, 10)
        self._drive_publisher = self.create_publisher(Float64, drive_cmd_topic, 10)
        self._gazebo_cmd_publisher = self.create_publisher(Twist, gz_cmd_vel_topic, 10)
        self.create_subscription(Twist, input_cmd_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            "cmd_vel_to_tricycle ready: input=%s steering=%s drive=%s gazebo=%s"
            % (input_cmd_topic, steering_cmd_topic, drive_cmd_topic, gz_cmd_vel_topic)
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        now = self.get_clock().now()
        delta_time = max(
            0.0, (now - self._last_update_time).nanoseconds / 1_000_000_000.0
        )
        self._last_update_time = now

        linear_velocity = clamp(
            float(message.linear.x),
            -self._max_wheel_angular_velocity * self._wheel_radius,
            self._max_wheel_angular_velocity * self._wheel_radius,
        )
        angular_velocity = float(message.angular.z)

        target_steering = self._compute_target_steering(
            linear_velocity, angular_velocity
        )
        steering_command = self._rate_limit_steering(target_steering, delta_time)
        wheel_velocity = clamp(
            linear_velocity / self._wheel_radius,
            -self._max_wheel_angular_velocity,
            self._max_wheel_angular_velocity,
        )

        gazebo_twist = Twist()
        gazebo_twist.linear.x = linear_velocity
        if abs(linear_velocity) > self._deadband_linear_velocity:
            # Rear-wheel steering uses the opposite yaw sign from the standard
            # front-steered bicycle model.
            gazebo_twist.angular.z = (
                -math.tan(steering_command) * linear_velocity / self._wheelbase
            )
        else:
            gazebo_twist.angular.z = 0.0

        steering_msg = Float64()
        steering_msg.data = steering_command
        drive_msg = Float64()
        drive_msg.data = wheel_velocity

        self._steering_publisher.publish(steering_msg)
        self._drive_publisher.publish(drive_msg)
        self._gazebo_cmd_publisher.publish(gazebo_twist)

    def _compute_target_steering(
        self, linear_velocity: float, angular_velocity: float
    ) -> float:
        if abs(linear_velocity) <= self._deadband_linear_velocity:
            if self._steering_hold_when_stopped:
                return self._current_steering
            return 0.0

        # Use atan on the curvature ratio so reverse motion does not wrap the
        # steering target toward pi and immediately saturate the joint limit.
        target = math.atan(-self._wheelbase * angular_velocity / linear_velocity)
        return clamp(target, -self._max_steering_angle, self._max_steering_angle)

    def _rate_limit_steering(self, target: float, delta_time: float) -> float:
        if self._max_steering_rate <= 0.0 or delta_time <= 0.0:
            self._current_steering = target
            return target

        max_delta = self._max_steering_rate * delta_time
        delta = clamp(
            target - self._current_steering,
            -max_delta,
            max_delta,
        )
        self._current_steering = clamp(
            self._current_steering + delta,
            -self._max_steering_angle,
            self._max_steering_angle,
        )
        return self._current_steering


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToTricycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
