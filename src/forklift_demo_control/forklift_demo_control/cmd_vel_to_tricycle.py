import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"
VALID_MOTION_MODES = {BODY_FIRST, FORKS_FIRST}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


class CmdVelToTricycle(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_tricycle")

        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("steering_cmd_topic", "/forklift/rear_steering_cmd")
        self.declare_parameter("drive_cmd_topic", "/forklift/rear_wheel_cmd")
        self.declare_parameter("gz_cmd_vel_topic", "/model/forklift_demo/cmd_vel")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("motion_mode", BODY_FIRST)
        self.declare_parameter("wheelbase", 1.2)
        self.declare_parameter("wheel_radius", 0.18)
        self.declare_parameter("max_steering_angle", 1.57079632679)
        self.declare_parameter("max_steering_rate", 1.4)
        self.declare_parameter("max_wheel_angular_velocity", 5.0)
        self.declare_parameter("reverse_velocity_scale", 0.5)
        self.declare_parameter("deadband_linear_velocity", 0.02)
        self.declare_parameter("steering_hold_when_stopped", True)
        self.declare_parameter("debug_logging", True)
        self.declare_parameter("debug_log_period_sec", 1.0)

        input_cmd_topic = self.get_parameter("input_cmd_topic").value
        steering_cmd_topic = self.get_parameter("steering_cmd_topic").value
        drive_cmd_topic = self.get_parameter("drive_cmd_topic").value
        gz_cmd_vel_topic = self.get_parameter("gz_cmd_vel_topic").value
        motion_mode_topic = self.get_parameter("motion_mode_topic").value

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
        self._reverse_velocity_scale = float(
            self.get_parameter("reverse_velocity_scale").value
        )
        self._deadband_linear_velocity = float(
            self.get_parameter("deadband_linear_velocity").value
        )
        self._steering_hold_when_stopped = bool(
            self.get_parameter("steering_hold_when_stopped").value
        )
        self._debug_logging = bool(self.get_parameter("debug_logging").value)
        self._debug_log_period = float(
            self.get_parameter("debug_log_period_sec").value
        )
        self._motion_mode = self._normalize_motion_mode(
            str(self.get_parameter("motion_mode").value)
        )

        self._current_steering = 0.0
        self._last_update_time = self.get_clock().now()
        self._last_debug_log_time = self.get_clock().now()
        self._last_saturation_log_time = self.get_clock().now()

        self._steering_publisher = self.create_publisher(Float64, steering_cmd_topic, 10)
        self._drive_publisher = self.create_publisher(Float64, drive_cmd_topic, 10)
        self._gazebo_cmd_publisher = self.create_publisher(Twist, gz_cmd_vel_topic, 10)
        motion_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Twist, input_cmd_topic, self._on_cmd_vel, 10)
        self.create_subscription(
            String,
            motion_mode_topic,
            self._on_motion_mode,
            motion_mode_qos,
        )

        self.get_logger().info(
            "cmd_vel_to_tricycle ready: input=%s steering=%s drive=%s gazebo=%s motion_mode=%s"
            % (
                input_cmd_topic,
                steering_cmd_topic,
                drive_cmd_topic,
                gz_cmd_vel_topic,
                self._motion_mode,
            )
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        now = self.get_clock().now()
        delta_time = max(
            0.0, (now - self._last_update_time).nanoseconds / 1_000_000_000.0
        )
        self._last_update_time = now

        input_linear_velocity = float(message.linear.x)
        input_angular_velocity = float(message.angular.z)
        linear_velocity = self._apply_motion_mode(input_linear_velocity)
        angular_velocity = self._adapt_angular_velocity(
            input_linear_velocity,
            linear_velocity,
            input_angular_velocity,
        )

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
        self._log_debug_state(
            now=now,
            input_linear_velocity=input_linear_velocity,
            input_angular_velocity=input_angular_velocity,
            angular_velocity=angular_velocity,
            target_steering=target_steering,
            steering_command=steering_command,
            wheel_velocity=wheel_velocity,
            gazebo_angular_velocity=gazebo_twist.angular.z,
            linear_velocity=linear_velocity,
        )

    def _on_motion_mode(self, message: String) -> None:
        next_mode = self._normalize_motion_mode(message.data)
        if next_mode == self._motion_mode:
            return

        self._motion_mode = next_mode
        self.get_logger().info("Motion mode switched to %s" % self._motion_mode)

    def _normalize_motion_mode(self, motion_mode: str) -> str:
        normalized = motion_mode.strip().upper()
        if normalized in VALID_MOTION_MODES:
            return normalized

        self.get_logger().warning(
            "Unsupported motion mode '%s', falling back to %s"
            % (motion_mode, BODY_FIRST)
        )
        return BODY_FIRST

    def _apply_motion_mode(self, linear_velocity: float) -> float:
        max_linear_velocity = self._max_wheel_angular_velocity * self._wheel_radius
        speed = min(abs(linear_velocity), max_linear_velocity)

        if self._motion_mode == FORKS_FIRST:
            reverse_limit = max_linear_velocity * max(0.0, self._reverse_velocity_scale)
            return -min(speed, reverse_limit)

        return speed

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

    def _adapt_angular_velocity(
        self,
        input_linear_velocity: float,
        output_linear_velocity: float,
        angular_velocity: float,
    ) -> float:
        # Preserve the curvature commanded by Nav2 after forcing motion mode
        # and reverse speed scaling at the low level.
        if (
            abs(input_linear_velocity) <= self._deadband_linear_velocity
            or abs(output_linear_velocity) <= self._deadband_linear_velocity
        ):
            return 0.0

        return angular_velocity * (output_linear_velocity / input_linear_velocity)

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

    def _log_debug_state(
        self,
        *,
        now,
        input_linear_velocity: float,
        input_angular_velocity: float,
        angular_velocity: float,
        target_steering: float,
        steering_command: float,
        wheel_velocity: float,
        gazebo_angular_velocity: float,
        linear_velocity: float,
    ) -> None:
        if self._debug_logging and (
            now - self._last_debug_log_time
        ).nanoseconds >= int(self._debug_log_period * 1_000_000_000.0):
            self.get_logger().info(
                "debug cmd_vel mode=%s in_v=%.3f out_v=%.3f in_omega=%.3f out_omega=%.3f "
                "target_steer_deg=%.1f steer_deg=%.1f wheel=%.3f gz_omega=%.3f"
                % (
                    self._motion_mode,
                    input_linear_velocity,
                    linear_velocity,
                    input_angular_velocity,
                    angular_velocity,
                    math.degrees(target_steering),
                    math.degrees(steering_command),
                    wheel_velocity,
                    gazebo_angular_velocity,
                )
            )
            self._last_debug_log_time = now

        steering_limit_margin = 1e-3
        saturated = (
            abs(target_steering) >= self._max_steering_angle - steering_limit_margin
            or abs(steering_command) >= self._max_steering_angle - steering_limit_margin
        )
        if saturated and (
            now - self._last_saturation_log_time
        ).nanoseconds >= 1_000_000_000:
            self.get_logger().warning(
                "steering saturation mode=%s in_v=%.3f out_v=%.3f in_omega=%.3f out_omega=%.3f "
                "target_steer_deg=%.1f steer_deg=%.1f limit_deg=%.1f"
                % (
                    self._motion_mode,
                    input_linear_velocity,
                    linear_velocity,
                    input_angular_velocity,
                    angular_velocity,
                    math.degrees(target_steering),
                    math.degrees(steering_command),
                    math.degrees(self._max_steering_angle),
                )
            )
            self._last_saturation_log_time = now


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
