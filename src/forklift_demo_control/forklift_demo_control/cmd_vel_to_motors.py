import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"
VALID_MOTION_MODES = {BODY_FIRST, FORKS_FIRST}


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class KinematicModel:
    base_frame: str
    cmd_vel_frame: str
    steering_joint_name: str
    drive_wheel_joint_name: str
    cmd_vel_frame_pose_xy_yaw: Tuple[float, float, float]
    steering_position_xy: Tuple[float, float]
    steering_limits: Tuple[float, float]
    wheel_radius: float
    wheel_velocity_limit: Optional[float]


class CmdVelToMotors(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_motors")

        self.declare_parameter("enabled", True)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("steering_cmd_topic", "/forklift/right_steering_cmd")
        self.declare_parameter("wheel_cmd_topic", "/forklift/right_wheel_cmd")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("motion_mode", BODY_FIRST)
        self.declare_parameter("reverse_velocity_scale", 0.5)
        self.declare_parameter("motion_mode_linear_deadband", 0.02)
        self.declare_parameter("cmd_vel_frame", "base_link")
        self.declare_parameter("steering_joint_name", "right_steering_joint")
        self.declare_parameter("drive_wheel_joint_name", "right_wheel_joint")
        self.declare_parameter("drive_wheel_radius", 0.0)
        self.declare_parameter("drive_wheel_velocity_sign", 1.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("motion_epsilon", 1e-4)
        self.declare_parameter("center_steering_on_stop", False)

        self._enabled = self._parse_bool_parameter(
            self.get_parameter("enabled").value
        )
        robot_description = str(self.get_parameter("robot_description").value)
        steering_joint_name = str(self.get_parameter("steering_joint_name").value)
        drive_wheel_joint_name = str(
            self.get_parameter("drive_wheel_joint_name").value
        )
        drive_wheel_radius = max(
            0.0, float(self.get_parameter("drive_wheel_radius").value)
        )
        drive_wheel_velocity_sign = float(
            self.get_parameter("drive_wheel_velocity_sign").value
        )
        self._drive_wheel_velocity_sign = (
            -1.0 if drive_wheel_velocity_sign < 0.0 else 1.0
        )
        cmd_vel_frame = str(self.get_parameter("cmd_vel_frame").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        steering_cmd_topic = str(self.get_parameter("steering_cmd_topic").value)
        wheel_cmd_topic = str(self.get_parameter("wheel_cmd_topic").value)
        motion_mode_topic = str(self.get_parameter("motion_mode_topic").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self._reverse_velocity_scale = max(
            0.0, float(self.get_parameter("reverse_velocity_scale").value)
        )
        self._motion_mode_linear_deadband = max(
            1e-6, float(self.get_parameter("motion_mode_linear_deadband").value)
        )
        self._command_timeout_sec = max(
            0.0, float(self.get_parameter("command_timeout_sec").value)
        )
        self._motion_epsilon = max(
            1e-6, float(self.get_parameter("motion_epsilon").value)
        )
        self._center_steering_on_stop = bool(
            self.get_parameter("center_steering_on_stop").value
        )
        self._motion_mode = self._normalize_motion_mode(
            str(self.get_parameter("motion_mode").value)
        )

        self._kinematic_model = self._build_kinematic_model(
            robot_description=robot_description,
            cmd_vel_frame=cmd_vel_frame,
            steering_joint_name=steering_joint_name,
            drive_wheel_joint_name=drive_wheel_joint_name,
            drive_wheel_radius_override=drive_wheel_radius,
        )

        self._latest_cmd = Twist()
        self._last_cmd_time = None
        self._last_steering_angle = 0.0

        self._steering_publisher = self.create_publisher(
            Float64, steering_cmd_topic, 10
        )
        self._wheel_publisher = self.create_publisher(Float64, wheel_cmd_topic, 10)
        motion_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 10
        )
        self._motion_mode_subscription = self.create_subscription(
            String, motion_mode_topic, self._motion_mode_callback, motion_mode_qos
        )
        self.create_timer(1.0 / publish_rate, self._publish_motor_commands)

        steering_x, steering_y = self._kinematic_model.steering_position_xy
        cmd_frame_x, cmd_frame_y, cmd_frame_yaw = (
            self._kinematic_model.cmd_vel_frame_pose_xy_yaw
        )
        wheel_velocity_limit = self._kinematic_model.wheel_velocity_limit
        wheel_velocity_limit_text = (
            f"{wheel_velocity_limit:.3f}"
            if wheel_velocity_limit is not None
            else "unbounded"
        )
        self.get_logger().info(
            "cmd_vel_to_motors ready: cmd_vel=%s steering_cmd=%s wheel_cmd=%s "
            "base=%s cmd_frame=%s cmd_frame_pose=(%.3f, %.3f, %.3f) "
            "steering_joint=%s drive_joint=%s steering_xy=(%.3f, %.3f) "
            "wheel_radius=%.3f steering_limits=[%.3f, %.3f] wheel_vel_limit=%s "
            "wheel_velocity_sign=%.1f motion_mode=%s reverse_velocity_scale=%.2f"
            % (
                cmd_vel_topic,
                steering_cmd_topic,
                wheel_cmd_topic,
                self._kinematic_model.base_frame,
                self._kinematic_model.cmd_vel_frame,
                cmd_frame_x,
                cmd_frame_y,
                cmd_frame_yaw,
                self._kinematic_model.steering_joint_name,
                self._kinematic_model.drive_wheel_joint_name,
                steering_x,
                steering_y,
                self._kinematic_model.wheel_radius,
                self._kinematic_model.steering_limits[0],
                self._kinematic_model.steering_limits[1],
                wheel_velocity_limit_text,
                self._drive_wheel_velocity_sign,
                self._motion_mode,
                self._reverse_velocity_scale,
            )
        )
        if not self._enabled:
            self.get_logger().warning(
                "cmd_vel_to_motors is disabled by parameter 'enabled:=false'; "
                "node will not publish wheel/steering commands."
            )

    def _cmd_vel_callback(self, message: Twist) -> None:
        self._latest_cmd = message
        self._last_cmd_time = self.get_clock().now()

    def _motion_mode_callback(self, message: String) -> None:
        next_mode = self._normalize_motion_mode(message.data)
        if next_mode == self._motion_mode:
            return

        self._motion_mode = next_mode
        self.get_logger().info("Motion mode switched to %s" % self._motion_mode)

    def _publish_motor_commands(self) -> None:
        if not self._enabled:
            return

        if self._command_is_stale():
            steering_angle = (
                0.0
                if self._center_steering_on_stop
                else self._last_steering_angle
            )
            wheel_velocity = 0.0
        else:
            steering_angle, wheel_velocity = self._twist_to_motor_command(
                self._latest_cmd
            )

        self._last_steering_angle = steering_angle
        self._publish_float64(self._steering_publisher, steering_angle)
        self._publish_float64(self._wheel_publisher, wheel_velocity)

    def _command_is_stale(self) -> bool:
        if self._last_cmd_time is None:
            return True

        if self._command_timeout_sec <= 0.0:
            return False

        age_sec = (
            self.get_clock().now() - self._last_cmd_time
        ).nanoseconds / 1_000_000_000.0
        return age_sec > self._command_timeout_sec

    def _twist_to_motor_command(self, message: Twist) -> Tuple[float, float]:
        adjusted_message = self._apply_motion_mode(message)
        steering_x, steering_y = self._kinematic_model.steering_position_xy
        cmd_frame_x, cmd_frame_y, cmd_frame_yaw = (
            self._kinematic_model.cmd_vel_frame_pose_xy_yaw
        )
        relative_x = steering_x - cmd_frame_x
        relative_y = steering_y - cmd_frame_y
        linear_vx = (
            math.cos(cmd_frame_yaw) * adjusted_message.linear.x
            - math.sin(cmd_frame_yaw) * adjusted_message.linear.y
        )
        linear_vy = (
            math.sin(cmd_frame_yaw) * adjusted_message.linear.x
            + math.cos(cmd_frame_yaw) * adjusted_message.linear.y
        )
        wheel_point_vx = linear_vx + (-adjusted_message.angular.z * relative_y)
        wheel_point_vy = linear_vy + (adjusted_message.angular.z * relative_x)
        wheel_linear_speed = math.hypot(wheel_point_vx, wheel_point_vy)

        if wheel_linear_speed < self._motion_epsilon:
            steering_angle = (
                0.0
                if self._center_steering_on_stop
                else self._last_steering_angle
            )
            return steering_angle, 0.0

        raw_steering_angle = math.atan2(wheel_point_vy, wheel_point_vx)
        steering_angle, wheel_direction = self._select_steering_solution(
            raw_steering_angle
        )
        wheel_angular_velocity = (
            wheel_direction * wheel_linear_speed / self._kinematic_model.wheel_radius
        )
        wheel_angular_velocity *= self._drive_wheel_velocity_sign
        wheel_velocity_limit = self._kinematic_model.wheel_velocity_limit
        if wheel_velocity_limit is not None:
            wheel_angular_velocity = max(
                -wheel_velocity_limit,
                min(wheel_velocity_limit, wheel_angular_velocity),
            )

        return steering_angle, wheel_angular_velocity

    def _normalize_motion_mode(self, motion_mode: str) -> str:
        normalized = motion_mode.strip().upper()
        if normalized in VALID_MOTION_MODES:
            return normalized

        self.get_logger().warning(
            "Unsupported motion mode '%s', falling back to %s"
            % (motion_mode, BODY_FIRST)
        )
        return BODY_FIRST

    def _apply_motion_mode(self, message: Twist) -> Twist:
        adjusted = Twist()
        adjusted.linear.x = float(message.linear.x)
        adjusted.linear.y = float(message.linear.y)
        adjusted.linear.z = float(message.linear.z)
        adjusted.angular.x = float(message.angular.x)
        adjusted.angular.y = float(message.angular.y)
        adjusted.angular.z = float(message.angular.z)

        input_linear_x = adjusted.linear.x
        if abs(input_linear_x) <= self._motion_mode_linear_deadband:
            return adjusted

        max_linear_velocity = self._max_linear_velocity()
        speed = min(abs(input_linear_x), max_linear_velocity)
        preferred_direction_sign = -1.0 if self._motion_mode == FORKS_FIRST else 1.0
        is_reverse_command = input_linear_x < 0.0
        if is_reverse_command:
            speed = min(speed, max_linear_velocity * self._reverse_velocity_scale)

        desired_linear_x = preferred_direction_sign * math.copysign(
            speed, input_linear_x
        )

        scale = desired_linear_x / input_linear_x
        adjusted.linear.x *= scale
        adjusted.linear.y *= scale
        adjusted.linear.z *= scale
        adjusted.angular.x *= scale
        adjusted.angular.y *= scale
        adjusted.angular.z *= scale
        return adjusted

    def _max_linear_velocity(self) -> float:
        wheel_velocity_limit = self._kinematic_model.wheel_velocity_limit
        if wheel_velocity_limit is None:
            return float("inf")
        return wheel_velocity_limit * self._kinematic_model.wheel_radius

    def _select_steering_solution(self, raw_angle: float) -> Tuple[float, float]:
        lower_limit, upper_limit = self._kinematic_model.steering_limits
        candidates = []
        for shift, wheel_direction in (
            (0.0, 1.0),
            (math.pi, -1.0),
            (-math.pi, -1.0),
        ):
            candidate_angle = normalize_angle(raw_angle + shift)
            if lower_limit - 1e-9 <= candidate_angle <= upper_limit + 1e-9:
                candidates.append((candidate_angle, wheel_direction))

        if not candidates:
            clamped_angle = max(
                lower_limit, min(upper_limit, normalize_angle(raw_angle))
            )
            return clamped_angle, 1.0

        return min(
            candidates,
            key=lambda candidate: abs(candidate[0] - self._last_steering_angle),
        )

    def _build_kinematic_model(
        self,
        robot_description: str,
        cmd_vel_frame: str,
        steering_joint_name: str,
        drive_wheel_joint_name: str,
        drive_wheel_radius_override: float,
    ) -> KinematicModel:
        if not robot_description.strip():
            raise ValueError(
                "Parameter 'robot_description' is empty, URDF is required."
            )

        root = ET.fromstring(robot_description)
        joint_map = {
            joint.get("name"): joint
            for joint in root.findall("joint")
            if joint.get("name")
        }
        link_map = {
            link.get("name"): link for link in root.findall("link") if link.get("name")
        }

        steering_joint = self._require_joint(joint_map, steering_joint_name)
        steering_parent = self._required_link_name(steering_joint, "parent")
        steering_child = self._required_link_name(steering_joint, "child")
        steering_origin_xyz = self._parse_xyz(steering_joint.find("origin"))
        steering_limits = self._parse_joint_limits(steering_joint)
        cmd_vel_frame_pose = self._resolve_link_pose(
            joint_map=joint_map,
            source_link=steering_parent,
            target_link=cmd_vel_frame,
        )

        drive_wheel_joint = self._require_joint(joint_map, drive_wheel_joint_name)
        drive_parent = self._required_link_name(drive_wheel_joint, "parent")
        if drive_parent != steering_child:
            raise ValueError(
                "Drive wheel joint '%s' must be attached to steering link '%s', got '%s'."
                % (drive_wheel_joint_name, steering_child, drive_parent)
            )

        drive_joint_origin = self._parse_xyz(drive_wheel_joint.find("origin"))
        if any(abs(component) > 1e-6 for component in drive_joint_origin):
            self.get_logger().warn(
                "Drive wheel joint '%s' has non-zero origin %s; using steering joint "
                "origin for kinematics."
                % (drive_wheel_joint_name, drive_joint_origin)
            )

        wheel_velocity_limit = self._parse_joint_velocity_limit(drive_wheel_joint)
        drive_wheel_link_name = self._required_link_name(drive_wheel_joint, "child")
        drive_wheel_link = link_map.get(drive_wheel_link_name)
        if drive_wheel_link is None:
            raise ValueError(
                "Drive wheel link '%s' is missing in URDF." % drive_wheel_link_name
            )

        wheel_radius = (
            drive_wheel_radius_override
            if drive_wheel_radius_override > 0.0
            else self._find_cylinder_radius(drive_wheel_link)
        )
        if wheel_radius <= 0.0:
            raise ValueError("Drive wheel radius must be positive.")

        return KinematicModel(
            base_frame=steering_parent,
            cmd_vel_frame=cmd_vel_frame,
            steering_joint_name=steering_joint_name,
            drive_wheel_joint_name=drive_wheel_joint_name,
            cmd_vel_frame_pose_xy_yaw=cmd_vel_frame_pose,
            steering_position_xy=(steering_origin_xyz[0], steering_origin_xyz[1]),
            steering_limits=steering_limits,
            wheel_radius=wheel_radius,
            wheel_velocity_limit=wheel_velocity_limit,
        )

    def _require_joint(self, joint_map, joint_name: str):
        joint = joint_map.get(joint_name)
        if joint is None:
            raise ValueError("Joint '%s' is missing in URDF." % joint_name)
        return joint

    def _required_link_name(self, joint_element: ET.Element, tag_name: str) -> str:
        link_element = joint_element.find(tag_name)
        if link_element is None:
            raise ValueError(
                "Joint '%s' is missing <%s>."
                % (joint_element.get("name", "<unknown>"), tag_name)
            )

        link_name = link_element.get("link")
        if not link_name:
            raise ValueError(
                "Joint '%s' <%s> is missing link attribute."
                % (joint_element.get("name", "<unknown>"), tag_name)
            )

        return link_name

    def _parse_xyz(
        self, origin_element: Optional[ET.Element]
    ) -> Tuple[float, float, float]:
        if origin_element is None:
            return 0.0, 0.0, 0.0

        xyz_text = origin_element.get("xyz", "0 0 0")
        parts = xyz_text.split()
        if len(parts) != 3:
            raise ValueError("Invalid origin xyz '%s'." % xyz_text)

        return tuple(float(part) for part in parts)

    def _parse_rpy(self, origin_element: Optional[ET.Element]) -> Tuple[float, float, float]:
        if origin_element is None:
            return 0.0, 0.0, 0.0

        rpy_text = origin_element.get("rpy", "0 0 0")
        parts = rpy_text.split()
        if len(parts) != 3:
            raise ValueError("Invalid origin rpy '%s'." % rpy_text)

        return tuple(float(part) for part in parts)

    def _resolve_link_pose(
        self,
        *,
        joint_map,
        source_link: str,
        target_link: str,
    ) -> Tuple[float, float, float]:
        if target_link == source_link:
            return 0.0, 0.0, 0.0

        child_to_joint = {}
        for joint in joint_map.values():
            child_element = joint.find("child")
            if child_element is None:
                continue
            child_name = child_element.get("link")
            if child_name:
                child_to_joint[child_name] = joint

        source_root, source_pose = self._resolve_pose_from_root(
            child_to_joint=child_to_joint, link_name=source_link
        )
        target_root, target_pose = self._resolve_pose_from_root(
            child_to_joint=child_to_joint, link_name=target_link
        )
        if source_root != target_root:
            raise ValueError(
                "Cannot resolve pose of link '%s' relative to '%s' because they do not "
                "share the same fixed-joint root."
                % (target_link, source_link)
            )

        source_x, source_y, source_yaw = source_pose
        target_x, target_y, target_yaw = target_pose
        delta_x = target_x - source_x
        delta_y = target_y - source_y
        relative_x = math.cos(source_yaw) * delta_x + math.sin(source_yaw) * delta_y
        relative_y = -math.sin(source_yaw) * delta_x + math.cos(source_yaw) * delta_y
        relative_yaw = normalize_angle(target_yaw - source_yaw)
        return relative_x, relative_y, relative_yaw

    def _resolve_pose_from_root(
        self, *, child_to_joint, link_name: str
    ) -> Tuple[str, Tuple[float, float, float]]:
        pose_chain = []
        current_link = link_name
        while True:
            joint = child_to_joint.get(current_link)
            if joint is None:
                break

            joint_type = joint.get("type", "")
            if joint_type != "fixed":
                raise ValueError(
                    "Link '%s' must be connected through fixed joints only, but joint "
                    "'%s' has type '%s'."
                    % (
                        link_name,
                        joint.get("name", "<unknown>"),
                        joint_type,
                    )
                )

            pose_chain.append(joint)
            current_link = self._required_link_name(joint, "parent")

        x = 0.0
        y = 0.0
        yaw = 0.0
        for joint in reversed(pose_chain):
            origin = joint.find("origin")
            origin_x, origin_y, _ = self._parse_xyz(origin)
            _, _, origin_yaw = self._parse_rpy(origin)
            rotated_x = math.cos(yaw) * origin_x - math.sin(yaw) * origin_y
            rotated_y = math.sin(yaw) * origin_x + math.cos(yaw) * origin_y
            x += rotated_x
            y += rotated_y
            yaw = normalize_angle(yaw + origin_yaw)

        return current_link, (x, y, yaw)

    def _parse_joint_limits(self, joint_element: ET.Element) -> Tuple[float, float]:
        limit_element = joint_element.find("limit")
        if limit_element is None:
            return -math.pi, math.pi

        lower_limit = float(limit_element.get("lower", str(-math.pi)))
        upper_limit = float(limit_element.get("upper", str(math.pi)))
        if lower_limit > upper_limit:
            raise ValueError(
                "Joint '%s' has invalid limits [%s, %s]."
                % (
                    joint_element.get("name", "<unknown>"),
                    lower_limit,
                    upper_limit,
                )
            )
        return lower_limit, upper_limit

    def _parse_joint_velocity_limit(
        self, joint_element: ET.Element
    ) -> Optional[float]:
        limit_element = joint_element.find("limit")
        if limit_element is None or limit_element.get("velocity") is None:
            return None
        return float(limit_element.get("velocity"))

    def _find_cylinder_radius(self, link_element: ET.Element) -> float:
        for tag_name in ("collision", "visual"):
            for geometry_parent in link_element.findall(tag_name):
                cylinder = geometry_parent.find("geometry/cylinder")
                if cylinder is not None and cylinder.get("radius") is not None:
                    return float(cylinder.get("radius"))

        raise ValueError(
            "Link '%s' does not contain a cylinder geometry to infer wheel radius."
            % link_element.get("name", "<unknown>")
        )

    def _publish_float64(self, publisher, value: float) -> None:
        message = Float64()
        message.data = value
        publisher.publish(message)

    def _parse_bool_parameter(self, raw_value) -> bool:
        if isinstance(raw_value, bool):
            return raw_value
        if isinstance(raw_value, (int, float)):
            return bool(raw_value)
        if isinstance(raw_value, str):
            normalized = raw_value.strip().lower()
            if normalized in ("1", "true", "yes", "on"):
                return True
            if normalized in ("0", "false", "no", "off"):
                return False
        raise ValueError(
            "Parameter 'enabled' must be a boolean-compatible value, got: %r"
            % (raw_value,)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelToMotors()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
