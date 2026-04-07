import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


class RvizTeleopMarker(Node):
    def __init__(self) -> None:
        super().__init__("rviz_teleop_marker")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("server_name", "forklift_teleop")
        self.declare_parameter("marker_name", "drive_joystick")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("marker_height", 0.9)
        self.declare_parameter("marker_scale", 0.45)
        self.declare_parameter("displacement_range", 0.35)
        self.declare_parameter("max_linear_velocity", 0.8)
        self.declare_parameter("max_angular_velocity", 1.2)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("deadman_timeout_sec", 1.0)
        self.declare_parameter("motion_mode_switch_threshold", 0.05)
        self.declare_parameter("auto_motion_mode", True)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._server_name = str(self.get_parameter("server_name").value)
        self._marker_name = str(self.get_parameter("marker_name").value)
        self._marker_height = float(self.get_parameter("marker_height").value)
        self._marker_scale = float(self.get_parameter("marker_scale").value)
        self._displacement_range = max(
            1e-3, float(self.get_parameter("displacement_range").value)
        )
        self._max_linear_velocity = float(
            self.get_parameter("max_linear_velocity").value
        )
        self._max_angular_velocity = float(
            self.get_parameter("max_angular_velocity").value
        )
        self._deadman_timeout_sec = max(
            0.1, float(self.get_parameter("deadman_timeout_sec").value)
        )
        self._motion_mode_switch_threshold = float(
            self.get_parameter("motion_mode_switch_threshold").value
        )
        self._auto_motion_mode = bool(self.get_parameter("auto_motion_mode").value)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        motion_mode_topic = str(self.get_parameter("motion_mode_topic").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        motion_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._motion_mode_publisher = self.create_publisher(
            String, motion_mode_topic, motion_mode_qos
        )

        self._server = InteractiveMarkerServer(self, self._server_name)
        self._drag_active = False
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._last_feedback_time = self.get_clock().now()
        self._pending_zero_publishes = 0
        self._current_motion_mode = BODY_FIRST

        self._create_marker()
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            "RViz teleop marker ready: update_topic=/%s/update cmd_vel=%s motion_mode=%s"
            % (self._server_name, cmd_vel_topic, motion_mode_topic)
        )

    def _create_marker(self) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self._frame_id
        marker.name = self._marker_name
        marker.description = "RViz joystick\nX drive, Y turn"
        marker.scale = self._marker_scale
        marker.pose = self._marker_pose(0.0, 0.0)

        visual_control = InteractiveMarkerControl()
        visual_control.name = "visual"
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        visual_control.always_visible = True
        visual_control.markers.append(self._make_disc_marker())
        visual_control.markers.append(self._make_arrow_marker())
        visual_control.markers.append(self._make_text_marker())
        marker.controls.append(visual_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "joystick_plane"
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        move_control.orientation_mode = InteractiveMarkerControl.FIXED
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(move_control)

        self._server.insert(marker, feedback_callback=self._process_feedback)
        self._server.applyChanges()

    def _make_disc_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = self._displacement_range * 2.0
        marker.scale.y = self._displacement_range * 2.0
        marker.scale.z = 0.03
        marker.color.r = 0.08
        marker.color.g = 0.65
        marker.color.b = 0.85
        marker.color.a = 0.55
        return marker

    def _make_arrow_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.ARROW
        marker.pose.position.x = self._displacement_range * 0.35
        marker.pose.position.z = 0.03
        marker.scale.x = self._displacement_range * 0.9
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.98
        marker.color.g = 0.98
        marker.color.b = 0.98
        marker.color.a = 0.95
        return marker

    def _make_text_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 0.26
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = "drag to drive"
        return marker

    def _process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self._drag_active = True
            self._last_feedback_time = self.get_clock().now()
            return

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._drag_active = True
            self._last_feedback_time = self.get_clock().now()
            self._update_command_from_pose(feedback.pose)
            return

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._stop_and_center()

    def _update_command_from_pose(self, pose: Pose) -> None:
        x_offset = clamp(pose.position.x, -self._displacement_range, self._displacement_range)
        y_offset = clamp(pose.position.y, -self._displacement_range, self._displacement_range)

        if (
            abs(x_offset - pose.position.x) > 1e-6
            or abs(y_offset - pose.position.y) > 1e-6
        ):
            self._set_marker_pose(x_offset, y_offset)

        self._command_linear = (
            x_offset / self._displacement_range * self._max_linear_velocity
        )
        self._command_angular = (
            y_offset / self._displacement_range * self._max_angular_velocity
        )

        if self._auto_motion_mode:
            self._update_motion_mode(self._command_linear)

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

    def _on_timer(self) -> None:
        if self._drag_active:
            age = (
                self.get_clock().now() - self._last_feedback_time
            ).nanoseconds / 1_000_000_000.0
            if age > self._deadman_timeout_sec:
                self._stop_and_center()
                return

            self._publish_cmd_vel(self._command_linear, self._command_angular)
            return

        if self._pending_zero_publishes > 0:
            self._publish_cmd_vel(0.0, 0.0)
            self._pending_zero_publishes -= 1

    def _stop_and_center(self) -> None:
        self._drag_active = False
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._pending_zero_publishes = 3
        self._set_marker_pose(0.0, 0.0)

    def _publish_cmd_vel(self, linear_velocity: float, angular_velocity: float) -> None:
        message = Twist()
        message.linear.x = linear_velocity
        message.angular.z = angular_velocity
        self._cmd_vel_publisher.publish(message)

    def _set_marker_pose(self, x_offset: float, y_offset: float) -> None:
        self._server.setPose(self._marker_name, self._marker_pose(x_offset, y_offset))
        self._server.applyChanges()

    def _marker_pose(self, x_offset: float, y_offset: float) -> Pose:
        pose = Pose()
        pose.position.x = x_offset
        pose.position.y = y_offset
        pose.position.z = self._marker_height
        pose.orientation.w = 1.0
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizTeleopMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
