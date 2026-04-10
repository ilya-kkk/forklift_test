import math
from functools import partial
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from forklift_demo_control.path_utils import (
    build_path,
    densify_polyline,
    round_polyline_corner,
)


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"


class HardcodedRouteSender(Node):
    def __init__(self) -> None:
        super().__init__("hardcoded_route_sender")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame_id", "tracking_link")
        self.declare_parameter("tick_period_sec", 0.2)
        self.declare_parameter("dispatch_period_sec", 30.0)
        self.declare_parameter(
            "path_publish_period_sec",
            float(self.get_parameter("dispatch_period_sec").value),
        )
        self.declare_parameter("retry_delay_sec", 5.0)
        self.declare_parameter("path_resolution", 0.08)
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 1.0)
        self.declare_parameter("horizontal_length", 6.0)
        self.declare_parameter("vertical_length", 6.0)
        self.declare_parameter("rounded_corner_radius", 1.0)
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("path_topic", "/demo_path")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("segment_completion_distance", 0.05)
        self.declare_parameter("controller_server_name", "controller_server")
        self.declare_parameter("controller_reverse_param", "")
        self.declare_parameter("mode_switch_settle_sec", 0.3)
        self.declare_parameter("debug_logging", True)
        self.declare_parameter("debug_log_period_sec", 1.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._tick_period = float(self.get_parameter("tick_period_sec").value)
        self._path_publish_period = float(
            self.get_parameter("path_publish_period_sec").value
        )
        self._retry_delay = float(self.get_parameter("retry_delay_sec").value)
        self._path_resolution = float(self.get_parameter("path_resolution").value)
        self._start_x = float(self.get_parameter("start_x").value)
        self._start_y = float(self.get_parameter("start_y").value)
        self._horizontal_length = float(
            self.get_parameter("horizontal_length").value
        )
        self._vertical_length = float(self.get_parameter("vertical_length").value)
        self._rounded_corner_radius = float(
            self.get_parameter("rounded_corner_radius").value
        )
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        path_topic = str(self.get_parameter("path_topic").value)
        motion_mode_topic = str(self.get_parameter("motion_mode_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._segment_completion_distance = float(
            self.get_parameter("segment_completion_distance").value
        )
        controller_server_name = str(
            self.get_parameter("controller_server_name").value
        )
        self._controller_reverse_param = str(
            self.get_parameter("controller_reverse_param").value
        )
        self._mode_switch_settle_sec = float(
            self.get_parameter("mode_switch_settle_sec").value
        )
        self._debug_logging = bool(self.get_parameter("debug_logging").value)
        self._debug_log_period = float(
            self.get_parameter("debug_log_period_sec").value
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_publisher(Path, path_topic, qos)
        self._motion_mode_publisher = self.create_publisher(
            String, motion_mode_topic, qos
        )
        self._action_client = ActionClient(self, FollowPath, "follow_path")
        self._controller_param_service = f"/{controller_server_name}/set_parameters"
        self._controller_param_client = self.create_client(
            SetParameters, self._controller_param_service
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self._active_goal_handle: Optional[ClientGoalHandle] = None
        self._goal_active = False
        self._dispatch_in_progress = False
        self._cancel_in_progress = False
        self._controller_param_update_in_progress = False
        self._controller_allow_reversing: Optional[bool] = None
        self._goal_token = 0
        self._active_goal_token = 0
        self._route_phase = 0
        self._active_route_phase: Optional[int] = None
        self._active_segment_target: Optional[Tuple[float, float]] = None
        self._latest_position: Optional[Tuple[float, float]] = None
        self._next_dispatch_time = self.get_clock().now()
        self._last_feedback_log_time = self.get_clock().now()
        self._last_debug_log_time = self.get_clock().now()
        self._last_published_phase = 0
        self._waiting_for_server_logged = False
        self._waiting_for_tf_logged = False
        self._waiting_for_param_service_logged = False

        self.create_timer(self._tick_period, self._tick)
        if self._path_publish_period > 0.0:
            self.create_timer(self._path_publish_period, self._republish_path)
        self._publish_motion_mode(self._motion_mode_for_phase(self._route_phase))
        self.get_logger().info("hardcoded_route_sender waiting for FollowPath server and TF")

    def _tick(self) -> None:
        if self._dispatch_in_progress or self._controller_param_update_in_progress:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            if not self._waiting_for_server_logged:
                self.get_logger().info("Waiting for /follow_path action server")
                self._waiting_for_server_logged = True
            return

        if not self._tf_buffer.can_transform(
            self._frame_id,
            self._base_frame_id,
            Time(),
            timeout=Duration(seconds=0.0),
        ):
            if not self._waiting_for_tf_logged:
                self.get_logger().info(
                    "Waiting for TF %s -> %s" % (self._frame_id, self._base_frame_id)
                )
                self._waiting_for_tf_logged = True
            return

        self._waiting_for_server_logged = False
        self._waiting_for_tf_logged = False

        if self.get_clock().now() < self._next_dispatch_time:
            return

        if self._goal_active and self._has_reached_active_target():
            self._request_phase_switch()
            return

        if self._goal_active or self._cancel_in_progress:
            return

        desired_allow_reversing = self._allow_reversing_for_phase(self._route_phase)
        if self._controller_allow_reversing != desired_allow_reversing:
            self._ensure_controller_reverse_mode(desired_allow_reversing)
            return

        self._dispatch_route(route_phase=self._route_phase)

    def _build_route(self, route_phase: int):
        end_x = self._start_x + self._horizontal_length
        end_y = self._start_y + self._vertical_length
        if route_phase == 0:
            raw_waypoints = [
                (self._start_x, self._start_y),
                (self._start_x, end_y),
                (end_x, end_y),
            ]
        else:
            raw_waypoints = [
                (end_x, end_y),
                (end_x, self._start_y),
                (self._start_x, self._start_y),
            ]
            raw_waypoints = round_polyline_corner(
                raw_waypoints,
                corner_index=1,
                radius=self._rounded_corner_radius,
                resolution=self._path_resolution,
            )

        dense_points = densify_polyline(raw_waypoints, self._path_resolution)
        return build_path(
            dense_points,
            self._frame_id,
            self.get_clock().now().to_msg(),
            yaw_offset=3.141592653589793 if route_phase == 0 else 0.0,
        )

    def _republish_path(self) -> None:
        if self._dispatch_in_progress:
            return

        path = self._build_route(route_phase=self._last_published_phase)
        self._path_publisher.publish(path)
        self._publish_motion_mode(self._motion_mode_for_phase(self._last_published_phase))

    def _dispatch_route(self, route_phase: int) -> None:
        route_name = "square_leg_1" if route_phase == 0 else "square_leg_2"
        path = self._build_route(route_phase=route_phase)
        self._last_published_phase = route_phase
        self._publish_motion_mode(self._motion_mode_for_phase(route_phase))
        self._log_route_dispatch(route_name, route_phase, path)
        self._path_publisher.publish(path)
        self._send_goal(path, route_name, route_phase)

    def _send_goal(self, path, route_name: str, route_phase: int) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self._goal_token += 1
        goal_token = self._goal_token
        self.get_logger().info(
            "Sending %s FollowPath goal with %d poses in %s mode"
            % (
                route_name,
                len(path.poses),
                self._motion_mode_for_phase(route_phase),
            )
        )
        self._dispatch_in_progress = True
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(
            partial(
                self._goal_response_callback,
                route_name=route_name,
                route_phase=route_phase,
                goal_token=goal_token,
                final_target=(
                    path.poses[-1].pose.position.x,
                    path.poses[-1].pose.position.y,
                ),
            )
        )

    def _goal_response_callback(
        self,
        future,
        route_name: str,
        route_phase: int,
        goal_token: int,
        final_target: Tuple[float, float],
    ) -> None:
        self._dispatch_in_progress = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("Failed to send %s goal: %s" % (route_name, exc))
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("%s goal was rejected" % route_name)
            if not self._goal_active:
                self._route_phase = route_phase
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        self._active_goal_handle = goal_handle
        self._goal_active = True
        self._active_goal_token = goal_token
        self._active_route_phase = route_phase
        self._active_segment_target = final_target
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._result_callback,
                route_name=route_name,
                route_phase=route_phase,
                goal_token=goal_token,
            )
        )

    def _result_callback(
        self, future, route_name: str, route_phase: int, goal_token: int
    ) -> None:
        if goal_token != self._active_goal_token:
            return

        self._goal_active = False
        self._active_goal_handle = None
        self._active_goal_token = 0
        self._active_route_phase = None
        self._active_segment_target = None
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("FollowPath result retrieval failed: %s" % exc)
            self._route_phase = route_phase
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        self.get_logger().info(
            "%s FollowPath finished with status %s" % (route_name, result.status)
        )

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._route_phase = (route_phase + 1) % 2
            self._next_dispatch_time = self.get_clock().now()
            return

        self._route_phase = route_phase
        self._next_dispatch_time = self.get_clock().now() + Duration(
            seconds=self._retry_delay
        )

    def _feedback_callback(self, feedback_msg) -> None:
        now = self.get_clock().now()
        if (now - self._last_feedback_log_time).nanoseconds < 2_000_000_000:
            return

        feedback = feedback_msg.feedback
        if self._debug_logging:
            self.get_logger().info(
                "debug feedback phase=%s mode=%s allow_reversing=%s "
                "distance_to_goal=%.2f speed=%.2f pose=%s target=%s target_dist=%.2f"
                % (
                    self._active_route_phase,
                    self._motion_mode_for_phase(
                        0 if self._active_route_phase is None else self._active_route_phase
                    ),
                    self._controller_allow_reversing,
                    feedback.distance_to_goal,
                    feedback.speed,
                    self._format_point(self._latest_position),
                    self._format_point(self._active_segment_target),
                    self._distance_to_active_target(),
                )
            )
        else:
            self.get_logger().info(
                "Feedback: distance_to_goal=%.2f speed=%.2f"
                % (feedback.distance_to_goal, feedback.speed)
            )
        self._last_feedback_log_time = now

        if self._debug_logging and (
            now - self._last_debug_log_time
        ).nanoseconds >= int(self._debug_log_period * 1_000_000_000.0):
            self.get_logger().info(
                "debug state route_phase=%d active_phase=%s mode=%s allow_reversing=%s "
                "pose=%s target=%s next_dispatch_in=%.2f"
                % (
                    self._route_phase,
                    self._active_route_phase,
                    self._motion_mode_for_phase(self._route_phase),
                    self._controller_allow_reversing,
                    self._format_point(self._latest_position),
                    self._format_point(self._active_segment_target),
                    max(
                        0.0,
                        (self._next_dispatch_time - now).nanoseconds / 1_000_000_000.0,
                    ),
                )
            )
            self._last_debug_log_time = now

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _has_reached_active_target(self) -> bool:
        return self._distance_to_active_target() < self._segment_completion_distance

    def _motion_mode_for_phase(self, route_phase: int) -> str:
        return FORKS_FIRST if route_phase == 0 else BODY_FIRST

    def _allow_reversing_for_phase(self, route_phase: int) -> bool:
        return route_phase == 0

    def _publish_motion_mode(self, motion_mode: str) -> None:
        motion_mode_msg = String()
        motion_mode_msg.data = motion_mode
        self._motion_mode_publisher.publish(motion_mode_msg)

    def _request_phase_switch(self) -> None:
        if self._cancel_in_progress:
            return

        completed_phase = self._active_route_phase
        next_phase = 0 if completed_phase is None else (completed_phase + 1) % 2
        self.get_logger().info(
            "Reached route phase %d target within %.2f m, switching to phase %d "
            "pose=%s target=%s"
            % (
                -1 if completed_phase is None else completed_phase,
                self._segment_completion_distance,
                next_phase,
                self._format_point(self._latest_position),
                self._format_point(self._active_segment_target),
            )
        )

        if self._active_goal_handle is None:
            self._complete_phase_switch(next_phase)
            return

        self._cancel_in_progress = True
        goal_token = self._active_goal_token
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            partial(
                self._cancel_goal_callback,
                goal_token=goal_token,
                next_phase=next_phase,
            )
        )

    def _cancel_goal_callback(
        self, future, goal_token: int, next_phase: int
    ) -> None:
        self._cancel_in_progress = False

        if goal_token != self._active_goal_token:
            return

        try:
            cancel_response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("Failed to cancel active FollowPath goal: %s" % exc)
            return

        if not cancel_response.goals_canceling:
            self.get_logger().warning(
                "Active FollowPath goal was not canceled, waiting for action result"
            )
            return

        self._complete_phase_switch(next_phase)

    def _complete_phase_switch(self, next_phase: int) -> None:
        self._goal_active = False
        self._active_goal_handle = None
        self._active_goal_token = 0
        self._active_route_phase = None
        self._active_segment_target = None
        self._route_phase = next_phase
        self._next_dispatch_time = self.get_clock().now()

    def _ensure_controller_reverse_mode(self, allow_reversing: bool) -> None:
        if not self._controller_reverse_param:
            self._controller_allow_reversing = allow_reversing
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._mode_switch_settle_sec
            )
            return

        if not self._controller_param_client.wait_for_service(timeout_sec=0.0):
            if not self._waiting_for_param_service_logged:
                self.get_logger().info(
                    "Waiting for parameter service %s to set %s=%s"
                    % (
                        self._controller_param_service,
                        self._controller_reverse_param,
                        allow_reversing,
                    )
                )
                self._waiting_for_param_service_logged = True
            return

        self._waiting_for_param_service_logged = False
        if self._debug_logging:
            self.get_logger().info(
                "Setting %s=%s for route_phase=%d mode=%s"
                % (
                    self._controller_reverse_param,
                    allow_reversing,
                    self._route_phase,
                    self._motion_mode_for_phase(self._route_phase),
                )
            )
        self._controller_param_update_in_progress = True
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                self._controller_reverse_param,
                value=allow_reversing,
            ).to_parameter_msg()
        ]
        future = self._controller_param_client.call_async(
            request
        )
        future.add_done_callback(
            partial(
                self._controller_reverse_mode_callback,
                allow_reversing=allow_reversing,
            )
        )

    def _controller_reverse_mode_callback(self, future, allow_reversing: bool) -> None:
        self._controller_param_update_in_progress = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning(
                "Failed to set %s=%s: %s"
                % (self._controller_reverse_param, allow_reversing, exc)
            )
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        results = [] if response is None else list(response.results)
        if not results or not all(result.successful for result in results):
            reasons = ", ".join(
                result.reason for result in results if getattr(result, "reason", "")
            )
            self.get_logger().warning(
                "Controller rejected %s=%s%s"
                % (
                    self._controller_reverse_param,
                    allow_reversing,
                    "" if not reasons else " (%s)" % reasons,
                )
            )
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        self._controller_allow_reversing = allow_reversing
        if self._debug_logging:
            self.get_logger().info(
                "Applied %s=%s for route_phase=%d"
                % (
                    self._controller_reverse_param,
                    allow_reversing,
                    self._route_phase,
                )
            )
        self._next_dispatch_time = self.get_clock().now() + Duration(
            seconds=self._mode_switch_settle_sec
        )

    def _log_route_dispatch(self, route_name: str, route_phase: int, path: Path) -> None:
        if not self._debug_logging or not path.poses:
            return

        start_pose = path.poses[0].pose
        end_pose = path.poses[-1].pose
        self.get_logger().info(
            "debug dispatch %s phase=%d mode=%s allow_reversing=%s "
            "start=(%.2f, %.2f, %.1fdeg) end=(%.2f, %.2f, %.1fdeg) poses=%d"
            % (
                route_name,
                route_phase,
                self._motion_mode_for_phase(route_phase),
                self._allow_reversing_for_phase(route_phase),
                start_pose.position.x,
                start_pose.position.y,
                self._yaw_degrees(start_pose.orientation),
                end_pose.position.x,
                end_pose.position.y,
                self._yaw_degrees(end_pose.orientation),
                len(path.poses),
            )
        )

    def _distance_to_active_target(self) -> float:
        if self._active_segment_target is None or self._latest_position is None:
            return float("inf")

        dx = self._active_segment_target[0] - self._latest_position[0]
        dy = self._active_segment_target[1] - self._latest_position[1]
        return (dx * dx + dy * dy) ** 0.5

    def _format_point(self, point: Optional[Tuple[float, float]]) -> str:
        if point is None:
            return "None"
        return "(%.2f, %.2f)" % point

    def _yaw_degrees(self, orientation) -> float:
        return math.degrees(2.0 * math.atan2(orientation.z, orientation.w))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardcodedRouteSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
