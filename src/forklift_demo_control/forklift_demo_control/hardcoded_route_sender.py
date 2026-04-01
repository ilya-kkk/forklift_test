from functools import partial
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from forklift_demo_control.path_utils import (
    build_l_waypoints,
    build_path,
    densify_polyline,
)


class HardcodedRouteSender(Node):
    def __init__(self) -> None:
        super().__init__("hardcoded_route_sender")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("dispatch_period_sec", 30.0)
        self.declare_parameter(
            "path_publish_period_sec",
            float(self.get_parameter("dispatch_period_sec").value),
        )
        self.declare_parameter("retry_delay_sec", 5.0)
        self.declare_parameter("path_resolution", 0.08)
        self.declare_parameter("start_x", -1.0)
        self.declare_parameter("start_y", -1.0)
        self.declare_parameter("horizontal_length", 4.0)
        self.declare_parameter("vertical_length", 4.0)
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("path_topic", "/demo_path")

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
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
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        path_topic = str(self.get_parameter("path_topic").value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_publisher(Path, path_topic, qos)
        self._action_client = ActionClient(self, FollowPath, "follow_path")
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._active_goal_handle: Optional[ClientGoalHandle] = None
        self._goal_active = False
        self._dispatch_in_progress = False
        self._route_phase = 0
        self._next_dispatch_time = self.get_clock().now()
        self._last_feedback_log_time = self.get_clock().now()
        self._last_published_phase = 0
        self._waiting_for_server_logged = False
        self._waiting_for_tf_logged = False

        self.create_timer(1.0, self._tick)
        if self._path_publish_period > 0.0:
            self.create_timer(self._path_publish_period, self._republish_path)
        self.get_logger().info("hardcoded_route_sender waiting for FollowPath server and TF")

    def _tick(self) -> None:
        if self._dispatch_in_progress:
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

        if self._goal_active:
            return

        route_phase = self._route_phase
        route_name = "square_leg_1" if route_phase == 0 else "square_leg_2"
        path = self._build_route(route_phase=route_phase)
        self._last_published_phase = route_phase
        self._path_publisher.publish(path)
        self._send_goal(path, route_name, route_phase)

    def _build_route(self, route_phase: int):
        if route_phase == 0:
            raw_waypoints = build_l_waypoints(
                start_x=self._start_x,
                start_y=self._start_y,
                horizontal_length=self._horizontal_length,
                vertical_length=self._vertical_length,
            )
        else:
            end_x = self._start_x + self._horizontal_length
            end_y = self._start_y + self._vertical_length
            raw_waypoints = [
                (end_x, end_y),
                (self._start_x, end_y),
                (self._start_x, self._start_y),
            ]

        dense_points = densify_polyline(raw_waypoints, self._path_resolution)
        return build_path(
            dense_points,
            self._frame_id,
            self.get_clock().now().to_msg(),
        )

    def _republish_path(self) -> None:
        if self._dispatch_in_progress:
            return

        path = self._build_route(route_phase=self._last_published_phase)
        self._path_publisher.publish(path)

    def _send_goal(self, path, route_name: str, route_phase: int) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self.get_logger().info(
            "Sending %s FollowPath goal with %d poses"
            % (route_name, len(path.poses))
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
            )
        )

    def _goal_response_callback(
        self, future, route_name: str, route_phase: int
    ) -> None:
        self._dispatch_in_progress = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("%s goal was rejected" % route_name)
            self._goal_active = False
            self._active_goal_handle = None
            self._next_dispatch_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay
            )
            return

        self._active_goal_handle = goal_handle
        self._goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._result_callback,
                route_name=route_name,
                route_phase=route_phase,
            )
        )

    def _result_callback(self, future, route_name: str, route_phase: int) -> None:
        self._goal_active = False
        self._active_goal_handle = None
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("FollowPath result retrieval failed: %s" % exc)
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
        self.get_logger().info(
            "Feedback: distance_to_goal=%.2f speed=%.2f"
            % (feedback.distance_to_goal, feedback.speed)
        )
        self._last_feedback_log_time = now


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
