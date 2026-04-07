import json
import math
from typing import Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from ros2_templates.srv import StringWithJson
from std_msgs.msg import Float64


Point2D = Tuple[float, float]


class DemoRouteLoop(Node):
    def __init__(self) -> None:
        super().__init__("demo_route_loop")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("route_service_name", "/robot_data/route/go_to_point")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("tick_period_sec", 0.2)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("goal_hold_sec", 0.5)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("target_point_4_id", 4)
        self.declare_parameter("target_point_6_id", 6)
        self.declare_parameter("target_point_7_id", 7)
        self.declare_parameter("fork_lift_topic", "/forklift/fork_lift_cmd")
        self.declare_parameter("fork_lift_lowered", 0.0)
        self.declare_parameter("fork_lift_raised", 0.24)
        self.declare_parameter("fork_lift_settle_sec", 2.0)

        map_service_name = str(self.get_parameter("map_service_name").value)
        route_service_name = str(self.get_parameter("route_service_name").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        fork_lift_topic = str(self.get_parameter("fork_lift_topic").value)
        self._tick_period = float(self.get_parameter("tick_period_sec").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self._goal_hold_sec = float(self.get_parameter("goal_hold_sec").value)
        self._retry_delay_sec = float(self.get_parameter("retry_delay_sec").value)
        self._target_point_4_id = int(self.get_parameter("target_point_4_id").value)
        self._target_point_6_id = int(self.get_parameter("target_point_6_id").value)
        self._target_point_7_id = int(self.get_parameter("target_point_7_id").value)
        self._fork_lift_lowered = float(
            self.get_parameter("fork_lift_lowered").value
        )
        self._fork_lift_raised = float(
            self.get_parameter("fork_lift_raised").value
        )
        self._fork_lift_settle_sec = float(
            self.get_parameter("fork_lift_settle_sec").value
        )

        self._map_client = self.create_client(StringWithJson, map_service_name)
        self._route_client = self.create_client(StringWithJson, route_service_name)
        self._fork_lift_publisher = self.create_publisher(Float64, fork_lift_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_timer(self._tick_period, self._tick)

        self._map_request_in_flight = False
        self._route_request_in_flight = False
        self._points_by_id: Dict[int, Point2D] = {}
        self._latest_position: Optional[Point2D] = None
        self._next_action_time = self.get_clock().now()
        self._arrival_since = None
        self._bootstrap_completed = False
        self._active_leg = None
        self._cycle_index = 0
        self._fork_lift_target = self._fork_lift_lowered
        self._startup_fork_lowering_completed = False

        self._cycle_legs = [
            {
                "name": "4_to_6_rear",
                "start": self._target_point_4_id,
                "goal": self._target_point_6_id,
                "arrival_mode": "rear",
            },
            {
                "name": "6_to_7_rear",
                "start": self._target_point_6_id,
                "goal": self._target_point_7_id,
                "arrival_mode": "rear",
            },
            {
                "name": "7_to_4_rear",
                "start": self._target_point_7_id,
                "goal": self._target_point_4_id,
                "arrival_mode": "rear",
            },
        ]

        self.get_logger().info(
            "demo_route_loop ready: bootstrap -> %d rear, cycle=(%d->%d rear, %d->%d rear, %d->%d rear)"
            % (
                self._target_point_4_id,
                self._target_point_4_id,
                self._target_point_6_id,
                self._target_point_6_id,
                self._target_point_7_id,
                self._target_point_7_id,
                self._target_point_4_id,
            )
        )

    def _tick(self) -> None:
        self._publish_fork_lift_target()

        if self.get_clock().now() < self._next_action_time:
            return

        if not self._points_by_id:
            self._request_map_if_needed()
            return

        if self._latest_position is None:
            return

        if self._route_request_in_flight:
            return

        if not self._startup_fork_lowering_completed:
            self._startup_fork_lowering_completed = True
            self._fork_lift_target = self._fork_lift_lowered
            self.get_logger().info(
                "Lowering forks to minimum before first movement target=%.2f"
                % self._fork_lift_target
            )
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._fork_lift_settle_sec
            )
            return

        if self._active_leg is None:
            self._dispatch_next_leg()
            return

        target_position = self._points_by_id.get(self._active_leg["goal"])
        if target_position is None:
            self.get_logger().warning(
                "Target point %s is missing in map, resetting demo loop"
                % self._active_leg["goal"]
            )
            self._reset_loop()
            return

        distance = math.hypot(
            target_position[0] - self._latest_position[0],
            target_position[1] - self._latest_position[1],
        )
        if distance > self._goal_tolerance:
            self._arrival_since = None
            return

        now = self.get_clock().now()
        if self._arrival_since is None:
            self._arrival_since = now
            return

        if (now - self._arrival_since) < Duration(seconds=self._goal_hold_sec):
            return

        self.get_logger().info(
            "Reached %s target=%d pose=(%.2f, %.2f)"
            % (
                self._active_leg["name"],
                self._active_leg["goal"],
                self._latest_position[0],
                self._latest_position[1],
            )
        )
        self._complete_active_leg()

    def _request_map_if_needed(self) -> None:
        if self._map_request_in_flight:
            return
        if not self._map_client.wait_for_service(timeout_sec=0.0):
            return

        request = StringWithJson.Request()
        request.message = "{}"
        self._map_request_in_flight = True
        future = self._map_client.call_async(request)
        future.add_done_callback(self._map_response_callback)

    def _map_response_callback(self, future) -> None:
        self._map_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("Map request failed: %s" % exc)
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        if response is None or not response.success:
            self.get_logger().warning("Map service returned an error")
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        try:
            payload = json.loads(response.message or "{}")
        except json.JSONDecodeError as exc:
            self.get_logger().warning("Map service returned invalid JSON: %s" % exc)
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        points_by_id: Dict[int, Point2D] = {}
        for point in payload.get("point", []):
            points_by_id[int(point["point_id"])] = (
                float(point["position_x"]),
                float(point["position_y"]),
            )

        self._points_by_id = points_by_id
        self.get_logger().info(
            "Loaded demo map with %d points" % len(self._points_by_id)
        )

    def _dispatch_next_leg(self) -> None:
        if not self._route_client.wait_for_service(timeout_sec=0.0):
            return

        if not self._bootstrap_completed:
            start_point_id = self._nearest_point_id()
            if start_point_id is None:
                return

            if start_point_id == self._target_point_4_id:
                self._bootstrap_completed = True
                self._cycle_index = 0
                self.get_logger().info(
                    "Skipping bootstrap: nearest point is already target point %d"
                    % self._target_point_4_id
                )
                leg = dict(self._cycle_legs[self._cycle_index])
            else:
                leg = {
                    "name": "bootstrap_to_4_rear",
                    "start": start_point_id,
                    "goal": self._target_point_4_id,
                    "arrival_mode": "rear",
                }
        else:
            leg = dict(self._cycle_legs[self._cycle_index])

        request_payload = {
            "start": leg["start"],
            "goal": leg["goal"],
            "arrival_mode": leg["arrival_mode"],
        }
        request = StringWithJson.Request()
        request.message = json.dumps(request_payload, ensure_ascii=False)

        self._route_request_in_flight = True
        self.get_logger().info(
            "Dispatching demo leg %s start=%s goal=%s arrival_mode=%s"
            % (
                leg["name"],
                leg["start"],
                leg["goal"],
                leg["arrival_mode"],
            )
        )
        future = self._route_client.call_async(request)
        future.add_done_callback(lambda result: self._route_response_callback(result, leg))

    def _route_response_callback(self, future, leg) -> None:
        self._route_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning(
                "Route request for %s failed: %s" % (leg["name"], exc)
            )
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        if response is None or not response.success:
            self.get_logger().warning(
                "Route service rejected %s: %s"
                % (leg["name"], "" if response is None else response.message)
            )
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        self._active_leg = leg
        self._arrival_since = None
        self._next_action_time = self.get_clock().now() + Duration(seconds=0.2)

    def _complete_active_leg(self) -> None:
        if self._active_leg is None:
            return

        next_action_delay = 0.2
        if self._active_leg["name"] == "4_to_6_rear":
            self._set_fork_lift_target(self._fork_lift_raised, "raise")
            next_action_delay = self._fork_lift_settle_sec
        elif self._active_leg["name"] == "6_to_7_rear":
            self._set_fork_lift_target(self._fork_lift_lowered, "lower")
            next_action_delay = self._fork_lift_settle_sec

        if not self._bootstrap_completed:
            self._bootstrap_completed = True
            self._cycle_index = 0
        else:
            self._cycle_index = (self._cycle_index + 1) % len(self._cycle_legs)

        self._active_leg = None
        self._arrival_since = None
        self._next_action_time = self.get_clock().now() + Duration(
            seconds=next_action_delay
        )

    def _nearest_point_id(self) -> Optional[int]:
        if self._latest_position is None or not self._points_by_id:
            return None

        best_point_id = None
        best_distance = float("inf")
        for point_id, point in self._points_by_id.items():
            distance = math.hypot(
                point[0] - self._latest_position[0],
                point[1] - self._latest_position[1],
            )
            if distance < best_distance:
                best_distance = distance
                best_point_id = point_id
        return best_point_id

    def _reset_loop(self) -> None:
        self._bootstrap_completed = False
        self._active_leg = None
        self._cycle_index = 0
        self._arrival_since = None
        self._fork_lift_target = self._fork_lift_lowered
        self._startup_fork_lowering_completed = False
        self._next_action_time = self.get_clock().now() + Duration(
            seconds=self._retry_delay_sec
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _publish_fork_lift_target(self) -> None:
        message = Float64()
        message.data = self._fork_lift_target
        self._fork_lift_publisher.publish(message)

    def _set_fork_lift_target(self, value: float, action: str) -> None:
        if abs(self._fork_lift_target - value) < 1e-9:
            return

        self._fork_lift_target = value
        self.get_logger().info(
            "Fork lift command: %s to %.2f" % (action, self._fork_lift_target)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoRouteLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
