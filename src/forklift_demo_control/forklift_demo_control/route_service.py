import json
import math
import threading
from dataclasses import dataclass
from functools import partial
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros2_templates.srv import StringWithJson
from std_msgs.msg import String

from forklift_demo_control.path_utils import (
    build_path,
    densify_polyline,
    is_right_angle_corner,
    smooth_right_angle_corners,
)


BODY_FIRST = "BODY_FIRST"
FORKS_FIRST = "FORKS_FIRST"
FRONT_ARRIVAL = "front"
REAR_ARRIVAL = "rear"
# User-facing "front" is the steering-wheel side of the robot, which is the
# negative-x side in the current model and therefore maps to FORKS_FIRST.
FRONT_MOTION_MODE = FORKS_FIRST
REAR_MOTION_MODE = BODY_FIRST

Point2D = Tuple[float, float]


@dataclass
class PlannedSegment:
    name: str
    path: Path
    motion_mode: str
    allow_reversing: bool


@dataclass
class PlannedRoute:
    request_id: int
    summary: Dict[str, Any]
    preview_path: Path
    segments: List[PlannedSegment]


class RouteServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("route_service")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("path_resolution", 0.08)
        self.declare_parameter("rounded_corner_radius", 0.0)
        self.declare_parameter("right_angle_tolerance_deg", 15.0)
        self.declare_parameter("rear_entry_extension", 0.0)
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("path_topic", "/demo_path")
        self.declare_parameter("motion_mode_topic", "/motion_mode")
        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("route_service_name", "/robot_data/route/go_to_point")
        self.declare_parameter("controller_server_name", "controller_server")
        self.declare_parameter("controller_reverse_param", "FollowPath.allow_reversing")
        self.declare_parameter("mode_switch_settle_sec", 0.3)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("debug_logging", True)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._path_resolution = float(self.get_parameter("path_resolution").value)
        self._rounded_corner_radius = float(
            self.get_parameter("rounded_corner_radius").value
        )
        self._right_angle_tolerance_deg = float(
            self.get_parameter("right_angle_tolerance_deg").value
        )
        self._rear_entry_extension = float(
            self.get_parameter("rear_entry_extension").value
        )
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._controller_reverse_param = str(
            self.get_parameter("controller_reverse_param").value
        )
        self._mode_switch_settle_sec = float(
            self.get_parameter("mode_switch_settle_sec").value
        )
        self._retry_delay_sec = float(self.get_parameter("retry_delay_sec").value)
        self._debug_logging = bool(self.get_parameter("debug_logging").value)

        path_topic = str(self.get_parameter("path_topic").value)
        motion_mode_topic = str(self.get_parameter("motion_mode_topic").value)
        map_service_name = str(self.get_parameter("map_service_name").value)
        route_service_name = str(self.get_parameter("route_service_name").value)
        controller_server_name = str(
            self.get_parameter("controller_server_name").value
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
        self._map_client = self.create_client(StringWithJson, map_service_name)
        self._controller_param_service = f"/{controller_server_name}/set_parameters"
        self._controller_param_client = self.create_client(
            SetParameters, self._controller_param_service
        )
        self.create_service(
            StringWithJson, route_service_name, self._handle_route_request
        )
        self.create_timer(0.1, self._tick)

        self._lock = threading.Lock()
        self._request_counter = 0
        self._pending_route: Optional[PlannedRoute] = None
        self._queued_route: Optional[PlannedRoute] = None
        self._active_route: Optional[PlannedRoute] = None
        self._active_segment_index = 0
        self._active_goal_handle: Optional[ClientGoalHandle] = None
        self._goal_active = False
        self._dispatch_in_progress = False
        self._cancel_in_progress = False
        self._controller_param_update_in_progress = False
        self._controller_allow_reversing: Optional[bool] = None
        self._goal_token = 0
        self._active_goal_token = 0
        self._next_action_time = self.get_clock().now()

        self.get_logger().info(
            "Route service ready: route=%s map=%s path_topic=%s"
            % (route_service_name, map_service_name, path_topic)
        )

    def _handle_route_request(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_json", "details": str(exc)}, ensure_ascii=False
            )
            return response

        try:
            request_data = self._normalize_request_payload(payload)
        except ValueError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_request", "details": str(exc)}, ensure_ascii=False
            )
            return response

        with self._lock:
            self._request_counter += 1
            request_id = self._request_counter

        worker = threading.Thread(
            target=self._build_route_in_background,
            args=(request_id, request_data),
            daemon=True,
        )
        worker.start()

        response.success = True
        response.message = json.dumps(
            {
                "accepted": True,
                "request_id": request_id,
                "start": request_data["start"],
                "goal": request_data["goal"],
                "arrival_mode": request_data["arrival_mode"],
            },
            ensure_ascii=False,
        )
        return response

    def _tick(self) -> None:
        if self._dispatch_in_progress or self._controller_param_update_in_progress:
            return

        if self.get_clock().now() < self._next_action_time:
            return

        pending_route = None
        with self._lock:
            if self._pending_route is not None:
                pending_route = self._pending_route
                self._pending_route = None

        if pending_route is not None:
            self._activate_route(pending_route)
            return

        if self._goal_active or self._cancel_in_progress:
            return

        if self._active_route is None:
            return

        self._ensure_segment_ready()

    def _build_route_in_background(
        self, request_id: int, request_data: Dict[str, Any]
    ) -> None:
        try:
            route = self._request_route_plan(request_id, request_data)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(
                "Failed to plan route request %d: %s" % (request_id, exc)
            )
            return

        if route is None:
            return

        with self._lock:
            self._pending_route = route

    def _request_route_plan(
        self, request_id: int, request_data: Dict[str, Any]
    ) -> Optional[PlannedRoute]:
        if not self._map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Map service is not available")
            return None

        map_request = StringWithJson.Request()
        map_request.message = "{}"
        map_response = self._map_client.call(map_request)
        if map_response is None or not map_response.success:
            self.get_logger().error("Map service returned an error")
            return None

        try:
            map_data = json.loads(map_response.message or "{}")
        except json.JSONDecodeError as exc:
            self.get_logger().error("Map service returned invalid JSON: %s" % exc)
            return None

        try:
            route = self._plan_route(request_id, request_data, map_data)
        except ValueError as exc:
            self.get_logger().error("Route request %d rejected: %s" % (request_id, exc))
            return None

        if self._debug_logging:
            self.get_logger().info(
                "Planned route request_id=%d start=%s goal=%s arrival_mode=%s nodes=%s segments=%d"
                % (
                    request_id,
                    request_data["start"],
                    request_data["goal"],
                    request_data["arrival_mode"],
                    route.summary["route_aliases"],
                    len(route.segments),
                )
            )
        return route

    def _activate_route(self, route: PlannedRoute) -> None:
        if self._goal_active:
            self._queued_route = route
            self._cancel_active_goal()
            return

        self._queued_route = None
        self._active_route = route
        self._active_segment_index = 0
        self._path_publisher.publish(route.preview_path)
        self._next_action_time = self.get_clock().now()
        self._ensure_segment_ready()

    def _ensure_segment_ready(self) -> None:
        segment = self._current_segment()
        if segment is None:
            self.get_logger().info(
                "Route request %d finished"
                % (-1 if self._active_route is None else self._active_route.request_id)
            )
            self._active_route = None
            self._start_queued_route_if_any()
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            return

        desired_allow_reversing = segment.allow_reversing
        if self._controller_allow_reversing != desired_allow_reversing:
            self._ensure_controller_reverse_mode(desired_allow_reversing)
            return

        self._dispatch_segment(segment)

    def _dispatch_segment(self, segment: PlannedSegment) -> None:
        self._publish_motion_mode(segment.motion_mode)
        self._path_publisher.publish(self._active_route.preview_path)

        goal = FollowPath.Goal()
        goal.path = segment.path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id

        self._goal_token += 1
        goal_token = self._goal_token
        self._dispatch_in_progress = True
        self.get_logger().info(
            "Dispatching %s request_id=%d segment=%d/%d mode=%s allow_reversing=%s poses=%d"
            % (
                segment.name,
                self._active_route.request_id,
                self._active_segment_index + 1,
                len(self._active_route.segments),
                segment.motion_mode,
                segment.allow_reversing,
                len(segment.path.poses),
            )
        )
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            partial(self._goal_response_callback, goal_token=goal_token)
        )

    def _goal_response_callback(self, future, goal_token: int) -> None:
        self._dispatch_in_progress = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("Failed to send FollowPath goal: %s" % exc)
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("FollowPath goal was rejected")
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        self._active_goal_handle = goal_handle
        self._goal_active = True
        self._active_goal_token = goal_token
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._result_callback, goal_token=goal_token)
        )

    def _result_callback(self, future, goal_token: int) -> None:
        if goal_token != self._active_goal_token:
            return

        self._goal_active = False
        self._active_goal_handle = None
        self._active_goal_token = 0
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("FollowPath result retrieval failed: %s" % exc)
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        self.get_logger().info("FollowPath finished with status %s" % result.status)

        if self._queued_route is not None and result.status == GoalStatus.STATUS_CANCELED:
            self._active_route = None
            self._start_queued_route_if_any()
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._active_segment_index += 1
            self._next_action_time = self.get_clock().now()
            self._ensure_segment_ready()
            return

        self.get_logger().warning("Route execution stopped after action status %s" % result.status)
        self._active_route = None
        self._start_queued_route_if_any()

    def _cancel_active_goal(self) -> None:
        if self._cancel_in_progress or self._active_goal_handle is None:
            return

        self._cancel_in_progress = True
        goal_token = self._active_goal_token
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            partial(self._cancel_goal_callback, goal_token=goal_token)
        )

    def _cancel_goal_callback(self, future, goal_token: int) -> None:
        self._cancel_in_progress = False
        if goal_token != self._active_goal_token:
            return

        try:
            cancel_response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning("Failed to cancel FollowPath goal: %s" % exc)
            return

        if not cancel_response.goals_canceling:
            self.get_logger().warning(
                "FollowPath goal was not canceled immediately; waiting for result"
            )
            return

        self._goal_active = False
        self._active_goal_handle = None
        self._active_goal_token = 0
        self._active_route = None
        self._next_action_time = self.get_clock().now()
        self._start_queued_route_if_any()

    def _ensure_controller_reverse_mode(self, allow_reversing: bool) -> None:
        if not self._controller_param_client.wait_for_service(timeout_sec=0.0):
            return

        self._controller_param_update_in_progress = True
        self.get_logger().info(
            "Setting %s=%s"
            % (self._controller_reverse_param, allow_reversing)
        )
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                self._controller_reverse_param,
                value=allow_reversing,
            ).to_parameter_msg()
        ]
        future = self._controller_param_client.call_async(request)
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
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        results = [] if response is None else list(response.results)
        if not results or not all(result.successful for result in results):
            self.get_logger().warning(
                "Controller rejected %s=%s"
                % (self._controller_reverse_param, allow_reversing)
            )
            self._next_action_time = self.get_clock().now() + Duration(
                seconds=self._retry_delay_sec
            )
            return

        self._controller_allow_reversing = allow_reversing
        self.get_logger().info(
            "Applied %s=%s"
            % (self._controller_reverse_param, allow_reversing)
        )
        self._next_action_time = self.get_clock().now() + Duration(
            seconds=self._mode_switch_settle_sec
        )

    def _current_segment(self) -> Optional[PlannedSegment]:
        if self._active_route is None:
            return None
        if self._active_segment_index >= len(self._active_route.segments):
            return None
        return self._active_route.segments[self._active_segment_index]

    def _start_queued_route_if_any(self) -> None:
        if self._queued_route is None:
            return

        route = self._queued_route
        self._queued_route = None
        self._active_route = route
        self._active_segment_index = 0
        self._path_publisher.publish(route.preview_path)
        self._next_action_time = self.get_clock().now()
        self._ensure_segment_ready()

    def _publish_motion_mode(self, motion_mode: str) -> None:
        motion_mode_message = String()
        motion_mode_message.data = motion_mode
        self._motion_mode_publisher.publish(motion_mode_message)

    def _normalize_request_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        start_value = self._extract_required_value(
            payload,
            ("start", "start_point", "from", "from_point", "source"),
            "start",
        )
        goal_value = self._extract_required_value(
            payload,
            ("goal", "goal_point", "target", "target_point", "to", "to_point"),
            "goal",
        )
        arrival_value = self._extract_required_value(
            payload,
            ("arrival_mode", "arrival_direction", "arrive_mode", "direction"),
            "arrival_mode",
        )
        return {
            "start": start_value,
            "goal": goal_value,
            "arrival_mode": self._normalize_arrival_mode(arrival_value),
        }

    def _extract_required_value(
        self, payload: Dict[str, Any], keys: Sequence[str], label: str
    ) -> Any:
        for key in keys:
            if key in payload:
                return payload[key]
        raise ValueError("missing '%s' field" % label)

    def _normalize_arrival_mode(self, value: Any) -> str:
        normalized = str(value).strip().lower()
        if normalized in {
            "front",
            "forward",
            "forks_first",
            "forks-first",
            "forksfirst",
            "передом",
            "перед",
        }:
            return FRONT_ARRIVAL
        if normalized in {
            "rear",
            "reverse",
            "backward",
            "body_first",
            "body-first",
            "bodyfirst",
            "задом",
            "зад",
        }:
            return REAR_ARRIVAL
        raise ValueError("unsupported arrival_mode '%s'" % value)

    def _plan_route(
        self,
        request_id: int,
        request_data: Dict[str, Any],
        map_data: Dict[str, Any],
    ) -> PlannedRoute:
        points_by_id, points_by_alias = self._index_points(map_data)
        start_point = self._resolve_point(request_data["start"], points_by_id, points_by_alias)
        goal_point = self._resolve_point(request_data["goal"], points_by_id, points_by_alias)
        route_point_ids = self._shortest_path(
            start_point["point_id"], goal_point["point_id"], points_by_id, map_data
        )
        route_points = [
            (
                float(points_by_id[point_id]["position_x"]),
                float(points_by_id[point_id]["position_y"]),
            )
            for point_id in route_point_ids
        ]
        route_aliases = [str(points_by_id[point_id]["alias"]) for point_id in route_point_ids]
        segments, preview_points = self._build_segments(
            route_points, request_data["arrival_mode"]
        )
        preview_path = self._build_preview_path(preview_points)
        summary = {
            "request_id": request_id,
            "start_point_id": start_point["point_id"],
            "start_alias": start_point["alias"],
            "goal_point_id": goal_point["point_id"],
            "goal_alias": goal_point["alias"],
            "arrival_mode": request_data["arrival_mode"],
            "route_aliases": route_aliases,
            "route_point_ids": route_point_ids,
            "segment_modes": [segment.motion_mode for segment in segments],
        }
        return PlannedRoute(
            request_id=request_id,
            summary=summary,
            preview_path=preview_path,
            segments=segments,
        )

    def _index_points(
        self, map_data: Dict[str, Any]
    ) -> Tuple[Dict[int, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
        points = list(map_data.get("point", []))
        if not points:
            raise ValueError("map contains no points")

        points_by_id: Dict[int, Dict[str, Any]] = {}
        points_by_alias: Dict[str, Dict[str, Any]] = {}
        for point in points:
            point_id = int(point["point_id"])
            alias = str(point["alias"])
            points_by_id[point_id] = point
            points_by_alias[alias] = point
        return points_by_id, points_by_alias

    def _resolve_point(
        self,
        value: Any,
        points_by_id: Dict[int, Dict[str, Any]],
        points_by_alias: Dict[str, Dict[str, Any]],
    ) -> Dict[str, Any]:
        if isinstance(value, dict):
            if "alias" in value:
                return self._resolve_point(value["alias"], points_by_id, points_by_alias)
            if "point_id" in value:
                return self._resolve_point(value["point_id"], points_by_id, points_by_alias)
            raise ValueError("point object must contain 'alias' or 'point_id'")

        if isinstance(value, int):
            if value in points_by_id:
                return points_by_id[value]
            raise ValueError("unknown point_id %s" % value)

        normalized = str(value).strip()
        if normalized in points_by_alias:
            return points_by_alias[normalized]
        if normalized.isdigit():
            point_id = int(normalized)
            if point_id in points_by_id:
                return points_by_id[point_id]
        raise ValueError("unknown point '%s'" % value)

    def _shortest_path(
        self,
        start_point_id: int,
        goal_point_id: int,
        points_by_id: Dict[int, Dict[str, Any]],
        map_data: Dict[str, Any],
    ) -> List[int]:
        if start_point_id == goal_point_id:
            return [start_point_id]

        adjacency: Dict[int, List[Tuple[int, float]]] = {}
        for path_item in map_data.get("path", []):
            source_id = int(path_item["start_point_id"])
            target_id = int(path_item["end_point_id"])
            if source_id not in points_by_id or target_id not in points_by_id:
                continue
            source = points_by_id[source_id]
            target = points_by_id[target_id]
            weight = math.hypot(
                float(target["position_x"]) - float(source["position_x"]),
                float(target["position_y"]) - float(source["position_y"]),
            )
            adjacency.setdefault(source_id, []).append((target_id, weight))

        distances: Dict[int, float] = {start_point_id: 0.0}
        previous: Dict[int, int] = {}
        visited = set()

        while True:
            current_id = None
            current_distance = float("inf")
            for point_id, distance in distances.items():
                if point_id in visited:
                    continue
                if distance < current_distance:
                    current_distance = distance
                    current_id = point_id

            if current_id is None:
                break
            if current_id == goal_point_id:
                break

            visited.add(current_id)
            for neighbor_id, weight in adjacency.get(current_id, []):
                candidate = current_distance + weight
                if candidate < distances.get(neighbor_id, float("inf")):
                    distances[neighbor_id] = candidate
                    previous[neighbor_id] = current_id

        if goal_point_id not in distances:
            raise ValueError(
                "no route from point_id %d to point_id %d"
                % (start_point_id, goal_point_id)
            )

        route = [goal_point_id]
        current_id = goal_point_id
        while current_id != start_point_id:
            current_id = previous[current_id]
            route.append(current_id)
        route.reverse()
        return route

    def _build_segments(
        self, route_points: Sequence[Point2D], arrival_mode: str
    ) -> Tuple[List[PlannedSegment], List[Point2D]]:
        if len(route_points) == 1:
            return [], list(route_points)

        if arrival_mode == FRONT_ARRIVAL:
            segment, preview_points = self._create_segment(
                name="route_full_front",
                route_points=route_points,
                motion_mode=FRONT_MOTION_MODE,
                allow_reversing=True,
            )
            return [segment], preview_points

        segments: List[PlannedSegment] = []
        preview_points: List[Point2D] = []

        prefix_route_points = list(route_points[:-1])
        rear_route_points = list(route_points[-2:])
        if self._should_use_rear_turn_maneuver(route_points):
            overshoot_point = self._extend_point(
                route_points[-3],
                route_points[-2],
                self._rear_entry_extension,
            )
            prefix_route_points.append(overshoot_point)
            rear_route_points = [overshoot_point, route_points[-2], route_points[-1]]

        if len(prefix_route_points) > 1:
            prefix_segment, prefix_points = self._create_segment(
                name="route_prefix_front",
                route_points=prefix_route_points,
                motion_mode=FRONT_MOTION_MODE,
                allow_reversing=True,
            )
            segments.append(prefix_segment)
            preview_points = prefix_points

        rear_segment, rear_points = self._create_segment(
            name="route_final_rear",
            route_points=rear_route_points,
            motion_mode=REAR_MOTION_MODE,
            allow_reversing=False,
        )
        segments.append(rear_segment)
        preview_points = self._merge_route_points(preview_points, rear_points)
        return segments, preview_points

    def _create_segment(
        self,
        *,
        name: str,
        route_points: Sequence[Point2D],
        motion_mode: str,
        allow_reversing: bool,
    ) -> Tuple[PlannedSegment, List[Point2D]]:
        shaped_points = self._shape_route_points(route_points)
        return (
            PlannedSegment(
                name=name,
                path=self._build_path_from_points(shaped_points, motion_mode),
                motion_mode=motion_mode,
                allow_reversing=allow_reversing,
            ),
            shaped_points,
        )

    def _shape_route_points(self, route_points: Sequence[Point2D]) -> List[Point2D]:
        return smooth_right_angle_corners(
            route_points,
            radius=self._rounded_corner_radius,
            resolution=self._path_resolution,
            tolerance_degrees=self._right_angle_tolerance_deg,
        )

    def _build_preview_path(self, route_points: Sequence[Point2D]) -> Path:
        return self._build_path_from_points(route_points, BODY_FIRST)

    def _build_path_from_points(
        self, route_points: Sequence[Point2D], motion_mode: str
    ) -> Path:
        yaw_offset = math.pi if motion_mode == FORKS_FIRST else 0.0
        dense_points = densify_polyline(route_points, self._path_resolution)
        return build_path(
            dense_points,
            self._frame_id,
            self.get_clock().now().to_msg(),
            yaw_offset=yaw_offset,
        )

    def _should_use_rear_turn_maneuver(
        self, route_points: Sequence[Point2D]
    ) -> bool:
        return (
            len(route_points) > 2
            and self._rear_entry_extension > 0.0
            and self._rounded_corner_radius > 0.0
            and is_right_angle_corner(
                route_points,
                len(route_points) - 2,
                tolerance_degrees=self._right_angle_tolerance_deg,
            )
        )

    def _extend_point(
        self,
        start_point: Point2D,
        end_point: Point2D,
        distance: float,
    ) -> Point2D:
        if distance <= 0.0:
            return end_point

        delta_x = end_point[0] - start_point[0]
        delta_y = end_point[1] - start_point[1]
        length = math.hypot(delta_x, delta_y)
        if length < 1e-9:
            return end_point

        scale = distance / length
        return (
            end_point[0] + delta_x * scale,
            end_point[1] + delta_y * scale,
        )

    def _merge_route_points(
        self,
        first_points: Sequence[Point2D],
        second_points: Sequence[Point2D],
    ) -> List[Point2D]:
        if not first_points:
            return list(second_points)
        if not second_points:
            return list(first_points)

        if math.hypot(
            first_points[-1][0] - second_points[0][0],
            first_points[-1][1] - second_points[0][1],
        ) < 1e-9:
            return list(first_points) + list(second_points[1:])

        return list(first_points) + list(second_points)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
