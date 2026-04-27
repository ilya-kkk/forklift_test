import heapq
import json
import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_templates.srv import StringWithJson
from tf2_ros import Buffer, TransformException, TransformListener


MapData = Dict[str, Any]
Point = Dict[str, Any]
Adjacency = Dict[int, List[Tuple[int, float]]]
RobotPose = Tuple[float, float, float]
PathPoint = Tuple[float, float]


class RouteServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("route_service")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("route_service_name", "/robot_data/route/go_to_point")
        self.declare_parameter("move_to_service_name", "/forklift_nav/move_to")
        self.declare_parameter("follow_path_action_name", "follow_path")
        self.declare_parameter("path_topic", "/route_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("follow_timeout_sec", 120.0)
        self.declare_parameter("nearest_point_tolerance", 0.20)
        self.declare_parameter("transform_timeout_sec", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("yaw_tolerance", 0.05)
        self.declare_parameter("yaw_angular_speed", 0.35)
        self.declare_parameter("yaw_timeout_sec", 20.0)
        self.declare_parameter("yaw_control_frequency", 20.0)

        self._map_service_name = str(self.get_parameter("map_service_name").value)
        route_service_name = str(self.get_parameter("route_service_name").value)
        move_to_service_name = str(self.get_parameter("move_to_service_name").value)
        follow_path_action_name = str(
            self.get_parameter("follow_path_action_name").value
        )
        path_topic = str(self.get_parameter("path_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._follow_timeout_sec = max(
            0.0, float(self.get_parameter("follow_timeout_sec").value)
        )
        self._nearest_point_tolerance = max(
            0.0, float(self.get_parameter("nearest_point_tolerance").value)
        )
        self._transform_timeout_sec = max(
            0.0, float(self.get_parameter("transform_timeout_sec").value)
        )
        self._yaw_tolerance = max(0.0, float(self.get_parameter("yaw_tolerance").value))
        self._yaw_angular_speed = max(
            0.01, float(self.get_parameter("yaw_angular_speed").value)
        )
        self._yaw_timeout_sec = max(
            0.0, float(self.get_parameter("yaw_timeout_sec").value)
        )
        self._yaw_control_frequency = max(
            1.0, float(self.get_parameter("yaw_control_frequency").value)
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_publisher(Path, path_topic, path_qos)
        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._map_client = self.create_client(StringWithJson, self._map_service_name)
        self._follow_path_client = ActionClient(self, FollowPath, follow_path_action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_service(
            StringWithJson, route_service_name, self._handle_route_request
        )
        self.create_service(
            StringWithJson, move_to_service_name, self._handle_move_to_request
        )

        self._lock = threading.Lock()
        self._active_request: Optional[Dict[str, Any]] = None

        self.get_logger().info(
            "Route service ready: route=%s move_to=%s map=%s follow_path=%s "
            "path_topic=%s cmd_vel=%s json_graph=true"
            % (
                route_service_name,
                move_to_service_name,
                self._map_service_name,
                follow_path_action_name,
                path_topic,
                cmd_vel_topic,
            )
        )

    def _handle_route_request(self, request, response):
        try:
            payload = self._decode_json_payload(request.message or "{}")
            pending_request = self._normalize_route_request(payload)
        except ValueError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_request", "details": str(exc)}, ensure_ascii=False
            )
            return response

        return self._accept_request(pending_request, response)

    def _handle_move_to_request(self, request, response):
        try:
            payload = self._decode_move_to_payload(request.message or "{}")
            pending_request = self._normalize_move_to_request(payload)
        except ValueError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_request", "details": str(exc)}, ensure_ascii=False
            )
            return response

        return self._accept_request(pending_request, response)

    def _accept_request(self, pending_request: Dict[str, Any], response):
        with self._lock:
            if self._active_request is not None:
                response.success = False
                response.message = json.dumps(
                    {
                        "error": "busy",
                        "details": "route execution already in progress",
                    },
                    ensure_ascii=False,
                )
                return response
            self._active_request = pending_request

        worker = threading.Thread(
            target=self._execute_route_request,
            args=(pending_request,),
            daemon=True,
        )
        worker.start()

        response.success = True
        response.message = json.dumps(
            {
                "accepted": True,
                "mode": pending_request["mode"],
                "goal": pending_request["goal_value"],
                "planner": "json_map_dijkstra",
                "controller": self._controller_id,
            },
            ensure_ascii=False,
        )
        return response

    def _decode_json_payload(self, message: str) -> Dict[str, Any]:
        try:
            payload = json.loads(message or "{}")
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("request message must be a JSON object")
        return payload

    def _decode_move_to_payload(self, message: str) -> Dict[str, Any]:
        try:
            payload = json.loads(message or "{}")
        except json.JSONDecodeError:
            payload = str(message).strip()

        if isinstance(payload, dict):
            return payload
        return {"goal": payload}

    def _normalize_route_request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
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
        return {
            "mode": "route",
            "start_value": start_value,
            "goal_value": goal_value,
        }

    def _normalize_move_to_request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        goal_value = self._extract_required_value(
            payload,
            ("id", "point_id", "goal", "goal_point", "target", "target_point", "to"),
            "id",
        )
        return {
            "mode": "move_to",
            "goal_value": goal_value,
        }

    def _execute_route_request(self, pending_request: Dict[str, Any]) -> None:
        try:
            resolved_request = self._resolve_route_request(pending_request)
            self._run_route_request(resolved_request)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error("Route execution failed: %s" % exc)
        finally:
            with self._lock:
                self._active_request = None

    def _resolve_route_request(self, pending_request: Dict[str, Any]) -> Dict[str, Any]:
        map_data = self._load_map_data()
        points_by_id, points_by_alias = self._index_points(map_data)
        adjacency = self._build_adjacency(map_data, points_by_id)
        goal_point = self._resolve_point(
            pending_request["goal_value"], points_by_id, points_by_alias
        )

        initial_pose = None
        nearest_distance = None
        if pending_request["mode"] == "move_to":
            current_pose = self._lookup_robot_pose()
            start_point, nearest_distance = self._nearest_point(
                current_pose, points_by_id
            )
            if nearest_distance > self._nearest_point_tolerance:
                initial_pose = current_pose
        else:
            start_point = self._resolve_point(
                pending_request["start_value"], points_by_id, points_by_alias
            )

        route_point_ids = self._shortest_path(
            int(start_point["point_id"]),
            int(goal_point["point_id"]),
            adjacency,
        )
        path = self._build_path(route_point_ids, points_by_id, initial_pose)

        return {
            "mode": pending_request["mode"],
            "path": path,
            "route_point_ids": route_point_ids,
            "start_point": start_point,
            "goal_point": goal_point,
            "nearest_distance": nearest_distance,
            "used_initial_pose": initial_pose is not None,
            "goal_yaw": self._goal_yaw(goal_point),
        }

    def _load_map_data(self) -> MapData:
        if not self._map_client.wait_for_service(timeout_sec=5.0):
            raise ValueError("map service is not available")

        request = StringWithJson.Request()
        request.message = "{}"
        map_response = self._call_service_and_wait(
            self._map_client,
            request,
            timeout_sec=5.0,
        )
        if map_response is None or not map_response.success:
            raise ValueError("map service returned an error")

        try:
            return json.loads(map_response.message or "{}")
        except json.JSONDecodeError as exc:
            raise ValueError(f"map service returned invalid JSON: {exc}") from exc

    def _run_route_request(self, resolved_request: Dict[str, Any]) -> None:
        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath action is not available")
            return

        path = resolved_request["path"]
        start_point = resolved_request["start_point"]
        goal_point = resolved_request["goal_point"]
        goal_yaw = resolved_request["goal_yaw"]

        self._path_publisher.publish(path)
        self._log_route(resolved_request, path)

        if len(path.poses) > 1:
            if not self._follow_path(path):
                return
        else:
            self.get_logger().info("Route execution skipped: already at drive goal")

        if goal_yaw is not None and not self._rotate_to_yaw(goal_yaw):
            return

        self.get_logger().info(
            "Route execution finished successfully at %s (%d)"
            % (goal_point["alias"], int(goal_point["point_id"]))
        )

    def _follow_path(self, path: Path) -> bool:
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = self._controller_id
        follow_goal.goal_checker_id = self._goal_checker_id
        follow_result = self._send_action_goal_and_wait(
            self._follow_path_client,
            follow_goal,
            timeout_sec=self._follow_timeout_sec,
        )
        if follow_result is None:
            self.get_logger().error("FollowPath action failed or timed out")
            return False
        return True

    def _log_route(self, resolved_request: Dict[str, Any], path: Path) -> None:
        start_point = resolved_request["start_point"]
        goal_point = resolved_request["goal_point"]
        mode = resolved_request["mode"]

        if mode == "move_to":
            nearest_distance = float(resolved_request["nearest_distance"])
            if resolved_request["used_initial_pose"]:
                self.get_logger().info(
                    "MoveTo snapped current pose to nearest JSON point %s (%d), distance=%.3fm"
                    % (
                        start_point["alias"],
                        int(start_point["point_id"]),
                        nearest_distance,
                    )
                )
            else:
                self.get_logger().info(
                    "MoveTo starts from nearest JSON point %s (%d), distance=%.3fm"
                    % (
                        start_point["alias"],
                        int(start_point["point_id"]),
                        nearest_distance,
                    )
                )

        self.get_logger().info(
            "Following JSON route from %s (%d) to %s (%d), poses=%d"
            % (
                start_point["alias"],
                int(start_point["point_id"]),
                goal_point["alias"],
                int(goal_point["point_id"]),
                len(path.poses),
            )
        )

        if resolved_request["goal_yaw"] is not None:
            self.get_logger().info(
                "Goal %s has explicit yaw=%.3f rad; final rotation will run after path"
                % (goal_point["alias"], float(resolved_request["goal_yaw"]))
            )

    def _index_points(
        self, map_data: MapData
    ) -> Tuple[Dict[int, Point], Dict[str, Point]]:
        points = list(map_data.get("point", []))
        if not points:
            raise ValueError("map contains no points")

        points_by_id: Dict[int, Point] = {}
        points_by_alias: Dict[str, Point] = {}
        for point in points:
            point_id = int(point["point_id"])
            alias = str(point["alias"])
            points_by_id[point_id] = point
            points_by_alias[alias] = point

        return points_by_id, points_by_alias

    def _resolve_point(
        self,
        value: Any,
        points_by_id: Dict[int, Point],
        points_by_alias: Dict[str, Point],
    ) -> Point:
        if isinstance(value, dict):
            if "alias" in value:
                return self._resolve_point(value["alias"], points_by_id, points_by_alias)
            if "point_id" in value:
                return self._resolve_point(
                    value["point_id"], points_by_id, points_by_alias
                )
            raise ValueError("point object must contain 'alias' or 'point_id'")

        if isinstance(value, int):
            if value in points_by_id:
                return points_by_id[value]
            raise ValueError(f"unknown point_id {value}")

        normalized = str(value).strip()
        if normalized in points_by_alias:
            return points_by_alias[normalized]
        if normalized.isdigit():
            point_id = int(normalized)
            if point_id in points_by_id:
                return points_by_id[point_id]

        raise ValueError(f"unknown point '{value}'")

    def _build_adjacency(
        self, map_data: MapData, points_by_id: Dict[int, Point]
    ) -> Adjacency:
        adjacency: Adjacency = {point_id: [] for point_id in points_by_id}
        for route_edge in map_data.get("path", []):
            if not self._is_released(route_edge):
                continue

            try:
                start_point_id = int(route_edge["start_point_id"])
                end_point_id = int(route_edge["end_point_id"])
            except (KeyError, TypeError, ValueError):
                continue

            if start_point_id not in points_by_id or end_point_id not in points_by_id:
                continue

            weight = self._point_distance(
                points_by_id[start_point_id], points_by_id[end_point_id]
            )
            adjacency[start_point_id].append((end_point_id, weight))

        return adjacency

    def _nearest_point(
        self, robot_pose: RobotPose, points_by_id: Dict[int, Point]
    ) -> Tuple[Point, float]:
        if not points_by_id:
            raise ValueError("map contains no points")

        robot_x, robot_y, _ = robot_pose
        nearest_point = min(
            points_by_id.values(),
            key=lambda point: self._distance_xy(
                robot_x,
                robot_y,
                float(point["position_x"]),
                float(point["position_y"]),
            ),
        )
        nearest_distance = self._distance_xy(
            robot_x,
            robot_y,
            float(nearest_point["position_x"]),
            float(nearest_point["position_y"]),
        )
        return nearest_point, nearest_distance

    def _shortest_path(
        self, start_point_id: int, goal_point_id: int, adjacency: Adjacency
    ) -> List[int]:
        if start_point_id not in adjacency:
            raise ValueError(f"start point_id {start_point_id} is not in JSON map")
        if goal_point_id not in adjacency:
            raise ValueError(f"goal point_id {goal_point_id} is not in JSON map")

        distances = {start_point_id: 0.0}
        previous: Dict[int, Optional[int]] = {start_point_id: None}
        queue = [(0.0, start_point_id)]

        while queue:
            distance, point_id = heapq.heappop(queue)
            if distance > distances.get(point_id, math.inf):
                continue
            if point_id == goal_point_id:
                break

            for neighbor_id, edge_weight in adjacency.get(point_id, []):
                new_distance = distance + edge_weight
                if new_distance >= distances.get(neighbor_id, math.inf):
                    continue
                distances[neighbor_id] = new_distance
                previous[neighbor_id] = point_id
                heapq.heappush(queue, (new_distance, neighbor_id))

        if goal_point_id not in previous:
            raise ValueError(
                "no JSON route from point_id %d to point_id %d"
                % (start_point_id, goal_point_id)
            )

        route_point_ids = []
        current: Optional[int] = goal_point_id
        while current is not None:
            route_point_ids.append(current)
            current = previous[current]
        route_point_ids.reverse()
        return route_point_ids

    def _build_path(
        self,
        route_point_ids: List[int],
        points_by_id: Dict[int, Point],
        initial_pose: Optional[RobotPose],
    ) -> Path:
        path_points: List[PathPoint] = []
        if initial_pose is not None:
            path_points.append((initial_pose[0], initial_pose[1]))

        for point_id in route_point_ids:
            point = points_by_id[point_id]
            path_points.append(
                (float(point["position_x"]), float(point["position_y"]))
            )

        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for index, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            yaw = self._path_point_yaw(path_points, index)
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = self._quaternion_from_yaw(yaw)
            path.poses.append(pose)
        return path

    def _path_point_yaw(self, path_points: List[PathPoint], index: int) -> float:
        if index < len(path_points) - 1:
            return self._yaw_between_xy(path_points[index], path_points[index + 1])
        if index > 0:
            return self._yaw_between_xy(path_points[index - 1], path_points[index])
        return 0.0

    def _yaw_between_xy(self, start: PathPoint, end: PathPoint) -> float:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if math.hypot(dx, dy) <= 1e-9:
            return 0.0
        return math.atan2(dy, dx)

    def _point_distance(self, lhs: Point, rhs: Point) -> float:
        return self._distance_xy(
            float(lhs["position_x"]),
            float(lhs["position_y"]),
            float(rhs["position_x"]),
            float(rhs["position_y"]),
        )

    def _distance_xy(self, ax: float, ay: float, bx: float, by: float) -> float:
        return math.hypot(ax - bx, ay - by)

    def _goal_yaw(self, point: Point) -> Optional[float]:
        for key in ("yaw", "goal_yaw", "arrival_yaw"):
            if key not in point:
                continue
            try:
                return float(point[key])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "point %s has invalid %s value: %r"
                    % (point.get("alias", point.get("point_id")), key, point[key])
                ) from exc
        return None

    def _rotate_to_yaw(self, target_yaw: float) -> bool:
        deadline = time.monotonic() + self._yaw_timeout_sec
        period = 1.0 / self._yaw_control_frequency

        while rclpy.ok():
            try:
                robot_pose = self._lookup_robot_pose(log_errors=False)
            except ValueError:
                if self._yaw_timeout_sec > 0.0 and time.monotonic() > deadline:
                    self._publish_stop()
                    self.get_logger().error(
                        "Final yaw rotation timed out: robot pose is not available"
                    )
                    return False
                time.sleep(period)
                continue

            _, _, current_yaw = robot_pose
            delta_yaw = self._normalize_angle(target_yaw - current_yaw)
            if abs(delta_yaw) <= self._yaw_tolerance:
                self._publish_stop()
                self.get_logger().info(
                    "Final yaw reached: current=%.3f target=%.3f"
                    % (current_yaw, target_yaw)
                )
                return True

            if self._yaw_timeout_sec > 0.0 and time.monotonic() > deadline:
                self._publish_stop()
                self.get_logger().error(
                    "Final yaw rotation timed out: current=%.3f target=%.3f"
                    % (current_yaw, target_yaw)
                )
                return False

            twist = Twist()
            twist.angular.z = math.copysign(self._yaw_angular_speed, delta_yaw)
            self._cmd_vel_publisher.publish(twist)
            time.sleep(period)

        self._publish_stop()
        return False

    def _publish_stop(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _lookup_robot_pose(self, *, log_errors: bool = True) -> RobotPose:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._robot_base_frame,
                Time(),
                timeout=RclpyDuration(seconds=self._transform_timeout_sec),
            )
        except TransformException as exc:
            if log_errors:
                self.get_logger().error(
                    "Failed to lookup %s -> %s transform: %s"
                    % (self._frame_id, self._robot_base_frame, exc)
                )
            raise ValueError("robot pose is not available") from exc

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _is_released(self, route_edge: Dict[str, Any]) -> bool:
        if "released" not in route_edge:
            return True
        return self._is_truthy(route_edge["released"])

    def _is_truthy(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in {
                "1",
                "true",
                "yes",
                "y",
                "on",
            }
        return False

    def _quaternion_from_yaw(self, yaw: float) -> Tuple[float, float, float, float]:
        half_yaw = yaw * 0.5
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def _call_service_and_wait(self, client, request, *, timeout_sec: float):
        response_event = threading.Event()
        state: Dict[str, Any] = {}

        def _response_callback(future) -> None:
            try:
                state["response"] = future.result()
            except Exception as exc:  # pragma: no cover - defensive logging
                state["error"] = str(exc)
            finally:
                response_event.set()

        future = client.call_async(request)
        future.add_done_callback(_response_callback)

        if not response_event.wait(timeout=timeout_sec if timeout_sec > 0.0 else None):
            return None

        if "error" in state:
            raise RuntimeError(state["error"])

        return state.get("response")

    def _send_action_goal_and_wait(
        self,
        client: ActionClient,
        goal,
        *,
        timeout_sec: float,
    ):
        goal_response_event = threading.Event()
        result_event = threading.Event()
        state: Dict[str, Any] = {}

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:  # pragma: no cover - defensive logging
                state["goal_error"] = str(exc)
                goal_response_event.set()
                result_event.set()
                return

            state["goal_handle"] = goal_handle
            goal_response_event.set()
            if goal_handle is None or not goal_handle.accepted:
                result_event.set()
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                state["result_response"] = future.result()
            except Exception as exc:  # pragma: no cover - defensive logging
                state["result_error"] = str(exc)
            result_event.set()

        client.send_goal_async(goal).add_done_callback(_goal_response_callback)

        if not goal_response_event.wait(timeout=timeout_sec if timeout_sec > 0.0 else None):
            return None
        if "goal_error" in state:
            self.get_logger().error("Action goal send failed: %s" % state["goal_error"])
            return None

        goal_handle = state.get("goal_handle")
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Action goal was rejected")
            return None

        if not result_event.wait(timeout=timeout_sec if timeout_sec > 0.0 else None):
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - defensive logging
                pass
            return None

        if "result_error" in state:
            self.get_logger().error(
                "Action result retrieval failed: %s" % state["result_error"]
            )
            return None

        result_response = state.get("result_response")
        if result_response is None:
            return None
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "Action finished with status %s" % result_response.status
            )
            return None

        return result_response.result

    def _extract_required_value(
        self, payload: Dict[str, Any], keys, label: str
    ) -> Any:
        for key in keys:
            if key in payload:
                return payload[key]
        raise ValueError("missing '%s' field" % label)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
