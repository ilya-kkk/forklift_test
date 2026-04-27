import json
import math
import threading
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import BackUp, ComputeRoute, FollowPath, Spin
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_templates.srv import StringWithJson
from tf2_ros import Buffer, TransformException, TransformListener

from forklift_demo_control.route_graph import index_points, resolve_point


class RouteServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("route_service")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("route_service_name", "/robot_data/route/go_to_point")
        self.declare_parameter("compute_route_action_name", "compute_route")
        self.declare_parameter("follow_path_action_name", "follow_path")
        self.declare_parameter("backup_action_name", "backup")
        self.declare_parameter("spin_action_name", "spin")
        self.declare_parameter("path_topic", "/route_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("compute_timeout_sec", 10.0)
        self.declare_parameter("follow_timeout_sec", 120.0)
        self.declare_parameter("backup_timeout_sec", 30.0)
        self.declare_parameter("reverse_entry_spin_timeout_sec", 20.0)
        self.declare_parameter("reverse_entry_yaw_tolerance", 0.05)
        self.declare_parameter("reverse_entry_enabled", True)
        self.declare_parameter("reverse_entry_speed", 0.10)
        self.declare_parameter("reverse_entry_min_distance", 0.05)
        self.declare_parameter("graph_filepath", "/tmp/forklift_demo_route_graph.geojson")
        self.declare_parameter("robot_base_frame", "base_link")

        self._map_service_name = str(self.get_parameter("map_service_name").value)
        route_service_name = str(self.get_parameter("route_service_name").value)
        compute_route_action_name = str(
            self.get_parameter("compute_route_action_name").value
        )
        follow_path_action_name = str(
            self.get_parameter("follow_path_action_name").value
        )
        backup_action_name = str(self.get_parameter("backup_action_name").value)
        spin_action_name = str(self.get_parameter("spin_action_name").value)
        path_topic = str(self.get_parameter("path_topic").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._compute_timeout_sec = max(
            0.0, float(self.get_parameter("compute_timeout_sec").value)
        )
        self._follow_timeout_sec = max(
            0.0, float(self.get_parameter("follow_timeout_sec").value)
        )
        self._backup_timeout_sec = max(
            0.0, float(self.get_parameter("backup_timeout_sec").value)
        )
        self._reverse_entry_spin_timeout_sec = max(
            0.0, float(self.get_parameter("reverse_entry_spin_timeout_sec").value)
        )
        self._reverse_entry_yaw_tolerance = max(
            0.0, float(self.get_parameter("reverse_entry_yaw_tolerance").value)
        )
        self._reverse_entry_enabled = bool(
            self.get_parameter("reverse_entry_enabled").value
        )
        self._reverse_entry_speed = max(
            0.01, float(self.get_parameter("reverse_entry_speed").value)
        )
        self._reverse_entry_min_distance = max(
            0.0, float(self.get_parameter("reverse_entry_min_distance").value)
        )
        self._graph_filepath = str(self.get_parameter("graph_filepath").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_publisher(Path, path_topic, qos)
        self._map_client = self.create_client(StringWithJson, self._map_service_name)
        self._compute_route_client = ActionClient(
            self, ComputeRoute, compute_route_action_name
        )
        self._follow_path_client = ActionClient(self, FollowPath, follow_path_action_name)
        self._backup_client = ActionClient(self, BackUp, backup_action_name)
        self._spin_client = ActionClient(self, Spin, spin_action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_service(
            StringWithJson, route_service_name, self._handle_route_request
        )

        self._lock = threading.Lock()
        self._active_request: Optional[Dict[str, Any]] = None

        self.get_logger().info(
            "Route service ready: route=%s map=%s compute_route=%s follow_path=%s "
            "backup=%s spin=%s path_topic=%s reverse_entry=%s"
            % (
                route_service_name,
                self._map_service_name,
                compute_route_action_name,
                follow_path_action_name,
                backup_action_name,
                spin_action_name,
                path_topic,
                str(self._reverse_entry_enabled).lower(),
            )
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
            pending_request = self._normalize_route_request(payload)
        except ValueError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_request", "details": str(exc)}, ensure_ascii=False
            )
            return response

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
                "start": pending_request["start_value"],
                "goal": pending_request["goal_value"],
                "arrival_mode": pending_request["arrival_mode"],
                "note": "route accepted; pallet/reverse-entry goals split the final edge into Spin + BackUp",
            },
            ensure_ascii=False,
        )
        return response

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
        arrival_mode = self._normalize_arrival_mode(payload.get("arrival_mode", "rear"))
        return {
            "start_value": start_value,
            "goal_value": goal_value,
            "arrival_mode": arrival_mode,
        }

    def _resolve_route_request(self, pending_request: Dict[str, Any]) -> Dict[str, Any]:
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
            map_data = json.loads(map_response.message or "{}")
        except json.JSONDecodeError as exc:
            raise ValueError(f"map service returned invalid JSON: {exc}") from exc

        points_by_id, points_by_alias = index_points(map_data)
        start_point = resolve_point(
            pending_request["start_value"], points_by_id, points_by_alias
        )
        goal_point = resolve_point(
            pending_request["goal_value"], points_by_id, points_by_alias
        )
        reverse_entry_point_ids = self._collect_reverse_entry_point_ids(map_data)
        return {
            "start_point_id": int(start_point["point_id"]),
            "start_alias": str(start_point["alias"]),
            "goal_point_id": int(goal_point["point_id"]),
            "goal_alias": str(goal_point["alias"]),
            "arrival_mode": pending_request["arrival_mode"],
            "goal_requires_reverse_entry": int(goal_point["point_id"])
            in reverse_entry_point_ids,
        }

    def _collect_reverse_entry_point_ids(self, map_data: Dict[str, Any]) -> set[int]:
        point_ids = {
            int(point["point_id"])
            for point in map_data.get("point", [])
            if "point_id" in point and self._point_requires_reverse_entry(point)
        }
        point_ids.update(self._collect_reverse_entry_point_ids_from_graph())
        return point_ids

    def _collect_reverse_entry_point_ids_from_graph(self) -> set[int]:
        if not self._graph_filepath:
            return set()
        try:
            with open(self._graph_filepath, "r", encoding="utf-8") as graph_file:
                graph = json.load(graph_file)
        except FileNotFoundError:
            return set()
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                "Failed to read route graph metadata from %s: %s"
                % (self._graph_filepath, exc)
            )
            return set()

        point_ids = set()
        for feature in graph.get("features", []):
            if feature.get("geometry", {}).get("type") != "Point":
                continue
            properties = feature.get("properties", {})
            metadata = properties.get("metadata", {})
            if self._point_requires_reverse_entry(properties) or (
                isinstance(metadata, dict)
                and self._point_requires_reverse_entry(metadata)
            ):
                try:
                    point_ids.add(int(properties["id"]))
                except (KeyError, TypeError, ValueError):
                    continue
        return point_ids

    def _point_requires_reverse_entry(self, point: Dict[str, Any]) -> bool:
        for key in (
            "reverse_entry",
            "reverse_final_edge",
            "pallet",
            "has_pallet",
            "pallet_present",
            "contains_pallet",
            "occupied_by_pallet",
        ):
            if key in point and self._is_truthy(point[key]):
                return True

        metadata = point.get("metadata")
        if isinstance(metadata, dict):
            return self._point_requires_reverse_entry(metadata)
        return False

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
                "pallet",
                "occupied",
            }
        return False

    def _execute_route_request(self, pending_request: Dict[str, Any]) -> None:
        try:
            resolved_request = self._resolve_route_request(pending_request)
            self._run_route_request(resolved_request)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error("Route execution failed: %s" % exc)
        finally:
            with self._lock:
                self._active_request = None

    def _run_route_request(self, resolved_request: Dict[str, Any]) -> None:
        if not self._compute_route_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Route Server action is not available")
            return

        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath action is not available")
            return

        if (
            self._reverse_entry_enabled
            and resolved_request["goal_requires_reverse_entry"]
        ):
            if not self._spin_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Spin action is not available")
                return
            if not self._backup_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("BackUp action is not available")
                return

        compute_goal = ComputeRoute.Goal()
        compute_goal.start_id = resolved_request["start_point_id"]
        compute_goal.goal_id = resolved_request["goal_point_id"]
        compute_goal.use_start = True
        compute_goal.use_poses = False

        self.get_logger().info(
            "Computing route from %s (%d) to %s (%d)"
            % (
                resolved_request["start_alias"],
                resolved_request["start_point_id"],
                resolved_request["goal_alias"],
                resolved_request["goal_point_id"],
            )
        )
        if (
            self._reverse_entry_enabled
            and resolved_request["goal_requires_reverse_entry"]
        ):
            self.get_logger().info(
                "Goal %s (%d) is marked for reverse entry; final route edge will use BackUp"
                % (
                    resolved_request["goal_alias"],
                    resolved_request["goal_point_id"],
                )
            )
        compute_result = self._send_action_goal_and_wait(
            self._compute_route_client,
            compute_goal,
            timeout_sec=self._compute_timeout_sec,
        )
        if compute_result is None:
            self.get_logger().error("ComputeRoute action failed or timed out")
            return

        path = compute_result.path
        if not path.poses:
            self.get_logger().error("Route Server returned an empty path")
            return

        self._path_publisher.publish(path)
        self.get_logger().info(
            "Route Server returned %d path poses and %d route nodes"
            % (len(path.poses), len(compute_result.route.nodes))
        )

        if (
            self._reverse_entry_enabled
            and resolved_request["goal_requires_reverse_entry"]
        ):
            if not self._run_reverse_entry_route(path, compute_result.route):
                self.get_logger().error("Reverse-entry route execution failed")
                return
        elif not self._follow_path(path):
            return

        self.get_logger().info("Route execution finished successfully")

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

    def _run_reverse_entry_route(self, path: Path, route) -> bool:
        final_edge_distance = self._final_route_edge_distance(route)
        if final_edge_distance is None:
            self.get_logger().error("Reverse-entry requested but route has no final edge")
            return False
        if final_edge_distance < self._reverse_entry_min_distance:
            self.get_logger().info(
                "Skipping reverse entry: final edge %.3fm is shorter than %.3fm"
                % (final_edge_distance, self._reverse_entry_min_distance)
            )
            return self._follow_path(path)

        prefix_path = self._path_before_final_edge(path, route)
        if prefix_path is not None and len(prefix_path.poses) >= 2:
            self.get_logger().info(
                "Following route prefix (%d poses), then backing up %.3fm into pallet point"
                % (len(prefix_path.poses), final_edge_distance)
            )
            if not self._follow_path(prefix_path):
                return False
        else:
            self.get_logger().info(
                "No route prefix before pallet entry; backing up %.3fm from current pose"
                % final_edge_distance
            )

        if not self._align_for_reverse_entry(route):
            return False

        backup_goal = BackUp.Goal()
        backup_goal.target.x = float(final_edge_distance)
        backup_goal.target.y = 0.0
        backup_goal.target.z = 0.0
        backup_goal.speed = float(self._reverse_entry_speed)
        backup_goal.time_allowance = self._duration_from_seconds(
            self._backup_timeout_sec
        )
        backup_result = self._send_action_goal_and_wait(
            self._backup_client,
            backup_goal,
            timeout_sec=self._backup_timeout_sec,
        )
        if backup_result is None:
            self.get_logger().error("BackUp action failed or timed out")
            return False
        if getattr(backup_result, "error_code", 0) != BackUp.Result.NONE:
            self.get_logger().error(
                "BackUp action returned error %s: %s"
                % (
                    getattr(backup_result, "error_code", "<unknown>"),
                    getattr(backup_result, "error_msg", ""),
                )
            )
            return False
        return True

    def _align_for_reverse_entry(self, route) -> bool:
        desired_yaw = self._reverse_entry_yaw(route)
        if desired_yaw is None:
            self.get_logger().error("Cannot compute reverse-entry yaw without final edge")
            return False

        frame_id = route.header.frame_id or "map"
        current_yaw = self._lookup_robot_yaw(frame_id)
        if current_yaw is None:
            return False

        delta_yaw = self._normalize_angle(desired_yaw - current_yaw)
        if abs(delta_yaw) <= self._reverse_entry_yaw_tolerance:
            self.get_logger().info(
                "Reverse-entry yaw already aligned: desired=%.3f current=%.3f"
                % (desired_yaw, current_yaw)
            )
            return True

        self.get_logger().info(
            "Aligning for reverse entry in %s: current=%.3f desired=%.3f spin=%.3f"
            % (frame_id, current_yaw, desired_yaw, delta_yaw)
        )
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(delta_yaw)
        spin_goal.time_allowance = self._duration_from_seconds(
            self._reverse_entry_spin_timeout_sec
        )
        spin_result = self._send_action_goal_and_wait(
            self._spin_client,
            spin_goal,
            timeout_sec=self._reverse_entry_spin_timeout_sec,
        )
        if spin_result is None:
            self.get_logger().error("Spin action failed or timed out")
            return False
        if getattr(spin_result, "error_code", 0) != Spin.Result.NONE:
            self.get_logger().error(
                "Spin action returned error %s: %s"
                % (
                    getattr(spin_result, "error_code", "<unknown>"),
                    getattr(spin_result, "error_msg", ""),
                )
            )
            return False
        return True

    def _reverse_entry_yaw(self, route) -> Optional[float]:
        if not getattr(route, "edges", None):
            return None
        final_edge = route.edges[-1]
        dx = float(final_edge.start.x - final_edge.end.x)
        dy = float(final_edge.start.y - final_edge.end.y)
        if math.hypot(dx, dy) <= 1e-9:
            return None
        return math.atan2(dy, dx)

    def _lookup_robot_yaw(self, target_frame: str) -> Optional[float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                self._robot_base_frame,
                Time(),
                timeout=RclpyDuration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().error(
                "Failed to lookup %s -> %s transform for reverse entry: %s"
                % (target_frame, self._robot_base_frame, exc)
            )
            return None

        rotation = transform.transform.rotation
        return self._yaw_from_quaternion(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _final_route_edge_distance(self, route) -> Optional[float]:
        if not getattr(route, "edges", None):
            return None
        final_edge = route.edges[-1]
        dx = float(final_edge.end.x - final_edge.start.x)
        dy = float(final_edge.end.y - final_edge.start.y)
        return math.hypot(dx, dy)

    def _path_before_final_edge(self, path: Path, route) -> Optional[Path]:
        if not path.poses or not getattr(route, "edges", None):
            return None

        final_edge = route.edges[-1]
        split_index = min(
            range(len(path.poses)),
            key=lambda index: self._distance_sq_to_point(
                path.poses[index].pose.position,
                final_edge.start,
            ),
        )
        if split_index <= 0:
            return None

        prefix_path = Path()
        prefix_path.header = path.header
        prefix_path.poses = list(path.poses[: split_index + 1])
        return prefix_path

    def _distance_sq_to_point(self, lhs, rhs) -> float:
        dx = float(lhs.x - rhs.x)
        dy = float(lhs.y - rhs.y)
        return dx * dx + dy * dy

    def _duration_from_seconds(self, seconds: float) -> Duration:
        whole_seconds = int(max(0.0, seconds))
        nanoseconds = int((max(0.0, seconds) - whole_seconds) * 1_000_000_000)
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
            self.get_logger().error("Action result retrieval failed: %s" % state["result_error"])
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
            return "front"
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
            return "rear"
        raise ValueError("unsupported arrival_mode '%s'" % value)


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
