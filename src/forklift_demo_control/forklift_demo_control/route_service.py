import json
import threading
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputeRoute, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros2_templates.srv import StringWithJson

from forklift_demo_control.route_graph import index_points, resolve_point


class RouteServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("route_service")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("route_service_name", "/robot_data/route/go_to_point")
        self.declare_parameter("compute_route_action_name", "compute_route")
        self.declare_parameter("follow_path_action_name", "follow_path")
        self.declare_parameter("path_topic", "/route_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("compute_timeout_sec", 10.0)
        self.declare_parameter("follow_timeout_sec", 120.0)

        self._map_service_name = str(self.get_parameter("map_service_name").value)
        route_service_name = str(self.get_parameter("route_service_name").value)
        compute_route_action_name = str(
            self.get_parameter("compute_route_action_name").value
        )
        follow_path_action_name = str(
            self.get_parameter("follow_path_action_name").value
        )
        path_topic = str(self.get_parameter("path_topic").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._compute_timeout_sec = max(
            0.0, float(self.get_parameter("compute_timeout_sec").value)
        )
        self._follow_timeout_sec = max(
            0.0, float(self.get_parameter("follow_timeout_sec").value)
        )

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
        self.create_service(
            StringWithJson, route_service_name, self._handle_route_request
        )

        self._lock = threading.Lock()
        self._active_request: Optional[Dict[str, Any]] = None

        self.get_logger().info(
            "Route service ready: route=%s map=%s compute_route=%s follow_path=%s path_topic=%s"
            % (
                route_service_name,
                self._map_service_name,
                compute_route_action_name,
                follow_path_action_name,
                path_topic,
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
                "note": "arrival_mode is accepted for compatibility, but the Route Server pipeline executes a single forward path",
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
        return {
            "start_point_id": int(start_point["point_id"]),
            "start_alias": str(start_point["alias"]),
            "goal_point_id": int(goal_point["point_id"]),
            "goal_alias": str(goal_point["alias"]),
            "arrival_mode": pending_request["arrival_mode"],
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

    def _run_route_request(self, resolved_request: Dict[str, Any]) -> None:
        if not self._compute_route_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Route Server action is not available")
            return

        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath action is not available")
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
            return

        self.get_logger().info("Route execution finished successfully")

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
