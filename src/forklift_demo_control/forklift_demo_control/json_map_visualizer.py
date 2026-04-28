import json
import math
import threading
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import Point as GeoPoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros2_templates.srv import StringWithJson
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


MapData = Dict[str, Any]
PointData = Dict[str, Any]


class JsonMapVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("json_map_visualizer")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("refresh_service_name", "/robot_data/map/visualize")
        self.declare_parameter("marker_topic", "/json_map/markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_sec", 0.0)
        self.declare_parameter("deduplicate_bidirectional_paths", True)
        self.declare_parameter("draw_labels", True)
        self.declare_parameter("point_scale", 0.14)
        self.declare_parameter("path_width", 0.045)

        self._map_service_name = str(self.get_parameter("map_service_name").value)
        refresh_service_name = str(self.get_parameter("refresh_service_name").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_period_sec = max(
            0.0, float(self.get_parameter("publish_period_sec").value)
        )
        self._deduplicate_bidirectional_paths = bool(
            self.get_parameter("deduplicate_bidirectional_paths").value
        )
        self._draw_labels = bool(self.get_parameter("draw_labels").value)
        self._point_scale = max(0.01, float(self.get_parameter("point_scale").value))
        self._path_width = max(0.001, float(self.get_parameter("path_width").value))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, marker_topic, qos
        )
        self._map_client = self.create_client(StringWithJson, self._map_service_name)
        self.create_service(
            StringWithJson, refresh_service_name, self._handle_refresh_request
        )

        self._refresh_lock = threading.Lock()
        self._refresh_in_progress = False

        if publish_period_sec > 0.0:
            self.create_timer(publish_period_sec, self._request_refresh)
        self._request_refresh()

        self.get_logger().info(
            "JSON map visualizer ready: map=%s markers=%s refresh=%s"
            % (self._map_service_name, marker_topic, refresh_service_name)
        )

    def _handle_refresh_request(self, request, response):
        del request
        accepted = self._request_refresh()
        response.success = accepted
        response.message = json.dumps(
            {
                "accepted": accepted,
                "details": "refresh started"
                if accepted
                else "refresh already in progress",
            },
            ensure_ascii=False,
        )
        return response

    def _request_refresh(self) -> bool:
        with self._refresh_lock:
            if self._refresh_in_progress:
                return False
            self._refresh_in_progress = True

        worker = threading.Thread(target=self._refresh_worker, daemon=True)
        worker.start()
        return True

    def _refresh_worker(self) -> None:
        try:
            map_data = self._load_map_data()
            markers = self._build_marker_array(map_data)
            self._marker_publisher.publish(markers)
            self.get_logger().info(
                "Published JSON map markers: points=%d paths=%d"
                % (
                    len(map_data.get("point", [])),
                    len(map_data.get("path", [])),
                )
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error("Failed to publish JSON map markers: %s" % exc)
        finally:
            with self._refresh_lock:
                self._refresh_in_progress = False

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

    def _build_marker_array(self, map_data: MapData) -> MarkerArray:
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker())

        points_by_id = self._index_points(map_data)
        marker_array.markers.append(self._path_marker(map_data, points_by_id))
        marker_array.markers.extend(self._point_markers(points_by_id))
        marker_array.markers.extend(self._yaw_markers(points_by_id))

        if self._draw_labels:
            marker_array.markers.extend(self._label_markers(points_by_id))

        return marker_array

    def _delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _path_marker(
        self, map_data: MapData, points_by_id: Dict[int, PointData]
    ) -> Marker:
        marker = self._base_marker("json_map_paths", 1)
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self._path_width
        marker.color = self._color(0.62, 0.68, 0.72, 0.9)

        seen_edges = set()
        for route_edge in map_data.get("path", []):
            if not self._is_truthy(route_edge.get("released", True)):
                continue

            try:
                start_id = int(route_edge["start_point_id"])
                end_id = int(route_edge["end_point_id"])
            except (KeyError, TypeError, ValueError):
                continue

            if start_id not in points_by_id or end_id not in points_by_id:
                continue

            edge_key = (
                tuple(sorted((start_id, end_id)))
                if self._deduplicate_bidirectional_paths
                else (start_id, end_id)
            )
            if edge_key in seen_edges:
                continue
            seen_edges.add(edge_key)

            marker.points.append(self._geo_point(points_by_id[start_id], z=0.03))
            marker.points.append(self._geo_point(points_by_id[end_id], z=0.03))

        return marker

    def _point_markers(self, points_by_id: Dict[int, PointData]) -> List[Marker]:
        markers = []
        for point_id, point in sorted(points_by_id.items()):
            marker = self._base_marker("json_map_points", point_id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = self._geo_point(point, z=0.12)
            marker.pose.orientation.w = 1.0
            marker.scale.x = self._point_scale
            marker.scale.y = self._point_scale
            marker.scale.z = self._point_scale
            marker.color = self._point_color(point)
            markers.append(marker)
        return markers

    def _label_markers(self, points_by_id: Dict[int, PointData]) -> List[Marker]:
        markers = []
        for point_id, point in sorted(points_by_id.items()):
            marker = self._base_marker("json_map_labels", point_id)
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = self._geo_point(point, z=0.42)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.22
            marker.color = self._color(0.92, 0.96, 1.0, 0.95)
            marker.text = str(point.get("alias", point_id))
            markers.append(marker)
        return markers

    def _yaw_markers(self, points_by_id: Dict[int, PointData]) -> List[Marker]:
        markers = []
        for point_id, point in sorted(points_by_id.items()):
            yaw = self._point_yaw(point)
            if yaw is None:
                continue

            marker = self._base_marker("json_map_yaw", point_id)
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points.append(self._geo_point(point, z=0.12))
            marker.points.append(self._yaw_end_point(point, yaw, length=0.35, z=0.12))
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.10
            marker.color = self._point_color(point)
            markers.append(marker)
        return markers

    def _base_marker(self, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = int(marker_id)
        return marker

    def _index_points(self, map_data: MapData) -> Dict[int, PointData]:
        points_by_id: Dict[int, PointData] = {}
        for point in map_data.get("point", []):
            try:
                points_by_id[int(point["point_id"])] = point
            except (KeyError, TypeError, ValueError):
                continue
        return points_by_id

    def _point_color(self, point: PointData) -> ColorRGBA:
        alias = str(point.get("alias", ""))
        if alias == "0000":
            return self._color(1.0, 0.84, 0.18, 1.0)
        return self._color(0.10, 0.48, 1.0, 1.0)

    def _geo_point(self, point: PointData, *, z: float) -> GeoPoint:
        geo_point = GeoPoint()
        geo_point.x = float(point["position_x"])
        geo_point.y = float(point["position_y"])
        geo_point.z = float(z)
        return geo_point

    def _yaw_end_point(
        self, point: PointData, yaw: float, *, length: float, z: float
    ) -> GeoPoint:
        geo_point = self._geo_point(point, z=z)
        geo_point.x += math.cos(yaw) * length
        geo_point.y += math.sin(yaw) * length
        return geo_point

    def _point_yaw(self, point: PointData) -> float | None:
        if "yaw" not in point:
            return None
        try:
            return float(point["yaw"])
        except (TypeError, ValueError):
            return None

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

    def _color(self, red: float, green: float, blue: float, alpha: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(red)
        color.g = float(green)
        color.b = float(blue)
        color.a = float(alpha)
        return color

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JsonMapVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
