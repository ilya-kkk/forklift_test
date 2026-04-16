import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from ros2_templates.srv import StringWithJson

from forklift_demo_control.route_graph import build_geojson_route_graph


class RouteGraphBuilder(Node):
    def __init__(self) -> None:
        super().__init__("route_graph_builder")

        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        self.declare_parameter("graph_filepath", "/tmp/forklift_demo_route_graph.geojson")
        self.declare_parameter("graph_frame_id", "map")
        self.declare_parameter("graph_name", "forklift_demo_route_graph")
        self.declare_parameter("wait_for_service_sec", 30.0)

        self._map_service_name = str(self.get_parameter("map_service_name").value)
        self._graph_filepath = Path(str(self.get_parameter("graph_filepath").value))
        self._graph_frame_id = str(self.get_parameter("graph_frame_id").value)
        self._graph_name = str(self.get_parameter("graph_name").value)
        self._wait_for_service_sec = max(
            0.0, float(self.get_parameter("wait_for_service_sec").value)
        )

        self._map_client = self.create_client(StringWithJson, self._map_service_name)

    def run(self) -> int:
        self.get_logger().info(
            "Waiting for map service %s to generate %s"
            % (self._map_service_name, self._graph_filepath)
        )
        if not self._map_client.wait_for_service(timeout_sec=self._wait_for_service_sec):
            self.get_logger().error(
                "Timed out waiting for map service %s" % self._map_service_name
            )
            return 1

        request = StringWithJson.Request()
        request.message = "{}"
        future = self._map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._wait_for_service_sec)
        if not future.done():
            self.get_logger().error(
                "Timed out waiting for map response from %s" % self._map_service_name
            )
            return 1

        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("Map service returned an error")
            return 1

        try:
            map_data = json.loads(response.message or "{}")
        except json.JSONDecodeError as exc:
            self.get_logger().error("Map service returned invalid JSON: %s" % exc)
            return 1

        try:
            graph = build_geojson_route_graph(
                map_data,
                frame_id=self._graph_frame_id,
                graph_name=self._graph_name,
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error("Failed to convert map JSON into route graph: %s" % exc)
            return 1

        self._graph_filepath.parent.mkdir(parents=True, exist_ok=True)
        self._graph_filepath.write_text(
            json.dumps(graph, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(
            "Route graph written to %s with %d features"
            % (self._graph_filepath, len(graph.get("features", [])))
        )
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteGraphBuilder()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
