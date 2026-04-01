import json

import rclpy
from rclpy.node import Node
from ros2_templates.srv import StringWithJson

from forklift_demo_control.map_data import build_demo_map


class MapServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("map_service")
        self.declare_parameter("map_service_name", "/robot_data/map/get_map")
        service_name = str(self.get_parameter("map_service_name").value)
        self.create_service(StringWithJson, service_name, self._handle_get_map)
        self.get_logger().info("Map service ready on %s" % service_name)

    def _handle_get_map(self, request, response):
        del request
        response.success = True
        response.message = json.dumps(build_demo_map(), ensure_ascii=False)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
