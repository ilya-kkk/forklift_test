import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from forklift_interfaces.srv import StringWithJson


class UpLidarMarkerService(Node):
    def __init__(self) -> None:
        super().__init__("up_lidar_marker_service")

        self.declare_parameter("marker_topic", "/debug/up_lidar_marker")
        self.declare_parameter("moving_topic", "/debug/up_lidar_marker/is_moving")
        self.declare_parameter("service_name", "/robot_data/marker/up_lidar/control")
        self.declare_parameter("frame_id", "up_lidar_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("sphere_diameter", 0.18)
        self.declare_parameter("enabled", True)

        marker_topic = str(self.get_parameter("marker_topic").value)
        moving_topic = str(self.get_parameter("moving_topic").value)
        service_name = str(self.get_parameter("service_name").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._sphere_diameter = float(self.get_parameter("sphere_diameter").value)
        self._enabled = bool(self.get_parameter("enabled").value)

        self._is_moving = False
        self._force_color = None

        self._publisher = self.create_publisher(Marker, marker_topic, 10)
        self.create_subscription(Bool, moving_topic, self._moving_callback, 10)
        self.create_service(StringWithJson, service_name, self._service_callback)
        self.create_timer(1.0 / publish_rate_hz, self._publish_marker)

        self.get_logger().info(
            "up_lidar_marker_service ready: marker_topic=%s moving_topic=%s service=%s frame=%s"
            % (marker_topic, moving_topic, service_name, self._frame_id)
        )

    def _moving_callback(self, msg: Bool) -> None:
        self._is_moving = bool(msg.data)

    def _service_callback(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps({"error": "invalid_json", "details": str(exc)})
            return response

        if "enabled" in payload:
            self._enabled = bool(payload["enabled"])

        color = payload.get("force_color")
        if color is None:
            self._force_color = None
        elif str(color).lower() in {"red", "green"}:
            self._force_color = str(color).lower()
        else:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_force_color", "details": "expected red|green|null"}
            )
            return response

        response.success = True
        response.message = json.dumps(
            {
                "enabled": self._enabled,
                "is_moving": self._is_moving,
                "force_color": self._force_color,
            }
        )
        return response

    def _publish_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "up_lidar_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if self._enabled else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = self._sphere_diameter
        marker.scale.y = self._sphere_diameter
        marker.scale.z = self._sphere_diameter

        color = self._force_color
        if color is None:
            color = "red" if self._is_moving else "green"

        if color == "red":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 0.9

        self._publisher.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UpLidarMarkerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
