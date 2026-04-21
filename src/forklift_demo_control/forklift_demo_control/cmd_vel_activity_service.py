import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from ros2_templates.srv import StringWithJson


class CmdVelActivityService(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_activity_service")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("moving_topic", "/debug/up_lidar_marker/is_moving")
        self.declare_parameter("service_name", "/robot_data/marker/cmd_vel_watch/control")
        self.declare_parameter("linear_threshold", 1e-4)
        self.declare_parameter("angular_threshold", 1e-4)
        self.declare_parameter("enabled", True)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        moving_topic = str(self.get_parameter("moving_topic").value)
        service_name = str(self.get_parameter("service_name").value)

        self._linear_threshold = abs(float(self.get_parameter("linear_threshold").value))
        self._angular_threshold = abs(float(self.get_parameter("angular_threshold").value))
        self._enabled = bool(self.get_parameter("enabled").value)
        self._is_moving = False

        self._publisher = self.create_publisher(Bool, moving_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_service(StringWithJson, service_name, self._service_callback)

        self.get_logger().info(
            "cmd_vel_activity_service ready: cmd_vel=%s moving_topic=%s service=%s"
            % (cmd_vel_topic, moving_topic, service_name)
        )

    def _service_callback(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps({"error": "invalid_json", "details": str(exc)})
            return response

        if "enabled" in payload:
            self._enabled = bool(payload["enabled"])

        if "linear_threshold" in payload:
            self._linear_threshold = abs(float(payload["linear_threshold"]))

        if "angular_threshold" in payload:
            self._angular_threshold = abs(float(payload["angular_threshold"]))

        response.success = True
        response.message = json.dumps(
            {
                "enabled": self._enabled,
                "is_moving": self._is_moving,
                "linear_threshold": self._linear_threshold,
                "angular_threshold": self._angular_threshold,
            }
        )
        return response

    def _cmd_vel_callback(self, msg: Twist) -> None:
        if not self._enabled:
            return

        linear_nonzero = (
            abs(msg.linear.x) > self._linear_threshold
            or abs(msg.linear.y) > self._linear_threshold
            or abs(msg.linear.z) > self._linear_threshold
        )
        angular_nonzero = (
            abs(msg.angular.x) > self._angular_threshold
            or abs(msg.angular.y) > self._angular_threshold
            or abs(msg.angular.z) > self._angular_threshold
        )
        next_is_moving = linear_nonzero or angular_nonzero
        if next_is_moving == self._is_moving:
            return

        self._is_moving = next_is_moving
        out = Bool()
        out.data = self._is_moving
        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelActivityService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
