import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from ros2_templates.srv import StringWithJson


class CmdVelActivityService(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_activity_service")

        self.declare_parameter("wheel_velocity_topic", "/forklift/right_wheel_cmd")
        self.declare_parameter("moving_topic", "/debug/up_lidar_marker/is_moving")
        self.declare_parameter("service_name", "/robot_data/marker/cmd_vel_watch/control")
        self.declare_parameter("wheel_velocity_threshold", 1e-4)
        self.declare_parameter("enabled", True)

        wheel_velocity_topic = str(self.get_parameter("wheel_velocity_topic").value)
        moving_topic = str(self.get_parameter("moving_topic").value)
        service_name = str(self.get_parameter("service_name").value)

        self._wheel_velocity_threshold = abs(
            float(self.get_parameter("wheel_velocity_threshold").value)
        )
        self._enabled = bool(self.get_parameter("enabled").value)
        self._is_moving = False

        self._publisher = self.create_publisher(Bool, moving_topic, 10)
        self.create_subscription(
            Float64, wheel_velocity_topic, self._wheel_velocity_callback, 10
        )
        self.create_service(StringWithJson, service_name, self._service_callback)

        self.get_logger().info(
            "cmd_vel_activity_service ready: wheel_velocity_topic=%s moving_topic=%s service=%s"
            % (wheel_velocity_topic, moving_topic, service_name)
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

        # Backward compatible keys from cmd_vel-based version.
        if "wheel_velocity_threshold" in payload:
            self._wheel_velocity_threshold = abs(float(payload["wheel_velocity_threshold"]))
        elif "linear_threshold" in payload:
            self._wheel_velocity_threshold = abs(float(payload["linear_threshold"]))
        elif "angular_threshold" in payload:
            self._wheel_velocity_threshold = abs(float(payload["angular_threshold"]))

        response.success = True
        response.message = json.dumps(
            {
                "enabled": self._enabled,
                "is_moving": self._is_moving,
                "wheel_velocity_threshold": self._wheel_velocity_threshold,
            }
        )
        return response

    def _wheel_velocity_callback(self, msg: Float64) -> None:
        if not self._enabled:
            return

        next_is_moving = abs(float(msg.data)) > self._wheel_velocity_threshold
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
