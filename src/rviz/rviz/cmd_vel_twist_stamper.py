import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelTwistStamper(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_twist_stamper")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_stamped")
        self.declare_parameter("frame_id", "base_link")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._twist_callback, 10)

        self.get_logger().info(
            "cmd_vel_twist_stamper ready: %s -> %s frame=%s"
            % (input_topic, output_topic, self._frame_id)
        )

    def _twist_callback(self, message: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = message
        self._publisher.publish(stamped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelTwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
