import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanSectorFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_sector_filter")

        self.declare_parameter("input_scan_topic", "/scan_raw")
        self.declare_parameter("output_scan_topic", "/scan")
        self.declare_parameter("blind_sector_center_deg", 0.0)
        self.declare_parameter("blind_sector_half_width_deg", 32.0)

        input_scan_topic = str(self.get_parameter("input_scan_topic").value)
        output_scan_topic = str(self.get_parameter("output_scan_topic").value)
        blind_sector_center_deg = float(
            self.get_parameter("blind_sector_center_deg").value
        )
        blind_sector_half_width_deg = float(
            self.get_parameter("blind_sector_half_width_deg").value
        )

        self._blind_sector_center = math.radians(blind_sector_center_deg)
        self._blind_sector_half_width = max(
            0.0, math.radians(blind_sector_half_width_deg)
        )

        self._publisher = self.create_publisher(
            LaserScan, output_scan_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            input_scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "scan_sector_filter ready: %s -> %s blind_center_deg=%.1f blind_half_width_deg=%.1f"
            % (
                input_scan_topic,
                output_scan_topic,
                blind_sector_center_deg,
                blind_sector_half_width_deg,
            )
        )

    def _scan_callback(self, message: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = message.header
        filtered.angle_min = message.angle_min
        filtered.angle_max = message.angle_max
        filtered.angle_increment = message.angle_increment
        filtered.time_increment = message.time_increment
        filtered.scan_time = message.scan_time
        filtered.range_min = message.range_min
        filtered.range_max = message.range_max
        filtered.ranges = list(message.ranges)
        filtered.intensities = list(message.intensities)

        current_angle = message.angle_min
        for index, _ in enumerate(filtered.ranges):
            if self._is_blinded(current_angle):
                filtered.ranges[index] = math.inf
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0
            current_angle += message.angle_increment

        self._publisher.publish(filtered)

    def _is_blinded(self, angle: float) -> bool:
        angle_delta = normalize_angle(angle - self._blind_sector_center)
        return abs(angle_delta) <= self._blind_sector_half_width


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSectorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
