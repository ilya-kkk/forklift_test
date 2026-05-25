import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node


class DetectionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_detection_monitor")

        self.declare_parameter("detections_topic", "/apriltag_detector/detections")
        self.declare_parameter("no_messages_warn_sec", 5.0)
        self.declare_parameter("empty_warn_sec", 5.0)
        self.declare_parameter("detections_log_sec", 2.0)

        detections_topic = str(self.get_parameter("detections_topic").value)
        self._no_messages_warn_sec = max(
            1.0, float(self.get_parameter("no_messages_warn_sec").value)
        )
        self._empty_warn_sec = max(
            1.0, float(self.get_parameter("empty_warn_sec").value)
        )
        self._detections_log_sec = max(
            0.5, float(self.get_parameter("detections_log_sec").value)
        )
        self._message_count = 0
        self._empty_count = 0
        self._detection_count = 0

        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self._detections_callback, 10
        )
        self.create_timer(self._no_messages_warn_sec, self._warn_if_no_messages)

        self.get_logger().info(
            "apriltag_detection_monitor ready: detections_topic=%s"
            % detections_topic
        )

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        self._message_count += 1
        detections = list(msg.detections)
        if not detections:
            self._empty_count += 1
            self.get_logger().warn(
                "apriltag detections empty: messages=%d empty=%d frame_id=%s stamp=%d.%09d; detector is receiving camera pairs but sees no valid tag"
                % (
                    self._message_count,
                    self._empty_count,
                    msg.header.frame_id,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                ),
                throttle_duration_sec=self._empty_warn_sec,
            )
            return

        self._detection_count += len(detections)
        ids = [str(detection.id) for detection in detections]
        self.get_logger().info(
            "apriltag detections: count=%d ids=[%s] frame_id=%s total=%d"
            % (
                len(detections),
                ", ".join(ids),
                msg.header.frame_id,
                self._detection_count,
            ),
            throttle_duration_sec=self._detections_log_sec,
        )

    def _warn_if_no_messages(self) -> None:
        if self._message_count > 0:
            return
        self.get_logger().warn(
            "no /apriltag_detector/detections messages yet; apriltag node may not be receiving synchronized image+camera_info",
            throttle_duration_sec=self._no_messages_warn_sec,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
