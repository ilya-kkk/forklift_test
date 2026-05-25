import json
from copy import deepcopy

import rclpy
from forklift_interfaces.srv import StringWithJson
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


class ImageGate(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_image_gate")

        self.declare_parameter("input_image_topic", "/camera/image")
        self.declare_parameter("input_camera_info_topic", "/camera/camera_info")
        self.declare_parameter("output_image_topic", "/apriltag_detector/image")
        self.declare_parameter("output_camera_info_topic", "/apriltag_detector/camera_info")
        self.declare_parameter("service_name", "/apriltag_detector/control")
        self.declare_parameter("enabled", False)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("restamp_with_now", True)

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        input_camera_info_topic = str(
            self.get_parameter("input_camera_info_topic").value
        )
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        output_camera_info_topic = str(
            self.get_parameter("output_camera_info_topic").value
        )
        service_name = str(self.get_parameter("service_name").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._restamp_with_now = bool(self.get_parameter("restamp_with_now").value)
        self._input_image_topic = input_image_topic
        self._input_camera_info_topic = input_camera_info_topic
        self._latest_image = None
        self._latest_camera_info = None
        self._image_count = 0
        self._camera_info_count = 0
        self._published_count = 0
        self._last_enabled = self._enabled
        self._logged_first_image = False
        self._logged_first_camera_info = False
        self._logged_first_publish = False
        output_qos = QoSProfile(depth=10)

        self._image_publisher = self.create_publisher(
            Image, output_image_topic, output_qos
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, output_camera_info_topic, output_qos
        )

        self.create_subscription(
            Image, input_image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            input_camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_service(StringWithJson, service_name, self._service_callback)
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest_pair)

        self.get_logger().info(
            "apriltag_image_gate ready: enabled=%s rate=%.2f image=%s->%s camera_info=%s->%s service=%s"
            % (
                self._enabled,
                publish_rate_hz,
                input_image_topic,
                output_image_topic,
                input_camera_info_topic,
                output_camera_info_topic,
                service_name,
            )
        )

    def _image_callback(self, msg: Image) -> None:
        self._image_count += 1
        self._latest_image = msg
        if not self._logged_first_image:
            self._logged_first_image = True
            self.get_logger().info(
                "first image received: topic=%s frame_id=%s stamp=%d.%09d size=%dx%d encoding=%s"
                % (
                    self._input_image_topic,
                    msg.header.frame_id,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.width,
                    msg.height,
                    msg.encoding,
                )
            )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info_count += 1
        self._latest_camera_info = msg
        if not self._logged_first_camera_info:
            self._logged_first_camera_info = True
            self.get_logger().info(
                "first camera_info received: topic=%s frame_id=%s stamp=%d.%09d size=%dx%d"
                % (
                    self._input_camera_info_topic,
                    msg.header.frame_id,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.width,
                    msg.height,
                )
            )

    def _publish_latest_pair(self) -> None:
        if not self._enabled:
            if self._last_enabled:
                self.get_logger().info("apriltag_image_gate disabled")
                self._last_enabled = False
            return

        if not self._last_enabled:
            self.get_logger().info("apriltag_image_gate enabled")
            self._last_enabled = True

        if self._latest_image is None:
            self.get_logger().warn(
                "apriltag detector enabled but no image received on /camera/image yet",
                throttle_duration_sec=5.0,
            )
            return

        if self._latest_camera_info is None:
            self.get_logger().warn(
                "apriltag detector enabled but no camera info received on /camera/camera_info yet",
                throttle_duration_sec=5.0,
            )
            return

        image = deepcopy(self._latest_image)
        camera_info = deepcopy(self._latest_camera_info)
        stamp = self.get_clock().now().to_msg() if self._restamp_with_now else image.header.stamp
        image.header.stamp = stamp
        camera_info.header.stamp = stamp
        if not camera_info.header.frame_id:
            camera_info.header.frame_id = image.header.frame_id

        self._camera_info_publisher.publish(camera_info)
        self._image_publisher.publish(image)
        self._published_count += 1
        if not self._logged_first_publish:
            self._logged_first_publish = True
            self.get_logger().info(
                "first gated camera pair published: image_frame=%s camera_info_frame=%s stamp=%d.%09d"
                % (
                    image.header.frame_id,
                    camera_info.header.frame_id,
                    image.header.stamp.sec,
                    image.header.stamp.nanosec,
                )
            )

        self.get_logger().info(
            "apriltag gate status: enabled=%s input_images=%d camera_infos=%d published_pairs=%d last_image_age=%.3fs"
            % (
                self._enabled,
                self._image_count,
                self._camera_info_count,
                self._published_count,
                self._latest_image_age_sec(),
            ),
            throttle_duration_sec=5.0,
        )

    def _service_callback(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps({"error": "invalid_json", "details": str(exc)})
            return response

        command = str(payload.get("command", "")).strip().lower()
        if "enabled" in payload:
            self._enabled = bool(payload["enabled"])
        elif command in {"enable", "start", "on"}:
            self._enabled = True
        elif command in {"disable", "stop", "off"}:
            self._enabled = False
        elif command not in {"", "status"}:
            response.success = False
            response.message = json.dumps(
                {
                    "error": "invalid_command",
                    "details": "expected enabled bool or command enable|disable|status",
                }
            )
            return response

        self.get_logger().info(
            "apriltag_image_gate service request: payload=%s enabled=%s"
            % (payload, self._enabled)
        )
        response.success = True
        response.message = json.dumps(
            {
                "enabled": self._enabled,
                "has_image": self._latest_image is not None,
                "has_camera_info": self._latest_camera_info is not None,
                "input_images": self._image_count,
                "input_camera_infos": self._camera_info_count,
                "published_pairs": self._published_count,
                "last_image_age_sec": self._latest_image_age_sec(),
            }
        )
        self.get_logger().info("apriltag_image_gate service response: %s" % response.message)
        return response

    def _latest_image_age_sec(self) -> float:
        if self._latest_image is None:
            return -1.0
        now = self.get_clock().now()
        stamp = Time.from_msg(self._latest_image.header.stamp)
        return max(0.0, (now.nanoseconds - stamp.nanoseconds) / 1e9)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
