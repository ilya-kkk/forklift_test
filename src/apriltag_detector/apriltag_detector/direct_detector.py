from __future__ import annotations

from copy import deepcopy

import cv2
import numpy as np
import rclpy
from apriltag import apriltag as AprilTagDetector
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class DirectAprilTagDetector(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_direct_detector")

        self.declare_parameter("input_image_topic", "/apriltag_detector/image")
        self.declare_parameter("input_camera_info_topic", "/apriltag_detector/camera_info")
        self.declare_parameter("detections_topic", "/apriltag_detector/detections")
        self.declare_parameter("target_frame", "camera_link")
        self.declare_parameter("family", "tag36h11")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("tf_publish_rate_hz", 10.0)
        self.declare_parameter("transform_timeout_sec", 0.02)
        self.declare_parameter("max_hamming", 2)
        self.declare_parameter("decimate", 1.0)
        self.declare_parameter("blur", 0.0)
        self.declare_parameter("refine_edges", True)
        self.declare_parameter("debug", False)
        self.declare_parameter("tag_ids", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("tag_frames", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("tag_sizes", Parameter.Type.DOUBLE_ARRAY)

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        input_camera_info_topic = str(
            self.get_parameter("input_camera_info_topic").value
        )
        detections_topic = str(self.get_parameter("detections_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._family = str(self.get_parameter("family").value)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        tf_publish_rate_hz = max(
            0.1, float(self.get_parameter("tf_publish_rate_hz").value)
        )
        self._timeout = float(self.get_parameter("transform_timeout_sec").value)
        self._camera_transform_rotation = _rotation_sequence(
            _rotation_matrix_y(-np.pi / 2.0),
            _rotation_matrix_x(-np.pi / 2.0),
            _rotation_matrix_z(np.pi),
        )

        tag_ids = [int(tag_id) for tag_id in self.get_parameter("tag_ids").value]
        tag_frames = [
            str(frame) for frame in self.get_parameter("tag_frames").value
        ]
        tag_sizes = [float(size) for size in self.get_parameter("tag_sizes").value]
        if len(tag_ids) != len(tag_frames) or len(tag_ids) != len(tag_sizes):
            raise ValueError("tag_ids, tag_frames and tag_sizes must have the same length")
        self._tag_frames = dict(zip(tag_ids, tag_frames))
        self._tag_sizes = dict(zip(tag_ids, tag_sizes))

        self._detector = AprilTagDetector(
            self._family,
            threads=2,
            maxhamming=int(self.get_parameter("max_hamming").value),
            decimate=float(self.get_parameter("decimate").value),
            blur=float(self.get_parameter("blur").value),
            refine_edges=bool(self.get_parameter("refine_edges").value),
            debug=bool(self.get_parameter("debug").value),
        )
        self._bridge = CvBridge()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        self._detections_pub = self.create_publisher(
            AprilTagDetectionArray, detections_topic, QoSProfile(depth=10)
        )

        self._latest_image: Image | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._last_processed_stamp: tuple[int, int] | None = None
        self._cached_transforms: dict[str, TransformStamped] = {}
        self._input_images = 0
        self._input_camera_infos = 0
        self._processed_images = 0
        self._detected_tags = 0

        self.create_subscription(
            Image, input_image_topic, self._image_callback, QoSProfile(depth=10)
        )
        self.create_subscription(
            CameraInfo,
            input_camera_info_topic,
            self._camera_info_callback,
            QoSProfile(depth=10),
        )
        self.create_timer(1.0 / publish_rate_hz, self._process_latest)
        self.create_timer(1.0 / tf_publish_rate_hz, self._publish_cached_transforms)

        self.get_logger().info(
            "apriltag_direct_detector ready: image=%s camera_info=%s detections=%s target_frame=%s tags=%d"
            % (
                input_image_topic,
                input_camera_info_topic,
                detections_topic,
                self._target_frame,
                len(self._tag_frames),
            )
        )

    def _image_callback(self, msg: Image) -> None:
        self._latest_image = msg
        self._input_images += 1
        if self._input_images == 1:
            self.get_logger().info(
                "direct detector first image: frame_id=%s stamp=%d.%09d size=%dx%d encoding=%s"
                % (
                    msg.header.frame_id,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.width,
                    msg.height,
                    msg.encoding,
                )
            )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg
        self._input_camera_infos += 1
        if self._input_camera_infos == 1:
            self.get_logger().info(
                "direct detector first camera_info: frame_id=%s stamp=%d.%09d size=%dx%d"
                % (
                    msg.header.frame_id,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.width,
                    msg.height,
                )
            )

    def _process_latest(self) -> None:
        if self._latest_image is None or self._latest_camera_info is None:
            self.get_logger().warn(
                "direct detector waiting for image+camera_info: images=%d camera_infos=%d"
                % (self._input_images, self._input_camera_infos),
                throttle_duration_sec=5.0,
            )
            return

        image = self._latest_image
        camera_info = self._latest_camera_info
        stamp_key = (image.header.stamp.sec, image.header.stamp.nanosec)
        if stamp_key == self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp_key

        try:
            gray = self._image_to_gray(image)
        except CvBridgeError as exc:
            self.get_logger().error("direct detector image conversion failed: %s" % exc)
            return

        try:
            detections = self._detector.detect(gray)
        except Exception as exc:
            self.get_logger().error("direct detector apriltag detect failed: %s" % exc)
            return

        array_msg = AprilTagDetectionArray()
        array_msg.header = image.header
        fx, fy, cx, cy = self._camera_intrinsics(camera_info)
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error(
                "direct detector invalid camera intrinsics: fx=%.3f fy=%.3f"
                % (fx, fy),
                throttle_duration_sec=5.0,
            )
            self._detections_pub.publish(array_msg)
            return

        for detection in detections:
            tag_id = int(detection["id"])
            if tag_id not in self._tag_frames:
                continue
            if int(detection.get("hamming", 0)) > int(
                self.get_parameter("max_hamming").value
            ):
                continue

            tag_size = self._tag_sizes[tag_id]
            try:
                transform_camera_tag = self._estimate_tag_pose(
                    detection, tag_size, fx, fy, cx, cy
                )
            except (KeyError, ValueError, cv2.error) as exc:
                self.get_logger().warn(
                    "direct detector cannot estimate pose for tag_id=%d: %s"
                    % (tag_id, exc),
                    throttle_duration_sec=5.0,
                )
                continue

            detection_msg = AprilTagDetection()
            detection_msg.family = self._family
            detection_msg.id = tag_id
            detection_msg.hamming = int(detection.get("hamming", 0))
            detection_msg.decision_margin = float(
                detection.get("decision_margin", detection.get("margin", 0.0))
            )
            detection_msg.goodness = float(detection.get("goodness", 0.0))
            detection_msg.centre = _point_from_xy(detection.get("center", [0.0, 0.0]))
            detection_msg.corners = [
                _point_from_xy(corner) for corner in self._detection_corners(detection)
            ]
            if "homography" in detection:
                detection_msg.homography = [
                    float(value)
                    for value in np.asarray(detection["homography"]).reshape(9)
                ]
            array_msg.detections.append(detection_msg)

            base_transform = self._make_base_transform(
                image.header.frame_id,
                self._tag_frames[tag_id],
                transform_camera_tag,
            )
            if base_transform is not None:
                self._cached_transforms[self._tag_frames[tag_id]] = base_transform

        self._detections_pub.publish(array_msg)
        self._processed_images += 1
        self._detected_tags += len(array_msg.detections)
        self.get_logger().info(
            "direct detector processed=%d detections=%d cached_tf=%d total_tags=%d"
            % (
                self._processed_images,
                len(array_msg.detections),
                len(self._cached_transforms),
                self._detected_tags,
            ),
            throttle_duration_sec=2.0,
        )
        self._publish_cached_transforms()

    def _image_to_gray(self, image: Image) -> np.ndarray:
        try:
            return self._bridge.imgmsg_to_cv2(image, desired_encoding="mono8")
        except CvBridgeError:
            cv_image = self._bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            if cv_image.ndim == 2:
                return cv_image
            if cv_image.shape[2] == 4:
                return cv2.cvtColor(cv_image, cv2.COLOR_RGBA2GRAY)
            return cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

    def _camera_intrinsics(self, camera_info: CameraInfo) -> tuple[float, float, float, float]:
        if camera_info.k[0] > 0.0 and camera_info.k[4] > 0.0:
            return camera_info.k[0], camera_info.k[4], camera_info.k[2], camera_info.k[5]
        return camera_info.p[0], camera_info.p[5], camera_info.p[2], camera_info.p[6]

    def _estimate_tag_pose(
        self,
        detection: dict,
        tag_size: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> np.ndarray:
        half_size = tag_size / 2.0
        object_points = np.array(
            [
                [-half_size, -half_size, 0.0],
                [half_size, -half_size, 0.0],
                [half_size, half_size, 0.0],
                [-half_size, half_size, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = self._detection_corners(detection)
        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        distortion = np.zeros((4, 1), dtype=np.float64)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            raise ValueError("cv2.solvePnP returned false")
        rotation, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = tvec.reshape(3)
        return transform

    def _detection_corners(self, detection: dict) -> np.ndarray:
        for key in ("lb-rb-rt-lt", "corners"):
            if key in detection:
                corners = np.asarray(detection[key], dtype=np.float64)
                break
        else:
            raise KeyError("detection has no corner coordinates")

        if corners.shape != (4, 2):
            raise ValueError("unexpected corner shape %s" % (corners.shape,))
        return corners

    def _make_base_transform(
        self,
        camera_frame: str,
        child_frame: str,
        transform_camera_tag: np.ndarray,
    ) -> TransformStamped | None:
        transform_camera_tag = self._camera_transform_rotation @ transform_camera_tag
        if self._target_frame == camera_frame:
            transform_target_tag = transform_camera_tag
        else:
            try:
                if self._timeout > 0.0:
                    transform_target_camera = self._buffer.lookup_transform(
                        self._target_frame,
                        camera_frame,
                        Time(),
                        timeout=Duration(seconds=self._timeout),
                    )
                else:
                    transform_target_camera = self._buffer.lookup_transform(
                        self._target_frame,
                        camera_frame,
                        Time(),
                    )
            except TransformException as exc:
                self.get_logger().warn(
                    "direct detector cannot lookup %s -> %s: %s"
                    % (self._target_frame, camera_frame, exc),
                    throttle_duration_sec=5.0,
                )
                return None

            transform_target_tag = (
                _matrix_from_transform(transform_target_camera) @ transform_camera_tag
            )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._target_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(transform_target_tag[0, 3])
        msg.transform.translation.y = float(transform_target_tag[1, 3])
        msg.transform.translation.z = float(transform_target_tag[2, 3])
        quat = _quaternion_from_matrix(transform_target_tag[:3, :3])
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        return msg

    def _publish_cached_transforms(self) -> None:
        if not self._cached_transforms:
            return
        now = self.get_clock().now().to_msg()
        transforms = []
        for transform in self._cached_transforms.values():
            updated = deepcopy(transform)
            updated.header.stamp = now
            transforms.append(updated)
        self._broadcaster.sendTransform(transforms)


def _point_from_xy(xy) -> Point:
    point = Point()
    point.x = float(xy[0])
    point.y = float(xy[1])
    return point


def _rotation_matrix_x(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]
    )
    return matrix


def _rotation_matrix_y(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]
    )
    return matrix


def _rotation_matrix_z(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return matrix


def _rotation_sequence(*rotations: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    for rotation in rotations:
        matrix = rotation @ matrix
    return matrix


def _matrix_from_transform(transform: TransformStamped) -> np.ndarray:
    matrix = np.eye(4)
    q = transform.transform.rotation
    matrix[:3, :3] = _matrix_from_quaternion([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ]
    return matrix


def _matrix_from_quaternion(quaternion: list[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = x * x + y * y + z * z + w * w
    if norm < np.finfo(float).eps:
        return np.eye(3)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ]
    )


def _quaternion_from_matrix(matrix: np.ndarray) -> list[float]:
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        return [
            float((m[2, 1] - m[1, 2]) / s),
            float((m[0, 2] - m[2, 0]) / s),
            float((m[1, 0] - m[0, 1]) / s),
            float(0.25 * s),
        ]
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return [
            float(0.25 * s),
            float((m[0, 1] + m[1, 0]) / s),
            float((m[0, 2] + m[2, 0]) / s),
            float((m[2, 1] - m[1, 2]) / s),
        ]
    if m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return [
            float((m[0, 1] + m[1, 0]) / s),
            float(0.25 * s),
            float((m[1, 2] + m[2, 1]) / s),
            float((m[0, 2] - m[2, 0]) / s),
        ]
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return [
        float((m[0, 2] + m[2, 0]) / s),
        float((m[1, 2] + m[2, 1]) / s),
        float(0.25 * s),
        float((m[1, 0] - m[0, 1]) / s),
    ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DirectAprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
