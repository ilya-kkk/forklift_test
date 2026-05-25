from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class BaseLinkTagTf(Node):
    def __init__(self) -> None:
        super().__init__("base_link_tag_tf")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("source_frames", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("output_frames", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("transform_timeout_sec", 0.05)
        self.declare_parameter("max_source_age_sec", 2.0)

        self._target_frame = str(self.get_parameter("target_frame").value)
        source_frames = [
            str(frame) for frame in self.get_parameter("source_frames").value
        ]
        output_frames = [
            str(frame) for frame in self.get_parameter("output_frames").value
        ]
        if len(source_frames) != len(output_frames):
            raise ValueError("source_frames and output_frames must have the same length")
        self._frame_pairs = list(zip(source_frames, output_frames))

        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timeout = float(self.get_parameter("transform_timeout_sec").value)
        self._max_source_age = float(self.get_parameter("max_source_age_sec").value)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        self._last_transforms = {}
        self._last_lookup_error = ""
        self._published_count = 0

        self.create_timer(1.0 / publish_rate_hz, self._publish_base_link_transforms)

        self.get_logger().info(
            "base_link_tag_tf ready: target_frame=%s frame_pairs=%d rate=%.2fHz"
            % (self._target_frame, len(self._frame_pairs), publish_rate_hz)
        )

    def _publish_base_link_transforms(self) -> None:
        now = self.get_clock().now()
        transforms = []
        updated = 0
        cached = 0
        self._last_lookup_error = ""

        for source_frame, output_frame in self._frame_pairs:
            transform = self._lookup_latest(source_frame)
            if transform is not None:
                transform.child_frame_id = output_frame
                self._last_transforms[output_frame] = deepcopy(transform)
                updated += 1
            else:
                transform = self._last_transforms.get(output_frame)
                if transform is None:
                    continue
                transform = deepcopy(transform)
                cached += 1

            transform.header.stamp = now.to_msg()
            transform.header.frame_id = self._target_frame
            transform.child_frame_id = output_frame
            transforms.append(transform)

        if transforms:
            self._broadcaster.sendTransform(transforms)
            self._published_count += len(transforms)
            self.get_logger().info(
                "base_link_tag_tf published=%d updated=%d cached=%d total=%d"
                % (len(transforms), updated, cached, self._published_count),
                throttle_duration_sec=5.0,
            )
        else:
            self.get_logger().warn(
                "base_link_tag_tf has no tag transforms yet: target=%s source_frames=%d last_lookup_error='%s'; check /apriltag_detector/detections"
                % (
                    self._target_frame,
                    len(self._frame_pairs),
                    self._last_lookup_error,
                ),
                throttle_duration_sec=5.0,
            )

    def _lookup_latest(self, source_frame: str) -> TransformStamped | None:
        try:
            if self._timeout > 0.0:
                transform = self._buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self._timeout),
                )
            else:
                transform = self._buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    Time(),
                )
        except TransformException as exc:
            if not self._last_lookup_error:
                self._last_lookup_error = "%s: %s" % (source_frame, exc)
            return None

        if self._is_too_old(transform):
            self.get_logger().warn(
                "stale source transform %s -> %s age=%.3fs"
                % (
                    self._target_frame,
                    source_frame,
                    self._transform_age_sec(transform),
                ),
                throttle_duration_sec=5.0,
            )
        return transform

    def _is_too_old(self, transform: TransformStamped) -> bool:
        age = self._transform_age_sec(transform)
        return age >= 0.0 and age > self._max_source_age

    def _transform_age_sec(self, transform: TransformStamped) -> float:
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return -1.0
        stamp = Time.from_msg(transform.header.stamp)
        return max(0.0, (now.nanoseconds - stamp.nanoseconds) / 1e9)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseLinkTagTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
