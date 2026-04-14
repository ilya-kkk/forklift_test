import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthImageToPointCloud(Node):
    def __init__(self) -> None:
        super().__init__("depth_image_to_pointcloud")

        self.declare_parameter("input_depth_topic", "/fork_depth")
        self.declare_parameter("input_camera_info_topic", "/fork_depth/camera_info")
        self.declare_parameter("output_pointcloud_topic", "/fork_depth/points")
        self.declare_parameter("output_frame_id", "fork_depth_camera_link")
        self.declare_parameter("depth_scale_16uc1", 0.001)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("rotate_x_deg", 0.0)
        self.declare_parameter("rotate_y_deg", 0.0)
        self.declare_parameter("rotate_z_deg", 0.0)

        self._input_depth_topic = str(self.get_parameter("input_depth_topic").value)
        self._input_camera_info_topic = str(
            self.get_parameter("input_camera_info_topic").value
        )
        self._output_pointcloud_topic = str(
            self.get_parameter("output_pointcloud_topic").value
        )
        self._output_frame_id = str(self.get_parameter("output_frame_id").value)
        self._depth_scale_16uc1 = float(self.get_parameter("depth_scale_16uc1").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._rotate_x_deg = float(self.get_parameter("rotate_x_deg").value)
        self._rotate_y_deg = float(self.get_parameter("rotate_y_deg").value)
        self._rotate_z_deg = float(self.get_parameter("rotate_z_deg").value)
        self._rotation_matrix = self._build_rotation_matrix(
            self._rotate_x_deg,
            self._rotate_y_deg,
            self._rotate_z_deg,
        )

        self._camera_info: CameraInfo | None = None
        self._warned_no_camera_info = False
        self._warned_bad_intrinsics = False
        self._warned_unsupported_encoding = False

        self.create_subscription(
            CameraInfo,
            self._input_camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self._input_depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )

        publisher_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            PointCloud2, self._output_pointcloud_topic, publisher_qos
        )

        self.get_logger().info(
            "depth_image_to_pointcloud ready: %s + %s -> %s frame=%s rot_xyz_deg=(%.1f, %.1f, %.1f)"
            % (
                self._input_depth_topic,
                self._input_camera_info_topic,
                self._output_pointcloud_topic,
                self._output_frame_id,
                self._rotate_x_deg,
                self._rotate_y_deg,
                self._rotate_z_deg,
            )
        )

    @staticmethod
    def _build_rotation_matrix(
        rotate_x_deg: float, rotate_y_deg: float, rotate_z_deg: float
    ) -> np.ndarray:
        rotate_x = np.deg2rad(rotate_x_deg)
        rotate_y = np.deg2rad(rotate_y_deg)
        rotate_z = np.deg2rad(rotate_z_deg)

        rotation_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(rotate_x), -np.sin(rotate_x)],
                [0.0, np.sin(rotate_x), np.cos(rotate_x)],
            ],
            dtype=np.float32,
        )
        rotation_y = np.array(
            [
                [np.cos(rotate_y), 0.0, np.sin(rotate_y)],
                [0.0, 1.0, 0.0],
                [-np.sin(rotate_y), 0.0, np.cos(rotate_y)],
            ],
            dtype=np.float32,
        )
        rotation_z = np.array(
            [
                [np.cos(rotate_z), -np.sin(rotate_z), 0.0],
                [np.sin(rotate_z), np.cos(rotate_z), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        return rotation_z @ rotation_y @ rotation_x

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _depth_callback(self, message: Image) -> None:
        if self._camera_info is None:
            if not self._warned_no_camera_info:
                self.get_logger().warning(
                    "Waiting for camera_info on %s before publishing point cloud"
                    % self._input_camera_info_topic
                )
                self._warned_no_camera_info = True
            return

        camera_info = self._camera_info
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            if not self._warned_bad_intrinsics:
                self.get_logger().warning(
                    "Invalid camera intrinsics fx=%.3f fy=%.3f on %s"
                    % (fx, fy, self._input_camera_info_topic)
                )
                self._warned_bad_intrinsics = True
            return

        depth = self._decode_depth_image(message)
        if depth is None:
            return

        point_cloud = self._build_point_cloud(
            depth=depth,
            image_header=message.header,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
        self._publisher.publish(point_cloud)

    def _decode_depth_image(self, message: Image) -> np.ndarray | None:
        encoding = message.encoding.strip().lower()
        if encoding in {"32fc1", "32f", "32fc"}:
            dtype = np.dtype(np.float32)
        elif encoding in {"16uc1", "mono16"}:
            dtype = np.dtype(np.uint16)
        else:
            if not self._warned_unsupported_encoding:
                self.get_logger().warning(
                    "Unsupported depth image encoding '%s' on %s"
                    % (message.encoding, self._input_depth_topic)
                )
                self._warned_unsupported_encoding = True
            return None

        if message.is_bigendian:
            dtype = dtype.newbyteorder(">")

        bytes_per_pixel = dtype.itemsize
        depth = np.ndarray(
            shape=(message.height, message.width),
            dtype=dtype,
            buffer=message.data,
            strides=(message.step, bytes_per_pixel),
        ).astype(np.float32, copy=False)

        if encoding in {"16uc1", "mono16"}:
            depth = depth * self._depth_scale_16uc1

        return depth

    def _build_point_cloud(
        self,
        depth: np.ndarray,
        image_header,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> PointCloud2:
        height, width = depth.shape
        u_coords = np.arange(width, dtype=np.float32)
        v_coords = np.arange(height, dtype=np.float32)
        u_grid, v_grid = np.meshgrid(u_coords, v_coords)

        valid = np.isfinite(depth) & (depth > 0.0)
        if self._min_depth_m > 0.0:
            valid &= depth >= self._min_depth_m
        if self._max_depth_m > 0.0:
            valid &= depth <= self._max_depth_m

        x = np.full(depth.shape, np.nan, dtype=np.float32)
        y = np.full(depth.shape, np.nan, dtype=np.float32)
        z = np.full(depth.shape, np.nan, dtype=np.float32)

        z[valid] = depth[valid]
        x[valid] = (u_grid[valid] - cx) * depth[valid] / fx
        y[valid] = (v_grid[valid] - cy) * depth[valid] / fy

        valid_points = np.column_stack((x[valid], y[valid], z[valid]))
        if valid_points.size > 0:
            rotated_points = valid_points @ self._rotation_matrix.T
            x[valid] = rotated_points[:, 0]
            y[valid] = rotated_points[:, 1]
            z[valid] = rotated_points[:, 2]

        points = np.empty(
            (height, width),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        points["x"] = x
        points["y"] = y
        points["z"] = z

        message = PointCloud2()
        message.header = image_header
        message.header.frame_id = self._output_frame_id or image_header.frame_id
        message.height = int(height)
        message.width = int(width)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * width
        message.is_dense = False
        message.data = np.ascontiguousarray(points).tobytes()
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthImageToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
