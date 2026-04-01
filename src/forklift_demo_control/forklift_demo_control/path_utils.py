import math
from typing import Iterable, List, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


Point2D = Tuple[float, float]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def build_l_waypoints(
    start_x: float = 1.0,
    start_y: float = 1.0,
    horizontal_length: float = 3.0,
    vertical_length: float = 3.0,
) -> List[Point2D]:
    return [
        (start_x, start_y),
        (start_x + horizontal_length, start_y),
        (start_x + horizontal_length, start_y + vertical_length),
    ]


def densify_polyline(points: Sequence[Point2D], resolution: float) -> List[Point2D]:
    if len(points) < 2:
        return list(points)

    if resolution <= 0.0:
        raise ValueError("resolution must be > 0")

    dense_points: List[Point2D] = [points[0]]
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance == 0.0:
            continue

        steps = max(1, int(math.ceil(distance / resolution)))
        for step in range(1, steps + 1):
            ratio = step / steps
            dense_points.append((start[0] + dx * ratio, start[1] + dy * ratio))

    return dense_points


def reverse_points(points: Iterable[Point2D]) -> List[Point2D]:
    return list(reversed(list(points)))


def build_path(points: Sequence[Point2D], frame_id: str, stamp) -> Path:
    if not points:
        raise ValueError("points must not be empty")

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp

    previous_yaw = 0.0
    for index, (x_coord, y_coord) in enumerate(points):
        if len(points) == 1:
            yaw = 0.0
        elif index == len(points) - 1:
            prev_x, prev_y = points[index - 1]
            yaw = math.atan2(y_coord - prev_y, x_coord - prev_x)
        else:
            next_x, next_y = points[index + 1]
            delta_x = next_x - x_coord
            delta_y = next_y - y_coord
            if math.hypot(delta_x, delta_y) < 1e-9:
                yaw = previous_yaw
            else:
                yaw = math.atan2(delta_y, delta_x)

        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x_coord
        pose.pose.position.y = y_coord
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        path.poses.append(pose)
        previous_yaw = yaw

    return path
