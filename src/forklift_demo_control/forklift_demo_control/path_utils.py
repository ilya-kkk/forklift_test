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


def round_polyline_corner(
    points: Sequence[Point2D],
    corner_index: int,
    radius: float,
    resolution: float,
) -> List[Point2D]:
    if len(points) < 3:
        return list(points)

    if corner_index <= 0 or corner_index >= len(points) - 1:
        raise ValueError("corner_index must refer to an interior point")

    if radius <= 0.0:
        return list(points)

    if resolution <= 0.0:
        raise ValueError("resolution must be > 0")

    previous_point = points[corner_index - 1]
    corner_point = points[corner_index]
    next_point = points[corner_index + 1]

    in_dx = corner_point[0] - previous_point[0]
    in_dy = corner_point[1] - previous_point[1]
    out_dx = next_point[0] - corner_point[0]
    out_dy = next_point[1] - corner_point[1]

    in_length = math.hypot(in_dx, in_dy)
    out_length = math.hypot(out_dx, out_dy)
    if in_length < 1e-9 or out_length < 1e-9:
        return list(points)

    in_unit = (in_dx / in_length, in_dy / in_length)
    out_unit = (out_dx / out_length, out_dy / out_length)

    dot_product = max(-1.0, min(1.0, in_unit[0] * out_unit[0] + in_unit[1] * out_unit[1]))
    turn_angle = math.acos(dot_product)
    if turn_angle < 1e-6 or abs(math.pi - turn_angle) < 1e-6:
        return list(points)

    tangent_distance = radius / math.tan(turn_angle * 0.5)
    tangent_distance = min(tangent_distance, in_length * 0.5, out_length * 0.5)
    if tangent_distance < 1e-6:
        return list(points)

    effective_radius = tangent_distance * math.tan(turn_angle * 0.5)
    start_tangent = (
        corner_point[0] - in_unit[0] * tangent_distance,
        corner_point[1] - in_unit[1] * tangent_distance,
    )
    end_tangent = (
        corner_point[0] + out_unit[0] * tangent_distance,
        corner_point[1] + out_unit[1] * tangent_distance,
    )

    bisector = (-in_unit[0] + out_unit[0], -in_unit[1] + out_unit[1])
    bisector_length = math.hypot(bisector[0], bisector[1])
    if bisector_length < 1e-9:
        return list(points)

    center_distance = effective_radius / math.sin(turn_angle * 0.5)
    center = (
        corner_point[0] + bisector[0] / bisector_length * center_distance,
        corner_point[1] + bisector[1] / bisector_length * center_distance,
    )

    start_angle = math.atan2(start_tangent[1] - center[1], start_tangent[0] - center[0])
    end_angle = math.atan2(end_tangent[1] - center[1], end_tangent[0] - center[0])
    turn_direction = in_unit[0] * out_unit[1] - in_unit[1] * out_unit[0]

    if turn_direction > 0.0:
        while end_angle <= start_angle:
            end_angle += 2.0 * math.pi
        sweep_angle = end_angle - start_angle
    else:
        while end_angle >= start_angle:
            end_angle -= 2.0 * math.pi
        sweep_angle = end_angle - start_angle

    arc_length = abs(sweep_angle) * effective_radius
    arc_steps = max(2, int(math.ceil(arc_length / resolution)))

    rounded_points: List[Point2D] = list(points[:corner_index])
    rounded_points.append(start_tangent)
    for step in range(1, arc_steps):
        ratio = step / arc_steps
        angle = start_angle + sweep_angle * ratio
        rounded_points.append(
            (
                center[0] + effective_radius * math.cos(angle),
                center[1] + effective_radius * math.sin(angle),
            )
        )
    rounded_points.append(end_tangent)
    rounded_points.extend(points[corner_index + 1 :])
    return rounded_points


def reverse_points(points: Iterable[Point2D]) -> List[Point2D]:
    return list(reversed(list(points)))


def build_path(
    points: Sequence[Point2D],
    frame_id: str,
    stamp,
    yaw_offset: float = 0.0,
) -> Path:
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

        yaw = math.atan2(math.sin(yaw + yaw_offset), math.cos(yaw + yaw_offset))

        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x_coord
        pose.pose.position.y = y_coord
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        path.poses.append(pose)
        previous_yaw = yaw

    return path
