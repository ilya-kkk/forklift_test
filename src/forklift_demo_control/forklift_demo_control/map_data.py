import json
import math
from typing import Any, Dict, List, Optional


MapJson = Dict[str, List[Dict[str, Any]]]
CAMERA_TO_BASE_X_DISTANCE = 0.65
ENTRY_OBJECT_DEPTH = 1.2
PRE_ENTRY_DISTANCE = CAMERA_TO_BASE_X_DISTANCE + ENTRY_OBJECT_DEPTH / 2.0


def build_demo_map() -> MapJson:
    points = []
    paths = []
    next_point_id = 1

    dock_id = next_point_id
    points.append(
        _build_point(alias="0000", point_id=dock_id, x=-10.0, y=0.0, marker_id=0)
    )
    next_point_id += 1

    x_values = [
        -7.15,
        -5.85,
        -4.55,
        -3.25,
        -1.95,
        -0.65,
        0.65,
        1.95,
        3.25,
        4.55,
        5.85,
        7.15,
    ]
    aisle_y_by_name = {"A": -3.5, "B": 3.5}
    aisle_nodes = {}
    for aisle_name, aisle_y in aisle_y_by_name.items():
        for x_index, aisle_x in enumerate(x_values):
            point_id = next_point_id
            points.append(
                _build_point(
                    alias=f"{point_id - 1:04d}",
                    point_id=point_id,
                    x=aisle_x,
                    y=aisle_y,
                    marker_id=point_id - 1,
                )
            )
            aisle_nodes[(aisle_name, x_index)] = point_id
            next_point_id += 1

    side_rows = [
        ("B", 1.5),
        ("B", 5.5),
    ]
    entry_records = []
    for aisle_name, entry_y in side_rows:
        for x_index, entry_x in enumerate(x_values):
            point_id = next_point_id
            end_point = _build_point(
                alias=f"{point_id - 1:04d}",
                point_id=point_id,
                x=entry_x,
                y=entry_y,
                marker_id=point_id - 1,
            )
            points.append(end_point)
            entry_records.append(
                {
                    "aisle_name": aisle_name,
                    "x_index": x_index,
                    "entry_x": entry_x,
                    "entry_y": entry_y,
                    "end_point": end_point,
                }
            )
            next_point_id += 1

    for entry_record in entry_records:
        aisle_name = entry_record["aisle_name"]
        x_index = entry_record["x_index"]
        entry_x = entry_record["entry_x"]
        entry_y = entry_record["entry_y"]
        end_point = entry_record["end_point"]
        aisle_y = aisle_y_by_name[aisle_name]
        direction_to_aisle = 1.0 if aisle_y > entry_y else -1.0
        approach_y = entry_y + direction_to_aisle * PRE_ENTRY_DISTANCE
        yaw_from_entry = math.atan2(approach_y - entry_y, 0.0)

        point_id = next_point_id
        approach_point = _build_point(
            alias=f"{point_id - 1:04d}",
            point_id=point_id,
            x=entry_x,
            y=approach_y,
            marker_id=point_id - 1,
            yaw=yaw_from_entry,
        )
        points.append(approach_point)

        aisle_point_id = aisle_nodes[(aisle_name, x_index)]
        _append_bidirectional_path(paths, aisle_point_id, point_id)
        _append_bidirectional_path(paths, point_id, int(end_point["point_id"]))
        next_point_id += 1

    for aisle_name in ("A", "B"):
        for x_index in range(len(x_values) - 1):
            aisle_point_id = aisle_nodes[(aisle_name, x_index)]
            next_aisle_point_id = aisle_nodes[(aisle_name, x_index + 1)]
            _append_bidirectional_path(paths, aisle_point_id, next_aisle_point_id)

    _append_bidirectional_path(paths, dock_id, aisle_nodes[("A", 0)])
    _append_bidirectional_path(paths, dock_id, aisle_nodes[("B", 0)])
    _append_bidirectional_path(paths, aisle_nodes[("A", 0)], aisle_nodes[("B", 0)])
    _append_bidirectional_path(
        paths,
        aisle_nodes[("A", len(x_values) - 1)],
        aisle_nodes[("B", len(x_values) - 1)],
    )

    return {"point": points, "path": paths}


def build_demo_map_json_string() -> str:
    return json.dumps(build_demo_map(), ensure_ascii=False)


def _build_point(
    *,
    alias: str,
    point_id: int,
    x: float,
    y: float,
    marker_id: int,
    yaw: Optional[float] = None,
) -> Dict[str, Any]:
    point = {
        "alias": alias,
        "point_type": "LP",
        "point_id": point_id,
        "released": True,
        "position_x": x,
        "position_y": y,
        "position_theta": yaw if yaw is not None else 0.0,
        "use_down_marker": True,
        "marker_group": {
            "marker_top_left_id": 0,
            "marker_top_right_id": marker_id,
            "marker_bottom_left_id": 0,
            "marker_bottom_right_id": 0,
        },
        "action": {
            "action_type": "none",
            "rec_file": "none",
            "use_object_marker": False,
        },
        "spin": True,
    }
    if yaw is not None:
        point["yaw"] = yaw
    return point


def _append_bidirectional_path(
    paths: List[Dict[str, Any]], start_point_id: int, end_point_id: int
) -> None:
    paths.append(
        _build_path(
            start_point_id=start_point_id,
            end_point_id=end_point_id,
            direction="forward",
        )
    )
    paths.append(
        _build_path(
            start_point_id=end_point_id,
            end_point_id=start_point_id,
            direction="backward",
        )
    )


def _build_path(
    *,
    start_point_id: int,
    end_point_id: int,
    direction: str,
) -> Dict[str, Any]:
    return {
        "alias": "demo_path",
        "start_point_type": "LP",
        "start_point_id": start_point_id,
        "end_point_type": "LP",
        "end_point_id": end_point_id,
        "released": True,
        "direction": direction,
        "move_type": "by_marker",
    }
