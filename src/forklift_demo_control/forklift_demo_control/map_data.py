import json
from typing import Any, Dict, List


MapJson = Dict[str, List[Dict[str, Any]]]


def build_demo_map() -> MapJson:
    points = [
        _build_point(alias="0000", point_id=1, x=-4.0, y=-4.0, marker_id=0),
        _build_point(alias="0001", point_id=2, x=-4.0, y=4.0, marker_id=1),
        _build_point(alias="0002", point_id=3, x=0.0, y=4.0, marker_id=2),
        _build_point(alias="0003", point_id=4, x=4.0, y=4.0, marker_id=3),
        _build_point(alias="0004", point_id=5, x=4.0, y=-4.0, marker_id=4),
        _build_point(alias="0005", point_id=6, x=0.0, y=1.0, marker_id=5),
        _build_point(alias="0006", point_id=7, x=-2.0, y=1.0, marker_id=6),
        _build_point(alias="0007", point_id=8, x=-2.0, y=-4.0, marker_id=7),
    ]

    paths = [
        _build_path(start_point_id=1, end_point_id=2, direction="forward"),
        _build_path(start_point_id=2, end_point_id=3, direction="forward"),
        _build_path(start_point_id=3, end_point_id=4, direction="forward"),
        _build_path(start_point_id=4, end_point_id=5, direction="forward"),
        _build_path(start_point_id=5, end_point_id=1, direction="forward"),
        _build_path(start_point_id=2, end_point_id=1, direction="backward"),
        _build_path(start_point_id=3, end_point_id=2, direction="backward"),
        _build_path(start_point_id=4, end_point_id=3, direction="backward"),
        _build_path(start_point_id=5, end_point_id=4, direction="backward"),
        _build_path(start_point_id=1, end_point_id=5, direction="backward"),
        _build_path(start_point_id=3, end_point_id=6, direction="forward"),
        _build_path(start_point_id=6, end_point_id=3, direction="backward"),
        _build_path(start_point_id=7, end_point_id=8, direction="forward"),
        _build_path(start_point_id=8, end_point_id=7, direction="backward"),
        _build_path(start_point_id=1, end_point_id=8, direction="forward"),
        _build_path(start_point_id=8, end_point_id=1, direction="backward"),
        _build_path(start_point_id=8, end_point_id=5, direction="forward"),
        _build_path(start_point_id=5, end_point_id=8, direction="backward"),
    ]
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
) -> Dict[str, Any]:
    return {
        "alias": alias,
        "point_type": "LP",
        "point_id": point_id,
        "released": True,
        "position_x": x,
        "position_y": y,
        "position_theta": 0.0,
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
