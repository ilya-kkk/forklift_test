import datetime as dt
from typing import Any, Dict, List, Tuple


MapDict = Dict[str, Any]
PointDict = Dict[str, Any]


def index_points(
    map_data: MapDict,
) -> Tuple[Dict[int, PointDict], Dict[str, PointDict]]:
    points = list(map_data.get("point", []))
    if not points:
        raise ValueError("map contains no points")

    points_by_id: Dict[int, PointDict] = {}
    points_by_alias: Dict[str, PointDict] = {}
    for point in points:
        point_id = int(point["point_id"])
        alias = str(point["alias"])
        points_by_id[point_id] = point
        points_by_alias[alias] = point

    return points_by_id, points_by_alias


def resolve_point(
    value: Any,
    points_by_id: Dict[int, PointDict],
    points_by_alias: Dict[str, PointDict],
) -> PointDict:
    if isinstance(value, dict):
        if "alias" in value:
            return resolve_point(value["alias"], points_by_id, points_by_alias)
        if "point_id" in value:
            return resolve_point(value["point_id"], points_by_id, points_by_alias)
        raise ValueError("point object must contain 'alias' or 'point_id'")

    if isinstance(value, int):
        if value in points_by_id:
            return points_by_id[value]
        raise ValueError(f"unknown point_id {value}")

    normalized = str(value).strip()
    if normalized in points_by_alias:
        return points_by_alias[normalized]
    if normalized.isdigit():
        point_id = int(normalized)
        if point_id in points_by_id:
            return points_by_id[point_id]

    raise ValueError(f"unknown point '{value}'")


def build_geojson_route_graph(
    map_data: MapDict,
    *,
    frame_id: str = "map",
    edge_id_offset: int = 10000,
    graph_name: str = "forklift_demo_route_graph",
) -> MapDict:
    points_by_id, _ = index_points(map_data)

    features: List[MapDict] = []
    for point_id in sorted(points_by_id):
        point = points_by_id[point_id]
        metadata = {
            "alias": str(point.get("alias", "")),
            "point_type": str(point.get("point_type", "")),
            "released": bool(point.get("released", True)),
            "spin": bool(point.get("spin", False)),
            "marker_group": dict(point.get("marker_group", {})),
            "action": dict(point.get("action", {})),
        }
        for key in (
            "reverse_entry",
            "reverse_final_edge",
            "pallet",
            "has_pallet",
            "pallet_present",
            "contains_pallet",
            "occupied_by_pallet",
        ):
            if key in point:
                metadata[key] = point[key]

        features.append(
            {
                "type": "Feature",
                "properties": {
                    "id": point_id,
                    "frame": frame_id,
                    "metadata": metadata,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [
                        float(point["position_x"]),
                        float(point["position_y"]),
                    ],
                },
            }
        )

    for index, path in enumerate(map_data.get("path", [])):
        start_point_id = int(path["start_point_id"])
        end_point_id = int(path["end_point_id"])
        if start_point_id not in points_by_id or end_point_id not in points_by_id:
            continue

        start_point = points_by_id[start_point_id]
        end_point = points_by_id[end_point_id]
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "id": edge_id_offset + index,
                    "startid": start_point_id,
                    "endid": end_point_id,
                    "metadata": {
                        "alias": str(path.get("alias", "")),
                        "direction": str(path.get("direction", "")),
                        "move_type": str(path.get("move_type", "")),
                        "released": bool(path.get("released", True)),
                    },
                },
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [
                        [
                            [
                                float(start_point["position_x"]),
                                float(start_point["position_y"]),
                            ],
                            [
                                float(end_point["position_x"]),
                                float(end_point["position_y"]),
                            ],
                        ]
                    ],
                },
            }
        )

    return {
        "type": "FeatureCollection",
        "name": graph_name,
        "date_generated": dt.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "features": features,
    }
