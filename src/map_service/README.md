# map_service

Пакет JSON-карты маршрутов.

## Что внутри
- `map_service/map_data.py` - генератор демо-карты склада.
- `map_service/map_service.py` - ROS-сервис выдачи карты.
- `config/map_service.yaml` - имя сервиса карты.

## Ответственность
- Отдает JSON-граф с `point` и `path` через `/robot_data/map/get_map`.
- Хранит текущую демо-геометрию точек и ребер.

## Связи
- Используется `navigation_forklift/route_service` для построения маршрута.
- Используется `rviz/json_map_visualizer` для отображения карты.
- Использует `forklift_interfaces/StringWithJson`.
