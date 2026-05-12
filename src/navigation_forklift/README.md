# navigation_forklift

Пакет навигационного уровня погрузчика.

## Что внутри
- `navigation_forklift/route_service.py` - сервисный оркестратор маршрутов.
- `config/route_service.yaml` - параметры `route_service`.
- `config/nav2_params.yaml` - параметры `controller_server`, `behavior_server`, costmap и lifecycle manager.
- `config/slam_toolbox.yaml` - параметры `slam_toolbox` для демо/навигации.

## Ответственность
- Принимает команды `/forklift_nav/move_to`, `/forklift_nav/revers_move_to`, `/robot_data/route/go_to_point`.
- Запрашивает карту у `map_service`.
- Строит путь по JSON-графу через Dijkstra.
- Отдает путь в Nav2 через `FollowPath`, `Spin`, `DriveOnHeading`.

## Связи
- Зависит от `map_service` через `/robot_data/map/get_map`.
- Зависит от `forklift_interfaces/StringWithJson`.
- Публикует `/route_path` для RViz.
- Использует TF `map -> base_link`.
- Выходные скорости Nav2 идут дальше в `collision_monitor` и `cmd_vel_to_motors`.
