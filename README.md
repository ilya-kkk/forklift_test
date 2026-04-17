# ROS 2 Forklift Navigation Stack

Стенд на `ROS 2 Humble + Gazebo Sim + Nav2` для rear-steer погрузчика.

Текущая архитектура:

`map_service (JSON graph)`  
`-> route_graph_builder (GeoJSON adapter)`  
`-> nav2_route / route_server`  
`-> route_service (thin compatibility layer)`  
`-> controller_server (fixed-lookahead Pure Pursuit)`  
`-> collision_monitor`  
`-> cmd_vel_to_motors`  
`-> Gazebo model`

## Что запущено

- `map_service`
  Отдает исходную JSON-карту в формате `point[]` / `path[]`.

- `route_graph_builder`
  При старте запрашивает карту у `map_service`, конвертирует ее в GeoJSON и сохраняет в `/tmp/forklift_demo_route_graph.geojson`.

- `nav2_route/route_server`
  Загружает GeoJSON route graph и вычисляет маршрут по графу.

- `route_service`
  Легкий compatibility-layer для старого API `/robot_data/route/go_to_point`.
  Сам путь не считает: он только резолвит точки через `map_service`, вызывает `ComputeRoute` у `route_server` и передает полученный `Path` в `FollowPath`.

- `nav2_controller/controller_server`
  Ведет робот по `Path` через `RegulatedPurePursuitController`, сконфигурированный как fixed-lookahead pure pursuit.

- `nav2_collision_monitor/collision_monitor`
  Фильтрует `cmd_vel` между `controller_server` и `cmd_vel_to_motors`.

- `cmd_vel_to_motors`
  Преобразует `/cmd_vel` в команды рулевого и ведущего колеса модели.

- `scan_sector_filter`
  Фильтрует лидар перед Nav2 и Collision Monitor.

- `slam_toolbox`
  Публикует `map -> odom`.

- `ros_gz_bridge`
  Связывает Gazebo и ROS по времени, lidar, odom, TF, joint states и motor commands.

## Что удалено из runtime

Из основного launch и runtime-пайплайна убраны:

- `demo_route_loop`
- `hardcoded_route_sender`
- `keyboard_teleop`
- `rviz_teleop_marker`
- ручное shortest-path планирование поверх JSON-карты
- corner smoothing / densify логика для runtime route planning
- runtime-переключение `FollowPath.allow_reversing`

## JSON карта

`map_service` сохраняет старый контракт и возвращает:

- `point[]`
  Узлы графа с `alias`, `point_id`, координатами и метаданными.

- `path[]`
  Направленные ребра графа между `start_point_id` и `end_point_id`.

Файл-источник:

- [map_data.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_data.py)

Сервис:

- [map_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_service.py)

## Конвертация в route graph

Конвертация живет отдельно от планирования:

- [route_graph.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_graph.py)
- [route_graph_builder.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_graph_builder.py)

Mapping:

- `point[]` -> GeoJSON `Point` features с `id = point_id`
- `path[]` -> GeoJSON `MultiLineString` features с `startid` / `endid`
- `alias`, `direction`, `marker_group`, `action` -> `metadata`

Route graph для `route_server` создается автоматически при старте launch.

## Route Server и сервис совместимости

`route_server` теперь единственный компонент, который считает маршрут по графу.

Файл его конфига:

- [route_server_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/route_server_params.yaml)

Старый внешний сервис `/robot_data/route/go_to_point` сохранен, но стал тонким фасадом:

- [route_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_service.py)

Он:

1. принимает старый JSON-запрос;
2. резолвит `start` / `goal` в `point_id` через `map_service`;
3. вызывает `nav2_msgs/action/ComputeRoute`;
4. публикует полученный путь в `/route_path`;
5. отправляет этот путь в `follow_path`.

Поле `arrival_mode` принимается только для совместимости контракта. Новый pipeline исполняет один forward path без ручной сегментации и без своего route geometry.

Пример запроса:

```bash
ros2 service call /robot_data/route/go_to_point ros2_templates/srv/StringWithJson \
  '{"message":"{\"start\":\"0001\",\"goal\":\"0004\",\"arrival_mode\":\"rear\"}"}'
```

## Контроллер

Контроллер упрощен до fixed-lookahead pure pursuit на базе `RegulatedPurePursuitController`.

Ключевые настройки:

- `lookahead_dist: 1.5`
- `min_lookahead_dist: 1.5`
- `max_lookahead_dist: 1.5`
- `use_velocity_scaled_lookahead_dist: false`
- `use_regulated_linear_velocity_scaling: false`
- `use_cost_regulated_linear_velocity_scaling: false`
- `use_rotate_to_heading: true`
- `allow_reversing: false`

Файл:

- [nav2_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/nav2_params.yaml)

## Collision Monitor

Collision Monitor стоит между выходом Nav2 и низким уровнем:

- `controller_server` remap `/cmd_vel -> /cmd_vel_raw`
- `collision_monitor` читает `/cmd_vel_raw`
- `collision_monitor` публикует итоговый `/cmd_vel`
- `cmd_vel_to_motors` читает `/cmd_vel`

Конфиг:

- [collision_monitor_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/collision_monitor_params.yaml)

Базовая конфигурация:

- `PolygonStop` перед роботом
- `PolygonSlow` шире и дальше перед роботом
- источник наблюдений: `/scan`
- базовая frame: `base_link`

## Launch

Главный launch:

- [sim_followpath.launch.py](/home/user/forklift_test/src/forklift_demo_description/launch/sim_followpath.launch.py)

Он делает следующее:

1. запускает Gazebo, bridge, `robot_state_publisher`, `slam_toolbox`, `controller_server`, `scan_sector_filter`, `map_service`, `cmd_vel_to_motors`;
2. запускает `route_graph_builder`;
3. после успешного завершения builder поднимает `route_server`, `collision_monitor`, `lifecycle_manager_navigation` и новый `route_service`;
4. опционально поднимает RViz.

Lifecycle manager теперь управляет:

- `controller_server`
- `route_server`
- `collision_monitor`

## Запуск

Полный запуск:

```bash
docker compose up --build
```

Только симуляция:

```bash
docker compose up sim
```

Отдельно RViz:

```bash
docker compose up rviz
```

Отдельно RQT steering для `/cmd_vel`:

```bash
docker compose up rqt
```

Важно:

- `docker build` теперь собирает только образ с системными зависимостями.
- `colcon build` выполняется уже внутри контейнера `sim`.
- `build/`, `install/`, `log/` вынесены в named volumes `colcon_*`, поэтому повторные `docker compose up sim` идут заметно быстрее и не требуют пересборки образа при правках в `src`.
- `docker compose build` нужен в основном после изменений в `Dockerfile`.

Локально без Docker:

```bash
colcon build --symlink-install --packages-select forklift_demo_control forklift_demo_description
source install/setup.bash
ros2 launch forklift_demo_description sim_followpath.launch.py
```

## Проверка

Проверить карту:

```bash
ros2 service call /robot_data/map/get_map ros2_templates/srv/StringWithJson '{"message":"{}"}'
```

Проверить route graph file внутри контейнера/окружения:

```bash
ls -l /tmp/forklift_demo_route_graph.geojson
```

Проверить action Route Server:

```bash
ros2 action list | grep compute_route
```

Проверить Collision Monitor:

```bash
ros2 node info /collision_monitor
```

## Основные файлы

- [map_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_service.py)
- [route_graph.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_graph.py)
- [route_graph_builder.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_graph_builder.py)
- [route_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_service.py)
- [cmd_vel_to_motors.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/cmd_vel_to_motors.py)
- [scan_sector_filter.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/scan_sector_filter.py)
- [nav2_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/nav2_params.yaml)
- [route_server_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/route_server_params.yaml)
- [collision_monitor_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/collision_monitor_params.yaml)
- [sim_followpath.launch.py](/home/user/forklift_test/src/forklift_demo_description/launch/sim_followpath.launch.py)
