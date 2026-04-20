# ROS 2 Forklift Navigation Stack

Стенд на `ROS 2 Jazzy + Gazebo Sim + Nav2` для rear-steer погрузчика.

## Текущая архитектура

`map_service (JSON)`
-> `route_graph_builder (JSON -> GeoJSON)`
-> `nav2_route/route_server (ComputeRoute)`
-> `route_service (compat API /robot_data/route/go_to_point)`
-> `controller_server (/cmd_vel_raw)`
-> `collision_monitor (/cmd_vel_raw -> /cmd_vel)`
-> `cmd_vel_to_motors (/cmd_vel -> steering/wheel commands)`
-> `Gazebo joint controllers`

Отдельно:
- `slam_toolbox` использует только верхний лидар `/scan_up`
- `scan_sector_filter` ограничивает/фильтрует нижние лидары для Nav2 и Collision Monitor

## Что запускается в `sim_followpath.launch.py`

Главный launch:
- поднимает Gazebo Sim (`gz_sim.launch.py`) с переключателем GUI/headless
- публикует `robot_description` и спавнит робота + паллету
- стартует `ros_gz_bridge`
- стартует `slam_toolbox`, Nav2 (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`)
- стартует низкоуровневые ноды управления:
  - `cmd_vel_to_motors`
  - `fork_position_controller`
  - `scan_sector_filter` (общий фронтальный фильтр `/scan_raw -> /scan`)
  - дополнительные секторные фильтры:
    - `/scan_left -> /scan_left_limited`
    - `/scan_right -> /scan_right_limited`
    - `/scan_raw -> /scan_rear_limited`
- стартует `map_service`
- запускает `route_graph_builder`; после его завершения поднимает:
  - `route_server`
  - `collision_monitor`
  - `lifecycle_manager_navigation`
  - `route_service`
- опционально поднимает RViz

## Launch-аргументы

- `launch_gz_gui` (default: `true`)  
  `true` -> Gazebo с GUI, `false` -> headless.
- `launch_rviz` (default: `false`)
- `enable_cmd_vel_to_motors` (default: `true`)
- `use_sim_time` (default: `true`)
- `world`, `x`, `y`, `yaw`

## Цепочка скоростей

- `controller_server` remap: `/cmd_vel -> /cmd_vel_raw`
- `collision_monitor` читает `/cmd_vel_raw`, публикует безопасный `/cmd_vel`
- `cmd_vel_to_motors` читает `/cmd_vel` и публикует:
  - `/forklift/right_steering_cmd`
  - `/forklift/right_wheel_cmd`

Параметры, зафиксированные в launch:
- `cmd_vel_topic: /cmd_vel`
- `drive_wheel_velocity_sign: -1.0`

Это сделано для соответствия ориентации ведущего колеса в URDF и движения `base_link` по `+X` при положительном `linear.x`.

## Источники лидаров

### SLAM

`slam_toolbox`:
- `scan_topic: /scan_up`

### Collision Monitor

`collision_monitor` берет только нижние лидары через ограниченные топики:
- `scan_left_limited`
- `scan_right_limited`
- `scan_rear_limited`

## Ограничение FOV нижних лидаров

Ограничение сделано надежно через ROS-фильтрацию (`scan_sector_filter`), чтобы не зависеть от внутреннего поведения рендера/сенсора в Gazebo:

- Левый: оставить `[-90, 180]`
- Правый: оставить `[-180, 90]`
- Задний: оставить `[-45, 45]`

RViz и Collision Monitor подписаны уже на `*_limited` топики.

## Collision Monitor (velocity polygons)

Используется `velocity_polygon` для двух зон:
- `VelocityPolygonStop` (`action_type: stop`)
- `VelocityPolygonSlow` (`action_type: slowdown`, `slowdown_ratio: 0.35`)

Текущие «рамки» применяются только для движения вперед (`forward`, `linear.x > 0.05`).
Для не-forward скоростей есть fallback-полигон `stopped`.

Топики публикации полигонов:
- `/collision_monitor/velocity_polygon_stop`
- `/collision_monitor/velocity_polygon_slowdown`

## Модель и физика

### Масса

Суммарная масса по URDF: `1178.1788836413 кг`.

### Центр масс `base_link`

`base_link` inertial origin:
- `xyz="0.94148 -7.6988E-08 0.25"`

### Трение колес/пола

Колеса (`right_wheel`, `left_wheel`, `fork_left_wheel`, `fork_right_wheel`):
- `mu1 = mu2 = 1000.0`

Пол (`square_room.sdf`):
- ODE `mu=200.0`, `mu2=200.0`, `slip1=0.0`, `slip2=0.0`
- Bullet `friction=200.0`, `friction2=200.0`

## Gazebo/ROS bridge

`bridge_config.yaml` включает:
- clock
- все lidar/camera topic-и
- `/odom`, `/tf`, `/joint_states`
- команды:
  - `/forklift/right_steering_cmd`
  - `/forklift/right_wheel_cmd`
  - `/forklift/fork_cmd`
  - `/forklift/fork_velocity_cmd`

## RViz конфиг

В `rviz/demo.rviz`:
- `RearLidar`, `LeftLidar`, `RightLidar`, `UpLidar` отображаются как:
  - `Style: Points`
  - `Color Transformer: FlatColor`
- `CollisionStop/CollisionSlowdown` подписаны на:
  - `/collision_monitor/velocity_polygon_stop`
  - `/collision_monitor/velocity_polygon_slowdown`
- `PlannedPath` line width: `0.1`

## Docker

### Полный запуск

```bash
docker compose up --build
```

### Только симуляция

```bash
docker compose up sim
```

### Отдельно RViz

```bash
docker compose up rviz
```

### Отдельно rqt_robot_steering

```bash
docker compose up rqt
```

`rqt_robot_steering` по умолчанию публикует в:
- `/cmd_vel_raw`

## Локальный запуск без Docker

```bash
colcon build --symlink-install --packages-select ros2_templates forklift_demo_control forklift_demo_description
source install/setup.bash
ros2 launch forklift_demo_description sim_followpath.launch.py launch_gz_gui:=true
```

## Быстрые проверки

### Проверка map service

```bash
ros2 service call /robot_data/map/get_map ros2_templates/srv/StringWithJson '{"message":"{}"}'
```

### Проверка route API совместимости

```bash
ros2 service call /robot_data/route/go_to_point ros2_templates/srv/StringWithJson \
  '{"message":"{\"start\":\"0001\",\"goal\":\"0004\",\"arrival_mode\":\"rear\"}"}'
```

### Проверка ограниченных сканов

```bash
ros2 topic echo /scan_left_limited --once | egrep "angle_min|angle_max"
ros2 topic echo /scan_right_limited --once | egrep "angle_min|angle_max"
ros2 topic echo /scan_rear_limited --once | egrep "angle_min|angle_max"
```

### Проверка полигонов collision monitor

```bash
ros2 topic list | grep collision_monitor
```

## Основные файлы

- `src/forklift_demo_description/launch/sim_followpath.launch.py`
- `src/forklift_demo_description/config/collision_monitor_params.yaml`
- `src/forklift_demo_description/config/nav2_params.yaml`
- `src/forklift_demo_description/config/slam_toolbox.yaml`
- `src/forklift_demo_description/config/route_server_params.yaml`
- `src/forklift_demo_description/config/bridge_config.yaml`
- `src/forklift_demo_description/urdf/forklift_demo.urdf.xacro`
- `src/forklift_demo_description/worlds/square_room.sdf`
- `src/forklift_demo_description/rviz/demo.rviz`
- `src/forklift_demo_control/forklift_demo_control/cmd_vel_to_motors.py`
- `src/forklift_demo_control/forklift_demo_control/scan_sector_filter.py`
- `src/forklift_demo_control/forklift_demo_control/fork_position_controller.py`
- `src/forklift_demo_control/forklift_demo_control/map_service.py`
- `src/forklift_demo_control/forklift_demo_control/route_graph_builder.py`
- `src/forklift_demo_control/forklift_demo_control/route_service.py`
