# ROS 2 Forklift Demo

Небольшой стенд на `ROS 2 Humble + Gazebo Sim + Nav2` для rear-steer погрузчика.

Система построена вокруг идеи "карта в виде графа точек и связей + сервис маршрута". Пользователь не публикует `Path` руками и не работает с координатами напрямую. Вместо этого он вызывает сервис, говорит "из точки X в точку Y, приехать front/rear", а дальше маршрут строится и исполняется автоматически.

## Из чего состоит система

Основные узлы:

- `map_service`
  Возвращает JSON-карту в формате `point/path`.

- `route_service`
  Принимает запрос "откуда -> куда -> как приехать", сам запрашивает карту, строит маршрут по графу и отправляет `FollowPath` в Nav2.

- `cmd_vel_to_tricycle`
  Берет `cmd_vel` от Nav2 и переводит его в команды именно для нашей rear-steer кинематики: угол заднего рулевого колеса, скорость ведущего колеса и `Twist` в Gazebo.

- `demo_route_loop`
  Опциональный демонстрационный узел. Если включен, сам по кругу вызывает сервис маршрута и имитирует перевозку палеты между несколькими точками.

- `controller_server` из Nav2
  Исполняет уже готовый путь через `RegulatedPurePursuitController`.

- `slam_toolbox`
  Публикует `map -> odom`.

- `ros_gz_bridge`
  Связывает Gazebo и ROS по времени, лидарам, одометрии, TF, joint states и управляющим топикам.

## Кто за что отвечает

### `/robot_data/map/get_map`

Сервис карты. Возвращает JSON с двумя массивами:

- `point`
  Описание точек карты: `point_id`, `alias`, координаты, метаданные.

- `path`
  Направленные связи между точками.

Источник данных: [map_data.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_data.py)

Обработчик сервиса: [map_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_service.py)

### `/robot_data/route/go_to_point`

Сервис маршрута. Принимает JSON примерно такого вида:

```json
{
  "start": "0001",
  "goal": "0004",
  "arrival_mode": "front"
}
```

`start` и `goal` можно передавать:

- по `alias`, например `"0001"`
- по `point_id`, например `2`

`arrival_mode` понимает:

- `front`, `forward`, `передом`
- `rear`, `reverse`, `backward`, `задом`

Обработчик сервиса: [route_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_service.py)

## Что значит `front` и `rear`

В пользовательской логике:

- `front` = сторона поворотных колес
- `rear` = сторона вил и паразитных передних колес

Важно: внутри низкого уровня есть свои режимы `BODY_FIRST` и `FORKS_FIRST`. Это внутренняя кинематическая деталь, а не то, что должен помнить пользователь сервиса.

Текущее соответствие такое:

- пользовательский `front` мапится на внутренний `FORKS_FIRST`
- пользовательский `rear` мапится на внутренний `BODY_FIRST`

## Как строится маршрут

Алгоритм в [route_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_service.py) в кратце такой:

1. Сервис принимает запрос `start / goal / arrival_mode`.
2. Запрашивает карту у `/robot_data/map/get_map`.
3. Индексирует точки:
   - по `point_id`
   - по `alias`
4. Разрешает `start` и `goal` в реальные точки карты.
5. Строит кратчайший путь по графу.
   Вес ребра = обычное евклидово расстояние между двумя точками.
6. Получает цепочку узлов, например `["0003", "0002", "0005"]`.
7. Превращает эту цепочку в ломаную и дополнительно "уплотняет" ее промежуточными точками, чтобы Nav2 вел робот не по редким вершинам, а по нормальному `Path`.
8. В зависимости от `arrival_mode` режет маршрут на сегменты:
   - `front`: весь маршрут одним сегментом
   - `rear`: весь маршрут, кроме последнего ребра, идет одним режимом; последний отрезок в цель идет отдельным сегментом другим режимом
9. Перед отправкой каждого сегмента:
   - публикует `/motion_mode`
   - переключает параметр `FollowPath.allow_reversing` у `controller_server`
10. Отправляет сегмент в action `follow_path`.

Итог: пользователь всегда вызывает один сервис, а внутри маршрут может превратиться в 1 или 2 `FollowPath`-сегмента в зависимости от того, как нужно приехать в цель.

## Как работает низкий уровень

[cmd_vel_to_tricycle.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/cmd_vel_to_tricycle.py) делает следующее:

- подписывается на `cmd_vel`
- подписывается на `/motion_mode`
- приводит линейную скорость к нужному направлению движения
- для reverse-режима дополнительно ограничивает скорость коэффициентом `reverse_velocity_scale`
- пересчитывает `omega`, чтобы при смене знака скорости не ломалась кривизна пути
- считает угол заднего рулевого колеса по модели rear steering
- ограничивает скорость изменения руля
- публикует:
  - угол заднего рулевого колеса
  - скорость ведущего колеса
  - `Twist` в Gazebo

То есть Nav2 по-прежнему работает как источник `cmd_vel`, а вся специфичная кинематика погрузчика живет в одном отдельном узле.

## Текущая карта

Карта задается в [map_data.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_data.py).

Точки:

- `0000` / `1`: `(-4.0, -4.0)`
- `0001` / `2`: `(-4.0, 4.0)`
- `0002` / `3`: `(0.0, 4.0)`
- `0003` / `4`: `(4.0, 4.0)`
- `0004` / `5`: `(4.0, -4.0)`
- `0005` / `6`: `(0.0, 1.0)`
- `0006` / `7`: `(-2.0, 1.0)`
- `0007` / `8`: `(-2.0, -4.0)`

Связи:

- `1 <-> 2`
- `2 <-> 3`
- `3 <-> 4`
- `4 <-> 5`
- `5 <-> 1`
- `3 <-> 6`
- `7 <-> 8`
- `1 <-> 8`
- `8 <-> 5`

ASCII-схема:

```text
2 (-4,4) -------- 3 (0,4) -------- 4 (4,4)
                     |
                     |
                  6 (0,1)

1 (-4,-4) ---- 8 (-2,-4) -------- 5 (4,-4)
                 |
                 |
              7 (-2,1)
```

Это ориентированный граф, но для большинства участков обе стороны заданы явными отдельными ребрами.

## Автодемо

По умолчанию launch поднимает `demo_route_loop`. Он нужен только для демонстрации без ручных сервисных вызовов.

Если хочешь управлять только сервисами вручную, отключай его так:

```bash
ros2 launch forklift_demo_description sim_followpath.launch.py run_demo_loop:=false
```

## Запуск

Полный запуск:

```bash
docker compose up --build
```

Только симуляция:

```bash
docker compose up --build sim
```

Только RViz:

```bash
docker compose up --build rviz
```

Главный launch: [sim_followpath.launch.py](/home/user/forklift_test/src/forklift_demo_description/launch/sim_followpath.launch.py)

Он поднимает:

- Gazebo
- bridge
- `robot_state_publisher`
- `slam_toolbox`
- `controller_server`
- `lifecycle_manager`
- `cmd_vel_to_tricycle`
- `map_service`
- `route_service`
- `demo_route_loop` при `run_demo_loop:=true`
- RViz при `launch_rviz:=true`

## Сборка внутри контейнера

Если уже находишься в `forklift_demo_sim`:

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros2_templates forklift_demo_control
source install/setup.bash
```

## Примеры вызова сервисов

Сначала зайти в контейнер:

```bash
docker exec -it forklift_demo_sim bash
```

Потом:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Получить карту:

```bash
ros2 service call /robot_data/map/get_map ros2_templates/srv/StringWithJson '{"message":"{}"}'
```

Поехать из `0001` в `0004` и приехать `front`:

```bash
ros2 service call /robot_data/route/go_to_point ros2_templates/srv/StringWithJson '{"message":"{\"start\":\"0001\",\"goal\":\"0004\",\"arrival_mode\":\"front\"}"}'
```

Поехать из `0001` в `0004` и приехать `rear`:

```bash
ros2 service call /robot_data/route/go_to_point ros2_templates/srv/StringWithJson '{"message":"{\"start\":\"0001\",\"goal\":\"0004\",\"arrival_mode\":\"rear\"}"}'
```

То же самое по `point_id`:

```bash
ros2 service call /robot_data/route/go_to_point ros2_templates/srv/StringWithJson '{"message":"{\"start\":2,\"goal\":5,\"arrival_mode\":\"front\"}"}'
```

## Ключевые файлы

- [map_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_service.py)
  Отдает карту.

- [route_service.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/route_service.py)
  Строит путь по карте и исполняет его через Nav2.

- [cmd_vel_to_tricycle.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/cmd_vel_to_tricycle.py)
  Преобразует `cmd_vel` в команды rear-steer базы.

- [demo_route_loop.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/demo_route_loop.py)
  Автоматический демонстрационный цикл.

- [map_data.py](/home/user/forklift_test/src/forklift_demo_control/forklift_demo_control/map_data.py)
  Описание точек и ребер графа.

- [controllers.yaml](/home/user/forklift_test/src/forklift_demo_description/config/controllers.yaml)
  Параметры low-level узла `cmd_vel_to_tricycle`.

- [nav2_params.yaml](/home/user/forklift_test/src/forklift_demo_description/config/nav2_params.yaml)
  Параметры `RegulatedPurePursuitController` и local costmap.

- [forklift_demo.urdf.xacro](/home/user/forklift_test/src/forklift_demo_description/urdf/forklift_demo.urdf.xacro)
  URDF/xacro для `robot_state_publisher`.

- [model.sdf](/home/user/forklift_test/src/forklift_demo_description/models/forklift_demo/model.sdf)
  Gazebo-модель робота.

- [StringWithJson.srv](/home/user/forklift_test/src/ros2_templates/srv/StringWithJson.srv)
  Общий сервисный интерфейс JSON-запросов и JSON-ответов.
