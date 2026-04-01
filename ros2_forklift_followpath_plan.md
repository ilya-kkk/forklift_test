# План для агента: ROS 2 path following для форклифта с Pure Pursuit без planner'а

## Цель

Собрать минимальный, воспроизводимый ROS 2 + Gazebo headless + RViz стенд, в котором простой форклифт-робот ездит по заранее заданному маршруту в форме буквы `Г` туда-обратно.

Источник маршрута в продовой архитектуре — внешний fleet manager. Для демо и теста маршрут нужно захардкодить внутри тестовой ROS 2 ноды и отправлять его каждые 20 секунд.

## Что должно получиться в конце

- Docker Compose поднимает весь стек одной командой.
- Gazebo запускается в headless режиме.
- RViz запускается и показывает робота, TF, карту, путь и лазер.
- Робот описан простым URDF из примитивов.
- Кинематика робота на текущем этапе: 
  - два передних неведущих колеса,
  - одно заднее ведущее поворотное колесо по центру.
- Источник одометрии / локализации: `slam_toolbox`.
- Path following реализован через `nav2_controller` + `RegulatedPurePursuitController`.
- Planner НЕ используется.
- Тестовая нода отправляет `FollowPath` goal c маршрутом в виде буквы `Г` раз в 20 секунд.
- Робот ездит по маршруту туда и обратно по кругу.

---

## Архитектурная идея

Использовать только control-слой Nav2, без planner server и без BT navigator:

1. **Gazebo** симулирует мир и робота.
2. **ros2_control** или минимальный мост публикует состояние робота и принимает команды.
3. **slam_toolbox** строит/поддерживает `map -> odom`.
4. **controller_server** из Nav2 принимает `FollowPath` action goal.
5. **Regulated Pure Pursuit** превращает путь в `cmd_vel`.
6. **Низкоуровневый converter node** превращает `cmd_vel` в:
   - скорость заднего колеса,
   - угол поворота заднего колеса.
7. **Тестовая route_sender node** отправляет заранее заданный путь в форме буквы `Г` каждые 20 секунд, чередуя прямой и обратный маршрут.

---

## Структура репозитория

Предлагаемая структура:

```text
repo/
  docker-compose.yml
  .env
  Dockerfile
  entrypoint.sh
  src/
    forklift_demo_description/
      package.xml
      CMakeLists.txt
      urdf/
        forklift_demo.urdf.xacro
      rviz/
        demo.rviz
      worlds/
        square_room.sdf
      launch/
        sim_followpath.launch.py
      config/
        nav2_params.yaml
        slam_toolbox.yaml
        controllers.yaml
    forklift_demo_control/
      package.xml
      setup.py
      resource/
      forklift_demo_control/
        __init__.py
        cmd_vel_to_tricycle.py
        hardcoded_route_sender.py
        path_utils.py
```

---

## Пакеты и технологии

### Базовые зависимости

- ROS 2 Humble
- Gazebo Sim / `ros_gz_sim`
- `ros_gz_bridge`
- `robot_state_publisher`
- `joint_state_publisher` или источник joint states из симуляции
- `xacro`
- `slam_toolbox`
- `nav2_controller`
- `nav2_regulated_pure_pursuit_controller`
- `nav2_msgs`
- `nav2_lifecycle_manager`
- `nav2_costmap_2d`
- `tf2_ros`
- `rviz2`
- `ros2_control`
- `ros2_controllers`
- `tricycle_controller` — только если получится быстро встроить без конфликтов с rear-steer схемой; иначе использовать собственный converter node

### Рекомендуемый путь реализации

На первом этапе НЕ пытаться полностью завести `tricycle_controller` в Gazebo, если это тормозит прогресс. Быстрее и надёжнее:

- использовать собственную ноду `cmd_vel_to_tricycle.py`,
- публиковать команды на steering и drive joints,
- вычислять команды из `Twist` по простой bicycle/tricycle модели.

Формулы первого приближения:

- `delta = atan(L * omega / max(|v|, eps))`
- `wheel_angular_velocity = v / wheel_radius`

Сделать saturations:

- max steering angle,
- max steering rate,
- max wheel speed.

---

## Мир в Gazebo

### Требование

Сделать квадратную комнату в SDF:

- пол,
- 4 стены,
- опционально 1-2 коробки как ориентиры для лидарного SLAM.

### Рекомендации

- Размер комнаты: например 8x8 или 10x10 метров.
- Стены достаточно высокие для уверенного lidar scan.
- В центре оставить свободное пространство.
- Стартовую позу робота выбрать так, чтобы маршрут буквой `Г` помещался целиком внутри комнаты.

---

## URDF робота

### Упростить модель до минимально достаточной

Создать URDF/Xacro из примитивов:

- `base_link` — коробка.
- `front_left_wheel_link` — цилиндр.
- `front_right_wheel_link` — цилиндр.
- `rear_steer_link` — промежуточный steering узел.
- `rear_drive_wheel_link` — цилиндр на steer joint.
- `laser_link` — lidar сверху или спереди.

### Joints

- два передних колеса — можно зафиксировать или дать им свободное вращение только для визуала;
- `rear_steering_joint` — revolute по `z`;
- `rear_wheel_joint` — continuous для тяги.

### Frames

Обязательно:

- `base_link`
- `laser_link`
- колёсные link/frame

### Геометрические параметры, которые вынести в xacro args

- длина базы,
- ширина базы,
- радиус колеса,
- wheelbase,
- положение лазера,
- limits угла steering.

---

## Сенсоры

### Лидар

Нужен 2D lidar plugin для Gazebo, чтобы `slam_toolbox` получал `/scan`.

Минимум:

- topic `/scan`
- `frame_id = laser_link`
- адекватный rate, например 10–15 Hz

### Одометрия

Поскольку пользователь указал, что источник одометрии — `slam_toolbox`, интерпретация для демо должна быть такой:

- `slam_toolbox` даёт `map -> odom` и позу на основе lidar + incoming odom,
- при этом для самой контроллерной петли всё равно нужен базовый `odom -> base_link` источник.

Поэтому в демо нужно сделать так:

1. Низкоуровневая симуляция публикует `odom -> base_link`.
2. `slam_toolbox` работает поверх `/scan` и `/odom`.
3. Nav2 controller использует глобальный frame `map`, robot base frame `base_link`, odom topic от симуляции.

Если попытаться вообще убрать `/odom`, `slam_toolbox` и tracking будут менее стабильны.

---

## Path following через Nav2 без planner'а

### Что запускать

Поднять только:

- `controller_server`
- `lifecycle_manager`
- локальную costmap
- `slam_toolbox`

Не поднимать:

- `planner_server`
- `bt_navigator`
- `behavior_server`
- `waypoint_follower`

### Controller plugin

Использовать:

- `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

### Важная логика

Тестовая нода НЕ должна публиковать путь просто в topic для исполнения.

Она должна:

- собрать `nav_msgs/Path`,
- отправить его как action goal типа `nav2_msgs/action/FollowPath` на action server `follow_path`.

### Дополнительно для визуализации

Параллельно публиковать этот же путь в topic, например:

- `/demo_path`

чтобы он был виден в RViz.

---

## route_sender: тестовая нода маршрута

### Поведение

Сделать Python-ноду `hardcoded_route_sender.py`, которая:

1. Ждёт, пока поднимутся `controller_server` и TF.
2. Создаёт маршрут в форме буквы `Г` в frame `map`.
3. Отправляет этот путь как `FollowPath` goal.
4. Публикует путь в `/demo_path`.
5. Через 20 секунд отправляет обратный маршрут.
6. Повторяет цикл бесконечно.

### Маршрут

Пример формы `Г`:

- старт: `(1.0, 1.0)`
- вправо до `(4.0, 1.0)`
- вверх до `(4.0, 4.0)`

Сделать path densification:

- интерполировать точки с шагом 0.05–0.1 м,
- задать orientation вдоль касательной.

### Почему это важно

Pure Pursuit лучше едет по плотному и гладкому пути, чем по 3 редким точкам.

---

## Низкоуровневая нода `cmd_vel_to_tricycle.py`

### Задача

Подписываться на `cmd_vel` и конвертировать команды базы в команды заднему steering/drive колесу.

### Вход

- `geometry_msgs/Twist` или `TwistStamped` — зависит от параметров `controller_server`

### Выход

Публикация команд на:

- `/rear_steering_joint_position_controller/command`
- `/rear_wheel_velocity_controller/command`

или на те топики, которые будут выбраны в реальном control graph.

### Формулы первого приближения

Пусть:

- `v = linear.x`
- `omega = angular.z`
- `L = wheelbase`
- `R = wheel_radius`

Тогда:

- `delta = atan2(L * omega, v)` при `|v| > eps`
- если `|v| <= eps`, то либо держать предыдущий steering angle, либо задавать 0
- `wheel_vel = v / R`

### Защиты

- clamp steering to `[-max_delta, max_delta]`
- clamp wheel velocity
- опционально limit steering rate

### Примечание

На текущем этапе цель — завести работающий demo, а не идеальную кинематику. Rear-steer и real forklift details можно уточнить позже.

---

## slam_toolbox

### Роль в демо

Использовать `slam_toolbox` как источник карты и `map -> odom` трансформации.

### Конфигурация

Подготовить `slam_toolbox.yaml` для симуляции:

- режим online async,
- scan topic `/scan`,
- base frame `base_link`,
- odom frame `odom`,
- map frame `map`.

### Важно

Проверить, что:

- `laser_link` корректно связан с `base_link`,
- `/scan` валиден,
- `/odom` публикуется,
- TF дерево непрерывно и без конфликтов.

---

## RViz

### Что показать в готовом профиле

Собрать `demo.rviz`, где отображаются:

- TF
- RobotModel
- LaserScan `/scan`
- Map `/map`
- Path `/demo_path`
- Odometry `/odom`
- TF axes при необходимости

### Поведение запуска

Docker Compose должен стартовать:

- headless Gazebo,
- ROS граф,
- RViz в отдельном контейнере или в том же контейнере с X11 пробросом.

Если X11 в окружении пользователя недоступен, предусмотреть fallback:

- запуск без RViz через compose profile,
- либо отдельную цель `docker compose --profile gui up`.

---

## Docker / Docker Compose

### Цель

Сделать полностью контейнеризованный запуск.

### Требования

- Один Dockerfile для ROS 2 окружения.
- Один `docker-compose.yml`.
- Сервис(ы):
  - `sim`
  - `rviz`
- Headless Gazebo должен стартовать без GUI.
- RViz должен стартовать отдельно.

### Рекомендуемая схема

#### Сервис `sim`

Запускает:

- Gazebo server headless
- robot_state_publisher
- spawn robot
- ros_gz bridge
- ros2_control / converter node
- slam_toolbox
- controller_server
- lifecycle_manager
- hardcoded_route_sender

#### Сервис `rviz`

Запускает:

- `rviz2 -d /ws/src/.../demo.rviz`

### Практические заметки

- Использовать `network_mode: host` на Linux для простоты DDS discovery.
- Пробросить `DISPLAY`, `.Xauthority`, `/tmp/.X11-unix` для RViz.
- Для headless Gazebo не поднимать клиентский GUI.

---

## Launch файл

### Нужен один основной launch

`sim_followpath.launch.py`, который:

1. Загружает world.
2. Поднимает Gazebo server в headless.
3. Публикует robot_description.
4. Спавнит робота в мир.
5. Запускает bridges.
6. Запускает `slam_toolbox`.
7. Запускает `controller_server`.
8. Запускает `lifecycle_manager`.
9. Запускает `cmd_vel_to_tricycle.py`.
10. Запускает `hardcoded_route_sender.py`.
11. Опционально запускает RViz, если флаг `launch_rviz:=true`.

### Аргументы launch

Добавить launch arguments:

- `use_sim_time:=true`
- `launch_rviz:=true|false`
- `world:=...`
- `x:=... y:=... yaw:=...`

---

## Навигационные параметры

### nav2_params.yaml

Нужно сконфигурировать:

- `controller_frequency`
- `controller_plugins: ["FollowPath"]`
- plugin `RegulatedPurePursuitController`
- `goal_checker_plugins`
- `progress_checker_plugins`
- `use_sim_time: true`

### Первичный тюнинг

Стартовые параметры:

- `desired_linear_vel: 0.2 - 0.4`
- `lookahead_dist: 0.6 - 1.0`
- `min_lookahead_dist: 0.3`
- `max_lookahead_dist: 1.2`
- `use_velocity_scaled_lookahead_dist: true`
- `transform_tolerance: 0.2`
- `use_collision_detection: false` на первом этапе, чтобы не ловить лишние стопы

---

## Контроллеры / joints

### Минимально рабочая версия

Если получится быстро, поднять joint controllers через `ros2_control`.

Иначе допустим временный вариант:

- простая нода, которая напрямую применяет joint команды через Gazebo transport / bridge.

Но предпочтительно всё же остаться в `ros2_control`, чтобы позже легче перейти к железу.

### Нужно проверить

- steering joint реально поворачивает wheel link вокруг вертикальной оси;
- drive wheel реально вращается;
- робот двигается при подаче low-level команд;
- odom соответствует движению.

---

## Порядок реализации

### Этап 1. Скелет проекта

- Создать 2 пакета: `forklift_demo_description`, `forklift_demo_control`.
- Подготовить `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`.
- Проверить, что workspace собирается в контейнере.

### Этап 2. Мир

- Создать `square_room.sdf`.
- Запустить headless Gazebo только с миром.
- Проверить, что мир стабильно стартует.

### Этап 3. URDF робота

- Создать `forklift_demo.urdf.xacro`.
- Поднять `robot_state_publisher`.
- Проверить TF и визуализацию в RViz.

### Этап 4. Спавн в Gazebo

- Импортировать робота в симуляцию.
- Убедиться, что модель появляется в мире.
- Проверить joint names и frames.

### Этап 5. Лидар и bridge

- Добавить lidar sensor/plugin.
- Вывести `/scan` в ROS.
- Проверить scan в RViz.

### Этап 6. Низкий уровень движения

- Реализовать `cmd_vel_to_tricycle.py`.
- Временно подать ручной `cmd_vel`.
- Добиться того, чтобы робот ездил и поворачивал.

### Этап 7. Одометрия и TF

- Обеспечить `/odom`.
- Проверить `odom -> base_link`.
- Проверить согласованность с движением.

### Этап 8. slam_toolbox

- Поднять `slam_toolbox`.
- Проверить `/map` и `map -> odom`.
- Проехать вручную и убедиться, что карта строится.

### Этап 9. Nav2 controller_server

- Поднять только controller stack.
- Проверить lifecycle nodes.
- Убедиться, что action server `follow_path` жив.

### Этап 10. hardcoded_route_sender

- Реализовать отправку `FollowPath` goal.
- Параллельно публиковать `/demo_path`.
- Убедиться, что робот начинает ехать по `Г`-маршруту.

### Этап 11. Цикл туда-обратно

- Реализовать чередование прямого и обратного маршрута раз в 20 секунд.
- Сделать отмену/замену предыдущего goal, если он ещё не завершён.

### Этап 12. Финальный launch и RViz

- Собрать единый launch.
- Проверить запуск из `docker compose up`.
- Проверить, что RViz открывается и показывает всё нужное.

---

## Acceptance criteria

Считать задачу завершённой, когда выполняются все условия:

1. `docker compose up` поднимает весь стенд без ручных действий.
2. В RViz видны:
   - робот,
   - scan,
   - path,
   - TF,
   - map.
3. Робот ездит по маршруту `Г` туда и обратно циклически.
4. Путь отправляется через `FollowPath` action, без planner server.
5. Gazebo работает в headless режиме.
6. `slam_toolbox` публикует карту и `map -> odom`.
7. Репозиторий содержит понятную документацию по запуску.

---

## Что агент должен логировать и проверять

### Проверки ROS graph

- `ros2 topic list`
- `ros2 action list`
- `ros2 node list`
- `ros2 param list`

### Проверки TF

- наличие `map`, `odom`, `base_link`, `laser_link`
- отсутствие конфликтующих TF publishers

### Проверки topics

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/map`
- `/cmd_vel`
- `/demo_path`

### Проверки action

- `FollowPath` server доступен
- goal принимается
- feedback приходит
- robot moves

### Проверки симуляции

- robot spawn успешен
- steering joint двигается
- drive wheel вращается
- robot pose меняется во времени

---

## Известные риски

1. **Rear-steer vs front-steer assumptions**
   - Некоторые готовые контроллеры и примеры ориентированы на переднее steer wheel.
   - На первом этапе использовать собственный `cmd_vel -> steering` converter.

2. **slam_toolbox без валидного odom**
   - Нужен базовый odom источник, даже если глобальная локализация идёт через SLAM.

3. **Headless Gazebo + RViz в Docker**
   - Возможны проблемы с X11/Wayland.
   - Нужен fallback без RViz или отдельный profile.

4. **Nav2 lifecycle**
   - Если не активировать lifecycle nodes, FollowPath action не заработает.

5. **Слишком редкий путь**
   - Pure Pursuit может ехать плохо по редким точкам.
   - Нужно densify path.

6. **Коллизии и costmap**
   - На первом этапе лучше отключить лишнюю collision regulation, пока базовая петля не заработала.

---

## Что НЕ делать на первом этапе

- Не пытаться сразу моделировать реальную асимметричную кинематику форклифта.
- Не добавлять planner server.
- Не делать сложный fleet manager integration.
- Не добавлять сложную динамику или паллеты.
- Не оптимизировать path smoothing сверх необходимости.

Сначала нужен рабочий минимальный demo pipeline.

---

## Что улучшить после MVP

- заменить центрированное заднее колесо на реальную механику пользователя;
- перейти от hardcoded path к внешнему API fleet manager;
- добавить route cancellation / preemption;
- добавить нормальную low-level odometry от joint states;
- протестировать `tricycle_controller` или кастомный rear-steer controller;
- добавить движение задним ходом;
- добавить препятствия и collision-aware local costmap;
- добавить метрики tracking error.

---

## Вопросы к пользователю

Ниже вопросы, ответы на которые помогут избежать переделок. Если ответов нет, агент должен использовать значения по умолчанию из раздела “Assumptions”.

1. **ROS 2 дистрибутив**: подтверждаем `Humble`?
2. **Какой Gazebo нужен**: классический Gazebo Classic или новый Gazebo Sim (`gz sim`)?
3. **Нужен ли именно `ros2_control`, или допустима MVP-реализация через собственные publishers команд в joints?**
4. **Какой lidar использовать для демо**:
   - 180° или 360°,
   - дальность 8–10 м достаточно?
5. **Какой размер комнаты взять по умолчанию**: 8x8 м или 10x10 м?
6. **Маршрут буквой `Г`**: подходит ли форма “вправо, потом вверх”, или нужна конкретная длина сегментов?
7. **Робот должен ехать только вперёд**, или сразу предусмотреть движение задним ходом?
8. **Нужно ли сохранять карту slam_toolbox на диск**, или достаточно live map в RViz?
9. **RViz должен запускаться автоматически всегда**, или допустим отдельный compose profile `gui`?
10. **Нужна ли поддержка Wayland**, или достаточно Linux + X11?

---

## Assumptions по умолчанию, если пользователь не ответит

- ROS 2 Humble
- Gazebo Sim (`ros_gz_sim`)
- Ubuntu 22.04 base image
- Комната 8x8 м
- 360° lidar, 10 м дальность
- Маршрут `Г`: 3 м вправо, затем 3 м вверх
- Только движение вперёд
- Карта не сохраняется на диск
- RViz запускается отдельным контейнером через X11
- MVP low-level control через собственную converter node, без жёсткой зависимости от `tricycle_controller`

---

## Что агент должен отдать в результате

1. Полный исходный код двух пакетов.
2. `Dockerfile`.
3. `docker-compose.yml`.
4. Единый launch-файл.
5. Конфиги Nav2 и SLAM.
6. RViz profile.
7. README с командами запуска.
8. Краткий smoke test checklist.

---

## Smoke test checklist

После сборки агент должен выполнить минимум:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_demo_description sim_followpath.launch.py
```

И затем проверить:

```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /map
ros2 action list | grep follow_path
ros2 topic echo /cmd_vel
```

Ожидаемое поведение:

- путь появляется в RViz,
- робот стартует,
- проезжает `Г`,
- через 20 секунд едет обратно.

