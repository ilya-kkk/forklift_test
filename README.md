# ROS 2 Forklift Demo

Короткое демо rear-steer погрузчика на `ROS 2 Humble + Gazebo Sim + Nav2`.

Сценарий сейчас такой:
- Gazebo запускается headless.
- RViz запускается отдельно.
- Робот бесконечно ездит по квадрату внутри комнаты.
- Квадрат собран из двух `L`-маршрутов, которые отправляются по очереди.

## Что здесь используется

- `Nav2 controller_server` без planner server и без BT navigator.
- Контроллер пути: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
- `slam_toolbox` для публикации `map -> odom`.
- `ros_gz_bridge` для `/clock`, `/scan`, `/odom`, `/tf`, `/joint_states` и управляющих топиков.
- Кастомный узел `cmd_vel_to_tricycle`, который переводит `cmd_vel` в команды заднего рулевого и ведущего колеса.

## Как едет робот

Алгоритм движения разбит на два уровня:

1. `hardcoded_route_sender.py`
   Строит путь, уплотняет его с шагом `0.08 м`, публикует в `/demo_path` и отправляет action `FollowPath`.

2. `controller_server`
   Считает управляющий `cmd_vel` по `Regulated Pure Pursuit`.

3. `cmd_vel_to_tricycle.py`
   Преобразует `cmd_vel` в rear-steer кинематику:
   - считает угол заднего рулевого колеса,
   - ограничивает скорость поворота руля,
   - публикует скорость заднего колеса,
   - публикует `Twist` в Gazebo.

Маршрут не один статичный отрезок. Узел по очереди шлет:
- нижняя + правая стороны квадрата,
- верхняя + левая стороны квадрата.

После успешного завершения текущей фазы сразу отправляется следующая, поэтому демо крутится бесконечно.

## Модель и мир

- Модель робота описана в `SDF` для Gazebo и в `xacro` для `robot_state_publisher`.
- Рулевая схема: одно заднее поворотное/ведущее колесо, два передних пассивных.
- Лимит заднего руления: `±85°` (`±1.48353 rad`).
- Мир максимально простой: пол и 4 стены, без внутренних препятствий.
- На роботе стоит `gpu_lidar`.

## Текущие параметры

- Размер квадрата по умолчанию: `4 x 4 м`.
- Старт робота: `(-1, -1)`.
- Целевая линейная скорость Nav2: `0.6 м/с`.
- Лимит угловой скорости колеса в low-level узле: `5.0 rad/s`.

## Визуализация

В RViz включены:
- `RobotModel`
- `TF`
- `LaserScan`
- `Map`
- `Path` из `/demo_path`

`Odometry` display сейчас отключен.

## Запуск

```bash
docker compose up --build
```

Только симуляция:

```bash
docker compose up --build sim
```

Отдельно RViz:

```bash
docker compose up --build rviz
```

## Ключевые файлы

- `src/forklift_demo_description/launch/sim_followpath.launch.py`
  Главный launch, поднимает Gazebo, bridge, Nav2, SLAM, route sender и RViz.

- `src/forklift_demo_control/forklift_demo_control/hardcoded_route_sender.py`
  Генерация маршрута и бесконечный цикл `FollowPath`.

- `src/forklift_demo_control/forklift_demo_control/cmd_vel_to_tricycle.py`
  Перевод `cmd_vel` в rear-steer команды.

- `src/forklift_demo_description/config/nav2_params.yaml`
  Конфиг `RegulatedPurePursuitController`.

- `src/forklift_demo_description/models/forklift_demo/model.sdf`
  Физическая модель робота в Gazebo.

- `src/forklift_demo_description/worlds/square_room.sdf`
  Простой world без препятствий.
