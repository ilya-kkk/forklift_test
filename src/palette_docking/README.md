# palette_docking

`palette_docking` выполняет поэтапный заезд погрузчика к палете из pre-picking
позиции. Пакет не управляет вилами, не строит глобальный маршрут и не делает retreat
после успешного заезда. Его зона ответственности: найти палетный AprilTag по TF,
выровняться, подъехать к целевой точке и выполнить финальный заезд под палету.

## Runtime

- Нода: `pallet_docking_controller`
- Сервис управления: `/palette_docking/control`
- Тип сервиса: `forklift_interfaces/srv/StringWithJson`
- Выход команд скорости: `/cmd_vel_pallet_docking`
- Основной источник цели: TF палетного тега, например `base_link -> pallet_b_south_08_tag`

Команды публикуются в отдельный docking-вход. В общем demo launch этот вход выбирает
`cmd_vel_arcestrator`; через `collision_monitor` проходит только navigation-выход.

## Система координат

Контроллер работает в режиме rear docking. TF тега читается в `target_frame` (обычно
`base_link`), затем XY-плоскость интерпретируется как frame, повернутый на `pi`:

```text
x = -transform.translation.x
y = -transform.translation.y
angle = atan2(y, x)
```

Из-за этого движение в сторону палеты публикуется как отрицательный `linear.x`, а
отъезд от палеты (`BACK_OUT`) публикуется как положительный `linear.x`.

## Управление

Запустить docking:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"start\"}'}"
```

Остановить и вернуть контроллер в `IDLE`:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"stop\"}'}"
```

Получить статус:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"status\"}'}"
```

Старт с разовыми параметрами:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"start\", \"tag_frame\": \"pallet_b_south_08_tag\", \"standoff_distance_m\": 1.0, \"approach_extra_drive_m\": 0.6, \"final_drive_distance_m\": 1.0, \"max_turn_angle_deg\": 35.0}'}"
```

Команды `enable` и `dock` работают как `start`. Команды `disable` и `cancel` работают
как `stop`. Команда `reset` очищает `failure_reason` и переводит контроллер в `IDLE`.

## Выбор тега

Если `tag_frame` пустой, контроллер перебирает `tag_frame_candidates` и выбирает
валидный TF с минимальным `abs(angle)`. Так выбирается тег, который ближе всего к
центральной оси робота в интерпретированном rear-docking frame.

Если нужен конкретный тег, его можно задать параметром:

```yaml
tag_frame: pallet_b_south_08_tag
```

или передать при старте:

```json
{"command": "start", "tag_frame": "pallet_b_south_08_tag"}
```

TF считается валидным, если transform найден и его возраст не больше `tag_timeout_sec`.

## Диагностические Логи

При старте контроллер пишет строку `palette docking start params` с ключевыми
параметрами текущей попытки: выбранный тег, `standoff_distance_m`,
`approach_extra_drive_m`, вычисленный `target_x`, допуски, финальную дистанцию и
параметры recovery.

Во время активного заезда контроллер пишет `palette docking diagnostic` не чаще, чем
раз в `diagnostic_log_period_sec`. По умолчанию это 1 секунда. В этих логах есть:

- состояние и время до таймаута;
- выбранный тег, `x`, `y`, `z`, `angle`, возраст TF;
- текущие ошибки `y_error`, `angle_error`, `x_error`;
- целевая `target_x` на этапе `APPROACH_STANDOFF`;
- публикуемые `cmd_linear` и `cmd_angular`;
- прогресс `final_drive_traveled` и `back_out_traveled`.

Отключить периодические диагностические логи можно значением:

```yaml
diagnostic_log_period_sec: 0.0
```

## Процесс Заезда

Обычный успешный путь:

```text
IDLE
  -> WAIT_FOR_TAG
  -> COMPENSATE_Y
  -> ALIGN_TO_TAG
  -> APPROACH_STANDOFF
  -> DOCK_UNDER_PALLET
  -> DONE
```

Recovery-путь при ошибке:

```text
любое активное состояние
  -> BACK_OUT
  -> FAILED
```

`DONE`, `FAILED` и `IDLE` являются остановочными состояниями: при входе в них
публикуется нулевая команда скорости.

## Состояния

### IDLE

Безопасное начальное состояние. Контроллер не выполняет движение. Вход в `IDLE` также
происходит по `stop`, `disable`, `cancel` или `reset`.

Переходы:
- `start` / `enable` / `dock` -> `WAIT_FOR_TAG`

### WAIT_FOR_TAG

Контроллер ждет валидный TF выбранного тега или одного из `tag_frame_candidates`.
Движение в этом состоянии не публикуется.

Переходы:
- TF найден -> `COMPENSATE_Y`
- TF не найден дольше `wait_for_tag_timeout_sec` -> `FAILED`
- `stop` / `cancel` -> `IDLE`

### COMPENSATE_Y

Компенсация боковой ошибки. Робот медленно едет в сторону палеты и рулит по `y`, чтобы
свести тег к центральной оси.

Ошибки:

```text
y_error = tag.y
angle_error = atan2(tag.y, tag.x)
```

Команда:

```text
linear.x  = -abs(lateral_linear_speed_mps)
angular.z = clamp(y_angular_gain * y_error, +/- max_angular_speed_radps)
```

Переходы:
- `abs(y_error) <= y_tolerance_m` -> `ALIGN_TO_TAG`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `compensate_y_timeout_sec` -> `BACK_OUT`
- потеря TF до таймаута -> нулевая скорость и ожидание TF
- потеря TF до конца таймаута -> `BACK_OUT`

### ALIGN_TO_TAG

Выравнивание по углу на тег. По умолчанию `align_linear_speed_mps = 0.0`, поэтому робот
поворачивает без намеренного продольного движения. Текущий допуск перед началом заезда:
`angle_tolerance_rad = 0.01` (примерно 0.57 градуса).

Команда:

```text
linear.x  = -abs(align_linear_speed_mps)
angular.z = clamp(angle_angular_gain * angle_error, +/- max_angular_speed_radps)
```

Переходы:
- `abs(angle_error) <= angle_tolerance_rad` -> `APPROACH_STANDOFF`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `align_timeout_sec` -> `BACK_OUT`
- потеря TF до таймаута -> нулевая скорость и ожидание TF
- потеря TF до конца таймаута -> `BACK_OUT`

### APPROACH_STANDOFF

Подъезд к целевой `x`-точке перед финальным заездом. Цель считается от
`standoff_distance_m` с дополнительным смещением глубже под палету:

```text
target_x = standoff_distance_m - approach_extra_drive_m
x_error  = tag.x - target_x
```

Команда:

```text
linear.x  = -clamp(approach_x_gain * x_error,
                   approach_min_linear_speed_mps,
                   approach_max_linear_speed_mps)
angular.z = clamp(approach_y_angular_gain * y_error +
                  approach_angle_angular_gain * angle_error,
                  +/- max_angular_speed_radps)
```

`approach_extra_drive_m = 0.6` означает, что целевая точка на этапе подхода смещается
на 0.6 м глубже относительно базового `standoff_distance_m`.

Переходы:
- `x_error <= x_tolerance_m` -> `DOCK_UNDER_PALLET`
- `abs(y_error) > y_reacquire_tolerance_m` -> `COMPENSATE_Y`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `approach_timeout_sec` -> `BACK_OUT`
- потеря TF до таймаута -> нулевая скорость и ожидание TF
- потеря TF до конца таймаута -> `BACK_OUT`

### DOCK_UNDER_PALLET

Финальный заезд под палету на фиксированную дистанцию `final_drive_distance_m`. Этот
этап может продолжаться даже при потере TF, потому что при въезде под палету тег может
уйти из камеры.

Команда:

```text
linear.x  = -abs(dock_linear_speed_mps)
angular.z = 0.0
```

Если TF еще виден, можно включить подруливание:

```yaml
dock_y_angular_gain: 0.1
dock_angle_angular_gain: 0.2
```

Финальная дистанция считается интегрированием отправленной скорости по времени:

```text
final_drive_traveled += abs(linear.x) * dt
```

Переходы:
- `final_drive_traveled >= final_drive_distance_m` -> `DONE`
- таймаут `dock_timeout_sec` -> `BACK_OUT`

### BACK_OUT

Recovery-состояние. Робот отъезжает от палеты на `back_out_distance_m`, затем завершает
попытку как неуспешную. Исходная причина сохраняется в `failure_reason`.

Команда:

```text
linear.x  = abs(back_out_linear_speed_mps)
angular.z = 0.0
```

Переходы:
- `back_out_traveled >= back_out_distance_m` -> `FAILED`
- таймаут `back_out_timeout_sec` -> `FAILED`

### DONE

Успешное завершение. Робот отправляет нулевую скорость и больше не двигается до
следующего `start`.

Переходы:
- `start` / `enable` / `dock` -> `WAIT_FOR_TAG`
- `reset` / `stop` -> `IDLE`

### FAILED

Неуспешное завершение. Робот отправляет нулевую скорость. Причина доступна в поле
`failure_reason` ответа `status`.

Переходы:
- `start` / `enable` / `dock` -> `WAIT_FOR_TAG`
- `reset` / `stop` -> `IDLE`

## Основные параметры

- `cmd_vel_topic`: топик публикации скорости, сейчас `/cmd_vel_pallet_docking`.
- `target_frame`: frame, в котором контроллер считает `x`, `y` и `angle`, обычно `base_link`.
- `tag_frame`: конкретный тег; пустая строка включает авто-выбор из `tag_frame_candidates`.
- `tag_timeout_sec`: максимальный возраст TF тега.
- `diagnostic_log_period_sec`: период INFO-логов диагностики активного состояния;
  `0.0` отключает периодические диагностические логи.
- `wait_for_tag_timeout_sec`: сколько ждать тег перед переходом в `FAILED`.
- `compensate_y_timeout_sec`: лимит времени на `COMPENSATE_Y`.
- `align_timeout_sec`: лимит времени на `ALIGN_TO_TAG`.
- `approach_timeout_sec`: лимит времени на `APPROACH_STANDOFF`.
- `dock_timeout_sec`: лимит времени на `DOCK_UNDER_PALLET`.
- `back_out_timeout_sec`: лимит времени на `BACK_OUT`.
- `standoff_distance_m`: базовая дистанция до тега/палеты перед финальным заездом.
- `approach_extra_drive_m`: дополнительный заезд на этапе `APPROACH_STANDOFF`.
- `final_drive_distance_m`: фиксированная дистанция финального заезда под палету.
- `back_out_distance_m`: дистанция отъезда при recovery.
- `y_tolerance_m`: допустимая боковая ошибка после `COMPENSATE_Y`.
- `y_reacquire_tolerance_m`: порог возврата из `APPROACH_STANDOFF` в `COMPENSATE_Y`.
- `angle_tolerance_rad`: допустимая угловая ошибка перед `APPROACH_STANDOFF`, сейчас
  `0.01` рад (примерно 0.57 градуса).
- `x_tolerance_m`: допуск по `x` перед переходом в `DOCK_UNDER_PALLET`.
- `max_turn_angle_rad`: максимальный разрешенный угол на тег перед recovery.

## Текущие Ограничения

- `final_drive_traveled` и `back_out_traveled` считаются по отправленной скорости и
  времени, а не по фактической odometry.
- Контроллер предполагает, что внешний navigation уже привел робота в pre-picking
  позицию.
- Контроллер не управляет вилами и не делает post-pick retreat.
