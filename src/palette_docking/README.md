# palette_docking

`palette_docking` выполняет финальный поэтапный заезд к палете из pre-picking позиции.
Пакет не управляет вилами, не делает retreat после успешного заезда и не берет на себя
полный picking-сценарий. Его ответственность заканчивается в момент, когда погрузчик
заехал вперед на заданную финальную дистанцию и остановился.

## Runtime

- Нода: `pallet_docking_controller`
- Сервис управления: `/palette_docking/control`
- Выход команд скорости: `/cmd_vel`
- Основной источник цели: TF палетного тега, например `base_link -> pallet_b_south_08_tag`

Команды публикуются напрямую в `/cmd_vel`.

## Запуск

В полном demo launch нода стартует в состоянии `IDLE` и не двигает робота, пока ее явно
не запустят сервисом.

Отдельный запуск:

```bash
ros2 run palette_docking pallet_docking_controller \
  --ros-args \
  --params-file /ws/install/palette_docking/share/palette_docking/config/pallet_docking_controller.yaml
```

Запустить docking:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"start\"}'}"
```

Остановить:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"stop\"}'}"
```

Статус:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"status\"}'}"
```

Старт с разовыми параметрами:

```bash
ros2 service call /palette_docking/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"start\", \"standoff_distance_m\": 0.9, \"final_drive_distance_m\": 1.1, \"max_turn_angle_deg\": 35.0}'}"
```

## Выбор тега

Если параметр `tag_frame` пустой, контроллер перебирает `tag_frame_candidates` и выбирает
валидный TF с минимальным `abs(atan2(y, x))` в `target_frame`. То есть берется тег,
который ближе всего к центральной оси робота в XY-плоскости.

Если нужен конкретный тег, можно задать его в конфиге:

```yaml
tag_frame: pallet_b_south_08_tag
```

или передать в сервисном запросе:

```json
{"command": "start", "tag_frame": "pallet_b_south_08_tag"}
```

## Состояния

### IDLE

Начальное безопасное состояние. Команды движения не публикуются, кроме нулевой команды
при входе в состояние. Переход из `IDLE` выполняется только командой `start`.

Переходы:
- `start` -> `WAIT_FOR_TAG`

### WAIT_FOR_TAG

Контроллер ждет валидный TF палетного тега. TF считается валидным, если он найден и его
возраст не больше `tag_timeout_sec`.

Переходы:
- TF найден -> `COMPENSATE_Y`
- TF не найден дольше `wait_for_tag_timeout_sec` -> `FAILED`
- `stop` -> `IDLE`

### COMPENSATE_Y

Первый активный этап. Робот медленно едет вперед и рулит по боковой ошибке `y`, чтобы
свести палету ближе к центральной оси робота.

Используемые ошибки:

```text
y_error = tag.y
angle_error = atan2(tag.y, tag.x)
```

Команда:

```text
linear.x  = lateral_linear_speed_mps
angular.z = clamp(y_angular_gain * y_error, +/- max_angular_speed_radps)
```

Ограничение безопасности: если `abs(angle_error) > max_turn_angle_rad`, контроллер не
пытается разворачивать робот на большой угол и переходит в `BACK_OUT`.

Переходы:
- `abs(y_error) <= y_tolerance_m` -> `ALIGN_TO_TAG`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `compensate_y_timeout_sec` -> `BACK_OUT`
- потеря TF до таймаута -> остановка и ожидание
- потеря TF до конца таймаута -> `BACK_OUT`

### ALIGN_TO_TAG

Второй этап. Робот поворачивается к палете, пока угол на тег в XY-плоскости не станет
достаточно близким к нулю.

Команда:

```text
linear.x  = align_linear_speed_mps
angular.z = clamp(angle_angular_gain * angle_error, +/- max_angular_speed_radps)
```

По умолчанию `align_linear_speed_mps = 0.0`, то есть это этап поворота без намеренного
продвижения вперед. Если конкретная кинематика требует небольшой скорости для поворота,
этот параметр можно поднять.

Переходы:
- `abs(angle_error) <= angle_tolerance_rad` -> `APPROACH_STANDOFF`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `align_timeout_sec` -> `BACK_OUT`

### APPROACH_STANDOFF

Третий этап. Робот подъезжает к точке перед палетой на заданную дистанцию по `x`.
Целевая дистанция задается параметром `standoff_distance_m`.

Команда:

```text
target_x  = standoff_distance_m - approach_extra_drive_m
x_error   = tag.x - target_x
linear.x  = clamp(approach_x_gain * x_error,
                  approach_min_linear_speed_mps,
                  approach_max_linear_speed_mps)
angular.z = clamp(approach_y_angular_gain * y_error +
                  approach_angle_angular_gain * angle_error,
                  +/- max_angular_speed_radps)
```

Если боковая ошибка снова стала слишком большой (`abs(y_error) > y_reacquire_tolerance_m`),
контроллер возвращается в `COMPENSATE_Y`, а не продолжает ехать вперед криво.

Переходы:
- `tag.x - (standoff_distance_m - approach_extra_drive_m) <= x_tolerance_m` -> `DOCK_UNDER_PALLET`
- `abs(y_error) > y_reacquire_tolerance_m` -> `COMPENSATE_Y`
- `abs(angle_error) > max_turn_angle_rad` -> `BACK_OUT`
- таймаут `approach_timeout_sec` -> `BACK_OUT`

### DOCK_UNDER_PALLET

Финальный этап. Робот едет вперед на фиксированную дистанцию `final_drive_distance_m`.
Это сделано намеренно: при въезде под палету тег может уйти из камеры, поэтому финальный
заезд не обязан зависеть от TF.

Команда:

```text
linear.x = dock_linear_speed_mps
```

По умолчанию `angular.z = 0.0`. Если нужно оставить небольшую корректировку по видимому
тегу, можно настроить:

```yaml
dock_y_angular_gain: 0.1
dock_angle_angular_gain: 0.2
```

Дистанция финального заезда считается интегрированием отправленной скорости по времени.
Это простая модель для MVP; если потребуется точность по фактическому перемещению, этот
этап стоит перевести на odometry-based distance.

Переходы:
- пройдена `final_drive_distance_m` -> `DONE`
- таймаут `dock_timeout_sec` -> `BACK_OUT`

### BACK_OUT

Recovery-состояние. Если робот не может безопасно повернуть или не смог дойти до точки
перед палетой в отведенное время, он отъезжает назад на `back_out_distance_m`.

Команда:

```text
linear.x = back_out_linear_speed_mps
angular.z = 0.0
```

Переходы:
- пройдена `back_out_distance_m` -> `FAILED`
- таймаут `back_out_timeout_sec` -> `FAILED`

### DONE

Успешное завершение. Робот отправляет нулевую команду скорости и больше не двигается до
следующего `start`.

### FAILED

Неуспешное завершение. Робот отправляет нулевую команду скорости. Причина доступна в
поле `failure_reason` ответа сервиса `status`.

## Основные параметры

- `target_frame`: frame, в котором контроллер считает `x`, `y` и угол. Обычно `base_link`.
- `tag_frame`: конкретный тег. Пустая строка включает авто-выбор из `tag_frame_candidates`.
- `max_turn_angle_rad`: максимальный разрешенный угол на тег. Если больше, робот не
  пытается разворачиваться на месте и уходит в `BACK_OUT`.
- `standoff_distance_m`: дистанция до палеты, на которой заканчивается подвод к точке
  перед палетой.
- `approach_extra_drive_m`: дополнительный заезд на этапе `APPROACH_STANDOFF` после
  достижения `standoff_distance_m`; позволяет сместить целевую `x`-точку глубже под палету
  (например, на полпалеты `0.6` м).
- `final_drive_distance_m`: фиксированная дистанция финального заезда под палету.
- `back_out_distance_m`: дистанция отъезда назад при невозможности выполнить docking.
- `y_tolerance_m`: допустимая боковая ошибка после компенсации.
- `angle_tolerance_rad`: допустимый угол перед этапом подъезда.
- `x_tolerance_m`: допуск по дистанции до standoff-точки.

## Ограничения текущей версии

- Финальная дистанция и back-out считаются интегрированием командной скорости, а не
  фактической odometry.
- Контроллер предполагает, что внешняя логика уже привела робота в pre-picking позицию.
- Контроллер не управляет вилами и не делает post-pick retreat.
