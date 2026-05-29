# cmd_vel_arcestrator

Оркестратор источников `cmd_vel`. Нода выбирает один из четырех входов и
публикует его в общий `/cmd_vel` для `cmd_vel_to_motors`.

## Runtime

- Нода: `cmd_vel_arcestrator`
- Сервис управления: `/cmd_vel_arcestrator/control`
- Тип сервиса: `forklift_interfaces/srv/StringWithJson`
- Статус: `/cmd_vel_arcestrator/status`, `std_msgs/String` с JSON
- Выход: `/cmd_vel`

## Источники

- `pallet_docking` -> `/cmd_vel_pallet_docking`
- `charging_docking` -> `/cmd_vel_charging_docking`
- `manual` -> `/cmd_vel_manual`
- `navigation` -> `/cmd_vel_nav`

В текущем launch только navigation проходит через `collision_monitor`:

```text
Nav2 / route_service -> /cmd_vel_nav_raw -> collision_monitor -> /cmd_vel_nav
```

Остальные источники идут в оркестратор напрямую.

## Управление

Выбрать источник и разрешить публикацию:

```bash
ros2 service call /cmd_vel_arcestrator/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"select_source\", \"source\": \"navigation\"}'}"
```

Доступные `source`: `pallet_docking`, `charging_docking`, `manual`, `navigation`.

Остановить выход, сохранив выбранный источник:

```bash
ros2 service call /cmd_vel_arcestrator/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"stop\"}'}"
```

Восстановить публикацию ранее выбранного источника:

```bash
ros2 service call /cmd_vel_arcestrator/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"release\"}'}"
```

Статус:

```bash
ros2 service call /cmd_vel_arcestrator/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"status\"}'}"
```

Если выбранный источник не публикует свежее сообщение дольше
`source_timeout_sec`, на `/cmd_vel` публикуется нулевая скорость.
