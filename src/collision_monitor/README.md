# collision_monitor

Пакет safety-фильтрации скорости и подготовки LaserScan для collision monitor.

## Что внутри
- `collision_monitor/scan_sector_filter.py` - фильтр секторов LaserScan.
- `config/collision_monitor_params.yaml` - параметры `nav2_collision_monitor`.
- `config/scan_sector_filters.yaml` - параметры четырех scan-фильтров.

## Ответственность
- Конфигурирует `nav2_collision_monitor` для navigation-цепочки
  `/cmd_vel_nav_raw -> /cmd_vel_nav`.
- Подготавливает `/scan`, `/scan_left_limited`, `/scan_right_limited`, `/scan_rear_limited`.

## Связи
- Получает raw scans из демо bridge или реальных lidar drivers.
- Получает `/cmd_vel_nav_raw` от Nav2 и `route_service`.
- Отдает безопасный navigation-выход `/cmd_vel_nav` в `cmd_vel_arcestrator`.
