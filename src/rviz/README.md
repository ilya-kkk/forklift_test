# rviz

Пакет RViz-конфигурации и вспомогательных debug-ноду визуализации.

## Что внутри
- `rviz/demo.rviz` - готовый RViz layout.
- `rviz/json_map_visualizer.py` - рисует JSON-карту как `MarkerArray`.
- `rviz/up_lidar_marker_service.py` - debug marker состояния верхнего лидара/движения.
- `rviz/cmd_vel_activity_service.py` - публикует `is_moving` по команде колеса.
- `rviz/cmd_vel_twist_stamper.py` - конвертирует `/cmd_vel` (`Twist`) в `/cmd_vel_stamped` (`TwistStamped`) для RViz display.
- `config/*.yaml` - параметры этих helper-ноду.

## Ответственность
- Визуализация и отладка, не core-control.
- Публикует `/json_map/markers` и `/debug/up_lidar_marker`.
- Публикует `/cmd_vel_stamped` для `rviz_default_plugins/TwistStamped`.

## Связи
- Читает карту из `map_service`.
- Читает wheel command из `cmd_vel_to_motors` для debug-состояния движения.
- Используется launch-пакетом `forklift_demo`.
