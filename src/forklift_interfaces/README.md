# forklift_interfaces

Общие ROS 2 интерфейсы проекта.

## Что внутри
- `srv/StringWithJson.srv` - совместимый JSON-over-service контракт.

## Ответственность
- Держит общие srv/msg/action типы, чтобы runtime-пакеты не зависели друг от друга ради интерфейсов.

## Связи
- Используется `navigation_forklift`, `map_service`, `rviz`.
- Заменяет старый пакет `ros2_templates`.
