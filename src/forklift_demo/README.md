# forklift_demo

Демо/simulation package для сборки всего стенда.

## Что внутри
- `launch/sim_followpath.launch.py` - полный запуск Gazebo + Nav2 + кастомные пакеты.
- `urdf/` - модель погрузчика и meshes.
- `worlds/` - Gazebo world.
- `models/` - Gazebo модели, включая euro pallet.
- `config/bridge_config.yaml` - `ros_gz_bridge` mapping.

## Ответственность
- Содержит только demo/sim assets и главный demo launch.
- Не содержит бизнес-логику навигации, карты, моторов или вил.

## Связи
- Запускает `navigation_forklift`, `cmd_vel_to_motors`, `fork_manager`, `map_service`, `collision_monitor`, `rviz`.
- Использует Gazebo packages `ros_gz_sim` и `ros_gz_bridge`.
