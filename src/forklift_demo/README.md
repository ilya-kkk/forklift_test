# forklift_demo

Демо/simulation package для сборки всего стенда.

## Что внутри
- `launch/sim_followpath.launch.py` - полный запуск Gazebo + Nav2 + кастомные пакеты.
- `urdf/` - модель погрузчика и meshes.
- `worlds/` - Gazebo world.
- `models/` - Gazebo модели, включая euro pallet и tagged pallet variants.
- `config/bridge_config.yaml` - `ros_gz_bridge` mapping.

## Ответственность
- Содержит только demo/sim assets и главный demo launch.
- Не содержит бизнес-логику навигации, карты, моторов или вил.

## Связи
- Запускает `navigation_forklift`, `cmd_vel_to_motors`, `fork_manager`, `map_service`, `collision_monitor`, `apriltag_detector`, `rviz`.
- Опционально запускает YASMIN Viewer: `launch_yasmin_viewer:=true`, web UI на `http://127.0.0.1:5000/`.
- Использует Gazebo packages `ros_gz_sim` и `ros_gz_bridge`.
