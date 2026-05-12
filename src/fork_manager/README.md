# fork_manager

Пакет управления вилами погрузчика.

## Что внутри
- `fork_manager/fork_position_controller.py` - P-контроллер позиции вил.
- `config/fork_position_controller.yaml` - параметры joint, топиков, лимитов и регулятора.
- `config/fork_slider_publisher.yaml` - конфиг slider-паблишера для ручного управления вилой в демо.

## Ответственность
- Принимает целевую позицию вил на `/forklift/fork_cmd`.
- Читает текущую позицию из `/joint_states`.
- Публикует velocity-команду `/forklift/fork_velocity_cmd`.

## Связи
- В демо velocity-команда уходит в Gazebo через `ros_gz_bridge`.
- На реальном роботе этот пакет должен быть связан с реальным lift actuator driver и safety limits.
