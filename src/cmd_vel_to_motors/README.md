# cmd_vel_to_motors

Пакет кинематики и преобразования velocity-команд в команды приводов.

## Что внутри
- `cmd_vel_to_motors/cmd_vel_to_motors.py` - ROS-нода `cmd_vel_to_motors`.
- `config/cmd_vel_to_motors.yaml` - параметры топиков, joint names, радиуса колеса, режима движения и steering gate.

## Ответственность
- Читает `/cmd_vel`.
- Парсит `robot_description`/URDF для геометрии рулевого и ведущего колеса.
- Публикует `/forklift/right_steering_cmd` и `/forklift/right_wheel_cmd`.
- Поддерживает режимы движения `BODY_FIRST` и `FORKS_FIRST`.

## Связи
- Получает `/cmd_vel` после `collision_monitor`.
- Получает `/joint_states` для текущего угла рулевого joint.
- В демо команды уходят через `ros_gz_bridge` в Gazebo; на реальном роботе должны идти в hardware driver.
