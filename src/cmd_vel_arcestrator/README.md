# cmd_vel_arcestrator

Пустой пакет-резерв для будущей оркестрации источников velocity-команд.

## Что внутри
- Пока только `config/`.

## Планируемая ответственность
- Арбитраж/оркестрация команд скорости.
- Выбор между manual, navigation, safety, docking/picking командами.
- Связь с `collision_monitor` и `cmd_vel_to_motors`.
