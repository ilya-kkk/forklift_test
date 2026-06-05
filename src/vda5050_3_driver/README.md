# vda5050_3_driver

Пакет-адаптер для будущей VDA5050 v3 интеграции.

## Архитектурная Граница

`vda5050_3_driver` не должен напрямую дергать `navigation_forklift`,
`fork_manager`, `palette_docking` или `cmd_vel_arcestrator`.

Правильный поток:

```text
MQTT VDA5050 topics
  -> vda5050_3_driver
  -> /robot_control_core/control
  -> robot_control_core capability adapters
```

`robot_control_core` является владельцем активной миссии, FSM, pause/cancel,
`actionStates`, `instantActionStates`, `nodeStates`, `edgeStates` и
`newBaseRequest`. VDA-драйвер должен оставаться протокольным слоем:

- подписаться на MQTT `order`, `instantActions`, `responses`, `zoneSet`;
- публиковать MQTT `state`, `connection`, `factsheet`, optional `visualization`;
- валидировать входящие JSON по VDA5050 v3 schemas;
- проверять `orderId`, `orderUpdateId`, stitching node, base/horizon;
- конвертировать accepted order/update в internal mission для
  `/robot_control_core/control`;
- конвертировать `/robot_control_core/status` обратно в VDA `state`.

## Mapping

```text
VDA edge traversal
  -> order topology / route execution, not an actionType

VDA finePositioning
  -> robot_control_core step: {"actionType": "finePositioning", "actionParameters": [{"key": "stationName", "value": "..."}]}

VDA pick
  -> robot_control_core step: {"actionType": "pick", "actionParameters": [{"key": "stationName", "value": "..."}, {"key": "loadId", "value": "..."}]}

VDA drop
  -> robot_control_core step: {"actionType": "drop", "actionParameters": [{"key": "loadId", "value": "..."}]}

instantAction startPause
  -> {"command": "startPause"}

instantAction stopPause
  -> {"command": "stopPause"}

instantAction cancelOrder
  -> {"command": "cancelOrder"}
```

## Минимальный План Реализации

1. MQTT client, topic naming и `connection` lifecycle.
2. JSON schema validation для VDA5050 v3.
3. OrderManager для acceptance/rejection.
4. Adapter: VDA order/update -> `start_mission` / `append_base`.
5. State publisher: `/robot_control_core/status` -> VDA `state`.
6. Factsheet publisher на основе capabilities из `robot_control_core`.
