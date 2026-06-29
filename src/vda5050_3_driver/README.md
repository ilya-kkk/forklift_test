# vda5050_3_driver

ROS2-адаптер VDA 5050 v3.0.0 для текущего `robot_control_core`.

Пакет принимает VDA `order` и `instantActions`, конвертирует поддержанные
действия в `/robot_control_core/control`, публикует VDA-подобные ROS topics
`state`, `connection` и `factsheet`.

MQTT-транспорт не реализуется внутри этого пакета. Для MQTT используется
скопированный в workspace пакет `mqtt_client`, который мостит MQTT payload как
primitive `std_msgs/String` в ROS topics и обратно.

## Текущий Статус

Реально подключено к `robot_control_core`:

```text
MQTT broker
  -> mqtt_client
  -> /vda5050/order, /vda5050/instantActions
  -> vda5050_3_bridge
  -> /robot_control_core/control
  -> robot_control_core
      -> /palette_docking/control
      -> /cmd_vel_arcestrator/control
      -> /forklift/fork_cmd
      -> /joint_states

/robot_control_core/status
  -> vda5050_3_bridge
  -> /vda5050/state
  -> mqtt_client
  -> MQTT broker
```

Важное ограничение: `robot_control_core` сейчас выполняет микродействия
`finePositioning`, `pick`, `drop`, но не исполняет VDA route edges. Поэтому
дефолтный factsheet не объявляет `navigationTypes`, а bridge принимает только
action-only orders: один текущий node, `edges: []`.

## Запуск

```bash
colcon build --packages-select \
  forklift_interfaces robot_control_core \
  mqtt_client_interfaces mqtt_client \
  vda5050_3_driver
source install/setup.bash
ros2 launch vda5050_3_driver vda5050_3_bridge.launch.py
```

## Запуск С MQTT

Поднять отдельный тестовый Mosquitto container:

```bash
docker compose -f docker-compose.vda5050-mqtt.yml up -d
```

Запустить VDA bridge вместе с ROS2 `mqtt_client`:

```bash
source install/setup.bash
ros2 launch vda5050_3_driver vda5050_with_mqtt_client.launch.py
```

Тестовый broker слушает host port `1884`, потому что `1883` часто уже занят
локальным Mosquitto. Конфиг MQTT bridge:

- [mqtt_client_vda5050_v3.yaml](config/mqtt_client_vda5050_v3.yaml)
- broker: `127.0.0.1:1884`
- MQTT prefix:

```text
vda5050/v3/ACCORD/forklift_sim
```

Минимальный MQTT smoke test:

```bash
mosquitto_sub -h 127.0.0.1 -p 1884 \
  -t 'vda5050/v3/ACCORD/forklift_sim/state'
```

В другом терминале:

```bash
mosquitto_pub -h 127.0.0.1 -p 1884 \
  -t 'vda5050/v3/ACCORD/forklift_sim/order' \
  -m '{"orderId":"mqtt-pick-001","orderUpdateId":0,"nodes":[{"nodeId":"current","sequenceId":0,"released":true,"actions":[{"actionType":"pick","actionId":"pick-mqtt-001","blockingType":"HARD","actionParameters":[{"key":"loadId","value":"pallet-mqtt-001"},{"key":"dock","value":false}]}]}],"edges":[]}'
```

VDA uses MQTT QoS 0 for `order`; publish после того, как `mqtt_client` уже
подключился к broker и VDA bridge подписался на ROS topic. Иначе раннее
сообщение может быть потеряно, что нормально для QoS 0.

## ROS Interfaces

Подписки:

| ROS topic | Type | Payload |
|---|---|---|
| `/vda5050/order` | `std_msgs/String` | VDA `order` JSON |
| `/vda5050/instantActions` | `std_msgs/String` | VDA `instantActions` JSON |
| `/robot_control_core/status` | `std_msgs/String` | JSON status ядра |

Публикации:

| ROS topic | Type | Payload |
|---|---|---|
| `/vda5050/state` | `std_msgs/String` | VDA `state` JSON |
| `/vda5050/connection` | `std_msgs/String` | VDA `connection` JSON |
| `/vda5050/factsheet` | `std_msgs/String` | VDA `factsheet` JSON |
| `/vda5050_3_driver/events` | `std_msgs/String` | accept/reject/debug events |

Сервисы:

| Service | Type | Что делает |
|---|---|---|
| `/vda5050_3_driver/order` | `forklift_interfaces/srv/StringWithJson` | принять VDA order |
| `/vda5050_3_driver/instant_actions` | `forklift_interfaces/srv/StringWithJson` | принять VDA instantActions |
| `/vda5050_3_driver/state` | `forklift_interfaces/srv/StringWithJson` | сразу вернуть и опубликовать state |
| `/vda5050_3_driver/factsheet` | `forklift_interfaces/srv/StringWithJson` | сразу вернуть и опубликовать factsheet |

## MQTT Topics Через mqtt_client

`mqtt_client` мостит эти topics как primitive string payload:

| MQTT topic | Direction | QoS |
|---|---|---|
| `<prefix>/order` | fleet -> robot | 0 |
| `<prefix>/instantActions` | fleet -> robot | 0 |
| `<prefix>/state` | robot -> fleet | 0 |
| `<prefix>/connection` | robot -> fleet | 1 |
| `<prefix>/factsheet` | robot -> fleet | 0 |

`connection` и `factsheet` публикуются retained. Last will задает
`CONNECTION_BROKEN`.

## Supported Action Types

Order actions, сейчас только scope `NODE`:

| actionType | Что вызывает в ядре | Основные параметры |
|---|---|---|
| `finePositioning` | `/palette_docking/control {"command":"start"}` через `robot_control_core` | `stationName`, `tag_frame`, `standoff_distance_m`, `approach_extra_drive_m`, `final_drive_distance_m`, `max_turn_angle_deg`, `max_turn_angle_rad` |
| `pick` | lower forks -> optional docking -> lift forks -> load state | `loadId`, `height`, `lift_position`, `dock`, `stationName`, `tag_frame` |
| `drop` | optional docking -> lower forks -> clear load state | `loadId`, `height`, `dock`, `stationName`, `tag_frame` |

Instant actions:

| actionType | Mapping |
|---|---|
| `startPause` | `/robot_control_core/control {"command":"startPause"}` |
| `stopPause` | `/robot_control_core/control {"command":"stopPause"}` |
| `cancelOrder` | `/robot_control_core/control {"command":"cancelOrder"}` |
| `stateRequest` | immediately publish `/vda5050/state` |
| `factsheetRequest` | immediately publish `/vda5050/factsheet` |
| `clearInstantActions` | clear finished/failed instant action states tracked by bridge |

`blockingType` values accepted by config: `NONE`, `SINGLE`, `SOFT`, `HARD`.
`robot_control_core` still executes actions serially; parallel VDA semantics are
not implemented yet.

## Какие Orders Сейчас Можно Отправлять

Без дополнительных navigation adapters безопасный формат такой:

```json
{
  "orderId": "pick-current-node-001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "current",
      "sequenceId": 0,
      "released": true,
      "actions": [
        {
          "actionType": "pick",
          "actionId": "pick-001",
          "blockingType": "HARD",
          "actionParameters": [
            {"key": "loadId", "value": "pallet-001"},
            {"key": "stationName", "value": "pallet_b_north_05_tag"},
            {"key": "dock", "value": true}
          ]
        }
      ]
    }
  ],
  "edges": []
}
```

Пример через ROS service:

```bash
ros2 service call /vda5050_3_driver/order forklift_interfaces/srv/StringWithJson \
  "{message: '{\"orderId\":\"pick-current-node-001\",\"orderUpdateId\":0,\"nodes\":[{\"nodeId\":\"current\",\"sequenceId\":0,\"released\":true,\"actions\":[{\"actionType\":\"pick\",\"actionId\":\"pick-001\",\"blockingType\":\"HARD\",\"actionParameters\":[{\"key\":\"loadId\",\"value\":\"pallet-001\"},{\"key\":\"stationName\",\"value\":\"pallet_b_north_05_tag\"},{\"key\":\"dock\",\"value\":true}]}]}],\"edges\":[]}'}"
```

Пример `finePositioning`:

```json
{
  "orderId": "dock-current-node-001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "current",
      "sequenceId": 0,
      "released": true,
      "actions": [
        {
          "actionType": "finePositioning",
          "actionId": "dock-001",
          "blockingType": "HARD",
          "actionParameters": [
            {"key": "stationName", "value": "pallet_b_north_05_tag"}
          ]
        }
      ]
    }
  ],
  "edges": []
}
```

Пример pause/resume/cancel:

```json
{
  "instantActions": [
    {"actionType": "startPause", "actionId": "pause-001"},
    {"actionType": "stopPause", "actionId": "resume-001"},
    {
      "actionType": "cancelOrder",
      "actionId": "cancel-001",
      "actionParameters": [
        {"key": "orderId", "value": "pick-current-node-001"}
      ]
    }
  ]
}
```

## Order Validation

Bridge проверяет минимум, который нужен для безопасного прокидывания в
`robot_control_core`:

- `orderId` непустой, `orderUpdateId` uint32;
- `nodes` непустой, `edges` массив;
- `len(edges) == len(nodes) - 1`;
- `sequenceId` непрерывны: node, edge, node, edge;
- первый node `released=true`;
- released edge разрешен только если оба соседних nodes released;
- после первого horizon элемента нельзя снова присылать released node/edge;
- `actionType` входит в `finePositioning`, `pick`, `drop`;
- edge actions отклоняются, пока `accept_edge_actions=false`;
- route orders с `edges` отклоняются, пока `allow_unconnected_route_orders=false`.

Ошибки публикуются в `/vda5050/state.errors` и возвращаются из service call.

## Что Пока Не Подключено

- VDA route traversal по `nodes/edges` к `navigation_forklift`;
- edge actions с корректной семантикой "работать во время движения по edge";
- `zoneSet`, zone actions, `responses`, `visualization`;
- полная JSON-schema validation по официальным schemas;
- позиция, батарея и safety из реальных сенсоров в VDA state;
- параллельное выполнение `NONE`/`SOFT` actions.

Чтобы включить route orders правильно, следующий шаг должен быть не в
`vda5050_3_driver`, а в `robot_control_core`: добавить capability, который берет
VDA edge/node target и вызывает текущий navigation route executor, после чего
factsheet можно будет объявлять `navigationTypes`.
