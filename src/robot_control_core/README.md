# robot_control_core

`robot_control_core` is the VDA-independent mission owner for the forklift stack.
It does not speak MQTT or VDA 5050 directly. It accepts an internal mission JSON,
executes micro-actions through ROS capabilities, and publishes a normalized robot
state that a VDA adapter, UI, or test client can consume.

## Runtime

- Node: `robot_control_core`
- Control service: `/robot_control_core/control`
- Service type: `forklift_interfaces/srv/StringWithJson`
- Status topic: `/robot_control_core/status`
- Status type: `std_msgs/String` with JSON payload

The node is the only intended owner of an active mission. Higher-level adapters
should call this service instead of directly calling navigation, fork, docking, or
motion-arbitration handles.

## Layers

```text
vda5050_3_driver / UI / tests
  -> /robot_control_core/control
robot_control_core
  -> NavigationCapability  -> /forklift_nav/move_to, /forklift_nav/revers_move_to
  -> ForkCapability        -> /forklift/fork_cmd + /joint_states
  -> DockingCapability     -> /palette_docking/control
  -> MotionCapability      -> /cmd_vel_arcestrator/control
```

## FSM

Top-level states:

```text
STARTUP -> IDLE -> ORDER_ACTIVE -> IDLE
                    |
                    +-> EXECUTING_BASE
                    +-> WAITING_FOR_BASE_EXTENSION
                    +-> PAUSED
                    +-> CANCELLING
                    +-> FAILED_RECOVERABLE
MANUAL / INTERVENED / SERVICE are operating modes that gate mission execution.
```

`ORDER_ACTIVE` runs a conservative action scheduler: one micro-action is active
at a time. This already respects VDA blocking semantics. The action model keeps
`blockingType` (`NONE`, `SINGLE`, `SOFT`, `HARD`) in state so the scheduler can
be expanded later to parallel `NONE`/`SOFT` execution without changing the API.

When released base actions finish and horizon actions are still present, the core
switches to `WAITING_FOR_BASE_EXTENSION`, stops motion via `cmd_vel_arcestrator`,
and reports `newBaseRequest: true`.

## Control Commands

Status:

```bash
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"status\"}'}"
```

Start a mission:

```bash
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"start_mission\", \"mission_id\": \"demo_pick\", \"steps\": [{\"type\": \"navigate\", \"target\": \"0001\"}, {\"type\": \"pick\", \"tag_frame\": \"pallet_b_south_08_tag\"}, {\"type\": \"navigate\", \"target\": \"0002\"}, {\"type\": \"drop\"}]}'}"
```

Pause/resume/cancel:

```bash
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"pause\"}'}"
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"resume\"}'}"
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"cancel\"}'}"
```

Release horizon into base:

```bash
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"append_base\", \"release_horizon_count\": 2}'}"
```

Set operating mode:

```bash
ros2 service call /robot_control_core/control forklift_interfaces/srv/StringWithJson "{message: '{\"command\": \"set_mode\", \"mode\": \"AUTOMATIC\"}'}"
```

## Mission Schema

Accepted `start_mission` fields:

- `mission_id`: internal mission identifier.
- `order_id`, `order_update_id`: optional VDA-facing identifiers passed through
  into status.
- `steps`: released base micro-actions.
- `base_steps`: alias for `steps`.
- `horizon_steps`: planned but unreleased micro-actions.
- `nodeStates`, `edgeStates`: optional pass-through VDA-style state arrays.

Supported micro-actions:

```json
{"type": "navigate", "target": "0001", "reverse": false}
{"type": "fork_position", "position": 0.0}
{"type": "dock", "tag_frame": "pallet_b_south_08_tag"}
{"type": "pick", "tag_frame": "pallet_b_south_08_tag", "load_id": "L1"}
{"type": "drop"}
{"type": "wait", "duration_sec": 2.0}
{"type": "set_load", "loaded": true, "load_id": "L1"}
```

`pick` expands to:

```text
fork_position(lower) -> dock -> fork_position(lift) -> set_load(true)
```

`drop` expands to:

```text
optional dock -> fork_position(lower) -> set_load(false)
```

Every step may include `actionId`/`action_id` and `blockingType`. If omitted, the
core generates an ID and uses `HARD`.

## VDA 5050 Mapping

The VDA driver should remain a protocol adapter:

- MQTT `order` -> validate JSON/schema -> convert released nodes/edges/actions to
  `start_mission` or `append_base`.
- MQTT `instantActions.startPause` -> `pause`.
- MQTT `instantActions.stopPause` -> `resume`.
- MQTT `instantActions.cancelOrder` -> `cancel`.
- Core `/robot_control_core/status` -> VDA `state`.

Recommended action mapping:

- VDA edge traversal -> `navigate`.
- VDA `finePositioning` -> `dock`.
- VDA `pick` -> `pick`.
- VDA `drop` -> `drop`.

## Current Limits

- The action scheduler is intentionally conservative and runs one action at a time.
- Navigation completion depends on `/forklift_nav/status`, added to
  `navigation_forklift/route_service`.
- Navigation cancellation is currently enforced by stopping `cmd_vel_arcestrator`;
  the underlying Nav2 route service does not yet cancel its active action goal.
- Load state is internal state only; there is no physical load sensor integration yet.
