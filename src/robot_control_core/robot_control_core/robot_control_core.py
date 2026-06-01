import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from forklift_interfaces.srv import StringWithJson
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


STARTUP = "STARTUP"
IDLE = "IDLE"
ORDER_ACTIVE = "ORDER_ACTIVE"
EXECUTING_BASE = "EXECUTING_BASE"
WAITING_FOR_BASE_EXTENSION = "WAITING_FOR_BASE_EXTENSION"
PAUSED = "PAUSED"
CANCELLING = "CANCELLING"
FAILED_RECOVERABLE = "FAILED_RECOVERABLE"

ACTION_WAITING = "WAITING"
ACTION_INITIALIZING = "INITIALIZING"
ACTION_RUNNING = "RUNNING"
ACTION_PAUSED = "PAUSED"
ACTION_RETRIABLE = "RETRIABLE"
ACTION_FINISHED = "FINISHED"
ACTION_FAILED = "FAILED"

END_ACTION_STATES = {ACTION_FINISHED, ACTION_FAILED}
NON_ORDER_MODES = {"MANUAL", "SERVICE", "TEACH_IN"}
VALID_OPERATING_MODES = {
    "AUTOMATIC",
    "SEMIAUTOMATIC",
    "INTERVENED",
    "MANUAL",
    "STARTUP",
    "SERVICE",
    "TEACH_IN",
}


@dataclass
class MicroAction:
    action_id: str
    action_type: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    blocking_type: str = "HARD"
    status: str = ACTION_WAITING
    result: str = ""
    runner: Dict[str, Any] = field(default_factory=dict)
    pause_allowed: bool = True
    cancel_allowed: bool = True


class RobotControlCore(Node):
    def __init__(self) -> None:
        super().__init__("robot_control_core")

        self.declare_parameter("control_service", "/robot_control_core/control")
        self.declare_parameter("status_topic", "/robot_control_core/status")
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("status_publish_period_sec", 1.0)
        self.declare_parameter(
            "motion_control_service", "/cmd_vel_arcestrator/control"
        )
        self.declare_parameter("navigation_move_to_service", "/forklift_nav/move_to")
        self.declare_parameter(
            "navigation_reverse_move_to_service", "/forklift_nav/revers_move_to"
        )
        self.declare_parameter("navigation_status_service", "/forklift_nav/status")
        self.declare_parameter("docking_control_service", "/palette_docking/control")
        self.declare_parameter("fork_cmd_topic", "/forklift/fork_cmd")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("fork_joint_name", "fork_joint")
        self.declare_parameter("fork_lower_position", 0.0)
        self.declare_parameter("fork_lift_position", 1.0)
        self.declare_parameter("fork_position_tolerance", 0.01)
        self.declare_parameter("fork_action_timeout_sec", 20.0)
        self.declare_parameter("service_call_timeout_sec", 2.0)
        self.declare_parameter("action_poll_period_sec", 0.25)

        control_service = str(self.get_parameter("control_service").value)
        status_topic = str(self.get_parameter("status_topic").value)
        tick_rate_hz = max(1.0, float(self.get_parameter("tick_rate_hz").value))
        self._status_period = max(
            0.0, float(self.get_parameter("status_publish_period_sec").value)
        )
        self._fork_joint_name = str(self.get_parameter("fork_joint_name").value)
        self._fork_lower_position = float(
            self.get_parameter("fork_lower_position").value
        )
        self._fork_lift_position = float(self.get_parameter("fork_lift_position").value)
        self._fork_tolerance = max(
            0.0, float(self.get_parameter("fork_position_tolerance").value)
        )
        self._fork_timeout = max(
            0.0, float(self.get_parameter("fork_action_timeout_sec").value)
        )
        self._service_timeout = max(
            0.1, float(self.get_parameter("service_call_timeout_sec").value)
        )
        self._poll_period = max(
            0.05, float(self.get_parameter("action_poll_period_sec").value)
        )

        self._motion_client = self.create_client(
            StringWithJson, str(self.get_parameter("motion_control_service").value)
        )
        self._nav_move_client = self.create_client(
            StringWithJson, str(self.get_parameter("navigation_move_to_service").value)
        )
        self._nav_reverse_client = self.create_client(
            StringWithJson,
            str(self.get_parameter("navigation_reverse_move_to_service").value),
        )
        self._nav_status_client = self.create_client(
            StringWithJson, str(self.get_parameter("navigation_status_service").value)
        )
        self._dock_client = self.create_client(
            StringWithJson, str(self.get_parameter("docking_control_service").value)
        )
        self._fork_pub = self.create_publisher(
            Float64, str(self.get_parameter("fork_cmd_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_callback,
            10,
        )
        self.create_service(StringWithJson, control_service, self._handle_control)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(1.0 / tick_rate_hz, self._tick)

        self._state = STARTUP
        self._substate = ""
        self._operating_mode = "STARTUP"
        self._mission_id = ""
        self._order_id = ""
        self._order_update_id = 0
        self._queue: List[MicroAction] = []
        self._horizon: List[MicroAction] = []
        self._actions: List[MicroAction] = []
        self._current_action: Optional[MicroAction] = None
        self._instant_action_states: List[Dict[str, Any]] = []
        self._node_states: List[Dict[str, Any]] = []
        self._edge_states: List[Dict[str, Any]] = []
        self._loads: List[Dict[str, Any]] = []
        self._errors: List[Dict[str, Any]] = []
        self._paused = False
        self._new_base_request = False
        self._last_status_at: Optional[Time] = None
        self._last_tick_started = False
        self._fork_position: Optional[float] = None
        self._action_counter = 0

        self.get_logger().info(
            "robot_control_core ready: control=%s status=%s tick=%.1fHz"
            % (control_service, status_topic, tick_rate_hz)
        )

    def _handle_control(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_json", "details": str(exc)}
            )
            return response

        command = str(payload.get("command", "status")).strip().lower()
        try:
            if command in {"", "status"}:
                response.success = True
            elif command in {"start", "start_mission", "run_mission"}:
                self._start_mission(payload)
                response.success = True
            elif command in {"append_base", "extend_base", "release_horizon"}:
                self._append_base(payload)
                response.success = True
            elif command in {"pause", "startpause", "start_pause"}:
                self._pause("control pause")
                self._record_instant_action("startPause", ACTION_FINISHED)
                response.success = True
            elif command in {"resume", "stoppause", "stop_pause", "release"}:
                self._resume("control resume")
                self._record_instant_action("stopPause", ACTION_FINISHED)
                response.success = True
            elif command in {"cancel", "cancelorder", "cancel_order"}:
                self._cancel("control cancel")
                self._record_instant_action("cancelOrder", ACTION_FINISHED)
                response.success = True
            elif command in {"set_mode", "mode"}:
                self._set_operating_mode(payload)
                response.success = True
            elif command in {"reset", "clear"}:
                self._reset()
                response.success = True
            elif command == "clear_errors":
                self._errors.clear()
                response.success = True
            else:
                response.success = False
                response.message = json.dumps(
                    {
                        "error": "invalid_command",
                        "details": (
                            "expected start_mission|append_base|pause|resume|"
                            "cancel|set_mode|reset|status"
                        ),
                    }
                )
                return response
        except ValueError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_request", "details": str(exc)}
            )
            return response

        response.message = json.dumps(self._status(), ensure_ascii=False)
        self._publish_status(force=True)
        return response

    def _tick(self) -> None:
        if not self._last_tick_started:
            self._last_tick_started = True
            self._operating_mode = "AUTOMATIC"
            self._state = IDLE

        if (
            self._state == ORDER_ACTIVE
            and not self._paused
            and self._substate == EXECUTING_BASE
        ):
            self._run_scheduler()

        self._publish_status()

    def _start_mission(self, payload: Dict[str, Any]) -> None:
        if self._state == ORDER_ACTIVE:
            raise ValueError("mission is already active")
        if self._operating_mode not in {"AUTOMATIC", "SEMIAUTOMATIC", "INTERVENED"}:
            raise ValueError("operating mode does not allow missions")

        base_steps = payload.get("base_steps", payload.get("steps", []))
        if not isinstance(base_steps, list):
            raise ValueError("steps/base_steps must be an array")
        horizon_steps = payload.get("horizon_steps", [])
        if not isinstance(horizon_steps, list):
            raise ValueError("horizon_steps must be an array")

        self._mission_id = str(payload.get("mission_id", payload.get("order_id", "")))
        self._order_id = str(payload.get("order_id", ""))
        self._order_update_id = int(payload.get("order_update_id", 0))
        self._node_states = list(payload.get("nodeStates", payload.get("node_states", [])))
        self._edge_states = list(payload.get("edgeStates", payload.get("edge_states", [])))
        self._errors.clear()
        self._queue = self._expand_steps(base_steps, prefix="base")
        self._horizon = self._expand_steps(horizon_steps, prefix="horizon")
        self._actions = [*self._queue, *self._horizon]
        self._current_action = None
        self._paused = False
        self._new_base_request = False
        self._state = ORDER_ACTIVE
        self._substate = EXECUTING_BASE
        self.get_logger().info(
            "mission started: mission_id=%s base_actions=%d horizon_actions=%d"
            % (self._mission_id or "<empty>", len(self._queue), len(self._horizon))
        )

    def _append_base(self, payload: Dict[str, Any]) -> None:
        if self._state != ORDER_ACTIVE:
            raise ValueError("no active mission")
        steps = payload.get("steps", payload.get("base_steps"))
        if steps is None:
            count = int(payload.get("release_horizon_count", len(self._horizon)))
            count = max(0, min(count, len(self._horizon)))
            self._queue.extend(self._horizon[:count])
            self._horizon = self._horizon[count:]
        else:
            if not isinstance(steps, list):
                raise ValueError("steps/base_steps must be an array")
            new_actions = self._expand_steps(steps, prefix="base")
            self._queue.extend(new_actions)
            self._actions.extend(new_actions)

        horizon_steps = payload.get("horizon_steps", [])
        if horizon_steps:
            if not isinstance(horizon_steps, list):
                raise ValueError("horizon_steps must be an array")
            new_horizon = self._expand_steps(horizon_steps, prefix="horizon")
            self._horizon.extend(new_horizon)
            self._actions.extend(new_horizon)

        self._order_update_id = int(payload.get("order_update_id", self._order_update_id))
        self._new_base_request = False
        self._substate = EXECUTING_BASE

    def _pause(self, reason: str) -> None:
        if self._state != ORDER_ACTIVE:
            return
        self._paused = True
        self._substate = PAUSED
        if self._current_action and self._current_action.status == ACTION_RUNNING:
            self._current_action.status = ACTION_PAUSED
        self._call_json(self._motion_client, {"command": "stop"})
        self.get_logger().info("mission paused: %s" % reason)

    def _resume(self, reason: str) -> None:
        if self._state != ORDER_ACTIVE:
            return
        if self._substate == FAILED_RECOVERABLE:
            raise ValueError("mission is failed recoverable; cancel or reset it first")
        self._paused = False
        self._substate = EXECUTING_BASE
        if self._current_action and self._current_action.status == ACTION_PAUSED:
            self._current_action.status = ACTION_RUNNING
        self.get_logger().info("mission resumed: %s" % reason)

    def _cancel(self, reason: str) -> None:
        if self._state != ORDER_ACTIVE:
            self._call_json(self._motion_client, {"command": "stop"})
            return
        self._substate = CANCELLING
        self._cancel_current_action()
        for action in self._actions:
            if action.status not in END_ACTION_STATES:
                action.status = ACTION_FAILED
                action.result = "cancelled"
        self._queue.clear()
        self._horizon.clear()
        self._node_states.clear()
        self._edge_states.clear()
        self._new_base_request = False
        self._paused = False
        self._call_json(self._motion_client, {"command": "stop"})
        self._state = IDLE
        self._substate = ""
        self.get_logger().warn("mission cancelled: %s" % reason)

    def _reset(self) -> None:
        self._cancel_current_action()
        self._queue.clear()
        self._horizon.clear()
        self._actions.clear()
        self._node_states.clear()
        self._edge_states.clear()
        self._errors.clear()
        self._mission_id = ""
        self._order_id = ""
        self._order_update_id = 0
        self._paused = False
        self._new_base_request = False
        self._state = IDLE
        self._substate = ""
        self._call_json(self._motion_client, {"command": "stop"})

    def _set_operating_mode(self, payload: Dict[str, Any]) -> None:
        mode = str(payload.get("mode", payload.get("operating_mode", ""))).upper()
        if mode not in VALID_OPERATING_MODES:
            raise ValueError("invalid operating mode: %s" % mode)
        self._operating_mode = mode
        if mode in NON_ORDER_MODES or mode == "STARTUP":
            self._cancel("mode changed to %s" % mode)
            self._state = mode
        elif mode == "INTERVENED":
            self._pause("mode changed to INTERVENED")
            self._state = ORDER_ACTIVE if self._current_action or self._queue else IDLE
        elif self._state in VALID_OPERATING_MODES:
            self._state = IDLE
        self.get_logger().info("operating mode set: %s" % mode)

    def _run_scheduler(self) -> None:
        if self._current_action is None and self._queue:
            self._current_action = self._queue.pop(0)
            self._start_action(self._current_action)

        if self._current_action is not None:
            self._poll_action(self._current_action)
            if self._current_action.status in END_ACTION_STATES:
                finished = self._current_action
                self._current_action = None
                if finished.status == ACTION_FAILED:
                    self._substate = FAILED_RECOVERABLE
                    self._errors.append(
                        {
                            "errorType": "ACTION_FAILED",
                            "errorLevel": "WARNING",
                            "errorReferences": [
                                {
                                    "referenceKey": "actionId",
                                    "referenceValue": finished.action_id,
                                }
                            ],
                            "errorDescription": finished.result,
                        }
                    )
                    self._call_json(self._motion_client, {"command": "stop"})
                    return

        if self._current_action is None and not self._queue:
            if self._horizon:
                self._substate = WAITING_FOR_BASE_EXTENSION
                self._new_base_request = True
                self._call_json(self._motion_client, {"command": "stop"})
                return
            self._state = IDLE
            self._substate = ""
            self._node_states.clear()
            self._edge_states.clear()
            self._new_base_request = False
            self._call_json(self._motion_client, {"command": "stop"})
            self.get_logger().info(
                "mission finished: mission_id=%s" % (self._mission_id or "<empty>")
            )

    def _start_action(self, action: MicroAction) -> None:
        action.status = ACTION_INITIALIZING
        action.runner = {
            "started_at": self.get_clock().now(),
            "last_poll_at": None,
            "phase": "start",
        }
        action_type = action.action_type

        if action_type == "navigate":
            self._start_navigation(action)
        elif action_type in {"dock", "finePositioning", "fine_positioning"}:
            self._start_docking(action)
        elif action_type == "fork_position":
            self._start_fork_position(action)
        elif action_type == "wait":
            action.status = ACTION_RUNNING
        elif action_type == "set_load":
            self._set_load(action)
        else:
            action.status = ACTION_FAILED
            action.result = "unsupported action type: %s" % action_type

    def _poll_action(self, action: MicroAction) -> None:
        if action.status in END_ACTION_STATES:
            return
        action_type = action.action_type
        if action_type == "navigate":
            self._poll_navigation(action)
        elif action_type in {"dock", "finePositioning", "fine_positioning"}:
            self._poll_docking(action)
        elif action_type == "fork_position":
            self._poll_fork_position(action)
        elif action_type == "wait":
            self._poll_wait(action)

    def _start_navigation(self, action: MicroAction) -> None:
        params = action.parameters
        reverse = bool(params.get("reverse", params.get("backwards", False)))
        client = self._nav_reverse_client if reverse else self._nav_move_client
        target = params.get("target", params.get("id", params.get("to")))
        payload = dict(params.get("payload", {}))
        if target is not None and "id" not in payload:
            payload["id"] = target
        if "id" not in payload:
            action.status = ACTION_FAILED
            action.result = "navigate action requires target/id/to"
            return

        self._call_json(self._motion_client, {"command": "select_source", "source": "navigation"})
        future = self._call_json(client, payload)
        if future is None:
            action.status = ACTION_FAILED
            action.result = "navigation service is not available"
            return
        action.runner.update({"future": future, "phase": "accept"})
        action.status = ACTION_RUNNING

    def _poll_navigation(self, action: MicroAction) -> None:
        phase = action.runner.get("phase")
        if phase == "accept":
            future = action.runner.get("future")
            if not future.done():
                self._fail_on_timeout(action, "navigation accept timeout")
                return
            if not self._response_success(future, action, "navigation rejected"):
                return
            action.runner["phase"] = "poll_status"
            action.runner["status_future"] = None
            return

        if phase != "poll_status":
            return
        status_future = action.runner.get("status_future")
        if status_future is None:
            if not self._poll_due(action):
                return
            future = self._call_json(self._nav_status_client, {"command": "status"})
            if future is None:
                action.status = ACTION_FINISHED
                action.result = "navigation accepted; no status service"
                return
            action.runner["status_future"] = future
            return

        if not status_future.done():
            return
        try:
            response = status_future.result()
            status = json.loads(response.message or "{}")
        except Exception as exc:
            action.status = ACTION_FAILED
            action.result = "navigation status failed: %s" % exc
            return
        action.runner["status_future"] = None
        if status.get("active", False):
            return
        last_result = status.get("last_result", {})
        if last_result.get("state") == "failed":
            action.status = ACTION_FAILED
            action.result = str(last_result.get("error", "navigation failed"))
            return
        action.status = ACTION_FINISHED
        action.result = "navigation finished"

    def _start_docking(self, action: MicroAction) -> None:
        payload = {"command": "start"}
        for key in (
            "tag_frame",
            "standoff_distance_m",
            "approach_extra_drive_m",
            "final_drive_distance_m",
            "max_turn_angle_deg",
            "max_turn_angle_rad",
        ):
            if key in action.parameters:
                payload[key] = action.parameters[key]
        self._call_json(
            self._motion_client, {"command": "select_source", "source": "pallet_docking"}
        )
        future = self._call_json(self._dock_client, payload)
        if future is None:
            action.status = ACTION_FAILED
            action.result = "docking service is not available"
            return
        action.runner.update({"future": future, "phase": "accept", "status_future": None})
        action.status = ACTION_RUNNING

    def _poll_docking(self, action: MicroAction) -> None:
        if action.runner.get("phase") == "accept":
            future = action.runner.get("future")
            if not future.done():
                self._fail_on_timeout(action, "docking accept timeout")
                return
            if not self._response_success(future, action, "docking rejected"):
                return
            action.runner["phase"] = "poll_status"
            return

        status_future = action.runner.get("status_future")
        if status_future is None:
            if not self._poll_due(action):
                return
            future = self._call_json(self._dock_client, {"command": "status"})
            if future is None:
                action.status = ACTION_FAILED
                action.result = "docking status service is not available"
                return
            action.runner["status_future"] = future
            return

        if not status_future.done():
            return
        try:
            response = status_future.result()
            status = json.loads(response.message or "{}")
        except Exception as exc:
            action.status = ACTION_FAILED
            action.result = "docking status failed: %s" % exc
            return
        action.runner["status_future"] = None
        docking_state = str(status.get("state", ""))
        if docking_state == "DONE":
            action.status = ACTION_FINISHED
            action.result = "docking done"
        elif docking_state == "FAILED":
            action.status = ACTION_FAILED
            action.result = str(status.get("failure_reason", "docking failed"))

    def _start_fork_position(self, action: MicroAction) -> None:
        target = float(action.parameters.get("position", self._fork_lower_position))
        action.runner["target"] = target
        self._publish_fork_target(target)
        action.status = ACTION_RUNNING

    def _poll_fork_position(self, action: MicroAction) -> None:
        target = float(action.runner["target"])
        if self._fork_position is not None:
            if math.fabs(self._fork_position - target) <= self._fork_tolerance:
                action.status = ACTION_FINISHED
                action.result = "fork position reached"
                return
        if self._elapsed(action.runner["started_at"]) > self._fork_timeout:
            action.status = ACTION_FAILED
            action.result = "fork position timeout"
            return
        if self._poll_due(action):
            self._publish_fork_target(target)

    def _poll_wait(self, action: MicroAction) -> None:
        duration = max(0.0, float(action.parameters.get("duration_sec", 0.0)))
        if self._elapsed(action.runner["started_at"]) >= duration:
            action.status = ACTION_FINISHED
            action.result = "wait finished"

    def _set_load(self, action: MicroAction) -> None:
        loaded = bool(action.parameters.get("loaded", True))
        load_id = str(action.parameters.get("load_id", "load"))
        if loaded:
            self._loads = [{"loadId": load_id}]
        else:
            self._loads = []
        action.status = ACTION_FINISHED
        action.result = "load set" if loaded else "load cleared"

    def _cancel_current_action(self) -> None:
        if self._current_action is None:
            return
        action = self._current_action
        if action.action_type in {"dock", "finePositioning", "fine_positioning"}:
            self._call_json(self._dock_client, {"command": "stop"})
        self._call_json(self._motion_client, {"command": "stop"})
        if action.status not in END_ACTION_STATES:
            action.status = ACTION_FAILED
            action.result = "cancelled"
        self._current_action = None

    def _response_success(self, future, action: MicroAction, default_error: str) -> bool:
        try:
            response = future.result()
        except Exception as exc:
            action.status = ACTION_FAILED
            action.result = "%s: %s" % (default_error, exc)
            return False
        if not response.success:
            action.status = ACTION_FAILED
            action.result = response.message or default_error
            return False
        return True

    def _fail_on_timeout(self, action: MicroAction, message: str) -> bool:
        if self._elapsed(action.runner["started_at"]) <= self._service_timeout:
            return False
        action.status = ACTION_FAILED
        action.result = message
        return True

    def _poll_due(self, action: MicroAction) -> bool:
        now = self.get_clock().now()
        last_poll_at = action.runner.get("last_poll_at")
        if last_poll_at is not None and self._seconds_since(last_poll_at, now) < self._poll_period:
            return False
        action.runner["last_poll_at"] = now
        return True

    def _call_json(self, client, payload: Dict[str, Any]):
        if not client.wait_for_service(timeout_sec=0.0):
            return None
        request = StringWithJson.Request()
        request.message = json.dumps(payload, ensure_ascii=False)
        return client.call_async(request)

    def _publish_fork_target(self, target: float) -> None:
        msg = Float64()
        msg.data = float(target)
        self._fork_pub.publish(msg)

    def _joint_state_callback(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self._fork_joint_name)
        except ValueError:
            return
        if index < len(msg.position):
            self._fork_position = float(msg.position[index])

    def _expand_steps(self, steps: List[Dict[str, Any]], *, prefix: str) -> List[MicroAction]:
        actions: List[MicroAction] = []
        for step in steps:
            if not isinstance(step, dict):
                raise ValueError("mission steps must be JSON objects")
            step_type = str(step.get("type", step.get("actionType", "")))
            if step_type == "pick":
                actions.extend(self._expand_pick(step, prefix=prefix))
            elif step_type == "drop":
                actions.extend(self._expand_drop(step, prefix=prefix))
            else:
                actions.append(self._make_action(step, prefix=prefix))
        return actions

    def _expand_pick(self, step: Dict[str, Any], *, prefix: str) -> List[MicroAction]:
        action_id = str(step.get("action_id", step.get("actionId", self._next_action_id(prefix, "pick"))))
        lower = float(step.get("lower_position", self._fork_lower_position))
        lift = float(step.get("lift_position", self._fork_lift_position))
        load_id = str(step.get("load_id", step.get("loadId", "load")))
        expanded = [
            {"type": "fork_position", "action_id": f"{action_id}:lower", "position": lower},
        ]
        if bool(step.get("dock", True)):
            dock_step = {"type": "dock", "action_id": f"{action_id}:dock"}
            if "tag_frame" in step:
                dock_step["tag_frame"] = step["tag_frame"]
            expanded.append(dock_step)
        expanded.extend(
            [
                {"type": "fork_position", "action_id": f"{action_id}:lift", "position": lift},
                {
                    "type": "set_load",
                    "action_id": f"{action_id}:load",
                    "loaded": True,
                    "load_id": load_id,
                    "blockingType": "NONE",
                },
            ]
        )
        return [self._make_action(item, prefix=prefix) for item in expanded]

    def _expand_drop(self, step: Dict[str, Any], *, prefix: str) -> List[MicroAction]:
        action_id = str(step.get("action_id", step.get("actionId", self._next_action_id(prefix, "drop"))))
        lower = float(step.get("lower_position", self._fork_lower_position))
        expanded = []
        if bool(step.get("dock", False)):
            dock_step = {"type": "dock", "action_id": f"{action_id}:dock"}
            if "tag_frame" in step:
                dock_step["tag_frame"] = step["tag_frame"]
            expanded.append(dock_step)
        expanded.extend(
            [
                {"type": "fork_position", "action_id": f"{action_id}:lower", "position": lower},
                {
                    "type": "set_load",
                    "action_id": f"{action_id}:load",
                    "loaded": False,
                    "blockingType": "NONE",
                },
            ]
        )
        return [self._make_action(item, prefix=prefix) for item in expanded]

    def _make_action(self, step: Dict[str, Any], *, prefix: str) -> MicroAction:
        action_type = str(step.get("type", step.get("actionType", "")))
        if not action_type:
            raise ValueError("mission step has no type/actionType")
        action_id = str(
            step.get("action_id", step.get("actionId", self._next_action_id(prefix, action_type)))
        )
        blocking_type = str(step.get("blockingType", step.get("blocking_type", "HARD"))).upper()
        if blocking_type not in {"NONE", "SINGLE", "SOFT", "HARD"}:
            raise ValueError("invalid blockingType for %s: %s" % (action_id, blocking_type))
        params = {
            key: value
            for key, value in step.items()
            if key not in {"type", "actionType", "action_id", "actionId", "blockingType", "blocking_type"}
        }
        return MicroAction(
            action_id=action_id,
            action_type=action_type,
            parameters=params,
            blocking_type=blocking_type,
            pause_allowed=bool(step.get("pauseAllowed", True)),
            cancel_allowed=bool(step.get("cancelAllowed", True)),
        )

    def _next_action_id(self, prefix: str, action_type: str) -> str:
        self._action_counter += 1
        return "%s_%03d_%s" % (prefix, self._action_counter, action_type)

    def _record_instant_action(self, action_type: str, status: str) -> None:
        self._instant_action_states.append(
            {
                "actionId": "%s_%d" % (action_type, len(self._instant_action_states) + 1),
                "actionType": action_type,
                "actionStatus": status,
            }
        )

    def _publish_status(self, *, force: bool = False) -> None:
        if not force and self._status_period <= 0.0:
            return
        now = self.get_clock().now()
        if (
            not force
            and self._last_status_at is not None
            and self._seconds_since(self._last_status_at, now) < self._status_period
        ):
            return
        self._last_status_at = now
        msg = String()
        msg.data = json.dumps(self._status(), ensure_ascii=False)
        self._status_pub.publish(msg)

    def _status(self) -> Dict[str, Any]:
        return {
            "state": self._state,
            "substate": self._substate,
            "operatingMode": self._operating_mode,
            "missionId": self._mission_id,
            "orderId": self._order_id,
            "orderUpdateId": self._order_update_id,
            "paused": self._paused,
            "newBaseRequest": self._new_base_request,
            "currentActionId": self._current_action.action_id if self._current_action else "",
            "queuedActionIds": [action.action_id for action in self._queue],
            "horizonActionIds": [action.action_id for action in self._horizon],
            "nodeStates": self._node_states,
            "edgeStates": self._edge_states,
            "actionStates": [self._action_state(action) for action in self._actions],
            "instantActionStates": self._instant_action_states,
            "loads": self._loads,
            "errors": self._errors,
            "forkPosition": self._fork_position,
        }

    def _action_state(self, action: MicroAction) -> Dict[str, Any]:
        return {
            "actionId": action.action_id,
            "actionType": action.action_type,
            "blockingType": action.blocking_type,
            "actionStatus": action.status,
            "actionResult": action.result,
        }

    def _elapsed(self, earlier: Time) -> float:
        return self._seconds_since(earlier, self.get_clock().now())

    def _seconds_since(self, earlier: Time, later: Time) -> float:
        seconds = (later - earlier).nanoseconds / 1_000_000_000.0
        return max(0.0, seconds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotControlCore()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._call_json(node._motion_client, {"command": "stop"})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
