import json
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from forklift_interfaces.srv import StringWithJson
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

try:
    from yasmin import Blackboard, State, StateMachine
except ModuleNotFoundError as exc:  # pragma: no cover - deployment guard
    raise ModuleNotFoundError(
        "robot_control_core requires YASMIN; install ros-$ROS_DISTRO-yasmin"
    ) from exc

try:
    from yasmin_viewer.yasmin_viewer_pub import YasminViewerPub
except ModuleNotFoundError:
    YasminViewerPub = None


STARTUP = "STARTUP"
IDLE = "IDLE"
AUTOMATIC = "AUTOMATIC"
SEMIAUTOMATIC = "SEMIAUTOMATIC"
INTERVENED = "INTERVENED"
MANUAL = "MANUAL"
SERVICE = "SERVICE"
TEACH_IN = "TEACH_IN"
ORDER_ACTIVE = "ORDER_ACTIVE"
DISPATCH = "DISPATCH"
EXECUTING_BASE = "EXECUTING_BASE"
WAITING_FOR_BASE_EXTENSION = "WAITING_FOR_BASE_EXTENSION"
PAUSED = "PAUSED"
CANCELLING = "CANCELLING"
FAILED_RECOVERABLE = "FAILED_RECOVERABLE"

ROBOT_SHUTDOWN = "shutdown"
MISSION_STARTED = "mission_started"
FSM_RUNNING = "running"
FSM_PAUSED = "paused"
FSM_RESUMED = "resumed"
FSM_NEED_BASE = "need_base"
FSM_BASE_EXTENDED = "base_extended"
FSM_FAILED = "failed"
FSM_FINISHED = "finished"
FSM_CANCELLED = "cancelled"

ACTION_WAITING = "WAITING"
ACTION_INITIALIZING = "INITIALIZING"
ACTION_RUNNING = "RUNNING"
ACTION_PAUSED = "PAUSED"
ACTION_RETRIABLE = "RETRIABLE"
ACTION_FINISHED = "FINISHED"
ACTION_FAILED = "FAILED"

END_ACTION_STATES = {ACTION_FINISHED, ACTION_FAILED}
SUPPORTED_ORDER_ACTION_TYPES = {"finePositioning", "pick", "drop"}
ORDER_MODES = {AUTOMATIC, SEMIAUTOMATIC, INTERVENED}
NON_ORDER_MODES = {MANUAL, SERVICE, TEACH_IN}
VALID_OPERATING_MODES = {
    AUTOMATIC,
    SEMIAUTOMATIC,
    INTERVENED,
    MANUAL,
    STARTUP,
    SERVICE,
    TEACH_IN,
}
OPERATING_MODE_ORDER = (
    STARTUP,
    AUTOMATIC,
    SEMIAUTOMATIC,
    INTERVENED,
    MANUAL,
    SERVICE,
    TEACH_IN,
)
MODE_OUTCOMES = tuple("to_%s" % mode for mode in OPERATING_MODE_ORDER)
STATUS_COMMANDS = {"", "status", "staterequest", "state_request"}
START_COMMANDS = {"start", "start_mission", "run_mission"}
APPEND_BASE_COMMANDS = {"append_base", "extend_base", "release_horizon"}
PAUSE_COMMANDS = {"pause", "startpause", "start_pause"}
RESUME_COMMANDS = {"resume", "stoppause", "stop_pause", "release"}
CANCEL_COMMANDS = {"cancel", "cancelorder", "cancel_order"}


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


class RobotModeState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__([*MODE_OUTCOMES, ROBOT_SHUTDOWN])
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_robot_mode(self._mode)
        while rclpy.ok():
            outcome = self._core._mode_transition_outcome(self._mode)
            if outcome:
                return outcome
            self._core._wait_for_mission_event()
        return ROBOT_SHUTDOWN


class MissionDispatchState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__(
            [
                IDLE,
                EXECUTING_BASE,
                WAITING_FOR_BASE_EXTENSION,
                PAUSED,
                FAILED_RECOVERABLE,
                CANCELLING,
                *MODE_OUTCOMES,
                ROBOT_SHUTDOWN,
            ]
        )
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_robot_mode(self._mode)
        return self._core._mission_dispatch_outcome(self._mode)


class MissionIdleState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__([MISSION_STARTED, *MODE_OUTCOMES, ROBOT_SHUTDOWN])
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_mission_idle(self._mode)
        while rclpy.ok():
            outcome = self._core._mode_transition_outcome(self._mode)
            if outcome:
                return outcome
            if self._core._mission_active_snapshot():
                return MISSION_STARTED
            self._core._wait_for_mission_event()
        return ROBOT_SHUTDOWN


class MissionExecutingState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__(
            [
                FSM_PAUSED,
                FSM_NEED_BASE,
                FSM_FAILED,
                FSM_FINISHED,
                FSM_CANCELLED,
                *MODE_OUTCOMES,
                ROBOT_SHUTDOWN,
            ]
        )
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_mission_substate(EXECUTING_BASE)
        while rclpy.ok():
            mode_outcome = self._core._mode_transition_outcome(self._mode)
            if mode_outcome:
                return mode_outcome
            outcome = self._core._execute_base_once()
            if outcome != FSM_RUNNING:
                return outcome
            self._core._wait_for_mission_event()
        return FSM_CANCELLED

    def cancel_state(self) -> None:
        self._core._request_cancel_from_fsm("yasmin cancel")
        super().cancel_state()


class MissionPausedState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__([FSM_RESUMED, FSM_CANCELLED, *MODE_OUTCOMES, ROBOT_SHUTDOWN])
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_mission_substate(PAUSED)
        self._core._on_paused_state_entered()
        while rclpy.ok():
            mode_outcome = self._core._mode_transition_outcome(self._mode)
            if mode_outcome:
                return mode_outcome
            outcome = self._core._paused_state_outcome()
            if outcome:
                return outcome
            self._core._wait_for_mission_event()
        return FSM_CANCELLED

    def cancel_state(self) -> None:
        self._core._request_cancel_from_fsm("yasmin cancel")
        super().cancel_state()


class MissionWaitingForBaseState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__(
            [FSM_BASE_EXTENDED, FSM_CANCELLED, *MODE_OUTCOMES, ROBOT_SHUTDOWN]
        )
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_mission_substate(WAITING_FOR_BASE_EXTENSION)
        self._core._on_waiting_for_base_entered()
        while rclpy.ok():
            mode_outcome = self._core._mode_transition_outcome(self._mode)
            if mode_outcome:
                return mode_outcome
            outcome = self._core._waiting_for_base_outcome()
            if outcome:
                return outcome
            self._core._wait_for_mission_event()
        return FSM_CANCELLED

    def cancel_state(self) -> None:
        self._core._request_cancel_from_fsm("yasmin cancel")
        super().cancel_state()


class MissionFailedRecoverableState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__([FSM_CANCELLED, *MODE_OUTCOMES, ROBOT_SHUTDOWN])
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_mission_substate(FAILED_RECOVERABLE)
        self._core._call_json(self._core._motion_client, {"command": "stop"})
        while rclpy.ok():
            mode_outcome = self._core._mode_transition_outcome(self._mode)
            if mode_outcome:
                return mode_outcome
            if self._core._cancel_requested_snapshot():
                return FSM_CANCELLED
            self._core._wait_for_mission_event()
        return FSM_CANCELLED

    def cancel_state(self) -> None:
        self._core._request_cancel_from_fsm("yasmin cancel")
        super().cancel_state()


class MissionCancellingState(State):
    def __init__(self, core: "RobotControlCore", mode: str) -> None:
        super().__init__([FSM_CANCELLED, ROBOT_SHUTDOWN])
        self._core = core
        self._mode = mode

    def execute(self, blackboard: Blackboard) -> str:
        del blackboard
        self._core._enter_robot_mode(self._mode)
        self._core._enter_mission_substate(CANCELLING)
        self._core._finish_cancelled_mission()
        return FSM_CANCELLED


class RobotControlCore(Node):
    def __init__(self) -> None:
        super().__init__("robot_control_core")

        self.declare_parameter("control_service", "/robot_control_core/control")
        self.declare_parameter("status_topic", "/robot_control_core/status")
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("status_publish_period_sec", 1.0)
        self.declare_parameter("enable_yasmin_viewer", True)
        self.declare_parameter("yasmin_viewer_publish_rate_hz", 4)
        self.declare_parameter(
            "motion_control_service", "/cmd_vel_arcestrator/control"
        )
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
        self._tick_period = 1.0 / tick_rate_hz
        self._status_period = max(
            0.0, float(self.get_parameter("status_publish_period_sec").value)
        )
        self._enable_yasmin_viewer = bool(
            self.get_parameter("enable_yasmin_viewer").value
        )
        self._yasmin_viewer_rate = max(
            1, int(self.get_parameter("yasmin_viewer_publish_rate_hz").value)
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
        self._operating_mode = STARTUP
        self._desired_operating_mode = STARTUP
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
        self._mission_lock = threading.RLock()
        self._mission_event = threading.Event()
        self._robot_fsm: Optional[StateMachine] = None
        self._robot_blackboard: Optional[Blackboard] = None
        self._robot_thread: Optional[threading.Thread] = None
        self._mode_fsms: Dict[str, StateMachine] = {}
        self._mission_fsm: Optional[StateMachine] = None
        self._yasmin_viewer_pubs: Dict[str, Any] = {}
        self._cancel_requested = False
        self._shutdown_requested = False
        self._yasmin_viewer_missing_logged = False

        self.get_logger().info(
            "robot_control_core ready: control=%s status=%s tick=%.1fHz"
            % (control_service, status_topic, tick_rate_hz)
        )
        self._start_robot_fsm()

    def destroy_node(self) -> bool:
        with self._mission_lock:
            self._shutdown_requested = True
            self._destroy_yasmin_viewer_pub_locked()
            thread = self._robot_thread

        self._mission_event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, 2.0 * self._tick_period))

        return super().destroy_node()

    def _handle_control(self, request, response):
        try:
            payload = json.loads(request.message or "{}")
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = json.dumps(
                {"error": "invalid_json", "details": str(exc)}
            )
            return response

        command = str(payload.get("command", "")).strip().lower()
        try:
            if command in STATUS_COMMANDS:
                response.success = True
            elif command in START_COMMANDS:
                self._start_mission(payload)
                response.success = True
            elif command in APPEND_BASE_COMMANDS:
                self._append_base(payload)
                response.success = True
            elif command in PAUSE_COMMANDS:
                self._pause("control pause")
                self._record_instant_action("startPause", ACTION_FINISHED)
                response.success = True
            elif command in RESUME_COMMANDS:
                self._resume("control resume")
                self._record_instant_action("stopPause", ACTION_FINISHED)
                response.success = True
            elif command in CANCEL_COMMANDS:
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
                with self._mission_lock:
                    self._errors.clear()
                response.success = True
            else:
                response.success = False
                response.message = json.dumps(
                    {
                        "error": "invalid_command",
                        "details": (
                            "expected start_mission|append_base|startPause|"
                            "stopPause|cancelOrder|set_mode|reset|"
                            "stateRequest/status"
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
            with self._mission_lock:
                self._desired_operating_mode = AUTOMATIC
            self._mission_event.set()

        self._publish_status()

    def _build_robot_fsm(self) -> StateMachine:
        sm = StateMachine(outcomes=[ROBOT_SHUTDOWN], handle_sigint=False)
        sm.set_name("robot_control_core")
        transitions = self._robot_mode_transitions()
        for mode in OPERATING_MODE_ORDER:
            if mode in ORDER_MODES:
                state = self._build_mode_mission_fsm(mode)
            else:
                state = RobotModeState(self, mode)
            sm.add_state(mode, state, {**transitions, ROBOT_SHUTDOWN: ROBOT_SHUTDOWN})
        sm.set_start_state(STARTUP)
        sm.add_transition_cb(self._on_robot_transition)
        sm.add_end_cb(self._on_robot_end)
        return sm

    def _build_mode_mission_fsm(self, mode: str) -> StateMachine:
        sm = StateMachine(outcomes=[*MODE_OUTCOMES, ROBOT_SHUTDOWN], handle_sigint=False)
        sm.set_name("%s_mode" % mode.lower())
        mode_transitions = self._terminal_mode_transitions()
        sm.add_state(
            DISPATCH,
            MissionDispatchState(self, mode),
            {
                IDLE: IDLE,
                EXECUTING_BASE: EXECUTING_BASE,
                WAITING_FOR_BASE_EXTENSION: WAITING_FOR_BASE_EXTENSION,
                PAUSED: PAUSED,
                FAILED_RECOVERABLE: FAILED_RECOVERABLE,
                CANCELLING: CANCELLING,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            IDLE,
            MissionIdleState(self, mode),
            {
                MISSION_STARTED: EXECUTING_BASE,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            EXECUTING_BASE,
            MissionExecutingState(self, mode),
            {
                FSM_PAUSED: PAUSED,
                FSM_NEED_BASE: WAITING_FOR_BASE_EXTENSION,
                FSM_FAILED: FAILED_RECOVERABLE,
                FSM_FINISHED: IDLE,
                FSM_CANCELLED: CANCELLING,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            PAUSED,
            MissionPausedState(self, mode),
            {
                FSM_RESUMED: EXECUTING_BASE,
                FSM_CANCELLED: CANCELLING,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            WAITING_FOR_BASE_EXTENSION,
            MissionWaitingForBaseState(self, mode),
            {
                FSM_BASE_EXTENDED: EXECUTING_BASE,
                FSM_CANCELLED: CANCELLING,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            FAILED_RECOVERABLE,
            MissionFailedRecoverableState(self, mode),
            {
                FSM_CANCELLED: CANCELLING,
                **mode_transitions,
                ROBOT_SHUTDOWN: ROBOT_SHUTDOWN,
            },
        )
        sm.add_state(
            CANCELLING,
            MissionCancellingState(self, mode),
            {FSM_CANCELLED: IDLE, ROBOT_SHUTDOWN: ROBOT_SHUTDOWN},
        )
        sm.set_start_state(DISPATCH)
        sm.add_transition_cb(self._on_yasmin_transition)
        sm.add_end_cb(self._on_yasmin_end)
        self._mode_fsms[mode] = sm
        return sm

    def _robot_mode_transitions(self) -> Dict[str, str]:
        return {"to_%s" % mode: mode for mode in OPERATING_MODE_ORDER}

    def _terminal_mode_transitions(self) -> Dict[str, str]:
        return {"to_%s" % mode: "to_%s" % mode for mode in OPERATING_MODE_ORDER}

    def _start_robot_fsm(self) -> None:
        with self._mission_lock:
            self._robot_fsm = self._build_robot_fsm()
            self._robot_fsm.validate()
            self._robot_blackboard = Blackboard()
            self._register_yasmin_viewers_locked()
            thread = threading.Thread(
                target=self._run_robot_fsm,
                name="robot_control_core_yasmin_robot_fsm",
                daemon=True,
            )
            self._robot_thread = thread
        thread.start()

    def _start_mission(self, payload: Dict[str, Any]) -> None:
        base_steps = payload.get("base_steps", payload.get("steps", []))
        if not isinstance(base_steps, list):
            raise ValueError("steps/base_steps must be an array")
        horizon_steps = payload.get("horizon_steps", [])
        if not isinstance(horizon_steps, list):
            raise ValueError("horizon_steps must be an array")

        with self._mission_lock:
            if self._mission_active_locked():
                raise ValueError("mission is already active")
            if self._operating_mode not in {
                "AUTOMATIC",
                "SEMIAUTOMATIC",
                "INTERVENED",
            }:
                raise ValueError("operating mode does not allow missions")

            self._mission_id = str(
                payload.get("mission_id", payload.get("order_id", ""))
            )
            self._order_id = str(payload.get("order_id", ""))
            self._order_update_id = int(payload.get("order_update_id", 0))
            self._node_states = list(
                payload.get("nodeStates", payload.get("node_states", []))
            )
            self._edge_states = list(
                payload.get("edgeStates", payload.get("edge_states", []))
            )
            self._errors.clear()
            self._queue = self._expand_steps(base_steps, prefix="base")
            self._horizon = self._expand_steps(horizon_steps, prefix="horizon")
            self._actions = [*self._queue, *self._horizon]
            self._current_action = None
            self._paused = False
            self._cancel_requested = False
            self._new_base_request = False
            self._state = ORDER_ACTIVE
            self._substate = EXECUTING_BASE
            self._mission_event.clear()

        self._mission_event.set()
        self.get_logger().info(
            "mission started: mission_id=%s base_actions=%d horizon_actions=%d"
            % (self._mission_id or "<empty>", len(self._queue), len(self._horizon))
        )

    def _append_base(self, payload: Dict[str, Any]) -> None:
        with self._mission_lock:
            if not self._mission_active_locked() or self._cancel_requested:
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

            self._order_update_id = int(
                payload.get("order_update_id", self._order_update_id)
            )
            self._new_base_request = False

        self._mission_event.set()

    def _pause(self, reason: str) -> None:
        should_stop = False
        with self._mission_lock:
            if not self._mission_active_locked() or self._cancel_requested:
                return
            self._paused = True
            should_stop = True
            if self._current_action and self._current_action.status == ACTION_RUNNING:
                self._current_action.status = ACTION_PAUSED

        self._mission_event.set()
        if should_stop:
            self._call_json(self._motion_client, {"command": "stop"})
            self.get_logger().info("mission paused: %s" % reason)

    def _resume(self, reason: str) -> None:
        with self._mission_lock:
            if not self._mission_active_locked() or self._cancel_requested:
                return
            if self._substate == FAILED_RECOVERABLE:
                raise ValueError("mission is failed recoverable; cancel or reset it first")
            self._paused = False
            if self._current_action and self._current_action.status == ACTION_PAUSED:
                self._current_action.status = ACTION_RUNNING

        self._mission_event.set()
        self.get_logger().info("mission resumed: %s" % reason)

    def _cancel(self, reason: str) -> None:
        with self._mission_lock:
            if not self._mission_active_locked():
                self._call_json(self._motion_client, {"command": "stop"})
                return
            self._cancel_requested = True
            self._substate = CANCELLING

        self._mission_event.set()
        self._call_json(self._motion_client, {"command": "stop"})
        self.get_logger().warn("mission cancel requested: %s" % reason)

    def _reset(self) -> None:
        with self._mission_lock:
            if self._mission_active_locked():
                self._cancel_requested = True
                self._substate = CANCELLING

        self._mission_event.set()

        with self._mission_lock:
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
            self._cancel_requested = False
            self._new_base_request = False
            self._state = self._idle_state_for_mode_locked()
            self._substate = ""
        self._call_json(self._motion_client, {"command": "stop"})

    def _set_operating_mode(self, payload: Dict[str, Any]) -> None:
        mode = str(payload.get("mode", payload.get("operating_mode", ""))).upper()
        if mode not in VALID_OPERATING_MODES:
            raise ValueError("invalid operating mode: %s" % mode)

        with self._mission_lock:
            self._desired_operating_mode = mode

        if mode in NON_ORDER_MODES or mode == "STARTUP":
            self._cancel("mode changed to %s" % mode)
        elif mode == "INTERVENED":
            self._pause("mode changed to INTERVENED")

        self._mission_event.set()
        self.get_logger().info("operating mode set: %s" % mode)

    def _run_robot_fsm(self) -> None:
        with self._mission_lock:
            fsm = self._robot_fsm
            blackboard = self._robot_blackboard or Blackboard()

        try:
            if fsm is not None:
                fsm(blackboard)
        except Exception as exc:  # pragma: no cover - defensive ROS runtime guard
            self.get_logger().error("robot FSM failed: %s" % exc)
            with self._mission_lock:
                self._errors.append(
                    {
                        "errorType": "FSM_FAILED",
                        "errorLevel": "FATAL",
                        "errorDescription": str(exc),
                    }
                )
                self._state = self._idle_state_for_mode_locked()
                self._substate = ""
                self._paused = False
                self._cancel_requested = False
            self._call_json(self._motion_client, {"command": "stop"})
            return

        with self._mission_lock:
            self._robot_thread = None

    def _enter_robot_mode(self, mode: str) -> None:
        with self._mission_lock:
            self._operating_mode = mode
            if mode in ORDER_MODES:
                self._mission_fsm = self._mode_fsms.get(mode)
                if not self._mission_active_locked():
                    self._state = IDLE
                    self._substate = ""
            else:
                self._mission_fsm = None
                if not self._mission_active_locked():
                    self._state = mode
                    self._substate = ""

    def _enter_mission_idle(self, mode: str) -> None:
        with self._mission_lock:
            self._operating_mode = mode
            self._mission_fsm = self._mode_fsms.get(mode)
            if not self._mission_active_locked():
                self._state = IDLE
                self._substate = ""
                self._paused = False
                self._cancel_requested = False
                self._new_base_request = False

    def _mode_transition_outcome(self, current_mode: str) -> str:
        with self._mission_lock:
            if self._shutdown_requested:
                return ROBOT_SHUTDOWN
            desired = self._desired_operating_mode
            if (
                desired != current_mode
                and (desired in NON_ORDER_MODES or desired == STARTUP)
                and self._mission_active_locked()
            ):
                self._cancel_requested = True
                self._substate = CANCELLING
                self._call_json(self._motion_client, {"command": "stop"})
                return ""
        if desired != current_mode:
            return "to_%s" % desired
        return ""

    def _mission_dispatch_outcome(self, mode: str) -> str:
        mode_outcome = self._mode_transition_outcome(mode)
        if mode_outcome:
            return mode_outcome
        if not rclpy.ok():
            return ROBOT_SHUTDOWN
        with self._mission_lock:
            if not self._mission_active_locked():
                return IDLE
            if self._cancel_requested:
                return CANCELLING
            if mode == INTERVENED:
                self._paused = True
                if self._current_action and self._current_action.status == ACTION_RUNNING:
                    self._current_action.status = ACTION_PAUSED
                return PAUSED
            if self._paused:
                return PAUSED
            if self._substate == WAITING_FOR_BASE_EXTENSION:
                return WAITING_FOR_BASE_EXTENSION
            if self._substate == FAILED_RECOVERABLE:
                return FAILED_RECOVERABLE
            if self._substate == CANCELLING:
                return CANCELLING
            return EXECUTING_BASE

    def _mission_active_snapshot(self) -> bool:
        with self._mission_lock:
            return self._mission_active_locked()

    def _on_robot_transition(
        self, blackboard: Blackboard, source: str, target: str, outcome: str
    ) -> None:
        del blackboard, outcome
        self.get_logger().info("robot FSM transition: %s -> %s" % (source, target))

    def _on_robot_end(self, blackboard: Blackboard, outcome: str) -> None:
        del blackboard
        self.get_logger().info("robot FSM ended: outcome=%s" % outcome)

    def _start_yasmin_viewer_pub_locked(
        self, fsm: StateMachine, fsm_name: str
    ) -> Optional[Any]:
        if not self._enable_yasmin_viewer:
            return None
        if YasminViewerPub is None:
            if not self._yasmin_viewer_missing_logged:
                self.get_logger().warn(
                    "YASMIN viewer publisher is disabled: install "
                    "ros-$ROS_DISTRO-yasmin-viewer"
                )
                self._yasmin_viewer_missing_logged = True
            return None
        try:
            return YasminViewerPub(
                fsm,
                fsm_name=fsm_name,
                rate=self._yasmin_viewer_rate,
                node=self,
            )
        except Exception as exc:  # pragma: no cover - defensive ROS runtime guard
            self.get_logger().warn("failed to start YASMIN viewer publisher: %s" % exc)
            return None

    def _register_yasmin_viewers_locked(self) -> None:
        self._destroy_yasmin_viewer_pub_locked()
        if self._robot_fsm is not None:
            self._register_yasmin_viewer_locked(
                "robot_control_core", self._robot_fsm
            )
        for mode in ORDER_MODES:
            fsm = self._mode_fsms.get(mode)
            if fsm is not None:
                self._register_yasmin_viewer_locked(
                    "%s_mode" % mode.lower(), fsm
                )

    def _register_yasmin_viewer_locked(
        self, fsm_name: str, fsm: StateMachine
    ) -> None:
        viewer_pub = self._start_yasmin_viewer_pub_locked(fsm, fsm_name)
        if viewer_pub is not None:
            self._yasmin_viewer_pubs[fsm_name] = viewer_pub

    def _destroy_yasmin_viewer_pub_locked(self) -> None:
        viewer_pubs = list(self._yasmin_viewer_pubs.values())
        self._yasmin_viewer_pubs.clear()
        for viewer_pub in viewer_pubs:
            timer = getattr(viewer_pub, "_timer", None)
            if timer is None:
                continue
            try:
                self.destroy_timer(timer)
            except Exception as exc:  # pragma: no cover - defensive ROS runtime guard
                self.get_logger().debug(
                    "failed to destroy YASMIN viewer timer: %s" % exc
                )

    def _on_yasmin_transition(
        self, blackboard: Blackboard, source: str, target: str, outcome: str
    ) -> None:
        del blackboard, source, outcome
        if target in {
            EXECUTING_BASE,
            PAUSED,
            WAITING_FOR_BASE_EXTENSION,
            FAILED_RECOVERABLE,
            CANCELLING,
        }:
            self._enter_mission_substate(target)

    def _on_yasmin_end(self, blackboard: Blackboard, outcome: str) -> None:
        del blackboard
        self.get_logger().info("mission FSM ended: outcome=%s" % outcome)

    def _mission_active_locked(self) -> bool:
        if self._state == ORDER_ACTIVE:
            return True
        if self._current_action is not None:
            return True
        if self._queue or self._horizon:
            return True
        return any(action.status not in END_ACTION_STATES for action in self._actions)

    def _idle_state_for_mode_locked(self) -> str:
        if self._operating_mode in NON_ORDER_MODES or self._operating_mode == STARTUP:
            return self._operating_mode
        return IDLE

    def _enter_mission_substate(self, substate: str) -> None:
        with self._mission_lock:
            if self._state not in NON_ORDER_MODES and self._state != "STARTUP":
                self._state = ORDER_ACTIVE
            self._substate = substate

    def _request_cancel_from_fsm(self, reason: str) -> None:
        with self._mission_lock:
            self._cancel_requested = True
            self._substate = CANCELLING
        self._mission_event.set()
        self.get_logger().warn("mission cancel requested: %s" % reason)

    def _wait_for_mission_event(self) -> None:
        self._mission_event.wait(timeout=self._tick_period)
        self._mission_event.clear()

    def _execute_base_once(self) -> str:
        with self._mission_lock:
            if self._cancel_requested:
                return FSM_CANCELLED
            if self._paused:
                return FSM_PAUSED

            if self._current_action is None and self._queue:
                self._current_action = self._queue.pop(0)
                self._start_action(self._current_action)

            if self._current_action is not None:
                self._poll_action(self._current_action)
                if self._current_action.status in END_ACTION_STATES:
                    finished = self._current_action
                    self._current_action = None
                    if finished.status == ACTION_FAILED:
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
                        return FSM_FAILED

            if self._current_action is None and not self._queue:
                if self._horizon:
                    self._new_base_request = True
                    self._call_json(self._motion_client, {"command": "stop"})
                    return FSM_NEED_BASE
                self._finish_successful_mission_locked()
                return FSM_FINISHED

        return FSM_RUNNING

    def _on_paused_state_entered(self) -> None:
        with self._mission_lock:
            self._paused = True
            if self._current_action and self._current_action.status == ACTION_RUNNING:
                self._current_action.status = ACTION_PAUSED
        self._call_json(self._motion_client, {"command": "stop"})

    def _paused_state_outcome(self) -> str:
        with self._mission_lock:
            if self._cancel_requested:
                return FSM_CANCELLED
            if not self._paused:
                return FSM_RESUMED
        return ""

    def _on_waiting_for_base_entered(self) -> None:
        with self._mission_lock:
            self._new_base_request = True
        self._call_json(self._motion_client, {"command": "stop"})

    def _waiting_for_base_outcome(self) -> str:
        with self._mission_lock:
            if self._cancel_requested:
                return FSM_CANCELLED
            if self._queue:
                self._new_base_request = False
                return FSM_BASE_EXTENDED
        return ""

    def _cancel_requested_snapshot(self) -> bool:
        with self._mission_lock:
            return self._cancel_requested

    def _finish_cancelled_mission(self) -> None:
        with self._mission_lock:
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
            self._cancel_requested = False
            self._state = self._idle_state_for_mode_locked()
            self._substate = ""
        self._call_json(self._motion_client, {"command": "stop"})

    def _finish_successful_mission_locked(self) -> None:
        self._state = self._idle_state_for_mode_locked()
        self._substate = ""
        self._node_states.clear()
        self._edge_states.clear()
        self._new_base_request = False
        self._paused = False
        self._cancel_requested = False
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

        if action_type == "finePositioning":
            self._start_docking(action)
        elif action_type == "pick":
            self._start_pick(action)
        elif action_type == "drop":
            self._start_drop(action)
        else:
            action.status = ACTION_FAILED
            action.result = "unsupported action type: %s" % action_type

    def _poll_action(self, action: MicroAction) -> None:
        if action.status in END_ACTION_STATES:
            return
        action_type = action.action_type
        if action_type == "finePositioning":
            self._poll_docking(action)
        elif action_type == "pick":
            self._poll_pick(action)
        elif action_type == "drop":
            self._poll_drop(action)

    def _start_docking(self, action: MicroAction) -> None:
        payload = {"command": "start"}
        tag_frame = self._station_reference(action.parameters)
        if tag_frame:
            payload["tag_frame"] = tag_frame
        for key in (
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

    def _start_pick(self, action: MicroAction) -> None:
        action.runner["phase"] = "lower"
        self._start_compound_fork(
            action, self._load_lower_position(action.parameters)
        )
        action.status = ACTION_RUNNING

    def _poll_pick(self, action: MicroAction) -> None:
        phase = action.runner.get("phase")
        child = action.runner.get("child")
        if not isinstance(child, MicroAction):
            action.status = ACTION_FAILED
            action.result = "pick has no active internal step"
            return

        if phase in {"lower", "lift"}:
            self._poll_fork_position(child)
        elif phase == "finePositioning":
            self._poll_docking(child)
        else:
            action.status = ACTION_FAILED
            action.result = "unknown pick phase: %s" % phase
            return

        if child.status == ACTION_FAILED:
            action.status = ACTION_FAILED
            action.result = "pick %s failed: %s" % (phase, child.result)
            return
        if child.status != ACTION_FINISHED:
            return

        if phase == "lower":
            if bool(action.parameters.get("dock", True)):
                action.runner["phase"] = "finePositioning"
                self._start_compound_fine_positioning(action)
            else:
                action.runner["phase"] = "lift"
                self._start_compound_fork(
                    action, self._load_lift_position(action.parameters)
                )
        elif phase == "finePositioning":
            action.runner["phase"] = "lift"
            self._start_compound_fork(action, self._load_lift_position(action.parameters))
        elif phase == "lift":
            self._set_load_state(action.parameters, loaded=True)
            action.status = ACTION_FINISHED
            action.result = "pick done"

    def _start_drop(self, action: MicroAction) -> None:
        if bool(action.parameters.get("dock", False)) or self._station_reference(
            action.parameters
        ):
            action.runner["phase"] = "finePositioning"
            self._start_compound_fine_positioning(action)
        else:
            action.runner["phase"] = "lower"
            self._start_compound_fork(action, self._load_lower_position(action.parameters))
        action.status = ACTION_RUNNING

    def _poll_drop(self, action: MicroAction) -> None:
        phase = action.runner.get("phase")
        child = action.runner.get("child")
        if not isinstance(child, MicroAction):
            action.status = ACTION_FAILED
            action.result = "drop has no active internal step"
            return

        if phase == "finePositioning":
            self._poll_docking(child)
        elif phase == "lower":
            self._poll_fork_position(child)
        else:
            action.status = ACTION_FAILED
            action.result = "unknown drop phase: %s" % phase
            return

        if child.status == ACTION_FAILED:
            action.status = ACTION_FAILED
            action.result = "drop %s failed: %s" % (phase, child.result)
            return
        if child.status != ACTION_FINISHED:
            return

        if phase == "finePositioning":
            action.runner["phase"] = "lower"
            self._start_compound_fork(action, self._load_lower_position(action.parameters))
        elif phase == "lower":
            self._set_load_state(action.parameters, loaded=False)
            action.status = ACTION_FINISHED
            action.result = "drop done"

    def _start_compound_fork(self, action: MicroAction, target: float) -> None:
        child = self._make_internal_action(
            action, "fork_position", {"position": float(target)}
        )
        action.runner["child"] = child
        self._start_fork_position(child)

    def _start_compound_fine_positioning(self, action: MicroAction) -> None:
        child = self._make_internal_action(
            action, "finePositioning", dict(action.parameters)
        )
        action.runner["child"] = child
        self._start_docking(child)

    def _make_internal_action(
        self, parent: MicroAction, action_type: str, parameters: Dict[str, Any]
    ) -> MicroAction:
        child = MicroAction(
            action_id="%s:%s" % (parent.action_id, action_type),
            action_type=action_type,
            parameters=parameters,
            blocking_type=parent.blocking_type,
        )
        child.runner = {
            "started_at": self.get_clock().now(),
            "last_poll_at": None,
            "phase": "start",
        }
        return child

    def _station_reference(self, params: Dict[str, Any]) -> str:
        for key in ("stationName", "station_name", "tag_frame"):
            value = params.get(key)
            if value not in (None, ""):
                return str(value)
        return ""

    def _load_id(self, params: Dict[str, Any]) -> str:
        return str(params.get("loadId", params.get("load_id", "load")))

    def _load_lower_position(self, params: Dict[str, Any]) -> float:
        return float(
            params.get("height", params.get("lower_position", self._fork_lower_position))
        )

    def _load_lift_position(self, params: Dict[str, Any]) -> float:
        return float(params.get("lift_position", self._fork_lift_position))

    def _set_load_state(self, params: Dict[str, Any], *, loaded: bool) -> None:
        if loaded:
            self._loads = [{"loadId": self._load_id(params)}]
        else:
            self._loads = []

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

    def _cancel_current_action(self) -> None:
        if self._current_action is None:
            return
        action = self._current_action
        if action.action_type in {"finePositioning", "pick", "drop"}:
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
            with self._mission_lock:
                self._fork_position = float(msg.position[index])

    def _expand_steps(self, steps: List[Dict[str, Any]], *, prefix: str) -> List[MicroAction]:
        actions: List[MicroAction] = []
        for step in steps:
            if not isinstance(step, dict):
                raise ValueError("mission steps must be JSON objects")
            actions.append(self._make_action(step, prefix=prefix))
        return actions

    def _make_action(self, step: Dict[str, Any], *, prefix: str) -> MicroAction:
        action_type = str(step.get("actionType", ""))
        if not action_type:
            raise ValueError("mission step has no actionType")
        if action_type not in SUPPORTED_ORDER_ACTION_TYPES:
            raise ValueError("unsupported VDA actionType: %s" % action_type)
        action_id_value = step.get("actionId")
        if action_id_value in (None, ""):
            action_id_value = self._next_action_id(prefix, action_type)
        action_id = str(action_id_value)
        blocking_type = str(step.get("blockingType", "HARD")).upper()
        if blocking_type not in {"NONE", "SINGLE", "SOFT", "HARD"}:
            raise ValueError("invalid blockingType for %s: %s" % (action_id, blocking_type))
        params = self._action_parameters(step)
        return MicroAction(
            action_id=action_id,
            action_type=action_type,
            parameters=params,
            blocking_type=blocking_type,
            pause_allowed=bool(step.get("pauseAllowed", True)),
            cancel_allowed=bool(step.get("cancelAllowed", True)),
        )

    def _action_parameters(self, step: Dict[str, Any]) -> Dict[str, Any]:
        params = {
            key: value
            for key, value in step.items()
            if key
            not in {
                "type",
                "actionType",
                "action_id",
                "actionId",
                "actionDescriptor",
                "blockingType",
                "blocking_type",
                "pauseAllowed",
                "cancelAllowed",
                "retriable",
                "actionParameters",
            }
        }
        action_parameters = step.get("actionParameters", [])
        if action_parameters:
            if not isinstance(action_parameters, list):
                raise ValueError("actionParameters must be an array")
            for item in action_parameters:
                if not isinstance(item, dict):
                    raise ValueError("actionParameters items must be JSON objects")
                key = item.get("key")
                if key in (None, ""):
                    raise ValueError("actionParameter has no key")
                params[str(key)] = item.get("value")
        return params

    def _next_action_id(self, prefix: str, action_type: str) -> str:
        self._action_counter += 1
        return "%s_%03d_%s" % (prefix, self._action_counter, action_type)

    def _record_instant_action(self, action_type: str, status: str) -> None:
        with self._mission_lock:
            self._instant_action_states.append(
                {
                    "actionId": "%s_%d"
                    % (action_type, len(self._instant_action_states) + 1),
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
        with self._mission_lock:
            fsm_state = ""
            if self._mission_fsm is not None:
                fsm_state = self._mission_fsm.get_current_state()
            substate = fsm_state or self._substate
            return {
                "state": self._state,
                "substate": substate,
                "operatingMode": self._operating_mode,
                "missionId": self._mission_id,
                "orderId": self._order_id,
                "orderUpdateId": self._order_update_id,
                "paused": self._paused,
                "newBaseRequest": self._new_base_request,
                "currentActionId": (
                    self._current_action.action_id if self._current_action else ""
                ),
                "queuedActionIds": [action.action_id for action in self._queue],
                "horizonActionIds": [action.action_id for action in self._horizon],
                "nodeStates": list(self._node_states),
                "edgeStates": list(self._edge_states),
                "actionStates": [self._action_state(action) for action in self._actions],
                "instantActionStates": list(self._instant_action_states),
                "loads": list(self._loads),
                "errors": list(self._errors),
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
