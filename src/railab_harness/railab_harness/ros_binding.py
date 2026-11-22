"""The bus seam of bus.py on ROS 2 (rclpy), for BOTH sides, against the package harness_module_msgs.

  /<instance>/command   service  harness_module_msgs/srv/Command      harness -> module
  /<instance>/state     topic    harness_module_msgs/msg/ModuleState  module -> harness
  /harness/heartbeat    topic    harness_module_msgs/msg/Heartbeat    harness -> every module node

A module instance is its namespace. The harness finds modules by looking in the ROS graph for .../state topics of
the ModuleState type: no name is listed anywhere, a second Bunny is just /bunny_2.

UNTESTED ON ROS: written on a machine without ROS (2026-11-20). The mapping functions at the end are plain Python and
were checked with stand-in message objects (tools/module_check_arm.py); everything that touches rclpy was not run.

rclpy and harness_module_msgs are imported lazily (inside RosBus), so the core and the checks import this file
without ROS. THE MAPPING between the contract's JSON dicts and the typed messages lives in ONE place: the functions
below the class. Two names differ: the JSON "do" is the ROS field `action` (a C++ keyword), the JSON "module" is
the ROS field `instance` (an OMG IDL keyword, and rosidl turns every .msg into IDL).
"""

import json
import math
import os
import threading

from . import messages as M
from .bus import Bus, Unreachable

PACKAGE = "harness_module_msgs"
STATE_TYPE = PACKAGE + "/msg/ModuleState"   # as the ROS graph spells a topic's type
COMMAND_SUFFIX, STATE_SUFFIX = "/command", "/state"
BEAT_TOPIC = "/harness/heartbeat"
DISCOVER_S = 1.0       # how often the harness looks for new .../state topics
QOS_DEPTH = 10
EXECUTOR_THREADS = 4
INTERNAL = "INTERNAL"  # a module's door raised (a bug in the adapter): refused, never a silent accept


def command_name(module: str) -> str:
    return f"/{module}{COMMAND_SUFFIX}"


def state_topic(module: str) -> str:
    return f"/{module}{STATE_SUFFIX}"


def module_of(topic: str):
    """The instance a .../state topic belongs to ("/bunny_1/state" -> "bunny_1"), else None."""
    if topic.startswith("/") and topic.endswith(STATE_SUFFIX) and len(topic) > len(STATE_SUFFIX) + 1:
        return topic[1:-len(STATE_SUFFIX)]
    return None


def ros_environment() -> None:
    """Fast DDS over UDP only, set before rclpy creates a participant: the old gateway learned on 2026-11-19 that its
    shared-memory port can be orphaned after a restart (tools/gateway_server.py, the comment above its PORT). An
    explicit FASTDDS_BUILTIN_TRANSPORTS wins."""
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")


class RosBus(Bus):
    """The three channels on ROS 2.

    node=None: the bus makes its own node `node_name` and spins it on its own executor thread (the gateway's way).
    node given: the caller owns it and spins it (a module process that shares one node with its device link).
    """

    def __init__(self, node=None, node_name: str = "railab_harness", threads: int = EXECUTOR_THREADS):
        ros_environment()
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from harness_module_msgs.msg import Heartbeat, ModuleState
        from harness_module_msgs.srv import Command

        self.Command, self.ModuleState, self.Heartbeat = Command, ModuleState, Heartbeat
        if not rclpy.ok():
            rclpy.init()
        self.node = node if node is not None else rclpy.create_node(node_name)
        self._group = ReentrantCallbackGroup()   # a slow listener never holds up a command
        self._lock = threading.Lock()
        self._served = {}       # module -> (state publisher, command service, beat subscription)
        self._clients = {}      # module -> command client
        self._watched = {}      # .../state topic -> its subscription
        self._listeners = []
        self._discover = None   # the discovery timer, made by the first on_state
        self._beat_pub = None
        self._seq = 0
        self._executor = None
        if node is None:
            from rclpy.executors import MultiThreadedExecutor
            self._executor = MultiThreadedExecutor(num_threads=threads)
            self._executor.add_node(self.node)
            threading.Thread(target=self._executor.spin, name="ros-bus-spin", daemon=True).start()

    # ------------------------------------------------------------ the module's side
    def serve(self, module, on_command, on_beat):
        def answer(request, response):
            try:
                asked = request_from_msg(request)
            except (ValueError, RecursionError) as error:   # RecursionError: JSON nested too deep
                return reply_to_msg(response, M.refuse(request.id, M.BAD_ARGS, str(error) or "args nested too deep"))
            try:
                reply = on_command(asked)
            except Exception as error:   # noqa: BLE001  an exception here would end the executor's thread
                reply = M.refuse(asked.get("id"), INTERNAL, f"{type(error).__name__}: {error}")
            return reply_to_msg(response, reply)

        with self._lock:
            if module in self._served:
                raise ValueError(f"{module} is already served by this bus")
            publisher = self.node.create_publisher(self.ModuleState, state_topic(module), QOS_DEPTH)
            service = self.node.create_service(self.Command, command_name(module), answer,
                                               callback_group=self._group)
            beat = self.node.create_subscription(self.Heartbeat, BEAT_TOPIC, lambda _beat: on_beat(), QOS_DEPTH,
                                                 callback_group=self._group)
            self._served[module] = (publisher, service, beat)

    def leave(self, module):
        with self._lock:
            served = self._served.pop(module, None)
        if served is None:
            return
        publisher, service, beat = served
        self.node.destroy_service(service)
        self.node.destroy_subscription(beat)
        self.node.destroy_publisher(publisher)

    def publish_state(self, state):
        with self._lock:
            served = self._served.get(state["module"])
        if served is not None:   # a late state of a module that has left goes nowhere
            served[0].publish(state_to_msg(self.ModuleState(), state))

    # ------------------------------------------------------------ the harness's side
    def call(self, module, request, timeout_s):
        client = self._client(module)
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=timeout_s):
            raise Unreachable(f"{module} offers no command service")
        answered = threading.Event()
        future = client.call_async(request_to_msg(self.Command.Request(), request))
        future.add_done_callback(lambda _future: answered.set())
        if not (answered.wait(timeout_s) or future.done()):
            forget = getattr(client, "remove_pending_request", None)   # rclpy >= Humble
            if forget is not None:
                forget(future)
            raise Unreachable(f"{module} did not answer within {timeout_s} s")
        response = future.result()
        if response is None:
            raise Unreachable(f"{module}: the call ended without a reply")
        return reply_from_msg(request.get("id"), response)

    def on_state(self, callback):
        with self._lock:
            self._listeners.append(callback)
            first = self._discover is None
            if first:
                self._discover = self.node.create_timer(DISCOVER_S, self._find_modules, callback_group=self._group)
        if first:
            self._find_modules()

        def stop():
            with self._lock:
                if callback in self._listeners:
                    self._listeners.remove(callback)
        return stop

    def beat(self):
        with self._lock:
            if self._beat_pub is None:
                self._beat_pub = self.node.create_publisher(self.Heartbeat, BEAT_TOPIC, QOS_DEPTH)
            self._seq += 1
            seq = self._seq
        beat = self.Heartbeat()
        beat.stamp = self.node.get_clock().now().to_msg()
        beat.seq = seq
        self._beat_pub.publish(beat)

    def _client(self, module):
        with self._lock:
            client = self._clients.get(module)
            if client is None:
                client = self._clients[module] = self.node.create_client(self.Command, command_name(module),
                                                                         callback_group=self._group)
        return client

    def _find_modules(self):
        """Subscribe to every .../state topic of the ModuleState type not yet heard; warm its command client.
        Never raises: it runs on the executor's thread (the discovery timer)."""
        try:
            self._find_modules_now()
        except Exception as error:   # noqa: BLE001  an exception here would end the executor's thread
            self.node.get_logger().warning(f"looking for modules: {type(error).__name__}: {error}")

    def _find_modules_now(self):
        for topic, types in self.node.get_topic_names_and_types():
            module = module_of(topic)
            if module is None or STATE_TYPE not in types:
                continue
            with self._lock:
                if topic in self._watched:
                    continue
                self._watched[topic] = self.node.create_subscription(self.ModuleState, topic, self._heard, QOS_DEPTH,
                                                                     callback_group=self._group)
            self._client(module)

    def _heard(self, message):
        """One .../state message, on the executor's thread: NEVER raises. WHY: an exception here ends the
        executor's spin thread — the gateway would hear nothing more while its tick kept beating, so every
        module would believe in a harness that no longer listens (nor answers a stop)."""
        try:
            state = state_from_msg(message)
        except Exception as error:   # noqa: BLE001  broken JSON (or nested too deep): dropped, its states stop
            self.node.get_logger().warning(   # counting as fresh
                f"a state on {state_topic(str(getattr(message, 'instance', '?')))} is not readable: "
                f"{type(error).__name__}: {error}")
            return
        with self._lock:
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(state)
            except Exception as error:   # noqa: BLE001  one listener's bug: said, the others still hear
                self.node.get_logger().warning(f"a state listener raised on {state_topic(str(state.get('module')))}: "
                                               f"{type(error).__name__}: {error}")

    # ------------------------------------------------------------ life
    def close(self):
        with self._lock:
            served = list(self._served)
        for module in served:
            self.leave(module)
        if self._executor is not None:
            self._executor.shutdown()
            self.node.destroy_node()


# ====================================================================================================================
# THE MAPPING: the contract's JSON dicts <-> the typed messages. The only place where a field name of either is spelled
# next to the other. JSON "do" = ROS `action`, JSON "module" = ROS `instance`; the module's own parts (args, set,
# describe, values, sources) travel as JSON text.
# ====================================================================================================================
def _json_object(text: str, what: str):
    if not text:
        return {}
    try:
        return json.loads(text)
    except ValueError as error:
        raise ValueError(f"{what} is not JSON: {error}") from None


def _dumps(value) -> str:
    return json.dumps(value, separators=(",", ":"))


def request_to_msg(message, request: dict):
    """A request dict -> Command.Request."""
    message.id = int(request["id"])
    if "do" in request:
        message.action = str(request["do"])
        message.args = _dumps(request.get("args") or {})
    if "set" in request:
        message.set = _dumps(request["set"])
    return message


def request_from_msg(message) -> dict:
    """Command.Request -> a request dict. Both or neither of action / set are passed on as they are: the module's own
    check (messages.check_request) refuses them, so its words are the ones the asker hears."""
    request = {"id": int(message.id)}
    if message.action or not message.set:
        request["do"] = message.action
        request["args"] = _json_object(message.args, "args")
    if message.set:
        request["set"] = _json_object(message.set, "set")
    return request


def reply_to_msg(message, reply: dict):
    """A reply dict -> Command.Response."""
    message.accepted = bool(reply.get("accepted"))
    message.code = "" if message.accepted else str(reply.get("code") or "")
    message.reason = "" if message.accepted else str(reply.get("reason") or "")
    return message


def reply_from_msg(rid, message) -> dict:
    """Command.Response -> a reply dict (the id is the request's: the response does not carry it)."""
    if message.accepted:
        return M.accept(rid)
    return M.refuse(rid, message.code, message.reason)


def split_seconds(t: float) -> tuple:
    """Float seconds -> (sec, nanosec), nanosec in [0, 1e9): THE ONE converter for ROS stamps / durations (2026-11-22:
    harness_planner.demo.to_msg kept its own copy without the carry — a sample a hair under a whole second encoded as
    (s, 0), its time fell back ~1 s and the arm node refused "time_from_start must increase strictly")."""
    sec, nanosec = divmod(int(round(float(t) * 1e9)), 1_000_000_000)
    return int(sec), int(nanosec)


def _put_stamp(stamp, t: float) -> None:
    stamp.sec, stamp.nanosec = split_seconds(t)


def state_to_msg(message, state: dict):
    """A state dict (Module.state()) -> ModuleState."""
    _put_stamp(message.stamp, float(state["time"]))
    message.instance, message.mode = state["module"], state["mode"]
    fault, current, last = state.get("fault") or {}, state.get("current"), state.get("last")
    message.fault_code, message.fault_reason = str(fault.get("code", "")), str(fault.get("reason", ""))
    message.has_current = current is not None
    if current:
        message.current_id, message.current_action = int(current["id"]), current["do"]
        message.current_progress = float(current.get("progress") or 0.0)
    message.has_last = last is not None
    if last:
        message.last_id, message.last_action = int(last["id"]), last["do"]
        message.last_result, message.last_reason = last["result"], str(last.get("reason") or "")
    message.describe = _dumps(state["describe"])
    message.values, message.sources = _dumps(state["values"]), _dumps(state["sources"])
    return message


def state_from_msg(message) -> dict:
    """ModuleState -> a state dict, the same shape Module.state() has (progress back to the 3 decimals the module
    sends: float32 on the wire)."""
    last = None
    if message.has_last:
        last = {"id": int(message.last_id), "do": message.last_action, "result": message.last_result}
        if message.last_reason:
            last["reason"] = message.last_reason
    current = None
    if message.has_current:
        current = {"id": int(message.current_id), "do": message.current_action,
                   "progress": round(float(message.current_progress), 3)}
    return {
        "time": message.stamp.sec + message.stamp.nanosec / 1e9,
        "module": message.instance,
        "describe": _json_object(message.describe, "describe") if message.describe else None,
        "mode": message.mode,
        "fault": {"code": message.fault_code, "reason": message.fault_reason} if message.fault_code else None,
        "current": current,
        "last": last,
        "values": _json_object(message.values, "values"),
        "sources": _json_object(message.sources, "sources"),
    }
