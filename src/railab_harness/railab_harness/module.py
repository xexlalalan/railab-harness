"""THE ADAPTER BASE: what every module node is.

A module node stands between one device and the harness. Downward it speaks the device's own
language (CAN, Modbus, a vendor SDK, a socket); upward every module looks the same: one `command`
to ask it, one `state` it pushes. This class is that upward half, so a new module is only:

    class Capper(Module):
        TYPE, VERSION = "capper_decapper", "0.1.0"
        ZONES = ["head_travel"]
        ACTIONS = {"cap": {"args": {"torque_nm": {"unit": "N·m", "range": [0.2, 3]}}, "uses": ["head_travel"]}}
        FIELDS = {"clamp": {"values": ["open", "closed", "moving"], "writable": False}}

        def do_cap(self, args, run): ...      # one method per action; returns = done
        def check_cap(self, args): ...        # optional: refuse before accepting (raise Refused)
        def set_<field>(self, value): ...     # optional: refuse a told value (raise Refused)
        def safe_state(self): ...             # make the device stop NOW (only what this node started)
        def on_stop_request(self): ...        # optional: what a `stop` of scope "device" does beyond that
        def check_clear_fault(self, args): ...   # optional: refuse clear_fault while the cause persists

and, from the adapter's own threads (a watch on the device): sensed / told / estimated / unknown
one field, sensed_many several in one state, set_ready, latch_fault.

What the base owns, so that no module gets it wrong in its own way: the mode, the fault latch,
one action at a time, stop, the values with their sources, the state stream, and going safe when
the harness falls silent.
"""

import threading
import time

from . import messages as M
from .messages import Refused
from .timing import DEFAULT, Timing


class Fault(Exception):
    """Raised by an action: the module is in trouble. Latched until clear_fault."""

    def __init__(self, code: str, reason: str):
        super().__init__(reason)
        self.code, self.reason = code, reason


class Failed(Exception):
    """Raised by an action that did not reach its end although the module is fine."""


class Run:
    """What a running action sees of the outside: is a stop wanted, and where to report progress."""

    def __init__(self, module: "Module", rid):
        self._module, self.id = module, rid
        self._stop = threading.Event()
        self.stop_reason = None

    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    def sleep(self, seconds: float) -> bool:
        """Wait; False as soon as a stop is wanted — `if not run.sleep(dt): return`."""
        return not self._stop.wait(seconds)

    def progress(self, fraction: float) -> None:
        self._module._progress(self, fraction)

    def ask_stop(self, reason: str) -> None:
        if not self.stopped:
            self.stop_reason = reason
            self._stop.set()


class Module:
    TYPE = "module"
    VERSION = "0"
    ACTIONS: dict = {}   # name -> {"args": {arg: SPEC}, "uses": [zone]}; stop / clear_fault are added
    FIELDS: dict = {}    # name -> SPEC + {"writable": bool}
    ZONES: list = []     # the spaces this module's own motion sweeps (names only: no geometry here)

    def __init__(self, name: str, bus, timing: Timing = DEFAULT, serial: str = ""):
        self.name, self.bus, self.timing = name, bus, timing
        self.describe = {
            "identity": {"type": self.TYPE, "serial": serial, "version": self.VERSION},
            "actions": {**self.ACTIONS, **M.COMMON_ACTIONS},
            "fields": dict(self.FIELDS),
            "zones": list(self.ZONES),
        }
        self._lock = threading.RLock()        # everything below
        self._publishing = threading.Lock()   # states leave in the order they became true
        self._ready = False
        self._fault = None
        self._current = None
        self._last = None
        self._action_run = None   # the running action's Run (not "_run": a subclass may own that name)
        self._worker = None
        self._values = {field: M.UNKNOWN for field in self.FIELDS}   # nothing is known until said
        self._sources = {field: M.UNKNOWN for field in self.FIELDS}
        self._harness_alive = True
        self._beat_at = time.monotonic()
        self._published_at = 0.0
        self._closing = threading.Event()
        self._ticker = threading.Thread(target=self._tick, name=f"{name}-tick", daemon=True)

    # ------------------------------------------------------------ for the subclass
    def safe_state(self) -> None:
        """Make the device stop NOW. Must be quick. It touches only what THIS node started (its own goal,
        its own job): an adapter may run beside another path to the same device, and the automatic safe
        state must never stop that path — with nothing of its own running it does nothing at all.
        Called: while an action of this node runs, on any stop (a `stop` request, no harness beat, the
        harness's own stop of scope "own"); and on a `stop` request of scope "device", close and a
        latched fault even when none runs."""

    def on_stop_request(self) -> None:
        """Optional: what a `stop` REQUEST of scope "device" (the default: the twin's stop) does beyond
        safe_state() — called right after it, never on the automatic safe state (no beat, close, a fault,
        the harness's own stop of scope "own"). WHY: a stop the twin asks for stops the device whoever
        moves it (another client's motion too); going safe by itself must not."""

    def check_clear_fault(self, args) -> None:
        """Optional: refuse clear_fault (raise Refused) while the device still has the fault's cause.
        Called only while a fault is latched, under the module's lock, just before the fault is
        cleared (nothing refuses after it). WHY: a fault cleared while its cause persists would let an
        action start on a device in trouble."""

    def set_ready(self, ready: bool) -> None:
        """Ready = the device is there and may take actions (after its own start-up)."""
        with self._lock:
            self._ready = ready
        self._publish()

    def sensed(self, field, value):
        """The device measured it."""
        self._put(field, value, M.SENSED)

    def told(self, field, value):
        """Someone outside said so (a `set`)."""
        self._put(field, value, M.TOLD)

    def estimated(self, field, value):
        """Worked out from what the module commanded, not measured."""
        self._put(field, value, M.ESTIMATED)

    def unknown(self, field):
        """The module no longer knows (a restart, a hand on the device, an interrupted move)."""
        self._put(field, M.UNKNOWN, M.UNKNOWN)

    def sensed_many(self, values: dict, ready=None) -> None:
        """Several fields the device measured (a value "unknown" = no longer known, source unknown) and,
        unless ready is None, the readiness — in ONE state, published always. WHY: a periodic sampler
        that called sensed() per field would push one state per field, each a half-updated picture."""
        with self._lock:
            undeclared = [field for field in values if field not in self._values]
            if undeclared:
                raise KeyError(f"{self.name} declares no field {undeclared[0]!r}")
            for field, value in values.items():
                self._values[field] = value
                self._sources[field] = M.UNKNOWN if value == M.UNKNOWN else M.SENSED
            if ready is not None:
                self._ready = bool(ready)
        self._publish()

    def latch_fault(self, code: str, reason: str) -> bool:
        """Latch a fault seen outside any action (a watch on the device's status): the running action
        is stopped, the device made safe (the automatic safe state), the fault kept until clear_fault.
        False when a fault was latched already (the first one stays). WHY: an action latches by raising
        Fault, but a device can fail while no action runs."""
        with self._lock:
            if self._fault is not None:
                return False
            self._fault = {"code": code, "reason": reason}
        self._stop(reason)
        self._publish()
        return True

    def value(self, field):
        with self._lock:
            return self._values[field]

    def need(self, field):
        """The value, or the refusal an action gives when it cannot work without it."""
        value = self.value(field)
        if value == M.UNKNOWN:
            raise Refused(M.UNKNOWN_VALUE, f"{field} is unknown")
        return value

    def running(self):
        """The name of the action in progress, or None."""
        with self._lock:
            return self._current["do"] if self._current else None

    # ------------------------------------------------------------ life
    def start(self) -> "Module":
        self.bus.serve(self.name, self.command, self._on_beat)
        self._publish()
        self._ticker.start()
        return self

    def close(self) -> None:
        self._closing.set()
        self._stop("module closing")
        for thread in (self._worker, self._ticker):
            if thread is not None and thread.is_alive():
                thread.join(2)
        self.bus.leave(self.name)

    # ------------------------------------------------------------ command
    def command(self, request: dict) -> dict:
        """The module's ONE door. Answers at once: taken, or refused with a code and a reason."""
        if not isinstance(request, dict):
            return M.refuse(None, M.BAD_ARGS, "a request is an object")
        rid = request.get("id")
        try:
            M.check_request(request, self.describe)
            if "set" in request:
                self._set(request["set"])
            elif request["do"] == M.STOP:
                args = request.get("args") or {}
                why = args.get("reason")
                reason = f"stop #{rid}" + (f": {why}" if why else "")
                if args.get("scope", M.STOP_DEVICE) == M.STOP_OWN:
                    self._stop(reason, quiet=True)   # the harness going safe: only an action of ours, if any
                else:
                    self._stop(reason, rid)
                    self.on_stop_request()
            else:
                self._do(rid, request["do"], request.get("args") or {})
        except Refused as refusal:
            return M.refuse(rid, refusal.code, refusal.reason)
        self._publish()
        return M.accept(rid)

    def _do(self, rid, action: str, args: dict) -> None:
        with self._lock:
            mode = self._mode()
            if mode == M.MODE_FAULT and action != M.CLEAR_FAULT:
                raise Refused(M.FAULT, f"in fault {self._fault['code']}: {self._fault['reason']}")
            if mode == M.MODE_BUSY:
                raise Refused(M.BUSY, f"{self._current['do']} (request {self._current['id']}) is running")
            if action == M.CLEAR_FAULT:
                if self._fault is not None:
                    self.check_clear_fault(args)
                self._fault = None
                self._last = {"id": rid, "do": action, "result": M.DONE}
                return
            if mode == M.MODE_NOT_READY:
                raise Refused(M.NOT_READY, "the module is not ready")
            if not self._harness_alive:
                raise Refused(M.NOT_READY, "no harness heartbeat: nobody could stop this action")
            check = getattr(self, "check_" + action, None)
            if check is not None:
                check(args)
            run = Run(self, rid)
            self._current = {"id": rid, "do": action, "progress": 0.0}
            self._action_run = run
            self._worker = threading.Thread(target=self._work, args=(run, action, args),
                                            name=f"{self.name}-{action}", daemon=True)
            self._worker.start()

    def _work(self, run: Run, action: str, args: dict) -> None:
        fault = None
        try:
            getattr(self, "do_" + action)(args, run)
            result, reason = (M.STOPPED, run.stop_reason) if run.stopped else (M.DONE, None)
        except Failed as failure:
            result, reason = M.FAILED, str(failure)
        except Fault as trouble:
            result, reason = M.FAILED, trouble.reason
            fault = {"code": trouble.code, "reason": trouble.reason}
        except Exception as error:   # a bug in the adapter is a fault too: never a silent "done"
            result, reason = M.FAILED, f"{type(error).__name__}: {error}"
            fault = {"code": "INTERNAL", "reason": reason}
        with self._lock:
            self._last = {"id": run.id, "do": action, "result": result}
            if reason:
                self._last["reason"] = reason
            self._fault = self._fault or fault
            self._current = self._action_run = None
        self._publish()

    def _stop(self, reason: str, rid=None, quiet: bool = False) -> None:
        """Never refused. A running action ends "stopped" with this reason; then the device is made safe.

        quiet = an AUTOMATIC stop (no harness beat, the harness's own stop of scope "own"): with no action
        of this node running it does nothing at all — no safe_state, no `last` (an action of the old path
        to the same device is not this node's to stop, and the result someone waits for stays)."""
        with self._lock:
            if self._action_run is not None:
                self._action_run.ask_stop(reason)
            elif quiet:
                return
            elif rid is not None:
                self._last = {"id": rid, "do": M.STOP, "result": M.DONE}
        self.safe_state()

    def _set(self, fields: dict) -> None:
        """Take told facts. Every hook is asked before anything is stored: all of the set, or none."""
        with self._lock:
            if self._fault is not None:
                raise Refused(M.FAULT, f"in fault {self._fault['code']}: {self._fault['reason']}")
            for field, value in fields.items():
                hook = getattr(self, "set_" + field, None)
                if hook is not None:
                    hook(value)
            for field, value in fields.items():
                self._values[field], self._sources[field] = value, M.TOLD

    # ------------------------------------------------------------ state
    def state(self) -> dict:
        with self._lock:
            return {
                "time": time.time(),
                "module": self.name,
                "describe": self.describe,
                "mode": self._mode(),
                "fault": dict(self._fault) if self._fault else None,
                "current": dict(self._current) if self._current else None,
                "last": dict(self._last) if self._last else None,
                "values": dict(self._values),
                "sources": dict(self._sources),
            }

    def _mode(self) -> str:
        if self._fault is not None:
            return M.MODE_FAULT
        if self._current is not None:
            return M.MODE_BUSY
        return M.MODE_READY if self._ready else M.MODE_NOT_READY

    def _put(self, field, value, source) -> None:
        with self._lock:
            if field not in self._values:
                raise KeyError(f"{self.name} declares no field {field!r}")
            if (self._values[field], self._sources[field]) == (value, source):
                return
            self._values[field], self._sources[field] = value, source
        self._publish()

    def _progress(self, run: Run, fraction: float) -> None:
        with self._lock:
            if self._action_run is not run:
                return
            self._current["progress"] = round(min(1.0, max(0.0, fraction)), 3)
        self._publish()

    def _publish(self) -> None:
        with self._publishing:
            self._published_at = time.monotonic()
            self.bus.publish_state(self.state())

    # ------------------------------------------------------------ liveness
    def _on_beat(self) -> None:
        self._beat_at = time.monotonic()
        self._harness_alive = True

    def _tick(self) -> None:
        """The idle beat upward, and the watch on the harness's beat downward."""
        timing = self.timing
        while not self._closing.wait(min(timing.idle_state_s, timing.module_silent_s) / 4):
            now = time.monotonic()
            if self._harness_alive and now - self._beat_at > timing.module_silent_s:
                self._harness_alive = False   # nobody supervises: do not keep moving
                self._stop("harness silent", quiet=True)
                self._publish()
            elif now - self._published_at >= timing.idle_state_s:
                self._publish()
