"""THE GENERIC DOOR: the one way from the twin to the hardware.

The gateway knows no module. For every request it asks, in this order:
  1. may this client command?            (the lease; `stop` is taken from anyone)
  2. is the module there?                (the registry: it announced itself and still talks)
  3. is the request one the module offers, with values in range?   (the module's OWN describe)
  4. are the zones this motion needs free?                         (zone occupancy)
then forwards the request VERBATIM and returns the module's own reply. It never changes a request:
do it, or refuse it with a reason. Whether the action then succeeds is told by the module's state.

It also beats (so modules know the harness lives), watches the controlling client, and logs every
request with its reply.

GOING SAFE BY ITSELF touches only what THIS layer started (operator 2026-11-20; the modules run BESIDE
the old gateway on 7803, which drives the same arm and Bunny): when the controlling client falls silent
(timing.client_silent_s) or its link ends, `stop_all` sends a `stop` of scope "own" to each module with
an action of this layer running — and to no other. A module then stops that action only (its own goal,
its own job); none running = nothing at all: no cancelled goal of the old path, no re-held arm, no abort.

THE WATCHDOG FOLLOWS THE ACTION: a client that gives control up (release_control, a hello without the key)
while an action of this layer runs stays watched until no such action runs — its silence or its link's
end still stops them. Taking control (anyone) hands the watch to the new holder.

Nothing slow runs on the beat's thread: every stop goes out from a thread of its own (a module that does
not answer would otherwise hold the beat back for up to two reply periods), and a state that cannot be
read (a malformed describe …) is dropped and said on stderr, never raised into the bus's thread.
"""

import itertools
import sys
import threading
import time

from . import __version__
from . import messages as M
from .audit import Audit
from .bus import Bus, Unreachable
from .lease import Lease
from .registry import Registry
from .timing import DEFAULT, Timing
from .zones import Zones

HARNESS = "harness"   # the client name of the gateway's own stops, in the log
INTERNAL = "INTERNAL"  # a stop that raised inside the gateway / the bus (a bug), as logged


class Gateway:
    def __init__(self, bus: Bus, timing: Timing = DEFAULT, audit_path=None):
        self.bus, self.timing = bus, timing
        self.registry = Registry(timing)
        self.zones = Zones(self.registry.online)
        self.lease = Lease()
        self.audit = Audit(audit_path)
        self._lock = threading.Lock()
        self._listeners = []
        self._own_ids = itertools.count(-1, -1)   # the gateway's own stops: negative, never a client's id
        self._closing = threading.Event()
        self._ticker = threading.Thread(target=self._tick, name="gateway-tick", daemon=True)
        self._unsubscribe = None
        # under _lock:
        self._started = {}     # module -> [request id, seen running?]: an action this gateway forwarded and the
                               # module took, until the module's own state shows it over (as zones.py frees a grant)
        self._keeper = None    # [client, heard at]: gave control up while an action of this layer ran
        self._stoppers = []    # the threads still sending a stop
        self._said = set()     # what was said on stderr already: modules with an unreadable state, listeners that raised

    # ------------------------------------------------------------ life
    def start(self) -> "Gateway":
        self._unsubscribe = self.bus.on_state(self._on_state)
        self.bus.beat()
        self._ticker.start()
        return self

    def close(self) -> None:
        """The harness goes away. The beat ends with it — which is what makes the modules go safe.
        A stop still on its way is waited for (it goes out, and into the log) first."""
        self._closing.set()
        if self._ticker.is_alive():
            self._ticker.join(2)
        if self._unsubscribe is not None:
            self._unsubscribe()
        self.join_stops(2 * self.timing.reply_s + 1)
        self.audit.close()

    # ------------------------------------------------------------ what a client may ask
    def hello(self, client: str) -> dict:
        """Who is there and what each can do — read from the modules' own states, nothing typed here."""
        self._heard(client)
        modules = {name: {"online": entry["online"], "describe": self.registry.describe(name)}
                   for name, entry in self.registry.snapshot().items()}
        return {"harness": __version__, "you": "control" if self.lease.held_by(client) else "view",
                "control": self.lease.holder, "modules": modules}

    def take_control(self, client: str) -> bool:
        """True when `client` holds control now. The new holder is the one watched from now on."""
        taken = self.lease.take(client)
        if taken:
            with self._lock:
                self._keeper = None
        return taken

    def release_control(self, client: str) -> bool:
        """True when `client` held control (it is free now). Releasing stops nothing — but while an action
        of this layer runs, `client` stays watched (its silence or its link's end stops those actions):
        giving control up must not leave a running motion without its watchdog."""
        released = self.lease.release(client)
        if released and self.running_actions():
            with self._lock:
                self._keeper = [client, time.monotonic()]
        return released

    def client_gone(self, client: str, why: str) -> None:
        """A client's link ended (closed, broken, a line too long). The controlling client's end — or the end
        of one that gave control up while an action of this layer still runs — = a silent client at once:
        the lease dropped, this layer's running actions stopped. Anyone else's end changes nothing."""
        if self.lease.release(client):
            self.stop_all(f"client {client} disconnected: {why}")
            return
        with self._lock:
            kept = self._keeper is not None and self._keeper[0] == client
            if kept:
                self._keeper = None
        if kept:
            print(f"gateway: {client} had given control up while an action ran", file=sys.stderr, flush=True)
            self.stop_all(f"client {client} disconnected: {why}")

    def ping(self, client: str) -> None:
        """A client with nothing to say still says it is alive."""
        self._heard(client)

    def states(self) -> dict:
        return self.registry.snapshot()

    def subscribe(self, callback):
        """Every module's states as ONE stream of {"module", "online", "state"}; returns the unsubscriber.

        "online" is the only thing the harness adds (it is the harness's own knowledge); a state is
        passed on as the module sent it.
        """
        with self._lock:
            self._listeners.append(callback)

        def stop():
            with self._lock:
                if callback in self._listeners:
                    self._listeners.remove(callback)
        return stop

    def handle(self, client: str, envelope: dict) -> dict:
        """One request from a client: {"module", "id", "do" | "set", ..., "claims"?} → the reply."""
        self._heard(client)
        reply, forwarded = self._decide(client, envelope)
        self.audit.write(client, envelope, reply, forwarded)
        return reply

    def stop_all(self, reason: str) -> list:
        """The harness going safe by itself: stop the actions THIS layer started — and nothing else.

        A `stop` of scope "own" goes to each module with such an action running (its state shows `current`,
        or this gateway forwarded an action the module took and has not yet seen end — a silent module's
        too: it may still hear); a module with none gets nothing. Each stop is sent from a thread of its own
        and this returns at once (a module that does not answer must not hold up the beat). Returns the
        modules a stop went to. A module whose describe offers no scope (an older one) gets a plain stop:
        it cannot tell the two apart, and its own running action must stop."""
        modules = self.running_actions()
        for module in modules:
            stop = ((self.registry.describe(module) or {}).get("actions") or {}).get(M.STOP) or {}
            args = {"reason": reason}
            if "scope" in (stop.get("args") or {}):
                args["scope"] = M.STOP_OWN
            request = {"id": next(self._own_ids), "do": M.STOP, "args": args}
            thread = threading.Thread(target=self._send_stop, args=(module, request), name=f"gateway-stop-{module}",
                                      daemon=True)
            with self._lock:
                self._stoppers = [t for t in self._stoppers if t.is_alive()] + [thread]
            thread.start()
        return modules

    def join_stops(self, timeout_s: float) -> bool:
        """Wait for the stops still on their way; True when none is left."""
        end = time.monotonic() + timeout_s
        with self._lock:
            stoppers = list(self._stoppers)
        for thread in stoppers:
            thread.join(max(0.0, end - time.monotonic()))
        return not any(thread.is_alive() for thread in stoppers)

    def running_actions(self) -> list:
        """The modules with an action of THIS layer running: its state shows `current` (every action a module
        runs came in through its command door), or this gateway forwarded one the module took and no state of
        the module has shown it over yet (its busy state may still be on its way)."""
        with self._lock:
            running = set(self._started)
        for module, entry in self.registry.snapshot().items():
            if (entry.get("state") or {}).get("current"):
                running.add(module)
        return sorted(running)

    # ------------------------------------------------------------ inside
    def _decide(self, client: str, envelope: dict):
        """(reply, was it forwarded to the module)."""
        if not isinstance(envelope, dict):
            return M.refuse(None, M.BAD_ARGS, "a request is an object"), False
        rid, module = envelope.get("id"), envelope.get("module")
        claims = envelope.get("claims")
        claims = [] if claims is None else claims
        request = {key: value for key, value in envelope.items() if key not in ("module", "claims")}
        is_stop = request.get("do") == M.STOP

        if not is_stop and not self.lease.held_by(client):
            holder = self.lease.holder
            return M.refuse(rid, M.NO_LEASE, f"control is {holder}'s" if holder else "take control first"), False
        describe = self.registry.describe(module) if isinstance(module, str) else None
        if describe is None:
            return M.refuse(rid, M.OFFLINE, f"no module {module!r} announced"), False
        if not is_stop and not self.registry.online(module):
            return M.refuse(rid, M.OFFLINE, f"{module} has gone silent"), False
        if not isinstance(claims, list) or not all(isinstance(zone, str) for zone in claims):
            return M.refuse(rid, M.BAD_ARGS, "claims is a list of zone names"), False
        try:
            M.check_request(request, describe)
        except M.Refused as refusal:
            return M.refuse(rid, refusal.code, refusal.reason), False

        holder = (module, rid)
        if "do" in request and not is_stop:
            unknown = [zone for zone in claims if zone not in self.registry.zones()]
            if unknown:
                return M.refuse(rid, M.BAD_ARGS, f"no module declares the zone {unknown[0]!r}"), False
            needed = [f"{module}/{zone}" for zone in describe["actions"][request["do"]].get("uses", [])]
            blocked = self.zones.acquire(holder, needed + list(claims))
            if blocked:
                return M.refuse(rid, M.INTERLOCK, blocked), False
        try:
            reply = self.bus.call(module, request, self.timing.reply_s)
        except Unreachable as gone:
            reply = M.refuse(rid, M.OFFLINE, str(gone))
        if not reply.get("accepted"):
            self.zones.release(holder)
        elif "do" in request and not is_stop:
            with self._lock:
                self._started[module] = [rid, False]
        return reply, True

    def _heard(self, client: str) -> None:
        """Anything from `client` says it is alive: for the lease, and for the watch on a client that gave
        control up while an action ran."""
        self.lease.touch(client)
        with self._lock:
            if self._keeper is not None and self._keeper[0] == client:
                self._keeper[1] = time.monotonic()

    def _send_stop(self, module: str, request: dict) -> None:
        try:
            reply = self.bus.call(module, request, self.timing.reply_s)
        except Unreachable as gone:
            reply = M.refuse(request["id"], M.OFFLINE, str(gone))
        except Exception as error:   # noqa: BLE001  a bug in a bus must not end silently
            reply = M.refuse(request["id"], INTERNAL, f"{type(error).__name__}: {error}")
            print(f"gateway: the stop to {module} raised: {reply['reason']}", file=sys.stderr, flush=True)
        self.audit.write(HARNESS, {"module": module, **request}, reply, True)

    def _on_state(self, state: dict) -> None:
        """Every module's state, on the bus's thread: never raises into it (an exception there would end the
        bus's thread while the beat goes on — modules would hear a harness that no longer listens)."""
        try:
            why = unreadable(state)
            if why:
                name = state.get("module") if isinstance(state, dict) else None
                key = name if isinstance(name, str) else "?"
                if key not in self._said:
                    self._said.add(key)
                    print(f"gateway: a state of {key} is dropped (its states stop counting as fresh): {why}",
                          file=sys.stderr, flush=True)
                return
            self._said.discard(state["module"])
            self.registry.update(state)
            self.zones.on_state(state)
            self._note_started(state)
        except Exception as error:   # noqa: BLE001
            print(f"gateway: a state could not be taken: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
            return
        self._tell({"module": state["module"], "online": True, "state": state})

    def _note_started(self, state: dict) -> None:
        """An action this gateway forwarded is over once the module's own state shows it: seen running and no
        longer, or already the module's last result (the rule zones.py frees a grant by)."""
        module = state["module"]
        current, last = state.get("current") or {}, state.get("last") or {}
        with self._lock:
            entry = self._started.get(module)
            if entry is None:
                return
            running = current.get("id")
            if running == entry[0]:
                entry[1] = True
            elif entry[1] or last.get("id") == entry[0] or running is not None:   # another one runs: ours is over
                del self._started[module]

    def _tell(self, item: dict) -> None:
        with self._lock:
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(item)
            except Exception as error:   # noqa: BLE001  one listener's bug must not starve the others
                if id(listener) not in self._said:   # said once per listener, not at every state
                    self._said.add(id(listener))
                    print(f"gateway: a state listener raised (said once): {type(error).__name__}: {error}",
                          file=sys.stderr, flush=True)

    def _tick(self) -> None:
        """One clock for the harness's three duties in time: beat, watch the client, notice silence. The beat
        goes first and nothing here waits on a module (the stops have their own threads)."""
        while not self._closing.wait(self.timing.beat_s):
            for duty in (self.bus.beat, self._watch_clients, self._say_offline):
                try:
                    duty()
                except Exception as error:   # noqa: BLE001  the beat must outlive one bad duty
                    print(f"gateway: {duty.__name__}: {type(error).__name__}: {error}", file=sys.stderr, flush=True)

    def _watch_clients(self) -> None:
        """The dead-man: the controlling client silent → lease dropped, this layer's running actions stopped;
        a client that gave control up while an action ran, silent → those actions stopped."""
        if self.lease.silent_for() > self.timing.client_silent_s:
            client = self.lease.drop()
            if client is not None:   # None: released between the two looks (the release made it the keeper)
                self.stop_all(f"client {client} silent")
        with self._lock:
            keeper = list(self._keeper) if self._keeper is not None else None
        if keeper is None:
            return
        client = keeper[0]
        running = self.running_actions()
        with self._lock:
            if self._keeper is None or self._keeper[0] != client:
                return   # taken over or gone meanwhile
            silent = time.monotonic() - self._keeper[1] > self.timing.client_silent_s   # its newest word
            if running and not silent:
                return
            self._keeper = None   # its actions are over, or it fell silent
        if running:
            self.stop_all(f"client {client} silent, had given control up")

    def _say_offline(self) -> None:
        for module in self.registry.went_offline():
            self._tell({"module": module, "online": False, "state": self.registry.state(module)})


# ------------------------------------------------------------ what a state must look like to be taken
def _zone_list(value) -> bool:
    return isinstance(value, list) and all(isinstance(zone, str) for zone in value)


def _spec_problem(name: str, spec) -> str:
    if not isinstance(spec, dict):
        return f"{name} is not an object"
    if "range" in spec:
        rng = spec["range"]
        if (not isinstance(rng, list) or len(rng) != 2
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in rng)):
            return f"{name}: range is not [lo, hi]"
    if "values" in spec and not isinstance(spec["values"], list):
        return f"{name}: values is not a list"
    return ""


def unreadable(state) -> str:
    """Why a state cannot be taken ("" = it can). WHY: the gateway reads every describe on every request
    (check_request, the zones, hello): one malformed describe kept in the registry would break the requests
    to every module."""
    if not isinstance(state, dict):
        return "a state is not an object"
    if not isinstance(state.get("module"), str) or not state["module"]:
        return "no module name"
    for part in ("current", "last", "fault"):
        if state.get(part) is not None and not isinstance(state[part], dict):
            return f"{part} is not an object"
    if (state.get("current") or {}) and "id" not in state["current"]:
        return "current has no id"
    describe = state.get("describe")
    if describe is None:
        return ""
    if not isinstance(describe, dict):
        return "describe is not an object"
    actions, fields = describe.get("actions"), describe.get("fields", {})
    if not isinstance(actions, dict) or not isinstance(fields, dict):
        return "describe: actions / fields are not objects"
    if not _zone_list(describe.get("zones", [])):
        return "describe: zones is not a list of names"
    for name, action in actions.items():
        if not isinstance(action, dict) or not isinstance(action.get("args", {}), dict) \
                or not _zone_list(action.get("uses", [])):
            return f"describe: action {name!r} is not {{args: {{}}, uses: []}}"
        for arg, spec in action.get("args", {}).items():
            problem = _spec_problem(f"{name}.{arg}", spec)
            if problem:
                return "describe: " + problem
    for field, spec in fields.items():
        problem = _spec_problem(f"field {field}", spec)
        if problem:
            return "describe: " + problem
    return ""
