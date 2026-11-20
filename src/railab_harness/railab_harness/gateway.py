"""THE GENERIC DOOR: the one way from the twin to the hardware.

The gateway knows no module. For every request it asks, in this order:
  1. may this client command?            (the lease; `stop` is taken from anyone)
  2. is the module there?                (the registry: it announced itself and still talks)
  3. is the request one the module offers, with values in range?   (the module's OWN describe)
  4. are the zones this motion needs free?                         (zone occupancy)
then forwards the request VERBATIM and returns the module's own reply. It never changes a request:
do it, or refuse it with a reason. Whether the action then succeeds is told by the module's state.

It also beats (so modules know the harness lives), watches the controlling client (silent → stop
everything), and logs every request with its reply.
"""

import itertools
import threading

from . import __version__
from . import messages as M
from .audit import Audit
from .bus import Bus, Unreachable
from .lease import Lease
from .registry import Registry
from .timing import DEFAULT, Timing
from .zones import Zones

HARNESS = "harness"   # the client name of the gateway's own stops, in the log


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

    # ------------------------------------------------------------ life
    def start(self) -> "Gateway":
        self._unsubscribe = self.bus.on_state(self._on_state)
        self.bus.beat()
        self._ticker.start()
        return self

    def close(self) -> None:
        """The harness goes away. The beat ends with it — which is what makes the modules go safe."""
        self._closing.set()
        if self._ticker.is_alive():
            self._ticker.join(2)
        if self._unsubscribe is not None:
            self._unsubscribe()
        self.audit.close()

    # ------------------------------------------------------------ what a client may ask
    def hello(self, client: str) -> dict:
        """Who is there and what each can do — read from the modules' own states, nothing typed here."""
        self.lease.touch(client)
        modules = {name: {"online": entry["online"], "describe": self.registry.describe(name)}
                   for name, entry in self.registry.snapshot().items()}
        return {"harness": __version__, "you": "control" if self.lease.held_by(client) else "view",
                "control": self.lease.holder, "modules": modules}

    def take_control(self, client: str) -> bool:
        return self.lease.take(client)

    def release_control(self, client: str) -> bool:
        """True when `client` held control (it is free now); releasing stops nothing."""
        return self.lease.release(client)

    def ping(self, client: str) -> None:
        """A client with nothing to say still says it is alive."""
        self.lease.touch(client)

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
        self.lease.touch(client)
        reply, forwarded = self._decide(client, envelope)
        self.audit.write(client, envelope, reply, forwarded)
        return reply

    def stop_all(self, reason: str) -> None:
        """Stop every module the harness has ever heard of (also the silent ones: they may still hear)."""
        for module in self.registry.modules():
            request = {"id": next(self._own_ids), "do": M.STOP, "args": {"reason": reason}}
            try:
                reply = self.bus.call(module, request, self.timing.reply_s)
            except Unreachable as gone:
                reply = M.refuse(request["id"], M.OFFLINE, str(gone))
            self.audit.write(HARNESS, {"module": module, **request}, reply, True)

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
            return M.refuse(rid, M.OFFLINE, f"no module {module!r} has announced itself"), False
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
        return reply, True

    def _on_state(self, state: dict) -> None:
        self.registry.update(state)
        self.zones.on_state(state)
        self._tell({"module": state["module"], "online": True, "state": state})

    def _tell(self, item: dict) -> None:
        with self._lock:
            listeners = list(self._listeners)
        for listener in listeners:
            listener(item)

    def _tick(self) -> None:
        """One clock for the harness's three duties in time: beat, watch the client, notice silence."""
        while not self._closing.wait(self.timing.beat_s):
            self.bus.beat()
            if self.lease.silent_for() > self.timing.client_silent_s:
                client = self.lease.drop()
                if client is not None:   # None: released between the two looks (a release stops nothing)
                    self.stop_all(f"client {client} silent")
            for module in self.registry.went_offline():
                self._tell({"module": module, "online": False, "state": self.registry.state(module)})
