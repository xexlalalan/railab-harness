"""THE TWIN'S DOOR: the Gateway on TCP, as newline-delimited JSON (one object per line, both ways).

One controlling client, any number of viewers (operator 2026-11-20). The access key is the one
today's gateway uses (KEY_FILE, created by `harness gateway key`): the right key in `hello` lets a
connection take control; without it a connection watches — and may still send `stop`, which the
gateway takes from anyone. The key is never printed, logged or sent back.

Client → door (every line has "op"):

  {"op": "hello", "client": "railab-wf", "key": "…"}   who I am; the key decides "access"
  {"op": "take_control"}   {"op": "release_control"}
  {"op": "ping"}                                       any line counts as alive: the controlling
                                                       client sends one at least every
                                                       timing.client_silent_s, else stop_all
  {"op": "subscribe"}   {"op": "unsubscribe"}          the merged state stream, pushed
  {"op": "request", "module": "bunny_1", "id": 21, "do": "platform_to", "args": {"z_mm": 120},
   "claims": ["…"]}                                    the contract's envelope + "op"

Door → client:

  an answer carries the op it answers:  {"op": "take_control", "ok": true}
                                        {"op": "take_control", "ok": false, "code", "reason"}
  a request's answer = the gateway's reply + the module:
                                        {"op": "request", "module": "bunny_1", "id": 21, "accepted": true}
  a pushed state (after subscribe):     {"op": "state", "module", "online", "state"}
  a line that is not an object with an op: {"op": "error", "code": "BAD_ARGS", "reason"}

Lines are handled one at a time in the order they arrive (a `set` sent before a `do` reaches the
module first); answers leave in that order. A request waits for the one before it on the same
connection (at most timing.reply_s when a module does not answer) — `stop` is taken from any
connection, so a client that wants a stop never queued may keep a second one.

Nothing a client does can hold up a module: a push only queues. When a client reads slower than
states arrive, only the NEWEST pending state of each module is still sent (a state is a picture,
not an event; an action's result stays in `last` until the next action of that module ends).

The CONTROLLING client's connection ending — closed, broken, or a line too long — is a silent
client at once: the lease is dropped and every module is stopped (Gateway.stop_all). A viewer's
ending changes nothing. `release_control` stops nothing (as on today's 7803).

Per connection the logic is `Connection` on any connected stream socket, so it is checked on a
`socket.socketpair()` without listening; `TcpFront.listen` / `serve_forever` only accept.
"""

import itertools
import json
import os
import secrets
import socket
import sys
import threading
import time
from collections import deque

from . import messages as M

# THE SAME KEY FILE as tools/gateway_server.py (KEY_FILE there; `harness gateway key` creates it).
# Read at every hello, so a key made while the door runs needs no restart.
KEY_FILE = os.path.expanduser("~/.config/railab-harness/gateway.key")
MAX_LINE_BYTES = 1 << 20      # a line longer than this ends the connection (a long path is ~100 kB)
MAX_CONNECTIONS = 32
ENVELOPE_KEYS = ("module", "id", "do", "set", "args", "claims")   # the contract's envelope, nothing else
NAME_MAX = 64
FLUSH_S = 1.0                 # a connection ended by the door first tries to deliver its last answer
ACCEPT_POLL_S = 0.5           # how often the accept loop looks whether the door is closing
INTERNAL = "INTERNAL"         # the gateway raised on a request: a bug, answered instead of a dropped link


def read_key():
    """The shared key, or None when there is none — the same file and reading as tools/gateway_server.py."""
    try:
        with open(KEY_FILE) as f:
            return f.read().strip() or None
    except OSError:
        return None


def key_matches(given, key) -> bool:
    if not key or not isinstance(given, str) or not given:
        return False
    return secrets.compare_digest(given.encode("utf-8"), key.encode("utf-8"))


def _no(op: str, code: str, reason: str) -> dict:
    return {"op": op, "ok": False, "code": code, "reason": reason}


def _short(text) -> str:
    text = "".join(c for c in str(text) if c.isprintable()).strip()
    return text[:NAME_MAX]


class TcpFront:
    """The door's listening side: accepts connections and keeps them until close()."""

    def __init__(self, gateway, read_key=read_key, max_connections: int = MAX_CONNECTIONS):
        self.gateway, self.read_key, self.max_connections = gateway, read_key, max_connections
        self._lock = threading.Lock()
        self._connections = set()
        self._numbers = itertools.count(1)
        self._listener = None
        self._closing = threading.Event()

    # ------------------------------------------------------------ connections
    def connection(self, sock, peer: str) -> "Connection":
        """A connection on a connected socket (accepted, or one end of a socketpair). Call .start()."""
        conn = Connection(self, sock, peer, next(self._numbers))
        with self._lock:
            self._connections.add(conn)
        return conn

    def connections(self) -> list:
        with self._lock:
            return list(self._connections)

    def _forget(self, conn) -> None:
        with self._lock:
            self._connections.discard(conn)

    # ------------------------------------------------------------ listening
    def listen(self, bind: str, port: int) -> None:
        """Open the listening socket (raises at once when the port is taken)."""
        family = socket.AF_INET6 if ":" in bind else socket.AF_INET
        self._listener = socket.create_server((bind, port), family=family)
        self._listener.settimeout(ACCEPT_POLL_S)

    def serve_forever(self) -> None:
        """Accept until close(). Blocking: run it in a thread."""
        listener = self._listener
        try:
            while not self._closing.is_set():
                try:
                    sock, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._closing.is_set():
                        break
                    raise
                sock.settimeout(None)
                if len(self.connections()) >= self.max_connections:
                    try:
                        sock.sendall((json.dumps({"op": "error", "code": M.BUSY, "reason":
                                                  f"the door has {self.max_connections} connections"}) + "\n").encode())
                    except OSError:
                        pass
                    sock.close()
                    continue
                self.connection(sock, f"{address[0]}:{address[1]}").start()
        finally:
            listener.close()

    def close(self) -> None:
        """Stop accepting and end every connection (the controller's ending stops every module)."""
        self._closing.set()
        for conn in self.connections():
            conn.close("the door is closing")


class Connection:
    """One client: a reader (lines in, handled in order) and a writer (answers and pushes out)."""

    def __init__(self, front: TcpFront, sock, peer: str, number: int):
        self.front, self.gateway, self.sock = front, front.gateway, sock
        self.peer, self.number = peer, number
        self.name = f"?@{peer}#{number}"   # the gateway's client name; fixed by the first hello
        self.access = "view"
        self._named = False
        self._wake = threading.Condition()   # guards everything below
        self._out = deque()                  # ("line", answer) | ("state", module), in order
        self._states = {}                    # module -> its newest pending push
        self._sending = False
        self._subscribed = False
        self._unsubscribe = None
        self._ending = self._ended = False
        self.ended_why = None
        self._reader = threading.Thread(target=self._read, name=f"front-{number}-read", daemon=True)
        self._writer = threading.Thread(target=self._write, name=f"front-{number}-write", daemon=True)

    # ------------------------------------------------------------ life
    def start(self) -> "Connection":
        self._writer.start()
        self._reader.start()
        return self

    def close(self, why: str = "closed by the harness") -> None:
        self._end(why)
        self.join(2)

    def join(self, timeout=None) -> None:
        if self._reader.is_alive() and threading.current_thread() is not self._reader:
            self._reader.join(timeout)

    @property
    def ended(self) -> bool:
        with self._wake:
            return self._ended

    def _end(self, why: str, flush_s: float = 0.0) -> None:
        """Once: no more pushes; the controller's ending stops every module; the socket is shut."""
        with self._wake:
            if self._ending:
                return
            self._ending = True
            if flush_s:
                self._wake.wait_for(lambda: not self._out and not self._sending, flush_s)
            self._ended, self.ended_why = True, why
            unsubscribe, self._unsubscribe, self._subscribed = self._unsubscribe, None, False
            self._wake.notify_all()
        if unsubscribe is not None:
            unsubscribe()
        if self.gateway.release_control(self.name):
            # the holder's end = a silent client at once (contract: client silence → stop ALL modules,
            # drop the lease); a viewer's end stops nothing
            self.gateway.stop_all(f"client {self.name} disconnected: {why}")
        try:
            self.sock.shutdown(socket.SHUT_RDWR)   # wakes the reader; it closes the socket last
        except OSError:
            pass

    # ------------------------------------------------------------ in
    def _read(self) -> None:
        buffer, why = b"", "the client closed the connection"
        try:
            while True:
                data = self.sock.recv(65536)
                if not data:
                    break
                buffer += data
                too_long = False
                while True:
                    line, newline, rest = buffer.partition(b"\n")
                    if not newline:
                        break
                    buffer = rest
                    if len(line) > MAX_LINE_BYTES:
                        too_long = True
                        break
                    if line.strip():
                        self._line(line)
                if too_long or len(buffer) > MAX_LINE_BYTES:
                    self._queue({"op": "error", "code": M.BAD_ARGS,
                                 "reason": f"a line longer than {MAX_LINE_BYTES} bytes: the connection is closed"})
                    why = "a line too long"
                    self._end(why, FLUSH_S)
                    break
        except OSError as error:
            why = f"connection broken ({error.strerror or type(error).__name__})"
        finally:
            self._end(why)
            if self._writer.is_alive():
                self._writer.join(2)
            self.sock.close()
            self.front._forget(self)

    def _line(self, raw: bytes) -> None:
        self.gateway.ping(self.name)   # any line from the client says it is alive
        try:
            message = json.loads(raw.decode("utf-8"))
        except (ValueError, RecursionError):
            self._queue({"op": "error", "code": M.BAD_ARGS, "reason": "a line must be one JSON object"})
            return
        if not isinstance(message, dict) or not isinstance(message.get("op"), str):
            self._queue({"op": "error", "code": M.BAD_ARGS, "reason": 'a line is a JSON object with "op"'})
            return
        handler = self._OPS.get(message["op"])
        if handler is None:
            op = _short(message["op"])
            self._queue(_no(op, M.BAD_ARGS, f"no op {op!r} (ops: {', '.join(sorted(self._OPS))})"))
            return
        answer = handler(self, message)
        if answer is not None:
            self._queue(answer)

    # ------------------------------------------------------------ the ops
    def _hello(self, message: dict) -> dict:
        if not self._named:
            label = _short(message.get("client") or "") or "client"
            self.name, self._named = f"{label}@{self.peer}#{self.number}", True
        key, given = self.front.read_key(), message.get("key")
        self.access = "control" if key_matches(given, key) else "view"
        if self.access == "view":
            self.gateway.release_control(self.name)   # a hello without the key gives control up (stops nothing)
        answer = {"op": "hello", "ok": True, "client": self.name, "access": self.access,
                  **self.gateway.hello(self.name)}
        if given and self.access == "view":
            answer["warning"] = "wrong key" if key else "no key on the harness (harness gateway key)"
        return answer

    def _take_control(self, message: dict) -> dict:
        if self.access != "control":
            return _no("take_control", M.NO_LEASE, "needs the gateway key (hello with the key)")
        if not self.gateway.take_control(self.name):
            return _no("take_control", M.NO_LEASE, f"control is {self.gateway.lease.holder}'s")
        return {"op": "take_control", "ok": True}

    def _release_control(self, message: dict) -> dict:
        self.gateway.release_control(self.name)
        return {"op": "release_control", "ok": True}

    def _ping(self, message: dict) -> dict:
        return {"op": "ping", "ok": True, "t": round(time.time(), 3)}

    def _subscribe(self, message: dict) -> None:
        with self._wake:
            fresh = not self._subscribed and not self._ended
            self._subscribed = self._subscribed or fresh
        if fresh:
            unsubscribe = self.gateway.subscribe(self._push)
            with self._wake:
                if self._ended:   # ended meanwhile: do not leave a listener behind
                    fresh = False
                else:
                    self._unsubscribe = unsubscribe
            if not fresh:
                unsubscribe()
        self._queue({"op": "subscribe", "ok": True})
        # then the picture so far: every module the harness has heard of
        for module, entry in self.gateway.states().items():
            self._push({"module": module, "online": entry["online"], "state": entry["state"]}, replace=False)

    def _unsubscribe_op(self, message: dict) -> dict:
        with self._wake:
            unsubscribe, self._unsubscribe, self._subscribed = self._unsubscribe, None, False
            self._out = deque(token for token in self._out if token[0] != "state")
            self._states.clear()
        if unsubscribe is not None:
            unsubscribe()
        return {"op": "unsubscribe", "ok": True}

    def _request(self, message: dict) -> dict:
        envelope = {key: value for key, value in message.items() if key != "op"}
        stray = sorted(_short(key) for key in envelope if key not in ENVELOPE_KEYS)
        claims = envelope.get("claims")
        why = None
        if stray:
            why = f"an envelope has no {', '.join(stray)} (it has {', '.join(ENVELOPE_KEYS)})"
        elif claims is not None and (not isinstance(claims, list) or not all(isinstance(z, str) for z in claims)):
            why = "claims is a list of zone names"
        if why is not None:
            # refused at the door, logged like any refusal — the stray keys by name only, never their values
            reply = M.refuse(envelope.get("id"), M.BAD_ARGS, why)
            self.gateway.audit.write(self.name, {k: v for k, v in envelope.items() if k in ENVELOPE_KEYS}, reply, False)
        else:
            try:
                reply = self.gateway.handle(self.name, envelope)
            except Exception as error:   # a gateway bug must not cut the link (that would stop everything)
                reply = M.refuse(envelope.get("id"), INTERNAL, f"the gateway failed: {type(error).__name__}: {error}")
                self.gateway.audit.write(self.name, envelope, reply, False)
                print(f"tcp_front: the gateway raised on a request of {self.name}: {reply['reason']}",
                      file=sys.stderr, flush=True)
        answer = {"op": "request"}
        if isinstance(envelope.get("module"), str):
            answer["module"] = envelope["module"]
        answer.update(reply)
        return answer

    _OPS = {"hello": _hello, "take_control": _take_control, "release_control": _release_control,
            "ping": _ping, "subscribe": _subscribe, "unsubscribe": _unsubscribe_op, "request": _request}

    # ------------------------------------------------------------ out
    def _queue(self, answer: dict) -> None:
        with self._wake:
            if not self._ended:
                self._out.append(("line", answer))
                self._wake.notify_all()

    def _push(self, item: dict, replace: bool = True) -> None:
        """A state for the client. Never blocks: called from the module's / the bus's threads.

        replace=False (the picture at subscribe) keeps a live state already pending: it is newer."""
        module = item.get("module")
        with self._wake:
            if self._ended or not self._subscribed:
                return
            if module not in self._states:
                self._out.append(("state", module))
            elif not replace:
                return
            self._states[module] = {"op": "state", **item}
            self._wake.notify_all()

    def _write(self) -> None:
        while True:
            with self._wake:
                while not self._out and not self._ended:
                    self._wake.wait()
                if self._ended:
                    return
                kind, payload = self._out.popleft()
                outgoing = self._states.pop(payload) if kind == "state" else payload
                self._sending = True
            try:
                try:
                    data = (json.dumps(outgoing, separators=(",", ":"), default=str) + "\n").encode("utf-8")
                except ValueError as error:   # not JSON (a circular value): skipped, said once on stderr
                    print(f"tcp_front: cannot encode a message for {self.name}: {error}", file=sys.stderr, flush=True)
                    continue
                try:
                    self.sock.sendall(data)
                except OSError as error:
                    self._end(f"cannot send ({error.strerror or type(error).__name__})")
                    return
            finally:
                with self._wake:
                    self._sending = False
                    self._wake.notify_all()
