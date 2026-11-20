#!/usr/bin/env python3
"""A walk-through of the module interface's middle layer in ONE process: 15 short scenes, no ROS,
no hardware, no listening socket (docs/module_interface.md).

    python3 tools/module_demo.py

Scenes 1–11 = the workflow repo's harness_skeleton/demo.py, ported: two simulated module nodes
(tools/module_sim: a Bunny, an arm) and the gateway talk over LocalBus. Scenes 12–15 = the twin's
door (src/railab_harness/railab_harness/tcp_front.py) on a socket.socketpair(): the key, a request
round trip, the pushed state, the controlling client's connection dropped. Every scene prints the
request, the reply, and the part of the module's state that changed, and checks it (✓ / ✗).
The periods run at a fifth of their proposed values and the simulated devices five times faster
than life, so the whole story takes a few seconds. Exit code 0 = every scene went as written.
"""

import itertools
import json
import os
import shutil
import socket
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "src", "railab_harness"), os.path.join(ROOT, "tools")]

from module_sim.arm_sim import Arm, SimArm            # noqa: E402
from module_sim.bunny_sim import Bunny, SimBunny      # noqa: E402
from railab_harness.bus import LocalBus               # noqa: E402
from railab_harness.gateway import Gateway            # noqa: E402
from railab_harness.tcp_front import TcpFront         # noqa: E402
from railab_harness.timing import Timing              # noqa: E402

TIMING = Timing().scaled(0.2)
SPEED = 5.0
TWIN, VIEWER = "twin", "viewer"
BUNNY, ARM = "bunny_1", "arm_1"
KEY = "walk-through-key"   # made up for the walk-through; the real key file is never read here


def do(action, **args):
    return {"do": action, "args": args}


def short(text, width=110):
    return text if len(text) <= width else text[:width - 3] + "…"


class Client:
    """A client's end of a socketpair: sends lines, keeps the answers and the pushed states apart."""

    def __init__(self, sock, keepalive_s=None):
        self.sock = sock
        self._cond = threading.Condition()
        self._send_lock = threading.Lock()
        self._done = threading.Event()
        self.answers, self.pushes = [], []
        self._threads = [threading.Thread(target=self._read, name="client-read", daemon=True)]
        if keepalive_s:   # a controlling client says it is alive between its requests
            self._threads.append(threading.Thread(target=self._keepalive, args=(keepalive_s,),
                                                  name="client-ping", daemon=True))
        for thread in self._threads:
            thread.start()

    def send(self, message):
        self.send_raw((json.dumps(message) + "\n").encode())

    def send_raw(self, data: bytes):
        with self._send_lock:
            self.sock.sendall(data)

    def ask(self, message, timeout=3.0):
        """Send one op; its answer (the first one with that op — for a request, that module and id)."""
        self.send(message)
        op = message["op"]
        return self.answer(lambda a: a.get("op") == op and (
            op != "request" or (a.get("module"), a.get("id")) == (message.get("module"), message.get("id"))), timeout)

    def answer(self, match, timeout=3.0):
        end = time.monotonic() + timeout
        with self._cond:
            while True:
                for i, a in enumerate(self.answers):
                    if match(a):
                        return self.answers.pop(i)
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)

    def saw(self, predicate) -> bool:
        with self._cond:
            return any(predicate(p) for p in self.pushes)

    def _read(self):
        buffer = b""
        try:
            while True:
                data = self.sock.recv(65536)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    message = json.loads(line)
                    with self._cond:
                        if message.get("op") == "state":
                            self.pushes.append(message)
                        elif message.get("op") != "ping":   # the keep-alive's answers
                            self.answers.append(message)
                        self._cond.notify_all()
        except OSError:
            pass
        finally:
            self._done.set()

    def _keepalive(self, period):
        while not self._done.wait(period):
            try:
                self.send({"op": "ping"})
            except OSError:
                return

    def close(self):
        """Hang up (no stop, no goodbye: what a crashed twin or a pulled cable looks like)."""
        self._done.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        for thread in self._threads:
            thread.join(2)
        self.sock.close()


class Walk:
    def __init__(self):
        self.tmp = tempfile.mkdtemp(prefix="module-demo-")
        self.log_path = os.path.join(self.tmp, "commands.jsonl")
        self.bus = LocalBus()
        self.bunny = Bunny(BUNNY, self.bus, TIMING, SimBunny(SPEED)).start()
        self.arm = Arm(ARM, self.bus, TIMING, SimArm(SPEED)).start()
        self.gateway = None
        self.front = None
        self.clients, self.doors = [], []
        self.stream = []      # everything the gateway's merged state stream delivered
        self.ids = itertools.count(1)
        self.wrong = []

    # ---------------------------------------------------------------- helpers
    def new_gateway(self):
        self.gateway = Gateway(self.bus, TIMING, self.log_path).start()
        self.gateway.subscribe(self.stream.append)

    def scene(self, number, title):
        print(f"\n{number:2}. {title}")

    def note(self, text):
        print(f"     {text}")

    def expect(self, ok, text):
        print(f"     {'✓' if ok else '✗'} {text}")
        if not ok:
            self.wrong.append(text)

    def ask(self, client, module, request, claims=None):
        rid = next(self.ids)
        envelope = {"module": module, "id": rid, **request}
        if claims:
            envelope["claims"] = claims
        reply = self.gateway.handle(client, envelope)
        what = (f"do {request['do']} {json.dumps(request.get('args', {}))}" if "do" in request
                else f"set {json.dumps(request['set'])}")
        what = what if len(what) <= 110 else what[:107] + "…"
        answer = "accepted" if reply["accepted"] else f"REFUSED {reply['code']}: {reply['reason']}"
        print(f"     {client} → {module} #{rid}  {what}" + (f"  claims {claims}" if claims else ""))
        print(f"     {' ' * len(client)} ← {answer}")
        return rid, reply

    def state(self, module):
        return self.gateway.states()[module]["state"]

    def show(self, module, *fields):
        s = self.state(module)
        parts = [f"mode {s['mode']}"] + [f"{f} = {json.dumps(s['values'][f])} ({s['sources'][f]})" for f in fields]
        if s["fault"]:
            parts.append("fault " + json.dumps(s["fault"]))
        if s["last"]:
            parts.append("last " + json.dumps(s["last"]))
        print(f"     state {module}: " + ", ".join(parts))

    def until(self, condition, timeout=8.0, ping=True):
        """Wait for a condition; meanwhile the twin keeps saying it is alive (unless told not to)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if ping:
                self.gateway.ping(TWIN)
            if condition():
                return True
            time.sleep(0.01)
        return False

    def ended(self, module, rid):
        return lambda: (self.state(module)["last"] or {}).get("id") == rid

    def was_busy_with(self, module, rid):
        return any(i["module"] == module and i["state"]["mode"] == "busy"
                   and (i["state"]["current"] or {}).get("id") == rid for i in self.stream)

    def long_path(self):
        """31 waypoints, 5 deg apart on joint 1, from where the arm is now (the first waypoint)."""
        here = list(self.state(ARM)["values"]["joints_deg"])
        return [[here[0] + 5.0 * k] + here[1:] for k in range(31)]

    def last_log(self):
        with open(self.log_path, encoding="utf-8") as log:
            return json.loads(log.readlines()[-1])

    # ---------------------------------------------------------------- helpers for the door (TCP)
    def connect(self, keepalive=True):
        """A new connection to the door over a socketpair: (the client's end, the door's Connection)."""
        mine, theirs = socket.socketpair()
        door = self.front.connection(theirs, "socketpair").start()
        client = Client(mine, TIMING.client_silent_s / 6 if keepalive else None)
        self.doors.append(door)
        self.clients.append(client)
        return client, door

    def wire(self, who, client, message, raw=None):
        """Send one line over TCP, print it (a key as ***) and its answer; the answer."""
        shown = json.dumps({k: ("***" if k == "key" else v) for k, v in message.items()}) if raw is None else repr(raw)
        if raw is None:
            answer = client.ask(message)
        else:
            client.send_raw(raw)
            answer = client.answer(lambda a: a.get("op") == "error")
        print(f"     {who} ⇒ {short(shown)}")
        if answer is None:
            text = "(no answer)"
        else:
            text = json.dumps({k: (sorted(v) if k == "modules" else v) for k, v in answer.items()})
        print(f"     {' ' * len(who)} ⇐ {short(text)}")
        return answer or {}

    def tcp_request(self, who, client, module, request):
        rid = next(self.ids)
        return rid, self.wire(who, client, {"op": "request", "module": module, "id": rid, **request})

    # ---------------------------------------------------------------- the story
    def run(self):
        self.scene(1, "hello: the gateway lists the modules from their own states (it started after them)")
        self.new_gateway()
        ok = self.until(lambda: all(self.gateway.states().get(m, {}).get("online") for m in (BUNNY, ARM)))
        hello = self.gateway.hello(TWIN)
        for name, module in sorted(hello["modules"].items()):
            d = module["describe"]
            self.note(f"{name}: {d['identity']['type']} {d['identity']['version']}, actions "
                      f"{sorted(d['actions'])}, zones {d['zones']}")
        self.expect(ok and sorted(hello["modules"]) == [ARM, BUNNY], "both modules known, nothing about them typed in the harness")

        self.scene(2, "a viewer may watch but not command")
        _, reply = self.ask(VIEWER, BUNNY, do("tare"))
        self.expect(reply.get("code") == "NO_LEASE", "refused NO_LEASE")
        self.expect(self.gateway.take_control(TWIN) and not self.gateway.take_control(VIEWER),
                    "the twin takes control; the viewer cannot take it too")

        self.scene(3, "the platform height is unknown (no limit switch): the module refuses to move it")
        self.show(BUNNY, "platform_mm")
        _, reply = self.ask(TWIN, BUNNY, do("platform_to", z_mm=60))
        tare, other = self.ask(TWIN, BUNNY, do("tare"))
        self.until(self.ended(BUNNY, tare))
        self.expect(reply.get("code") == "UNKNOWN_VALUE" and other["accepted"] and self.state(BUNNY)["mode"] == "ready",
                    "refused UNKNOWN_VALUE — by the module; it stays ready and an action that needs no height is taken")

        self.scene(4, "the twin measured tag 3 with the arm's camera and TELLS the module its height")
        _, reply = self.ask(TWIN, BUNNY, {"set": {"platform_mm": 87.3}})
        self.show(BUNNY, "platform_mm")
        s = self.state(BUNNY)
        self.expect(reply["accepted"] and s["values"]["platform_mm"] == 87.3 and s["sources"]["platform_mm"] == "told",
                    "accepted; platform_mm = 87.3, source told")

        self.scene(5, "now the move is taken: accepted at once, busy, then done — the result is in the state")
        rid, reply = self.ask(TWIN, BUNNY, do("platform_to", z_mm=60))
        done = self.until(self.ended(BUNNY, rid))
        self.show(BUNNY, "platform_mm")
        s = self.state(BUNNY)
        self.expect(reply["accepted"] and self.was_busy_with(BUNNY, rid), "accepted, and the stream showed mode busy with this request")
        self.expect(done and s["last"]["result"] == "done" and s["values"]["platform_mm"] == 60.0
                    and s["sources"]["platform_mm"] == "estimated", "done; platform_mm = 60.0, source estimated (its own step count)")

        self.scene(6, "out of range: the gateway refuses from the module's own describe, the module never sees it")
        _, reply = self.ask(TWIN, BUNNY, do("platform_to", z_mm=300))
        self.expect(reply.get("code") == "OUT_OF_RANGE" and self.last_log()["forwarded"] is False,
                    "refused OUT_OF_RANGE; the command log says forwarded = false")
        _, reply = self.ask(TWIN, BUNNY, do("tare"), claims=7)
        self.expect(reply.get("code") == "BAD_ARGS" and self.last_log()["forwarded"] is False,
                    "claims that are not a list of zone names: refused BAD_ARGS by the gateway itself, never forwarded")

        self.scene(7, "interlock by zone: the housing swings, so an arm path through its swing is refused — and the other way round")
        housing, reply = self.ask(TWIN, BUNNY, do("housing_to", target="open"))
        self.note(f"zones held: {json.dumps(self.gateway.zones.held())}")
        path = [[0, 0, 0, 0, 0, 0], [10, 20, -30, 0, 40, 0], [20, 35, -50, 0, 60, 0], [0, 0, 0, 0, 0, 0]]   # from the arm
        _, refused = self.ask(TWIN, ARM, do("follow_path", waypoints_deg=path), claims=[f"{BUNNY}/housing_swing"])
        self.expect(reply["accepted"] and refused.get("code") == "INTERLOCK", "the arm is refused INTERLOCK while the housing moves")
        self.until(self.ended(BUNNY, housing))
        self.show(BUNNY, "housing")
        moving, reply = self.ask(TWIN, ARM, do("follow_path", waypoints_deg=path), claims=[f"{BUNNY}/housing_swing"])
        _, refused = self.ask(TWIN, BUNNY, do("housing_to", target="closed"))
        self.expect(reply["accepted"] and refused.get("code") == "INTERLOCK",
                    "the housing ended → the arm is accepted; now the housing is refused INTERLOCK while the arm is in its swing")
        self.until(self.ended(ARM, moving))
        self.expect(self.gateway.zones.held() == {}, "the arm's path ended → no zone held (freed by the modules' states, no timer)")

        self.scene(8, "stop pre-empts a running action — and anyone may send it")
        self.ask(TWIN, BUNNY, {"set": {"pan_loaded": True, "powder_id": "9618329212"}})
        weigh, _ = self.ask(TWIN, BUNNY, do("weigh_to", target_g=5.0))
        self.until(lambda: ((self.state(BUNNY)["current"] or {}).get("progress") or 0) >= 0.2)
        self.note(f"running: {json.dumps(self.state(BUNNY)['current'])}")
        _, reply = self.ask(VIEWER, BUNNY, do("stop"))
        stopped = self.until(self.ended(BUNNY, weigh))
        self.show(BUNNY, "weight_g")
        last = self.state(BUNNY)["last"]
        self.expect(reply["accepted"] and stopped and last["result"] == "stopped" and "stop #" in last.get("reason", ""),
                    "the viewer's stop is accepted; weigh_to ended stopped, the dose so far is in weight_g")

        self.scene(9, "a fault latches until it is cleared")
        self.bunny.device.jam = True
        self.note("(the simulated housing motor will stall on its next move)")
        self.ask(TWIN, BUNNY, do("housing_to", target="closed"))
        self.until(lambda: self.state(BUNNY)["mode"] == "fault")
        self.show(BUNNY, "housing")
        _, refused = self.ask(TWIN, BUNNY, do("tare"))
        s = self.state(BUNNY)
        self.expect(s["mode"] == "fault" and s["fault"]["code"] == "HOUSING_STALL" and s["last"]["result"] == "failed"
                    and refused.get("code") == "FAULT", "mode fault with its code; the next action is refused FAULT")
        _, reply = self.ask(TWIN, BUNNY, do("clear_fault"))
        s = self.state(BUNNY)
        self.expect(reply["accepted"] and s["mode"] == "ready" and s["fault"] is None, "clear_fault → ready, fault null")

        self.scene(10, "the harness dies: its beat stops, and the module goes safe BY ITSELF")
        weigh, _ = self.ask(TWIN, BUNNY, do("weigh_to", target_g=50.0))
        self.until(lambda: self.state(BUNNY)["mode"] == "busy")
        halts, died = self.bunny.device.halts, time.monotonic()
        self.gateway.close()
        self.note("(the gateway is closed: no beat, nobody to send a stop)")
        safe = self.until(lambda: (self.bunny.state()["last"] or {}).get("id") == weigh, ping=False)
        last = self.bunny.state()["last"]
        self.note(f"bunny_1 by itself after {time.monotonic() - died:.2f} s (module-silent limit {TIMING.module_silent_s:.2f} s): "
                  f"last {json.dumps(last)}")
        self.expect(safe and last["result"] == "stopped" and last.get("reason") == "harness silent"
                    and self.bunny.device.halts > halts, "weigh_to ended stopped, reason \"harness silent\"; the motors were cut")
        started = time.monotonic()
        self.new_gateway()
        back = self.until(lambda: all(self.gateway.states().get(m, {}).get("online") for m in (BUNNY, ARM)), ping=False)
        self.expect(back and sorted(self.gateway.hello(TWIN)["modules"]) == [ARM, BUNNY],
                    f"a NEW gateway knows both modules again after {time.monotonic() - started:.2f} s — from their states alone")

        self.scene(11, "the controlling client goes silent: the gateway drops the lease and stops the actions it started")
        self.gateway.take_control(TWIN)
        moving, a = self.ask(TWIN, ARM, do("follow_path", waypoints_deg=self.long_path()))
        weigh, b = self.ask(TWIN, BUNNY, do("weigh_to", target_g=50.0))
        self.until(lambda: self.state(ARM)["mode"] == "busy" and self.state(BUNNY)["mode"] == "busy")
        self.note("(the twin stops talking: no request, no ping)")
        silent = time.monotonic()
        stopped = self.until(lambda: self.gateway.lease.holder is None and self.ended(ARM, moving)()
                             and self.ended(BUNNY, weigh)(), ping=False)
        self.note(f"after {time.monotonic() - silent:.2f} s (client-silent limit {TIMING.client_silent_s:.2f} s):")
        self.show(ARM)
        self.show(BUNNY)
        lasts = [self.state(m)["last"] for m in (ARM, BUNNY)]
        self.expect(a["accepted"] and b["accepted"] and stopped and all(
            l["result"] == "stopped" and "client twin silent" in l.get("reason", "") for l in lasts),
            "both actions ended stopped (two modules were busy at once: no global job slot); the lease is free")
        self.gateway.join_stops(2)   # each stop goes out from a thread of its own (the beat never waits for one)
        self.expect(self.last_log()["client"] == "harness" and self.last_log()["request"]["args"].get("scope") == "own",
                    "the harness's own stops (scope \"own\": only what this layer started) are in the command log too")

        # ------------------------------------------------------------ the twin's door: TCP (tcp_front.py)
        self.scene(12, "the twin's door over TCP (here a socketpair): the key decides who may take control")
        self.front = TcpFront(self.gateway, read_key=lambda: KEY)
        viewer, _ = self.connect(keepalive=False)
        seen = self.wire("viewer", viewer, {"op": "hello", "client": "viewer"})
        self.wire("viewer", viewer, {"op": "subscribe"})
        picture = self.until(lambda: viewer.saw(lambda p: p["module"] == ARM) and viewer.saw(lambda p: p["module"] == BUNNY),
                             timeout=2, ping=False)
        twin, door = self.connect()
        wrong = self.wire("twin", twin, {"op": "hello", "client": "twin", "key": "not-the-key"})
        refused = self.wire("twin", twin, {"op": "take_control"})
        right = self.wire("twin", twin, {"op": "hello", "client": "twin", "key": KEY})
        took = self.wire("twin", twin, {"op": "take_control"})
        self.wire("twin", twin, {"op": "subscribe"})
        self.expect(seen.get("access") == "view" and "warning" not in seen and sorted(seen.get("modules", {})) == [ARM, BUNNY],
                    "a hello without a key: access view, both modules listed (from their own states)")
        self.expect(wrong.get("access") == "view" and wrong.get("warning") == "wrong key"
                    and refused.get("ok") is False and refused.get("code") == "NO_LEASE",
                    "a wrong key: access view, warning \"wrong key\"; take_control refused NO_LEASE")
        self.expect(right.get("access") == "control" and took.get("ok") is True
                    and self.gateway.lease.holder == door.name == right.get("client"),
                    f"the right key: access control; take_control taken — the lease is held by {door.name}")
        with open(self.log_path, encoding="utf-8") as log:
            logged_text = log.read()
        self.expect(KEY not in json.dumps([seen, wrong, right]) and KEY not in logged_text,
                    "the key is in no answer and not in the command log")

        self.scene(13, "a request round trip over TCP: the contract's envelope in, the gateway's reply out, the result in the pushed state")
        rid, reply = self.tcp_request("twin", twin, BUNNY, do("platform_to", z_mm=20))
        done = self.until(lambda: twin.saw(lambda p: p["module"] == BUNNY and (p["state"]["last"] or {}).get("id") == rid
                                           and p["state"]["last"]["result"] == "done"), ping=False)
        self.show(BUNNY, "platform_mm")
        self.expect(reply.get("accepted") is True and (reply.get("module"), reply.get("id")) == (BUNNY, rid) and done
                    and self.state(BUNNY)["values"]["platform_mm"] == 20.0,
                    "accepted (the answer names module and id); the pushed state showed it done; platform_mm = 20.0")
        _, out = self.tcp_request("twin", twin, BUNNY, do("platform_to", z_mm=300))
        self.expect(out.get("code") == "OUT_OF_RANGE" and self.last_log()["forwarded"] is False,
                    "out of range: refused OUT_OF_RANGE by the gateway, as in scene 6")
        error = self.wire("twin", twin, None, raw=b"this is not JSON\n")
        _, stray = self.tcp_request("twin", twin, BUNNY, {**do("tare"), "priority": 1})
        logged = self.last_log()
        self.expect(error.get("code") == "BAD_ARGS" and stray.get("code") == "BAD_ARGS" and logged["forwarded"] is False
                    and "priority" not in logged["request"],
                    "a line that is not JSON: an error, the link stays; a key the envelope does not have: refused at the door, never forwarded")

        self.scene(14, "a viewer sees every module's state pushed, without asking — and cannot command")
        busy = viewer.saw(lambda p: p["module"] == BUNNY and (p["state"]["current"] or {}).get("id") == rid)
        ended = viewer.saw(lambda p: p["module"] == BUNNY and (p["state"]["last"] or {}).get("id") == rid)
        self.note(f"the viewer has {len(viewer.pushes)} pushed states")
        _, tried = self.tcp_request("viewer", viewer, BUNNY, do("tare"))
        self.expect(picture, "on subscribe it got the picture so far: a state of each module")
        self.expect(busy and ended, f"it saw request #{rid} busy and then its result, pushed")
        self.expect(tried.get("code") == "NO_LEASE", "its own request is refused NO_LEASE")

        self.scene(15, "the controlling client's connection drops: the door stops this layer's actions at once and frees the lease")
        moving, a = self.tcp_request("twin", twin, ARM, do("follow_path", waypoints_deg=self.long_path()))
        weigh, b = self.tcp_request("twin", twin, BUNNY, do("weigh_to", target_g=90.0))
        self.until(lambda: self.state(ARM)["mode"] == "busy" and self.state(BUNNY)["mode"] == "busy", ping=False)
        other, other_door = self.connect(keepalive=False)
        self.wire("viewer 2", other, {"op": "hello", "client": "viewer-2"})
        other.close()
        hung_up = self.until(lambda: other_door.ended, timeout=2, ping=False)
        self.expect(hung_up and self.state(ARM)["mode"] == "busy" and self.state(BUNNY)["mode"] == "busy"
                    and self.gateway.lease.holder == door.name,
                    "a viewer hangs up meanwhile: its link ends, nothing is stopped, the lease stays the twin's")
        self.note("(the twin's socket closes: no stop sent, no silence waited for)")
        gone = time.monotonic()
        twin.close()
        stopped = self.until(lambda: self.gateway.lease.holder is None and self.ended(ARM, moving)()
                             and self.ended(BUNNY, weigh)(), ping=False)
        took_s = time.monotonic() - gone
        self.note(f"after {took_s:.2f} s (client-silent limit {TIMING.client_silent_s:.2f} s): the door's link ended: "
                  f"{door.ended_why!r}")
        self.show(ARM)
        self.show(BUNNY)
        lasts = [self.state(m)["last"] for m in (ARM, BUNNY)]
        self.expect(a.get("accepted") and b.get("accepted") and stopped and took_s < TIMING.client_silent_s and all(
            l["result"] == "stopped" and "disconnected" in l.get("reason", "") for l in lasts),
            "both actions ended stopped, reason \"…disconnected…\", before any silence limit; the lease is free")
        pushed = self.until(lambda: all(viewer.saw(lambda p, m=m, i=i: p["module"] == m and (p["state"]["last"] or {}).get("id") == i)
                                        for m, i in ((ARM, moving), (BUNNY, weigh))), timeout=2, ping=False)
        self.gateway.join_stops(2)
        self.expect(pushed and door.ended and self.last_log()["client"] == "harness",
                    "the viewer was pushed both results; the stops are in the command log as the harness's")

    def close(self):
        for client in self.clients:
            client.close()
        if self.front is not None:
            self.front.close()
        for door in self.doors:
            door.join(2)
        if self.gateway is not None:
            self.gateway.close()
        self.bunny.close()
        self.arm.close()
        shutil.rmtree(self.tmp, ignore_errors=True)


def main():
    walk = Walk()
    try:
        walk.run()
    finally:
        walk.close()
    time.sleep(0.05)
    left = [t.name for t in threading.enumerate() if t is not threading.main_thread()]
    walk.expect(not left, f"no thread left running{': ' + ', '.join(left) if left else ''}")
    print(f"\n{'ALL SCENES AS WRITTEN' if not walk.wrong else 'NOT AS WRITTEN: ' + '; '.join(walk.wrong)}")
    return 1 if walk.wrong else 0


if __name__ == "__main__":
    sys.exit(main())
