#!/usr/bin/env python3
"""Checks the Bunny's and the camera's adapter nodes (src/harness_modules/harness_modules/bunny_module.py,
camera_module.py) on a LocalBus with FAKE links — direct calls, no ROS, no socket, no hardware.

    ~/.venvs/railab-wf/bin/python tools/module_check_bunny.py        # ~6 s

The fakes stand for today's bunny_node + the Bunny's own server (what /bunny/status and /bunny/* answer) and for
tag_node (one TagArray per frame). One line per check; exit 0 only if every check holds.

NOT CHECKED HERE (no ROS on the machine it was written on): RosBunnyLink, RosCameraLink and both main()s — the
service calls, the status subscription, the TagArray conversion, the parameters, the RosBus they run on.
"""

import itertools
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ("src/railab_harness", "src/harness_modules"):
    sys.path.insert(0, str(ROOT / sub))

from railab_harness import messages as M  # noqa: E402
from railab_harness.bus import LocalBus  # noqa: E402
from railab_harness.gateway import Gateway  # noqa: E402
from railab_harness.timing import DEFAULT  # noqa: E402
from harness_modules import bunny_module as B  # noqa: E402
from harness_modules import camera_module as C  # noqa: E402

POWDER = "9618329212"   # the demo's NaCl barcode
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(("ok    " if ok else "FAIL  ") + name + ("" if ok or not detail else f"  — {detail}"))


def until(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


# ----------------------------------------------------------------------------- the fakes
class FakeBunny(B.BunnyLink):
    """bunny_node + the Bunny's harness_server, as far as the adapter can see them."""

    def __init__(self):
        self.listener = None
        self.counter = 0              # the server's platform counter (its state["platform"])
        self.housing_word = "closed"
        self.weight = 0.0
        self.powder = None            # the Bunny's current powder (core.current_powder_id)
        self.scan = None              # its last_scan
        self.sent = []                # every call that reached the Bunny (reads of the counter excluded)
        self.aborts = 0
        self.housing_fails = None     # words: the next housing swing fails with them
        self.cut_next_move = False    # the next platform move: the connection drops part way
        self.weigh = {"success": True, "outcome": "finished", "actual_g": 1.0, "message": '{"outcome": "finished"}'}
        self.hold = None              # threading.Event: a weigh_to runs until it is set (by abort)

    # what bunny_node publishes on /bunny/status
    def on_status(self, callback):
        self.listener = callback

    def status(self, **override):
        status = {"online": True, "unit": "fake-bunny", "stm32": True, "housing": self.housing_word,
                  "weight_g": self.weight, "busy": None, "load_open": False, "powder_id": self.powder,
                  "powder_name": "", "last_scan": self.scan}
        status.update(override)
        self.listener(status)

    def offline(self, why):
        self.listener({"online": False, "error": why})

    # its services
    def housing(self, verb):
        self.sent.append(("housing", verb))
        if self.housing_fails:
            words, self.housing_fails = self.housing_fails, None
            self.housing_word = "between"
            return B.Pending.now(error=B.BunnyError(words))
        self.housing_word = {"housing_open": "open", "housing_half": "half", "housing_close": "closed"}[verb]
        return B.Pending.now({"housing": self.housing_word, "housing_frac": 1.0})

    def platform(self, steps):
        if steps == 0:
            return B.Pending.now(self.counter)
        self.sent.append(("platform", steps))
        if self.cut_next_move:
            self.cut_next_move = False
            self.counter += steps // 2           # it went part of the way; the reply never says how far
            return B.Pending.now(error=B.BunnyError("Bunny closed the connection"))
        self.counter += steps
        return B.Pending.now(self.counter)

    def tare(self):
        self.sent.append(("tare",))
        self.weight = 0.0
        return B.Pending.now({"g": 0.0})

    def weigh_to(self, target_g, powder_id):
        self.sent.append(("weigh_to", target_g, powder_id))
        pending = B.Pending()

        def dose():
            if self.hold is not None:
                aborted = self.hold.wait(10)
                reply = ({"success": False, "outcome": "stopped", "actual_g": 0.42, "message": "{}"} if aborted
                         else {"success": False, "outcome": "error", "actual_g": 0.0, "message": "never aborted"})
            else:
                reply = dict(self.weigh)
            if powder_id:
                self.powder = powder_id
            pending.finish(reply)
        threading.Thread(target=dose, daemon=True).start()
        return pending

    def dispense_volume(self, volume_ml):
        self.sent.append(("dispense_volume", volume_ml))
        return B.Pending.now({"success": True, "steps": 18000, "steps_planned": 18000, "actual_ml": volume_ml,
                              "message": "{}"})

    def abort(self):
        self.aborts += 1
        if self.hold is not None:
            self.hold.set()


class FakeCamera(C.CameraLink):
    """tag_node: one TagArray per frame; the link keeps the newest."""

    def __init__(self):
        self._newest = None
        self.frames = 0

    def frame(self, tag_id=3):
        tag = {"family": "tag36h11", "id": tag_id, "frame": "gripper_cam_optical",
               "position_mm": [1.5, -2.0, 180.0], "orientation_xyzw": [0, 0, 0, 1], "size_mm": 24.0}
        self._newest = ({"frame": "gripper_cam_optical", "tags": [tag]}, time.monotonic())
        self.frames += 1

    def newest(self):
        if self._newest is None:
            return None, None
        return self._newest[0], time.monotonic() - self._newest[1]


# ----------------------------------------------------------------------------- the walk
def main():
    ids = itertools.count(1)
    bus = LocalBus()
    gateway = Gateway(bus).start()                 # the harness: it beats; its hello reads the describes
    camera_states = []
    bus.on_state(lambda s: camera_states.append(time.monotonic()) if s["module"] == "camera_1" else None)
    logs = []
    fake = FakeBunny()
    # no watch thread: the walk calls watch_once() itself, and every call reads the counter (poll_s 0)
    bunny = B.BunnyModule("bunny_1", bus, fake, log=logs.append, watch_s=None, poll_s=0.0).start()
    cam_link = FakeCamera()
    camera = C.CameraModule("camera_1", bus, cam_link).start()

    def do(module, action, **args):
        rid = next(ids)
        return rid, bus.call(module, {"id": rid, "do": action, "args": args}, 1.0)

    def tell(module, **fields):
        return bus.call(module, {"id": next(ids), "set": fields}, 1.0)

    def last(module, rid, timeout=3.0):
        until(lambda: (module.state()["last"] or {}).get("id") == rid, timeout)
        return module.state()["last"] or {}

    def value(field):
        state = bunny.state()
        return state["values"][field], state["sources"][field]

    def refused(reply, code):
        return reply.get("accepted") is False and reply.get("code") == code

    # -- describe
    hello = gateway.hello("check")
    d = (hello["modules"].get("bunny_1") or {}).get("describe") or {}
    dc = (hello["modules"].get("camera_1") or {}).get("describe") or {}
    check("describe: bunny_1 announces its actions, fields and zones; the gateway's hello lists both modules",
          set(d.get("actions", {})) == {"housing_to", "platform_to", "tare", "weigh_to", "dispense_volume", "stop",
                                        "clear_fault"}
          and {f for f, s in d.get("fields", {}).items() if s.get("writable")} == {"platform_mm", "pan_loaded",
                                                                                  "powder_id"}
          and d["fields"]["housing"]["values"] == B.HOUSING_WORDS
          and d.get("zones") == ["housing_swing", "platform_travel"]
          and d["actions"]["platform_to"]["uses"] == ["platform_travel"]
          and set(dc.get("actions", {})) == {"stop", "clear_fault"} and set(dc.get("fields", {})) == {"tags", "frame_age_s"},
          str(d)[:300])

    state = bunny.state()
    check("before any /bunny/status: not_ready, every value unknown",
          state["mode"] == "not_ready" and set(state["values"].values()) == {M.UNKNOWN}, str(state["values"]))

    fake.status()
    check("the Bunny online: ready; housing and weight sensed; platform_mm still unknown",
          bunny.state()["mode"] == "ready" and value("housing") == ("closed", "sensed")
          and value("weight_g") == (0.0, "sensed") and value("platform_mm") == (M.UNKNOWN, M.UNKNOWN))

    # -- the platform
    _, reply = do("bunny_1", "platform_to", z_mm=50)
    check("platform_to while the height is unknown → UNKNOWN_VALUE, the module stays ready",
          refused(reply, M.UNKNOWN_VALUE) and bunny.state()["mode"] == "ready", str(reply))

    reply = tell("bunny_1", platform_mm=40.0)
    check("set platform_mm before the Bunny's step counter was read → NOT_READY", refused(reply, M.NOT_READY), str(reply))

    bunny.watch_once()                             # reads the counter (the fake starts at 0)
    reply = tell("bunny_1", platform_mm=40.0)
    rid, reply2 = do("bunny_1", "platform_to", z_mm=60)
    done = last(bunny, rid)
    check("told 40 mm → platform_to 60 → +1000 steps sent, done, 60.0 mm estimated",
          reply.get("accepted") and reply2.get("accepted") and done.get("result") == "done"
          and fake.sent[-1] == ("platform", 1000) and value("platform_mm") == (60.0, "estimated"),
          f"{reply} {reply2} {done} {fake.sent[-1:]} {value('platform_mm')}")

    sent = len(fake.sent)
    _, reply = do("bunny_1", "platform_to", z_mm=120)
    gateway.take_control("check")
    reply2 = gateway.handle("check", {"module": "bunny_1", "id": next(ids), "do": "platform_to", "args": {"z_mm": 120}})
    gateway.release_control("check")
    reply3 = tell("bunny_1", platform_mm=-5)
    check("out of range: platform_to 120 mm and set −5 mm → OUT_OF_RANGE (module and gateway), nothing moved",
          refused(reply, M.OUT_OF_RANGE) and refused(reply2, M.OUT_OF_RANGE) and refused(reply3, M.OUT_OF_RANGE)
          and len(fake.sent) == sent and value("platform_mm") == (60.0, "estimated"), f"{reply} {reply2} {reply3}")

    fake.cut_next_move = True
    rid, reply = do("bunny_1", "platform_to", z_mm=30)
    done = last(bunny, rid)
    _, reply2 = do("bunny_1", "platform_to", z_mm=40)
    check("an interrupted move (the connection drops part way) → failed, platform_mm unknown, platform_to refused",
          reply.get("accepted") and done.get("result") == "failed" and "closed the connection" in done.get("reason", "")
          and value("platform_mm") == (M.UNKNOWN, M.UNKNOWN) and refused(reply2, M.UNKNOWN_VALUE), f"{done} {reply2}")

    bunny.watch_once()                             # anchors the counter again
    tell("bunny_1", platform_mm=30.0)
    fake.counter += 250                            # another client of the Bunny's server moved the platform
    bunny.watch_once()
    check("the Bunny's counter changes with no command of ours → platform_mm unknown",
          value("platform_mm") == (M.UNKNOWN, M.UNKNOWN) and any("no move of ours" in line for line in logs),
          f"{value('platform_mm')} {logs[-1:]}")

    # -- housing
    fake.status()
    sent = len(fake.sent)
    rid, reply = do("bunny_1", "housing_to", target="closed")
    done = last(bunny, rid)
    rid2, _ = do("bunny_1", "housing_to", target="open")
    done2 = last(bunny, rid2)
    check("housing_to the state it reports → done at once, nothing sent; to open → housing_open sent, open sensed",
          done.get("result") == "done" and fake.sent[sent:] == [("housing", "housing_open")]
          and done2.get("result") == "done" and value("housing") == ("open", "sensed"), f"{done} {fake.sent[sent:]}")

    words = "housing did not close: the motor stalled (see the Bunny log)"
    fake.housing_fails = words
    rid, _ = do("bunny_1", "housing_to", target="closed")
    done = last(bunny, rid)
    state = bunny.state()
    _, reply = do("bunny_1", "tare")
    _, reply2 = do("bunny_1", "clear_fault")
    fake.status()
    check("a failed housing swing → failed + a LATCHED fault with the Bunny's words; only clear_fault frees it",
          done.get("result") == "failed" and state["mode"] == "fault" and state["fault"] == {
              "code": B.HOUSING_FAILED, "reason": words} and refused(reply, M.FAULT) and reply2.get("accepted")
          and bunny.state()["mode"] == "ready" and value("housing") == ("between", "sensed"), f"{done} {state['fault']}")

    # -- weighing
    _, reply = do("bunny_1", "weigh_to", target_g=1.0)
    reply2 = tell("bunny_1", powder_id="NaCl")
    reply3 = tell("bunny_1", pan_loaded=True, powder_id=POWDER)
    rid, _ = do("bunny_1", "weigh_to", target_g=1.0)
    done = last(bunny, rid)
    check("weigh_to: refused UNKNOWN_VALUE until pan_loaded is told; a name is no barcode (BAD_ARGS); "
          "the told powder_id rides on WeighTo",
          refused(reply, M.UNKNOWN_VALUE) and refused(reply2, M.BAD_ARGS) and reply3.get("accepted")
          and done.get("result") == "done" and fake.sent[-1] == ("weigh_to", 1.0, POWDER)
          and value("weight_g") == (1.0, "sensed"), f"{reply} {reply2} {done} {fake.sent[-1:]}")

    fake.weigh = {"success": False, "outcome": "stopped", "actual_g": 0.3, "message": "{}"}
    rid, _ = do("bunny_1", "weigh_to", target_g=1.0)
    done = last(bunny, rid)
    fake.weigh = {"success": False, "outcome": "error", "actual_g": 0.0,
                  "message": "powder id 9618329212 not in the database"}
    rid2, _ = do("bunny_1", "weigh_to", target_g=1.0)
    done2 = last(bunny, rid2)
    check("weigh_to outcomes: 'stopped' (on the Bunny) → stopped; 'error' → failed with the Bunny's words, no fault",
          done.get("result") == "stopped" and "stopped on the Bunny" in done.get("reason", "")
          and done2.get("result") == "failed" and "not in the database" in done2.get("reason", "")
          and bunny.state()["mode"] == "ready", f"{done} {done2}")

    fake.weigh = {"success": True, "outcome": "finished", "actual_g": 1.0, "message": "{}"}
    fake.hold = threading.Event()
    aborts = fake.aborts
    rid, _ = do("bunny_1", "weigh_to", target_g=2.0)
    until(lambda: bunny.state()["mode"] == "busy")
    t0 = time.monotonic()
    _, reply = do("bunny_1", "stop")
    answered_s = time.monotonic() - t0
    done = last(bunny, rid)
    fake.hold = None
    check("stop during a long weigh_to: answered at once, the abort reaches the Bunny, the dose ends stopped",
          reply.get("accepted") and answered_s < 0.2 and fake.aborts > aborts and done.get("result") == "stopped"
          and done.get("reason", "").startswith(f"stop #") and "the Bunny: stopped" in done.get("reason", ""),
          f"{answered_s:.3f} s {done}")

    fake.scan = {"code": "4006381333931", "name": "KCl"}
    fake.status()
    check("a bottle scanned on the Bunny's own reader → powder_id sensed",
          value("powder_id") == ("4006381333931", "sensed"), str(value("powder_id")))

    # -- offline
    bunny.watch_once()
    tell("bunny_1", platform_mm=30.0, pan_loaded=True, powder_id=POWDER)
    fake.offline("[Errno 111] Connection refused")
    state = bunny.state()
    _, reply = do("bunny_1", "tare")
    fake.powder = POWDER                           # the Bunny still holds the powder of its last dose
    fake.status()
    bunny.watch_once()
    state2 = bunny.state()
    tell("bunny_1", pan_loaded=True)
    _, reply2 = do("bunny_1", "weigh_to", target_g=1.0)
    check("the Bunny offline → not_ready, every value unknown; back online → ready but still unknown "
          "(platform_mm, the told facts)",
          state["mode"] == "not_ready" and set(state["values"].values()) == {M.UNKNOWN} and refused(reply, M.NOT_READY)
          and state2["mode"] == "ready" and state2["values"]["platform_mm"] == M.UNKNOWN
          and state2["values"]["powder_id"] == M.UNKNOWN, f"{state['values']} {state2['values']}")
    check("powder_id unknown while the Bunny holds a powder → weigh_to refused UNKNOWN_VALUE (it would not measure fresh)",
          refused(reply2, M.UNKNOWN_VALUE) and POWDER in reply2.get("reason", ""), str(reply2))

    bunny.stale_s = 0.2
    time.sleep(0.3)
    bunny.watch_once()
    stale_state = bunny.state()
    bunny.stale_s = B.STATUS_STALE_S
    fake.status()
    check("no /bunny/status for stale_s (bunny_node gone) → not_ready, weight_g unknown",
          stale_state["mode"] == "not_ready" and stale_state["values"]["weight_g"] == M.UNKNOWN, str(stale_state["mode"]))

    # -- the beat stops
    timing = DEFAULT.scaled(0.1)                   # goes safe after 0.3 s without a beat
    bus3, fake3 = LocalBus(), FakeBunny()
    idle = B.BunnyModule("bunny_8", bus3, fake3, timing=timing, watch_s=None).start()
    fake3.status()
    time.sleep(3 * timing.module_silent_s)         # no beat ever: the automatic safe state, nothing of ours runs
    silent_aborts = fake3.aborts
    reply = bus3.call("bunny_8", {"id": 1, "do": "stop", "args": {}}, 1.0)
    stop_aborts = fake3.aborts
    idle.close()
    check("idle and no beat (or closed): no abort — a dose another client started is not touched; "
          "a stop REQUEST aborts the Bunny whoever doses",
          silent_aborts == 0 and reply.get("accepted") and stop_aborts == 1 and fake3.aborts == 1,
          f"aborts: silent {silent_aborts}, after the stop {stop_aborts}, after close {fake3.aborts}")

    bus2, fake2 = LocalBus(), FakeBunny()
    lone = B.BunnyModule("bunny_9", bus2, fake2, timing=timing, watch_s=None).start()
    beating = threading.Event()
    beating.set()

    def beat():
        while beating.is_set():
            bus2.beat()
            time.sleep(timing.beat_s)
    threading.Thread(target=beat, daemon=True).start()
    fake2.status()
    bus2.call("bunny_9", {"id": 1, "set": {"pan_loaded": True, "powder_id": POWDER}}, 1.0)
    fake2.hold = threading.Event()
    bus2.call("bunny_9", {"id": 2, "do": "weigh_to", "args": {"target_g": 1.0}}, 1.0)
    until(lambda: lone.state()["mode"] == "busy")
    beating.clear()                                # the harness falls silent
    until(lambda: (lone.state()["last"] or {}).get("id") == 2, 3.0)
    gone = lone.state()["last"] or {}
    reply = bus2.call("bunny_9", {"id": 3, "do": "tare", "args": {}}, 1.0)
    check("the harness's beat stops → the Bunny is aborted by itself, the dose ends stopped, actions refused NOT_READY",
          fake2.aborts >= 1 and gone.get("result") == "stopped" and "harness silent" in gone.get("reason", "")
          and refused(reply, M.NOT_READY), f"{gone} {reply}")
    lone.close()

    # -- the camera
    state = camera.state()
    check("camera before any frame: not_ready, tags and frame_age_s unknown",
          state["mode"] == "not_ready" and state["values"] == {"tags": M.UNKNOWN, "frame_age_s": M.UNKNOWN},
          str(state["values"]))
    feeding = threading.Event()
    feeding.set()

    def feed():
        while feeding.is_set():
            cam_link.frame()
            time.sleep(1 / 30)
    threading.Thread(target=feed, daemon=True).start()
    time.sleep(0.6)                                # settle on the stream
    t0, frames0 = time.monotonic(), cam_link.frames
    time.sleep(2.0)
    window = [t for t in camera_states if t >= t0]
    frames = cam_link.frames - frames0
    state = camera.state()
    check(f"camera at ~30 fps: ready, tags sensed; {len(window)} states in 2 s for {frames} frames "
          f"(one per {C.STATE_PERIOD_S} s, never one per frame)",
          state["mode"] == "ready" and state["sources"]["tags"] == "sensed" and state["values"]["tags"][0]["id"] == 3
          and frames >= 40 and len(window) <= 2.0 / C.STATE_PERIOD_S + 1, f"{len(window)} states {state['mode']}")
    feeding.clear()
    until(lambda: camera.state()["mode"] == "not_ready", C.FRESH_S + 2 * C.STATE_PERIOD_S + 1)
    state = camera.state()
    check("camera: frames stop → not_ready, tags unknown, frame_age_s still says how old the newest frame is",
          state["mode"] == "not_ready" and state["values"]["tags"] == M.UNKNOWN
          and isinstance(state["values"]["frame_age_s"], float) and state["values"]["frame_age_s"] > C.FRESH_S,
          str(state["values"]))

    # the link's conversion is plain Python: stand-in message objects in TagArray's shape
    from types import SimpleNamespace as S
    tag = S(id=3, family="tag36h11", size_m=0.024, center_px=[640.0, 360.0], corners_px=[1.0] * 8, size_px=88.4,
            pose=S(position=S(x=0.0015, y=-0.002, z=0.18), orientation=S(x=0.0, y=0.0, z=0.0, w=1.0)),
            distance_m=0.18, reproj_err_px=0.21, in_center_square=True, depth_m=0.181, depth_size_m=0.0241)
    frame = C.frame_from_msg(S(header=S(frame_id="gripper_cam_optical"), tags=[tag]))
    got = frame["tags"][0]
    check("TagArray → tags in mm (stand-in messages): pose 1.5 / −2.0 / 180.0 mm, size 24 mm, the optical frame",
          frame["frame"] == "gripper_cam_optical" and got["position_mm"] == [1.5, -2.0, 180.0]
          and got["size_mm"] == 24.0 and got["depth_mm"] == 181.0 and got["id"] == 3 and got["family"] == "tag36h11",
          str(got))

    camera.close()
    bunny.close()
    gateway.close()
    print(f"{sum(RESULTS)} / {len(RESULTS)} checks hold")
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
