#!/usr/bin/env python3
"""The arm's adapter node (src/harness_modules/harness_modules/arm_module.py) checked by direct calls: the Arm module on
a LocalBus, a FAKE link standing for the arm node (piper_node), the REAL planner (harness_planner.trajectory) on a
synthetic arm model. No ROS, no hardware, no socket, nothing started outside this process. One line per check; exit 0
only if every check holds.

    python3 tools/module_check_arm.py

Also checks ros_binding's JSON <-> message mapping with stand-in message objects (its rclpy part never runs here).
"""
import collections
import itertools
import math
import os
import struct
import sys
import threading
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "src", p) for p in ("railab_harness", "harness_modules", "harness_planner", "harness_arm")]

import numpy as np  # noqa: E402

from harness_arm.kinematics import PiperKinematics  # noqa: E402
from harness_modules import arm_module as A  # noqa: E402
from harness_planner.trajectory import Limits  # noqa: E402
from railab_harness import messages as M  # noqa: E402
from railab_harness import ros_binding as R  # noqa: E402
from railab_harness.bus import LocalBus  # noqa: E402
from railab_harness.timing import Timing  # noqa: E402

TIMING = Timing().scaled(0.2)          # beat 0.2 s; a module hearing none for 0.6 s goes safe
# A PiPER-like MDH chain (d, a, alpha, theta offset) - NOT the SDK's table (pyAgxArm is not on this machine): it only
# has to be a sound 6-joint arm for the planner's TCP checks. The joint limits are the real ones (harness_arm).
SYNTH_MDH = [(0.123, 0.0, 0.0, 0.0), (0.0, 0.0, -math.pi / 2, -3.0058), (0.0, 0.28503, 0.0, -1.7939),
             (0.25075, -0.02198, math.pi / 2, 0.0), (0.0, 0.0, -math.pi / 2, 0.0), (0.091, 0.0, math.pi / 2, 0.0)]
OFFSETS = [0.0, 0.0, 0.0, 0.0, math.radians(7.5), 0.0]   # a joint-5 zero offset, as calib/kinematics.yaml has
START_RAD = [0.2, 1.2, -1.0, 0.1, 0.3, -0.2]            # where the fake arm stands (controller joints)
GAP_MAX_MM = 89.6
CAP_RAD_S = [A.FOLLOW_VEL_CAP_RAD_S * A.FOLLOW_J1_FACTOR] + [A.FOLLOW_VEL_CAP_RAD_S] * 5   # at factor 2
RID = itertools.count(1)


# ====================================================================================================== the fake arm
class FakeGoal:
    """A trajectory played at `rate` x real time on the fake arm; cancel freezes it where it is."""

    def __init__(self, arm, traj, rate):
        self.arm, self.t, self.q, self.rate = arm, np.asarray(traj.t), np.asarray(traj.q), rate
        self._done, self._cancel = threading.Event(), threading.Event()
        self._progress, self._result = 0.0, None

    def start(self):
        threading.Thread(target=self._play, daemon=True).start()

    def _play(self):
        t0, T = time.monotonic(), float(self.t[-1])
        while True:
            tt = (time.monotonic() - t0) * self.rate
            if self._cancel.is_set():
                self._result = (False, f"cancelled at {tt:.1f} s of {T:.1f} s; stopped where it was")
                break
            if tt >= T:
                self.arm.set_q(self.q[-1])
                self._progress, self._result = 1.0, (True, f"done in {T:.1f} s")
                break
            self.arm.set_q([np.interp(tt, self.t, self.q[:, j]) for j in range(6)])
            self._progress = tt / T
            time.sleep(0.005)
        self.arm.goal_ended(self)
        self._done.set()

    def done(self):
        return self._done.is_set()

    def progress(self):
        return self._progress

    def result(self):
        return self._result

    def cancel(self):
        self._cancel.set()


class FakeArm(A.ArmLink):
    """The arm node as the adapter sees it, with the node's own refusals where they matter here."""

    def __init__(self):
        self.lock = threading.Lock()
        self.q, self.width_m = list(START_RAD), 0.040
        self.st = {"connected": True, "all_enabled": True, "teach_mode": False, "errors": [], "fault": "", "busy": False}
        self.age = 0.0          # how old every reading looks (raise it: the arm node fell silent)
        self.rate = 10.0        # trajectories play this many times faster than real time
        self.calls, self.trajs, self.goal = [], [], None

    def snapshot(self):
        with self.lock:
            return {"status": dict(self.st, errors=list(self.st["errors"]), age_s=self.age),
                    "joints": {"q": list(self.q), "age_s": self.age},
                    "gripper": {"present": True, "width_m": self.width_m, "age_s": self.age}}

    def set_q(self, q):
        with self.lock:
            self.q = [float(v) for v in q]

    def goal_ended(self, goal):
        with self.lock:
            if self.goal is goal:
                self.st["busy"] = False

    def follow(self, traj, label):
        self.calls.append("follow")
        with self.lock:
            st = self.st
            if st["busy"] or st["fault"] or not st["all_enabled"] or st["teach_mode"]:
                raise A.LinkError("rejected by the arm node (busy, fault, teach mode or not enabled)")
            if max(abs(a - b) for a, b in zip(traj.q[0], self.q)) > 0.2:     # the node's traj_start_tol_rad
                raise A.LinkError("refused: the trajectory starts away from the current joints")
            st["busy"] = True
            self.trajs.append(traj)
            goal = self.goal = FakeGoal(self, traj, self.rate)
        goal.start()
        return goal

    def _cancel_goal(self):
        goal = self.goal
        if goal is not None:
            goal.cancel()

    def halt(self):
        self.calls.append("halt")
        self._cancel_goal()

    def halt_all(self):
        self.calls.append("halt_all")
        self._cancel_goal()

    def _hold(self):
        with self.lock:
            bits = [e for e in self.st["errors"] if not A.JOINT_FAULT.match(e)]
            if bits:
                return False, "refused: error bits set: " + ",".join(bits)
            self.st.update(all_enabled=True, teach_mode=False, fault="")
            return True, "holding"

    def hold(self):
        self.calls.append("hold")
        return self._hold()

    def hold_soon(self):
        self.calls.append("hold_soon")
        self._hold()

    def release(self):
        self.calls.append("release")
        with self.lock:
            self.st["all_enabled"] = False
        return True, "all joints disabled (arm is limp)"

    def grip(self, gap_m, force_n, timeout_s):
        self.calls.append(("grip", round(gap_m, 4), force_n))
        time.sleep(0.05)
        with self.lock:
            self.width_m = gap_m
        return {"success": True, "stalled": False, "width_m": gap_m, "message": f"width {gap_m:.4f} m"}

    def grip_here(self, force_n):
        self.calls.append("grip_here")


class Beat:
    """The harness's heartbeat on the LocalBus; `on` cleared = the harness falls silent."""

    def __init__(self, bus):
        self.bus, self.on, self.end = bus, threading.Event(), threading.Event()
        self.on.set()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.end.wait(TIMING.beat_s):
            if self.on.is_set():
                self.bus.beat()


# ========================================================================================================== helpers
class Check(AssertionError):
    pass


def need(cond, words):
    if not cond:
        raise Check(words)


def wait_for(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return bool(cond())


def ask(arm, do=None, args=None, set_=None):
    rid = next(RID)
    request = {"id": rid, "set": set_} if set_ is not None else {"id": rid, "do": do, "args": args or {}}
    return rid, arm.command(request)


def last_of(arm, rid):
    last = arm.state()["last"]
    return last if last and last["id"] == rid else None


def ended(arm, rid, timeout=10.0):
    need(wait_for(lambda: last_of(arm, rid) is not None, timeout), f"request {rid} did not end within {timeout} s")
    return last_of(arm, rid)


def here(arm):
    return list(arm.state()["values"]["joints_deg"])


def moved(start, deltas):
    return [a + d for a, d in zip(start, deltas)]


def refused(reply, code, *words):
    need(not reply["accepted"], f"accepted, expected {code}")
    need(reply["code"] == code, f"refused {reply['code']} ({reply['reason']}), expected {code}")
    for w in words:
        need(w in reply["reason"], f"'{w}' not in the reason: {reply['reason']}")
    return reply["reason"]


def accepted(reply):
    need(reply["accepted"], f"refused {reply.get('code')}: {reply.get('reason')}")


def run_path(c, deltas, speeds=None):
    """A path from where the arm is by `deltas` (deg, one or more waypoints); waits for its end."""
    start = here(c.arm)
    waypoints = [start] + [moved(start, d) for d in deltas]
    args = {"waypoints_deg": waypoints}
    if speeds is not None:
        args["speeds"] = speeds
    rid, reply = ask(c.arm, "follow_path", args)
    accepted(reply)
    return rid, ended(c.arm, rid, 20.0)


# =========================================================================================================== checks
def c_describe(c):
    st = c.arm.state()
    d, v, s = st["describe"], st["values"], st["sources"]
    need(set(d["actions"]) == {"follow_path", "grip_to", "hold", "release", "calibrate_handeye", "calibrate_absolute",
                               "stop", "clear_fault"}, f"actions {sorted(d['actions'])}")
    need(set(d["fields"]) == {"joints_deg", "gap_mm", "held", "handeye_error_mm", "absolute_error_mm"},
         f"fields {sorted(d['fields'])}")
    need(d["zones"] == [] and d["identity"]["type"] == "piper", "zones / identity")
    need(st["mode"] == "ready", f"mode {st['mode']}")
    want = [math.degrees(q + o) for q, o in zip(START_RAD, OFFSETS)]
    need(max(abs(a - b) for a, b in zip(v["joints_deg"], want)) < 2e-3, f"joints_deg {v['joints_deg']} != {want}")
    need(v["held"] is True and v["gap_mm"] == 40.0, f"held {v['held']}, gap {v['gap_mm']}")
    need(s["joints_deg"] == s["gap_mm"] == s["held"] == "sensed", f"sources {s}")
    need(v["handeye_error_mm"] == v["absolute_error_mm"] == "unknown", "calibration errors not unknown")
    need(d["actions"]["grip_to"]["args"]["gap_mm"]["range"] == [0.0, GAP_MAX_MM], "gap range")
    return (f"8 actions, 5 fields, no zones; ready; joints_deg = controller + offsets (j5 {v['joints_deg'][4]:.2f} "
            f"deg), held, gap 40.0 mm sensed; calibration errors unknown")


def c_good_path(c):
    start = here(c.arm)
    q_before = list(c.fake.q)
    w0 = moved(start, [0, 0.3, 0, 0, 0, 0])                      # 0.3 deg off: within START_TOL_DEG
    w1 = moved(start, [10, 5, -8, 0, 6, 4])
    w2 = moved(start, [20, 8, -12, -5, 10, 0])
    n_traj = len(c.fake.trajs)
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [w0, w1, w2], "speeds": [1.0, 2.0]})
    accepted(reply)
    last = ended(c.arm, rid, 20.0)
    need(last["result"] == "done", f"last {last}")
    need(len(c.fake.trajs) == n_traj + 1, "no trajectory reached the arm node")
    traj = c.fake.trajs[-1]
    need(np.abs(np.asarray(traj.q[0]) - q_before).max() < 1e-9, "the first sample is not where the arm was")
    end = c.model.to_controller_rad(w2)
    need(np.abs(np.asarray(traj.q[-1]) - end).max() < 1e-9, "the last sample is not the last waypoint")
    v = np.abs(np.diff(traj.q, axis=0)) / np.diff(traj.t)[:, None]
    peak = v.max(axis=0)
    need(all(p <= cap * 1.001 for p, cap in zip(peak, CAP_RAD_S)), f"joint speeds {peak} over {CAP_RAD_S}")
    seen = [p for p in c.progress[rid] if 0.0 < p < 1.0]
    need(seen, "no progress between 0 and 1 in the states")
    need(max(abs(a - b) for a, b in zip(here(c.arm), w2)) < 0.01, "joints_deg not at the last waypoint")
    return (f"done; {len(traj.t)} samples over {traj.duration:.2f} s, sample 1 = the arm's posture exactly (waypoint 1 "
            f"was 0.3 deg off), the last = waypoint 3; peak joint speeds {np.round(peak, 3).tolist()} rad/s within "
            f"the caps; progress seen {len(seen)} times")


def c_start_gap(c):
    start = here(c.arm)
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [moved(start, [0, 0, 5, 0, 0, 0]),
                                                              moved(start, [5, 0, 0, 0, 0, 0])]})
    return refused(reply, M.OUT_OF_RANGE, "joint 3", "from where the arm is")


def c_limit(c):
    start = here(c.arm)
    hi5 = c.model.limits_model_deg()[4][1]
    beyond = list(start)
    beyond[4] = hi5 + 2.0                                       # the old gateway clamped up to 10 deg past
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [start, moved(start, [3, 0, 0, 0, 0, 0]), beyond]})
    why = refused(reply, M.OUT_OF_RANGE, "waypoint 3", "joint 5", f"{hi5:.2f}")
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [start, moved(start, [3, 0, 0, 0, 0, 0])],
                                            "speeds": [3.0]})
    refused(reply, M.OUT_OF_RANGE, "segment 1")
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [start, moved(start, [3, 0, 0, 0, 0, 0])],
                                            "speeds": [1.0, 1.0]})
    refused(reply, M.BAD_ARGS, "1 numbers")
    need(not c.fake.st["busy"], "something moved")
    return why + " | speed 3 -> OUT_OF_RANGE, 2 speeds for 1 segment -> BAD_ARGS"


def c_stop_mid_path(c):
    c.fake.rate = 1.0
    start = here(c.arm)
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [start, moved(start, [40, 0, 0, 0, 0, 0])],
                                            "speeds": [0.25]})
    accepted(reply)
    need(wait_for(lambda: any(p > 0.05 for p in c.progress[rid]), 5.0), "no progress")
    n = len(c.fake.calls)
    sid, sreply = ask(c.arm, "stop", {"reason": "the check"})
    accepted(sreply)
    last = ended(c.arm, rid, 5.0)
    need(last["result"] == "stopped" and last["reason"].startswith(f"stop #{sid}"), f"last {last}")
    calls = c.fake.calls[n:]
    need("halt" in calls and "halt_all" in calls, f"calls {calls}")
    need(c.fake.goal.done() and c.fake.goal.result()[0] is False, "the goal was not cancelled")
    q1 = list(c.fake.q)
    time.sleep(0.2)
    need(q1 == c.fake.q, "the arm still moves")
    need(c.arm.state()["mode"] == "ready", "not ready after the stop")
    c.fake.rate = 10.0
    return (f"stopped at {max(c.progress[rid]):.2f} of the path; last = stopped ('{last['reason']}'); its goal and "
            f"every arm goal cancelled ({', '.join(calls)}), the arm frozen; ready")


def c_release_hold(c):
    rid, reply = ask(c.arm, "release")
    accepted(reply)
    need(ended(c.arm, rid)["result"] == "done", "release did not end done")
    need(c.arm.state()["values"]["held"] is False, "held is not False")
    start = here(c.arm)
    path = {"waypoints_deg": [start, moved(start, [4, 0, 0, 0, 0, 0])]}
    _, reply = ask(c.arm, "follow_path", path)
    why = refused(reply, M.NOT_READY, "send hold first")
    hid, reply = ask(c.arm, "hold")
    accepted(reply)
    need(ended(c.arm, hid)["result"] == "done", "hold did not end done")
    need(c.arm.state()["values"]["held"] is True, "held is not True")
    _, last = run_path(c, [[4, 0, 0, 0, 0, 0]])
    need(last["result"] == "done", f"the path after hold: {last}")
    return f"release done, held False; follow_path -> NOT_READY ({why}); hold done; the same path done"


def c_stop_released(c):
    rid, reply = ask(c.arm, "release")
    accepted(reply)
    ended(c.arm, rid)
    n = len(c.fake.calls)
    sid, reply = ask(c.arm, "stop")
    accepted(reply)
    need("hold_soon" in c.fake.calls[n:], f"calls {c.fake.calls[n:]}")
    need(wait_for(lambda: c.arm.state()["values"]["held"] is True, 2.0), "the arm was left limp")
    return "release, then stop -> the arm held where it is (stop never leaves it limp)"


def c_fault(c):
    with c.fake.lock:
        c.fake.st.update(errors=["arm_overcurrent"], fault="error bits: arm_overcurrent")
    need(wait_for(lambda: c.arm.state()["mode"] == M.MODE_FAULT, 2.0), "no fault latched")
    fault = c.arm.state()["fault"]
    need(fault["code"] == A.FAULT_ARM and "error bits: arm_overcurrent" in fault["reason"], f"fault {fault}")
    start = here(c.arm)
    path = {"waypoints_deg": [start, moved(start, [3, 0, 0, 0, 0, 0])]}
    refused(ask(c.arm, "follow_path", path)[1], M.FAULT)
    still = refused(ask(c.arm, "clear_fault")[1], M.FAULT, "still reports error bits")
    with c.fake.lock:
        c.fake.st["errors"] = []                                 # the cause gone; the node keeps its words latched
    accepted(ask(c.arm, "clear_fault")[1])
    need(wait_for(lambda: c.arm.state()["mode"] == M.MODE_READY, 2.0), "not ready after clear_fault")
    time.sleep(3 * A.SENSE_PERIOD_S)
    need(c.arm.state()["mode"] == M.MODE_READY, "latched again from the same words")
    nodes = refused(ask(c.arm, "follow_path", path)[1], M.NOT_READY, "send hold to clear it")
    hid, reply = ask(c.arm, "hold")
    accepted(reply)
    need(ended(c.arm, hid)["result"] == "done", "hold did not end done")
    _, last = run_path(c, [[3, 0, 0, 0, 0, 0]])
    need(last["result"] == "done", f"the path after hold: {last}")
    return (f"latched '{fault['reason']}'; follow_path -> FAULT; clear_fault -> FAULT ({still}); cause gone -> "
            f"clear_fault taken; follow_path -> NOT_READY ({nodes}); hold; the path done")


def c_silent_node(c):
    c.fake.age = 5.0
    need(wait_for(lambda: (c.arm.state()["fault"] or {}).get("code") == A.FAULT_SILENT, 2.0), "no silence fault")
    v = c.arm.state()["values"]
    need(v["joints_deg"] == v["gap_mm"] == v["held"] == "unknown", f"values {v}")
    refused(ask(c.arm, "clear_fault")[1], M.FAULT, "no fresh status")
    c.fake.age = 0.0
    need(wait_for(lambda: c.arm.state()["values"]["held"] is True, 2.0), "values not back")
    accepted(ask(c.arm, "clear_fault")[1])
    need(wait_for(lambda: c.arm.state()["mode"] == M.MODE_READY, 2.0), "not ready")
    return (f"status 5 s old -> fault {A.FAULT_SILENT}, joints / gap / held unknown, clear_fault refused; status back "
            "-> clear_fault taken, ready")


def c_beat(c):
    c.fake.rate = 1.0
    start = here(c.arm)
    rid, reply = ask(c.arm, "follow_path", {"waypoints_deg": [start, moved(start, [-30, 0, 0, 0, 0, 0])],
                                            "speeds": [0.25]})
    accepted(reply)
    need(wait_for(lambda: any(p > 0.02 for p in c.progress[rid]), 5.0), "no progress")
    n = len(c.fake.calls)
    c.beat.on.clear()                                           # the harness falls silent
    last = ended(c.arm, rid, 3.0)
    need(last["result"] == "stopped" and last["reason"] == "harness silent", f"last {last}")
    calls = c.fake.calls[n:]
    need("halt" in calls and "halt_all" not in calls, f"calls {calls}")
    later = here(c.arm)
    _, reply = ask(c.arm, "follow_path", {"waypoints_deg": [later, moved(later, [2, 0, 0, 0, 0, 0])]})
    why = refused(reply, M.NOT_READY, "heartbeat")
    c.beat.on.set()
    time.sleep(3 * TIMING.beat_s)
    c.fake.rate = 10.0
    _, last2 = run_path(c, [[2, 0, 0, 0, 0, 0]])
    need(last2["result"] == "done", f"after the beat came back: {last2}")
    return (f"beat stopped -> the path stopped ('harness silent'), only its own goal cancelled ({', '.join(calls)}); "
            f"follow_path -> NOT_READY ({why}); beat back -> a path done")


def c_calibrations(c):
    a = refused(ask(c.arm, "calibrate_handeye")[1], M.NOT_READY, "not ported yet")
    refused(ask(c.arm, "calibrate_absolute")[1], M.NOT_READY, "not ported yet")
    return "both NOT_READY: " + a[:80] + " ..."


def c_grip(c):
    refused(ask(c.arm, "grip_to", {"gap_mm": 95.0})[1], M.OUT_OF_RANGE, f"{GAP_MAX_MM}")
    refused(ask(c.arm, "grip_to", {"gap_mm": 20.0, "force_n": 0.0})[1], M.OUT_OF_RANGE, "force_n")
    rid, reply = ask(c.arm, "grip_to", {"gap_mm": 20.0, "force_n": 5.0})
    accepted(reply)
    need(ended(c.arm, rid)["result"] == "done", "grip did not end done")
    need(("grip", 0.02, 5.0) in c.fake.calls, f"calls {c.fake.calls[-3:]}")
    need(wait_for(lambda: c.arm.state()["values"]["gap_mm"] == 20.0, 1.0), "gap_mm not 20")
    return f"95 mm -> OUT_OF_RANGE (widest {GAP_MAX_MM}), force 0 -> OUT_OF_RANGE, 20 mm at 5 N done, gap 20.0 sensed"


def _stand_ins():
    def ns(**kw):
        return types.SimpleNamespace(**kw)
    state = lambda: ns(stamp=ns(sec=0, nanosec=0), instance="", mode="", fault_code="", fault_reason="",  # noqa: E731
                       has_current=False, current_id=0, current_action="", current_progress=0.0, has_last=False,
                       last_id=0, last_action="", last_result="", last_reason="", describe="", values="", sources="")
    request = lambda: ns(id=0, action="", args="", set="")  # noqa: E731
    response = lambda: ns(accepted=False, code="", reason="")  # noqa: E731
    return state, request, response


def c_mapping(c):
    StateMsg, RequestMsg, ResponseMsg = _stand_ins()
    state = dict(c.arm.state(), time=1763600000.123456,
                 current={"id": 7, "do": "follow_path", "progress": 0.62},
                 last={"id": 6, "do": "hold", "result": "stopped", "reason": "stop #9"},
                 fault={"code": "ARM_FAULT", "reason": "arm node: feedback lost"})
    msg = R.state_to_msg(StateMsg(), state)
    msg.current_progress = struct.unpack("f", struct.pack("f", msg.current_progress))[0]   # float32 on the wire
    back = R.state_from_msg(msg)
    need(abs(back.pop("time") - state["time"]) < 1e-6, "time")
    need(back == {k: v for k, v in state.items() if k != "time"}, f"state differs: {back}")
    need(msg.instance == state["module"] and not hasattr(msg, "module"), "instance")
    for request in ({"id": -3, "do": "follow_path", "args": {"waypoints_deg": [[1.5] * 6, [2.5] * 6]}},
                    {"id": 4, "set": {"platform_mm": 87.3}}, {"id": 5, "do": "stop", "args": {}}):
        m = R.request_to_msg(RequestMsg(), request)
        need(R.request_from_msg(m) == request, f"request {request} came back {R.request_from_msg(m)}")
    need(R.request_to_msg(RequestMsg(), {"id": 1, "do": "hold", "args": {}}).action == "hold", "do -> action")
    for reply in (M.accept(8), M.refuse(9, M.OUT_OF_RANGE, "z_mm = 300 is outside 0 ... 109 mm")):
        need(R.reply_from_msg(reply["id"], R.reply_to_msg(ResponseMsg(), reply)) == reply, f"reply {reply}")
    broken = RequestMsg()
    broken.id, broken.action, broken.args = 1, "hold", "{not json"
    try:
        R.request_from_msg(broken)
        need(False, "broken args JSON was taken")
    except ValueError:
        pass
    need(R.module_of("/bunny_1/state") == "bunny_1" and R.module_of("/state") is None
         and R.module_of("/bunny_1/command") is None, "module_of")
    return "state (incl. current / last / fault, float32 progress), do / set requests, replies, broken JSON, topic names"


CHECKS = [
    ("describe", c_describe),
    ("a good path", c_good_path),
    ("a start gap is refused", c_start_gap),
    ("an out-of-limit waypoint is refused", c_limit),
    ("stop mid-path", c_stop_mid_path),
    ("release -> refused -> hold", c_release_hold),
    ("stop while released holds", c_stop_released),
    ("a fault latches and clears", c_fault),
    ("the arm node falls silent", c_silent_node),
    ("the beat stops -> safe state", c_beat),
    ("the two calibrations", c_calibrations),
    ("grip_to", c_grip),
    ("ros_binding mapping", c_mapping),
]


def main():
    lim = Limits(tcp_offset_m=0.1441, floor_enabled=False, workspace_min=(-5.0, -5.0, -5.0),
                 workspace_max=(5.0, 5.0, 5.0), ik_margin_rad=0.02)
    c = types.SimpleNamespace(bus=LocalBus(), fake=FakeArm(), progress=collections.defaultdict(list))
    c.model = A.ArmModel(PiperKinematics(SYNTH_MDH, OFFSETS), lim, {"gripper_max_m": GAP_MAX_MM / 1000.0})

    def heard(state):
        if state.get("current"):
            c.progress[state["current"]["id"]].append(state["current"]["progress"])
    c.bus.on_state(heard)
    c.arm = A.Arm("arm_1", c.bus, c.fake, c.model, timing=TIMING, serial="CHECK").start()
    c.beat = Beat(c.bus)
    results = []
    try:
        if not wait_for(lambda: c.arm.state()["mode"] == M.MODE_READY, 3.0):
            print("FAIL  start: the arm module did not become ready")
            return 1
        for name, fn in CHECKS:
            try:
                detail, ok = fn(c), True
            except Exception as error:   # noqa: BLE001  a check that raises is a failed check
                detail, ok = f"{type(error).__name__}: {error}", False
            results.append(ok)
            print(f"{'ok  ' if ok else 'FAIL'}  {name}: {detail}", flush=True)
    finally:
        c.beat.end.set()
        c.arm.close()
    print(f"{sum(results)} / {len(results)} checks hold")
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
