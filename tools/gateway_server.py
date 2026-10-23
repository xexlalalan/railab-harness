#!/usr/bin/env python3
"""Twin control gateway: the workflow app's digital twin watches and drives the harness modules
(TCP 7803, all interfaces, newline-delimited JSON, persistent connections, many clients).
Contract: PROJECT_SPEC.md "2026-10-22 — Twin control gateway".

  {"id": 1, "cmd": "hello", "args": {"key": "...", "client": "railab-wf"}}
      -> {"id": 1, "ok": true, "result": {"version": 1, "access": "control"|"view", "modules": {...}}}
  {"id": 2, "cmd": "subscribe", "args": {"rate_hz": 10}}  -> {"event": "state", ...} at that rate
  {"id": 3, "cmd": "take_control"} / "release_control" / "ping" / "stop"
  {"id": 4, "cmd": "arm.check_j", "args": {"joints_deg": [...]}}     dry run, nothing moves
  {"id": 5, "cmd": "arm.move_j", "args": {"joints_deg": [...], "speed_percent": 10}}
  {"id": 6, "cmd": "arm.hold"} / "arm.gripper" {gap_mm, force_n} / "bunny.housing" {to} / "bunny.platform" {steps}
  {"id": 7, "cmd": "arm.jog_pose", "args": {"tip_mm_deg": [x, y, z, rx, ry, rz], "speed_percent": 10}}
  {"id": 8, "cmd": "arm.jog_gripper", "args": {"gap_mm": 30, "force_n": 1}}
      LIVE driving from the twin's sliders: answered at once; the LATEST target wins — the gateway moves the
      fingertip there (straight line, /arm/move_pose, the node's IK / floor / workspace checks) as soon as the
      current move ends, then the next latest one. Progress in the state event: control.jog.

The gateway owns no hardware: it is an rclpy client of the module nodes, like harness_cmd.py. There is
deliberately NO arm release / disable command. `stop` is accepted from anyone. The controller's connection
is a dead-man: silent for DEADMAN_S (or closed) while its motion runs -> the goal is cancelled (the arm
freezes, stays enabled) and the control lease is dropped.

  python3 tools/gateway_server.py          serve (process `gateway` under `harness nodes`)
  python3 tools/gateway_server.py --key    create the shared key if missing, print it (`harness gateway key`)
"""
import json, math, os, secrets, socketserver, sys, threading, time

PORT = int(os.environ.get("GATEWAY_PORT", 7803))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY_FILE = os.path.expanduser("~/.config/railab-harness/gateway.key")
PIPER_YAML = os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")
KIN_FILE = os.path.join(ROOT, "calib", "kinematics.yaml")
VERSION = 1
DEADMAN_S = 3.0
JOINTS = ["joint%d" % i for i in range(1, 7)]
ARM_ACTIONS = ("/arm/move_j", "/arm/move_pose", "/arm/follow_trajectory")


def read_key():
    try:
        with open(KEY_FILE) as f:
            return f.read().strip() or None
    except OSError:
        return None


def make_key():
    k = read_key()
    if k:
        return k
    os.makedirs(os.path.dirname(KEY_FILE), exist_ok=True)
    k = secrets.token_urlsafe(18)
    fd = os.open(KEY_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(k + "\n")
    return k


class Gateway:
    """ROS side: latest module state, the arm model for check_j, service/action calls."""

    def __init__(self):
        import rclpy, yaml
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState
        from harness_msgs.msg import ArmStatus, GripperStatus
        from harness_msgs.action import MoveJ
        from harness_arm.kinematics import PiperKinematics, JOINT_LIMITS_RAD

        with open(PIPER_YAML) as f:
            self.P = yaml.safe_load(f)["piper"]["ros__parameters"]
        self.kin = PiperKinematics.from_file(KIN_FILE, str(self.P.get("kinematics_model", "piper")))
        self.limits = JOINT_LIMITS_RAD
        rclpy.init()
        self.n = n = Node("harness_gateway")
        self.last = {}
        put = lambda k: (lambda m: self.last.__setitem__(k, (m, time.time())))
        n.create_subscription(ArmStatus, "/arm/status", put("arm"), 10)
        n.create_subscription(GripperStatus, "/arm/gripper_status", put("grip"), 10)
        n.create_subscription(JointState, "/joint_states", put("joints"), 10)
        n.create_subscription(String, "/bunny/status", lambda m: self.last.__setitem__("bunny", (json.loads(m.data), time.time())), 10)
        self.MoveJ = MoveJ
        self.ac_j = ActionClient(n, MoveJ, "/arm/move_j")
        self.clients = {}
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(n)
        threading.Thread(target=ex.spin, daemon=True).start()

    # ------------------------------------------------------------------ ROS helpers
    def fresh(self, key, max_age):
        v = self.last.get(key)
        return v[0] if v and time.time() - v[1] < max_age else None

    def call(self, srv, name, req, timeout=30.0):
        cli = self.clients.get(name) or self.clients.setdefault(name, self.n.create_client(srv, name))
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("%s not answering (node not running?)" % name)
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise RuntimeError("%s timed out" % name)
            time.sleep(0.01)
        return fut.result()

    def trigger(self, name, timeout=30.0):
        from std_srvs.srv import Trigger
        r = self.call(Trigger, name, Trigger.Request(), timeout)
        if not r.success:
            raise RuntimeError(r.message)
        return {"message": r.message}

    def cancel_arm(self):
        """Cancel every goal on every arm action (zero goal id + zero stamp = all goals): the node freezes the arm."""
        from action_msgs.srv import CancelGoal
        out = []
        for a in ARM_ACTIONS:
            try:
                r = self.call(CancelGoal, a + "/_action/cancel_goal", CancelGoal.Request(), 3.0)
                out += [a] * len(r.goals_canceling)
            except RuntimeError:
                pass
        return out

    def joints_rad(self):
        m = self.fresh("joints", 1.0)
        if m is None or not all(j in m.name for j in JOINTS):
            return None
        return [float(m.position[m.name.index(j)]) for j in JOINTS]

    # ------------------------------------------------------------------ arm model
    def tip_and_clearance(self, q):
        import numpy as np
        T = self.kin.fk(q)
        tip = T[:3, 3] + float(self.P.get("tcp_offset_m", 0.0)) * T[:3, 2]
        a, b, c = self.P.get("floor_plane", [0.0, 0.0, 0.0])
        return T, tip, float(tip[2] - (a * tip[0] + b * tip[1] + c))

    def check_j(self, q_deg):
        """Dry run of a joint target: limits, jump from now, corrected FK, fingertip floor clearance along the way."""
        from harness_arm.kinematics import xyzrpy_from_T
        if not isinstance(q_deg, list) or len(q_deg) != 6:
            raise ValueError("joints_deg needs 6 values")
        q = [math.radians(float(x)) for x in q_deg]
        reasons = []
        for i, (qi, (lo, hi)) in enumerate(zip(q, self.limits)):
            if qi < lo - 1e-3 or qi > hi + 1e-3:
                reasons.append("joint %d at %.1f deg is outside its limits [%.1f, %.1f]" % (i + 1, math.degrees(qi), math.degrees(lo), math.degrees(hi)))
        now = self.joints_rad()
        jump = None
        if now is None:
            reasons.append("no arm feedback (/joint_states)")
        else:
            d = [abs(a - b) for a, b in zip(q, now)]
            k = max(range(6), key=lambda i: d[i])
            jump = {"joint": k + 1, "deg": round(math.degrees(d[k]), 2)}
            if d[k] > float(self.P.get("max_joint_delta_rad", 1.2)):
                reasons.append("joint %d would jump %.1f deg (> %.1f deg per move): go in smaller steps"
                               % (k + 1, math.degrees(d[k]), math.degrees(float(self.P.get("max_joint_delta_rad", 1.2)))))
        T, tip, clear = self.tip_and_clearance(q)
        path_clear = clear
        if now is not None:     # move_j is joint-interpolated: sample the straight joint-space path
            for s in range(1, 20):
                f = s / 20.0
                path_clear = min(path_clear, self.tip_and_clearance([a + (b - a) * f for a, b in zip(now, q)])[2])
        margin = float(self.P.get("floor_margin_m", 0.015))
        if self.P.get("floor_enabled", False) and path_clear < margin:
            reasons.append("fingertip would come %.1f mm above the floor (margin %.0f mm)" % (path_clear * 1000, margin * 1000))
        lo, hi = self.P.get("workspace_min"), self.P.get("workspace_max")
        if lo and hi and not all(lo[i] <= T[i, 3] <= hi[i] for i in range(3)):
            reasons.append("gripper base outside the workspace box")
        x = xyzrpy_from_T(T)
        return {"ok": not reasons, "reasons": reasons, "jump": jump,
                "flange_mm_deg": [round(v * 1000, 1) for v in x[:3]] + [round(math.degrees(v), 2) for v in x[3:]],
                "tip_mm": [round(float(v) * 1000, 1) for v in tip],
                "floor_clearance_mm": round(clear * 1000, 1), "path_min_clearance_mm": round(path_clear * 1000, 1)}

    def flange_from_tip(self, tip_mm_deg):
        """[x y z mm, rx ry rz deg] of the fingertip (fixed-axis XYZ) -> flange pose [m, rad] for /arm/move_pose."""
        from harness_arm.kinematics import T_from_xyzrpy
        v = [float(a) for a in tip_mm_deg]
        if len(v) != 6:
            raise ValueError("tip_mm_deg needs x y z rx ry rz")
        T = T_from_xyzrpy([v[0] / 1000, v[1] / 1000, v[2] / 1000] + [math.radians(a) for a in v[3:]])
        p = T[:3, 3] - float(self.P.get("tcp_offset_m", 0.0)) * T[:3, 2]
        return [float(p[0]), float(p[1]), float(p[2])] + [math.radians(a) for a in v[3:]]

    def modules(self):
        return {"arm": {"joints": 6, "limits_deg": [[round(math.degrees(lo), 2), round(math.degrees(hi), 2)] for lo, hi in self.limits],
                        "gripper_max_mm": round(float(self.P.get("gripper_max_m", 0.07)) * 1000, 1),
                        "max_speed_percent": int(self.P.get("max_speed_percent", 30)),
                        "default_speed_percent": int(self.P.get("default_speed_percent", 10)),
                        "max_joint_delta_deg": round(math.degrees(float(self.P.get("max_joint_delta_rad", 1.2))), 1)},
                "bunny": {"housing": ["open", "half", "close"], "platform": "steps"}}

    def state(self):
        a, g, q = self.fresh("arm", 1.0), self.fresh("grip", 1.0), self.joints_rad()
        arm = {"online": bool(a and a.connected and q is not None)}
        if a is not None:
            arm.update(enabled=list(a.joints_enabled), all_enabled=a.all_enabled, ctrl_mode=a.ctrl_mode,
                       arm_status=a.arm_status, busy=a.busy, fault=a.fault, errors=list(a.errors),
                       teach=a.teach_mode, feedback_age_s=round(a.feedback_age_s, 3), speed_percent=a.speed_percent)
        if q is not None:
            from harness_arm.kinematics import xyzrpy_from_T
            x = xyzrpy_from_T(self.kin.fk(q))
            arm["joints_deg"] = [round(math.degrees(v), 3) for v in q]
            arm["flange_mm_deg"] = [round(v * 1000, 1) for v in x[:3]] + [round(math.degrees(v), 2) for v in x[3:]]
            _, tip, _ = self.tip_and_clearance(q)        # the fingertip: same orientation, tcp_offset along flange z
            arm["tip_mm_deg"] = [round(float(v) * 1000, 1) for v in tip] + arm["flange_mm_deg"][3:]
        if g is not None:
            arm["gripper"] = {"gap_mm": round(g.width_m * 1000, 2), "force_n": round(g.force_n, 2), "powered": g.enabled}
        b = self.fresh("bunny", 3.0)
        bunny = {"online": bool(b and b.get("online"))}
        if b:
            bunny.update({k: b.get(k) for k in ("unit", "housing", "weight_g", "dispensing", "busy", "load_open")})
        return {"arm": arm, "bunny": bunny}


# ---------------------------------------------------------------------------- server
gw = None
lock = threading.Lock()
ctl = {"conn": None, "motion": None}     # control lease (a Conn) and the gateway-started motion {conn, label}
# live slider driving: the latest target of each kind wins (arm.jog_pose / arm.jog_gripper)
jog = {"pose": None, "grip": None, "busy": False, "error": "", "last": None, "ev": threading.Event()}


def jog_worker():
    """Moves to the latest jog target, one at a time; a target that arrives while moving waits, older ones
    are dropped. Runs only for the controller (its connection must stay alive: dead-man as for move_j)."""
    from harness_msgs.action import MovePose
    from harness_msgs.srv import SetGripper
    ac = None
    while True:
        jog["ev"].wait(); jog["ev"].clear()
        while jog["pose"] or jog["grip"]:
            owner = ctl["conn"]
            if owner is None or not owner.alive:
                jog["pose"] = jog["grip"] = None
                break
            if jog["grip"]:
                g, jog["grip"] = jog["grip"], None
                try:
                    r = gw.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=g["gap_mm"] / 1000.0, force=g["force_n"]), 20)
                    jog["error"] = "" if r.success else "gripper: " + r.message
                except RuntimeError as e:
                    jog["error"] = "gripper: %s" % e
                continue
            t, jog["pose"] = jog["pose"], None
            jog["busy"] = True
            try:
                if ac is None:
                    from rclpy.action import ActionClient
                    ac = ActionClient(gw.n, MovePose, "/arm/move_pose")
                if not ac.wait_for_server(timeout_sec=3.0):
                    raise RuntimeError("arm node not answering")
                goal = MovePose.Goal(pose=gw.flange_from_tip(t["tip"]), linear=True, speed_percent=t["speed"], timeout_s=30.0)
                gf = ac.send_goal_async(goal)
                t0 = time.time()
                while not gf.done():
                    if time.time() - t0 > 5:
                        raise RuntimeError("goal not accepted in time")
                    time.sleep(0.01)
                gh = gf.result()
                if not gh.accepted:
                    raise RuntimeError("rejected by the arm node (not enabled, busy, fault or teach mode)")
                ctl["motion"] = {"conn": owner, "label": "jog"}
                rf = gh.get_result_async()
                while not rf.done():
                    if not owner.alive or time.time() - owner.last_rx > DEADMAN_S:
                        gh.cancel_goal_async()
                        jog["pose"] = jog["grip"] = None
                        raise RuntimeError("dead-man: controller silent for %.0f s - arm frozen" % DEADMAN_S)
                    time.sleep(0.03)
                r = rf.result().result
                jog["error"] = "" if r.success else r.message
                jog["last"] = t["tip"]
            except Exception as e:     # noqa: BLE001  reported in the state event
                jog["error"] = str(e)
            finally:
                ctl["motion"] = None
                jog["busy"] = False


class Conn(socketserver.StreamRequestHandler):
    def send(self, obj):
        data = (json.dumps(obj) + "\n").encode()
        with self.wlock:
            try:
                self.wfile.write(data); self.wfile.flush()
            except OSError:
                self.alive = False

    def handle(self):
        self.wlock, self.alive, self.access, self.client = threading.Lock(), True, "view", "?"
        self.last_rx, self.sub_rate = time.time(), 0.0
        self.peer = "%s:%d" % self.client_address
        threading.Thread(target=self.pump_state, daemon=True).start()
        try:
            for line in self.rfile:
                self.last_rx = time.time()
                try:
                    req = json.loads(line.decode())
                except ValueError:
                    self.send({"ok": False, "error": "not JSON"}); continue
                threading.Thread(target=self.dispatch, args=(req,), daemon=True).start()
        except OSError:
            pass
        finally:
            self.alive = False
            with lock:
                if ctl["conn"] is self:
                    ctl["conn"] = None
            m = ctl["motion"]
            if m and m["conn"] is self:     # dead-man: the controller went away mid-motion
                gw.cancel_arm()

    def pump_state(self):
        while self.alive:
            if self.sub_rate > 0:
                st = gw.state()
                c = ctl["conn"]
                st["control"] = {"held_by": c.client if c else None, "you": c is self,
                                 "jog": {"busy": jog["busy"] or bool(jog["pose"]), "error": jog["error"]}}
                st["event"], st["t"] = "state", round(time.time(), 3)
                self.send(st)
                time.sleep(1.0 / self.sub_rate)
            else:
                time.sleep(0.2)

    def dispatch(self, req):
        rid, cmd, args = req.get("id"), req.get("cmd"), req.get("args") or {}
        try:
            res = self.run(cmd, args, rid)
            self.send({"id": rid, "ok": True, "result": res})
        except Exception as e:     # noqa: BLE001  every failure goes back to the client
            self.send({"id": rid, "ok": False, "error": str(e)})

    def need(self, level):
        if level == "key" and self.access != "control":
            raise PermissionError("needs the gateway key (hello with key)")
        if level == "control" and ctl["conn"] is not self:
            raise PermissionError("take control first" if ctl["conn"] is None else "control held by %s" % ctl["conn"].client)

    def run(self, cmd, a, rid):
        if cmd == "hello":
            key = read_key()
            self.client = "%s@%s" % (a.get("client") or "client", self.peer)
            given = a.get("key") or ""
            self.access = "control" if key and given and secrets.compare_digest(given, key) else "view"
            r = {"version": VERSION, "access": self.access, "modules": gw.modules()}
            if given and self.access == "view":
                r["warning"] = "wrong key" if key else "no key on the harness (harness gateway key)"
            return r
        if cmd == "ping":
            return {"t": time.time()}
        if cmd == "subscribe":
            self.sub_rate = max(0.5, min(20.0, float(a.get("rate_hz", 10))))
            return {"rate_hz": self.sub_rate}
        if cmd == "unsubscribe":
            self.sub_rate = 0.0
            return {}
        if cmd == "state":
            return gw.state()
        if cmd == "stop":                  # anyone, never refused
            jog["pose"] = jog["grip"] = None
            r = {"cancelled": gw.cancel_arm()}
            b = gw.fresh("bunny", 3.0)
            if b and (b.get("dispensing") or b.get("busy")):
                try:
                    gw.trigger("/bunny/abort", 15); r["bunny"] = "abort sent"
                except RuntimeError as e:
                    r["bunny"] = str(e)
            return r
        if cmd == "take_control":
            self.need("key")
            with lock:
                if ctl["conn"] not in (None, self) and ctl["conn"].alive:
                    raise PermissionError("control held by %s" % ctl["conn"].client)
                ctl["conn"] = self
            return {"control": True}
        if cmd == "release_control":
            with lock:
                if ctl["conn"] is self:
                    ctl["conn"] = None
            return {"control": False}
        if cmd == "arm.check_j":
            self.need("key")
            return gw.check_j(a.get("joints_deg"))
        if cmd == "arm.move_j":
            self.need("control")
            chk = gw.check_j(a.get("joints_deg"))
            if not chk["ok"]:
                raise RuntimeError("refused: " + "; ".join(chk["reasons"]))
            speed = int(a.get("speed_percent") or gw.P.get("default_speed_percent", 10))
            return self.motion(chk, [math.radians(float(x)) for x in a["joints_deg"]], speed, rid)
        if cmd == "arm.jog_pose":
            self.need("control")
            tip = [float(v) for v in a.get("tip_mm_deg") or []]
            gw.flange_from_tip(tip)                      # validates the shape
            speed = max(1, min(int(gw.P.get("max_speed_percent", 30)), int(a.get("speed_percent") or 10)))
            jog["pose"], jog["error"] = {"tip": tip, "speed": speed}, ""
            jog["ev"].set()
            return {"queued": True}
        if cmd == "arm.jog_gripper":
            self.need("control")
            jog["grip"] = {"gap_mm": float(a["gap_mm"]), "force_n": float(a.get("force_n") or 1.0)}
            jog["ev"].set()
            return {"queued": True}
        if cmd == "arm.hold":
            self.need("control")
            return gw.trigger("/arm/hold", 15)
        if cmd == "arm.gripper":
            self.need("control")
            from harness_msgs.srv import SetGripper
            r = gw.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=float(a["gap_mm"]) / 1000.0,
                                                                       force=float(a.get("force_n") or 0.0)), 20)
            if not r.success:
                raise RuntimeError(r.message)
            return {"gap_mm": round(r.width_m * 1000, 2), "stalled": r.stalled, "message": r.message}
        if cmd == "bunny.housing":
            self.need("control")
            to = a.get("to")
            if to not in ("open", "half", "close"):
                raise ValueError("to must be open | half | close")
            return gw.trigger("/bunny/housing_" + to, 90)
        if cmd == "bunny.platform":
            self.need("control")
            from harness_msgs.srv import LiquidPush
            steps = int(a.get("steps", 0))
            r = gw.call(LiquidPush, "/bunny/platform", LiquidPush.Request(steps=steps, rate=0, wait=False), 10 + abs(steps) * 0.005)
            if not r.success:
                raise RuntimeError(r.message)
            return {"position_steps": r.steps_done}
        raise ValueError("unknown cmd %r" % cmd)

    def motion(self, chk, q, speed, rid):
        """Run /arm/move_j; progress events; the dead-man watch cancels it if this connection goes quiet."""
        if not gw.ac_j.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("arm node not answering")
        g = gw.MoveJ.Goal(joints_rad=q, speed_percent=speed, timeout_s=60.0)

        def fb(m):
            self.send({"event": "progress", "id": rid, "joint_error_deg": round(math.degrees(m.feedback.max_joint_error_rad), 2)})
        gf = gw.ac_j.send_goal_async(g, feedback_callback=fb)
        t0 = time.time()
        while not gf.done():
            if time.time() - t0 > 5:
                raise RuntimeError("goal not accepted in time")
            time.sleep(0.01)
        gh = gf.result()
        if not gh.accepted:
            raise RuntimeError("rejected by the arm node (busy, fault, teach mode or no feedback; see state)")
        ctl["motion"] = {"conn": self, "label": "move_j"}
        try:
            rf = gh.get_result_async()
            while not rf.done():
                if not self.alive or time.time() - self.last_rx > DEADMAN_S:
                    gh.cancel_goal_async()
                    with lock:
                        if ctl["conn"] is self:
                            ctl["conn"] = None
                    raise RuntimeError("dead-man: controller silent for %.0f s - arm frozen" % DEADMAN_S)
                if time.time() - t0 > 70:
                    gh.cancel_goal_async()
                    raise RuntimeError("move timed out - arm frozen")
                time.sleep(0.05)
            r = rf.result().result
        finally:
            ctl["motion"] = None
        if not r.success:
            raise RuntimeError(r.message)
        return {"success": True, "message": r.message, "speed_percent": speed,
                "final_joints_deg": [round(math.degrees(v), 3) for v in r.final_joints_rad], "check": chk}


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    global gw
    if sys.argv[1:2] == ["--key"]:
        print(make_key())
        return
    gw = Gateway()
    # rclpy takes SIGTERM for its own shutdown, which leaves the TCP server running: `harness nodes
    # restart gateway` (pkill = SIGTERM) must end the process
    import signal
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    threading.Thread(target=jog_worker, daemon=True).start()
    print("gateway: listening on 0.0.0.0:%d (key %s)" % (PORT, "set" if read_key() else "MISSING: view-only"), flush=True)
    Server(("0.0.0.0", PORT), Conn).serve_forever()


if __name__ == "__main__":
    main()
