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
  v2 (2026-10-23, spec "Release, hand-eye and tag teaching through the gateway"):
  {"cmd": "arm.release"}                                  joints 1-6 limp (the gripper keeps holding)
  {"cmd": "calib.handeye", "args": {"tag_id": 8}}         job: hand-eye around where the arm holds; state.calib
  {"cmd": "teach.plan_views", "args": {"from_model_deg": [...6], "backoff_mm": 40, "tilt_deg": 1.5}}   dry run
  {"cmd": "teach.fine", "args": {"tag_id": 7, "black_mm": 37.5, "tilt_deg": 3, "views_model_deg": {...}}}   job: the cross
  {"cmd": "arm.follow_joints", "args": {"waypoints_model_deg": [[...6], ...]}}   job: a planner's joint path

The gateway owns no hardware: it is an rclpy client of the module nodes, like harness_cmd.py. Releasing the
arm is the explicit arm.release (operator 2026-10-23; before that it was CLI-only). `stop` is accepted from
anyone. The controller's connection is a dead-man: silent for DEADMAN_S (or closed) while its motion or job
runs -> the goal is cancelled / the job aborted (the arm freezes, stays enabled) and the control lease is dropped.

  python3 tools/gateway_server.py          serve (process `gateway` under `harness nodes`)
  python3 tools/gateway_server.py --key    create the shared key if missing, print it (`harness gateway key`)
"""
import json, math, os, secrets, signal, socketserver, subprocess, sys, threading, time

PORT = int(os.environ.get("GATEWAY_PORT", 7803))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY_FILE = os.path.expanduser("~/.config/railab-harness/gateway.key")
PIPER_YAML = os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")
# the calibration / planner packages from source, so a new module there needs no colcon rebuild
sys.path[:0] = [os.path.join(ROOT, "src", "harness_calibration"), os.path.join(ROOT, "src", "harness_planner")]
KIN_FILE = os.path.join(ROOT, "calib", "kinematics.yaml")
HANDEYE_FILE = os.path.join(ROOT, "calib", "handeye.yaml")
TEACH_TAGS = os.path.join(ROOT, "calib", "teach", "tags.yaml")      # teach.fine results, by tag id
VERSION = 2
DEADMAN_S = 3.0
HOLDER_STALE_S = 10.0     # a control holder silent this long (no ping, no command) can be taken over
JOINTS = ["joint%d" % i for i in range(1, 7)]
ARM_ACTIONS = ("/arm/move_j", "/arm/move_pose", "/arm/follow_trajectory")
HANDEYE_ACCEPT_RMS_MM = 3.0      # calib.handeye installs a new calibration only at or below this rms
HANDEYE_SETTLE_S = 3.0           # wait after every hand-eye move before the images (operator: the arm shakes ~3 s)
TEACH_VEL_RAD_S, TEACH_ACC_RAD_S2 = 0.35, 0.7     # arm.follow_joints: joint speed / acceleration of the path
FOLLOW_START_TOL_RAD = 0.1       # its first waypoint: this close to where the arm is
FOLLOW_CLAMP_RAD = math.radians(3.0)   # waypoints beyond the controller limits by up to this are clamped inside


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
        from harness_msgs.msg import ArmStatus, GripperStatus, TagArray
        from harness_msgs.action import FollowTrajectory, MoveJ
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
        n.create_subscription(TagArray, "/camera/tags", put("tags"), 10)
        self.MoveJ, self.FollowTrajectory = MoveJ, FollowTrajectory
        self.ac_j = ActionClient(n, MoveJ, "/arm/move_j")
        self.ac_t = ActionClient(n, FollowTrajectory, "/arm/follow_trajectory")
        self.clients = {}
        self.hc = None                       # HarnessClient for teach.fine (created on first use)
        self.load_handeye()
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

    def load_handeye(self):
        """calib/handeye.yaml -> self.Tfc (gripper base -> camera, m); None without one."""
        import numpy as np, yaml
        try:
            with open(HANDEYE_FILE) as f:
                d = yaml.safe_load(f) or {}
            self.Tfc = np.array(d["flange_to_camera"]["matrix"], float)
            self.he_info = {"date": d.get("date"), "rms_mm": d.get("rms_mm", d.get("residual_rms_mm"))}
        except (OSError, KeyError, TypeError, ValueError):
            self.Tfc, self.he_info = None, {"date": None, "rms_mm": None}

    def camera(self):
        """Tags the gripper camera sees now, posed in the arm base (corrected FK of the latest joints x
        hand-eye x camera->tag as the detector solved it with assumed_mm)."""
        import numpy as np
        from harness_vision.geom import T_from_pose_msg
        m, q = self.fresh("tags", 1.0), self.joints_rad()
        out = {"online": m is not None, "handeye": self.he_info, "tags": []}
        if m is None or q is None or self.Tfc is None:
            return out
        Tbc = self.kin.fk(q) @ self.Tfc
        out["cam_mm"] = [round(float(v) * 1000, 2) for v in Tbc[:3, 3]]
        for t in m.tags:
            Tbt = Tbc @ T_from_pose_msg(t.pose)
            out["tags"].append({"id": int(t.id), "size_px": round(t.size_px, 1), "center_px": [round(v, 1) for v in t.center_px],
                                "distance_mm": round(t.distance_m * 1000, 1), "assumed_mm": round(t.size_m * 1000, 2),
                                "base_mm": [round(float(v) * 1000, 2) for v in Tbt[:3, 3]],
                                "base_R": [[round(float(v), 6) for v in row] for row in Tbt[:3, :3]],
                                "reproj_px": round(t.reproj_err_px, 2)})
        return out

    def teach_client(self):
        if self.hc is None:
            from harness_calibration.client import HarnessClient
            self.hc = HarnessClient("gateway_teach")
        return self.hc

    def plan_views(self, q_model_deg, backoff_mm, tilt_deg):
        """teach.plan_views, a dry run: the five-view cross for a viewing posture (corrected-model degrees) with the
        camera backoff_mm further back along its optical axis and +-tilt_deg tilts. Every view solved by IK inside
        the joint limits (seeded from the posture: same wrist / elbow branch), its fingertip checked against the
        floor and the workspace box; nothing moves. Obstacles are the caller's (the twin's) to check."""
        import numpy as np
        from harness_calibration.fine import backed_off, cross_targets
        if self.Tfc is None:
            raise RuntimeError("no hand-eye calibration (calib/handeye.yaml)")
        if not isinstance(q_model_deg, list) or len(q_model_deg) != 6:
            raise ValueError("from_model_deg needs 6 joint angles")
        off = np.array(self.kin.offsets, float)
        q = np.radians(np.array(q_model_deg, float)) - off
        margin = float(self.P.get("ik_limit_margin_rad", 0.02))
        lo = np.array([l[0] for l in self.limits]) + margin
        hi = np.array([l[1] for l in self.limits]) - margin
        T0 = backed_off(self.kin.fk(q), self.Tfc, float(backoff_mm) / 1000.0)
        fl_margin = float(self.P.get("floor_margin_m", 0.015))
        wlo, whi = self.P.get("workspace_min"), self.P.get("workspace_max")
        out, reasons, q_c = [], [], None
        for name, T in cross_targets(T0, self.Tfc, float(tilt_deg)):
            qs, pe, ae = self.kin.ik(T, q_c if q_c is not None else np.clip(q, lo + 1e-6, hi - 1e-6), margin=margin)
            why = None
            if pe > 1e-3 or ae > 5e-3:
                why = "out of reach inside the joint limits (best %.1f mm / %.2f deg)" % (pe * 1000, math.degrees(ae))
            else:
                Tq, _, clear = self.tip_and_clearance(list(qs))
                if self.P.get("floor_enabled", False) and clear < fl_margin:
                    why = "the fingertip would be %.0f mm above the floor (margin %.0f mm)" % (clear * 1000, fl_margin * 1000)
                elif wlo and whi and not all(wlo[i] <= Tq[i, 3] <= whi[i] for i in range(3)):
                    why = "the gripper base would leave the workspace box"
                elif q_c is not None and float(np.abs(qs - q_c).max()) > float(self.P.get("max_joint_delta_rad", 1.2)):
                    why = "a joint would jump %.0f deg (another wrist branch)" % math.degrees(float(np.abs(qs - q_c).max()))
            out.append({"view": name, "ok": why is None, "reason": why,
                        "joints_model_deg": [round(math.degrees(float(v + o)), 4) for v, o in zip(qs, off)]})
            if why:
                reasons.append("%s: %s" % (name, why))
                if name == "centre":
                    break                                     # no centre view, no cross
            elif name == "centre":
                q_c = qs
        return {"ok": not reasons, "reasons": reasons, "backoff_mm": float(backoff_mm), "tilt_deg": float(tilt_deg),
                "centre_model_deg": out[0]["joints_model_deg"], "views": out[1:]}

    def fine(self, tid, black_mm, abort, tilt_deg=3.0, views_model_deg=None):
        """teach.fine: the five-view cross on tag tid from where the camera looks now (harness_calibration.fine);
        views_model_deg {view: 6 corrected-model degrees} = go to exactly these (teach.plan_views) postures."""
        import datetime as dt, numpy as np, yaml
        from harness_calibration.fine import Aborted, cross_locate
        from harness_arm.kinematics import xyzrpy_from_T
        if self.Tfc is None:
            raise RuntimeError("no hand-eye calibration (calib/handeye.yaml)")
        c = self.teach_client()
        c.wait_ready()
        if not (c.status and c.status.all_enabled):
            raise RuntimeError("the arm is not holding: press Hold first")
        t = c.tag(tid)
        if t is None:
            raise RuntimeError("tag %d is not in view from here" % tid)
        scale = (black_mm / 1000.0) / t.size_m if black_mm > 0 else 1.0
        vj = None
        if views_model_deg:
            off = list(self.kin.offsets)
            vj = {k: [math.radians(float(v)) - o for v, o in zip(q, off)] for k, q in views_model_deg.items()}
        try:
            r = cross_locate(c, tid, self.Tfc, scale=scale, abort=abort.is_set, tilt_deg=float(tilt_deg), view_joints=vj)
        except Aborted:
            raise RuntimeError(JOB.get("why") or "aborted")
        q = self.joints_rad()
        res = {"ok": r["ok"], "reasons": r["reasons"], "tag_id": tid, "black_mm": black_mm or round(t.size_m * 1000, 2),
               "spread_mm": r["spread_mm"], "spread_deg": r["spread_deg"], "views": r["views"], "size_px": r["size_px"],
               "joints_deg": [round(math.degrees(v), 3) for v in q] if q else None}
        if r["T"] is not None:
            T = r["T"]
            res["base_mm"] = [round(float(v) * 1000, 2) for v in T[:3, 3]]
            res["base_R"] = [[round(float(v), 6) for v in row] for row in T[:3, :3]]
        if r["ok"]:
            try:
                with open(TEACH_TAGS) as f:
                    doc = yaml.safe_load(f) or {}
            except OSError:
                doc = {}
            doc.setdefault("tags", {})[int(tid)] = {
                "base_to_tag_matrix": T.tolist(), "base_to_tag_xyzrpy": xyzrpy_from_T(T), "spread_mm": r["spread_mm"],
                "spread_deg": r["spread_deg"], "black_mm": res["black_mm"], "size_px": r["size_px"], "joints_rad": q,
                "handeye_date": self.he_info.get("date"), "date": dt.datetime.now().isoformat(timespec="seconds")}
            os.makedirs(os.path.dirname(TEACH_TAGS), exist_ok=True)
            with open(TEACH_TAGS, "w") as f:
                yaml.safe_dump(doc, f, sort_keys=False)
        return res

    def follow_joints(self, conn, wps_deg, label, abort):
        """arm.follow_joints: a planner's joint path (corrected-model degrees) -> validated, streamed trajectory."""
        import numpy as np
        from harness_planner.demo import load_model, to_msg
        from harness_planner.trajectory import PlanError, plan_joint_path
        q = self.joints_rad()
        if q is None:
            raise RuntimeError("no arm feedback")
        st = self.fresh("arm", 1.0)
        if st is None or not st.all_enabled:
            raise RuntimeError("the arm is not holding: press Hold first")
        if not isinstance(wps_deg, list) or len(wps_deg) < 1 or any(not isinstance(w, list) or len(w) != 6 for w in wps_deg):
            raise ValueError("waypoints_model_deg needs a list of 6-joint waypoints")
        off = np.array(self.kin.offsets, float)
        W = np.radians(np.array(wps_deg, float)) - off             # corrected-model angles -> controller joints
        lo = np.array([l[0] for l in self.limits]) + 0.02
        hi = np.array([l[1] for l in self.limits]) - 0.02
        over = np.maximum(lo - W, W - hi).max(axis=1)
        if (over > FOLLOW_CLAMP_RAD).any():
            k = int(np.argmax(over))
            raise RuntimeError("refused: waypoint %d is %.1f deg beyond a joint limit" % (k + 1, math.degrees(over[k])))
        W = np.clip(W, lo, hi)
        d0 = float(np.abs(W[0] - np.array(q)).max())
        if d0 > FOLLOW_START_TOL_RAD:
            raise RuntimeError("refused: the path starts %.1f deg from where the arm is; plan it from the current posture" % math.degrees(d0))
        W[0] = q
        kin, lim = load_model(ROOT)
        lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2 = TEACH_VEL_RAD_S, TEACH_ACC_RAD_S2
        try:
            tr = plan_joint_path(self.kin, W, lim)
        except PlanError as e:
            raise RuntimeError("refused: %s" % e)
        if not self.ac_t.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("/arm/follow_trajectory not available")
        g = self.FollowTrajectory.Goal(trajectory=to_msg(tr), label=label or "twin path")
        gf = self.ac_t.send_goal_async(g)
        t0 = time.time()
        while not gf.done():
            if time.time() - t0 > 5:
                raise RuntimeError("goal not accepted in time")
            time.sleep(0.01)
        gh = gf.result()
        if not gh.accepted:
            raise RuntimeError("rejected by the arm node (busy, fault, teach mode or not enabled)")
        rf = gh.get_result_async()
        while not rf.done():
            if abort.is_set():
                gh.cancel_goal_async()
                raise RuntimeError(JOB.get("why") or "aborted - arm frozen")
            if time.time() - t0 > tr.duration + 60:
                gh.cancel_goal_async()
                raise RuntimeError("no result long after the path should have ended - arm frozen")
            time.sleep(0.05)
        r = rf.result().result
        if not r.success:
            raise RuntimeError(r.message)
        return {"success": True, "message": r.message, "seconds": round(r.seconds, 1),
                "max_tracking_error_deg": round(math.degrees(r.max_tracking_error_rad), 2), "plan": tr.report}

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
            # the corrected model's angles (raw + calib/kinematics.yaml zero offsets): what the twin mirrors
            arm["joints_model_deg"] = [round(math.degrees(v + o), 3) for v, o in zip(q, self.kin.offsets)]
            arm["flange_mm_deg"] = [round(v * 1000, 1) for v in x[:3]] + [round(math.degrees(v), 2) for v in x[3:]]
            _, tip, _ = self.tip_and_clearance(q)        # the fingertip: same orientation, tcp_offset along flange z
            arm["tip_mm_deg"] = [round(float(v) * 1000, 1) for v in tip] + arm["flange_mm_deg"][3:]
        if g is not None:
            arm["gripper"] = {"gap_mm": round(g.width_m * 1000, 2), "force_n": round(g.force_n, 2), "powered": g.enabled}
        b = self.fresh("bunny", 3.0)
        bunny = {"online": bool(b and b.get("online"))}
        if b:
            bunny.update({k: b.get(k) for k in ("unit", "housing", "weight_g", "dispensing", "busy", "load_open")})
        calib = {"busy": JOB["busy"], "kind": JOB["kind"], "lines": JOB["lines"][-8:], "result": JOB["result"]}
        return {"arm": arm, "bunny": bunny, "camera": self.camera(), "calib": calib}


# ---------------------------------------------------------------------------- server
gw = None
lock = threading.Lock()
ctl = {"conn": None, "motion": None}     # control lease (a Conn) and the gateway-started motion {conn, label}
# live slider driving: the latest target of each kind wins (arm.jog_pose / arm.jog_gripper)
jog = {"pose": None, "grip": None, "busy": False, "error": "", "last": None, "ev": threading.Event()}
# the one running job (calib.handeye / teach.fine / arm.follow_joints) and the last result
JOB = {"busy": False, "kind": "", "lines": [], "result": None, "proc": None, "abort": threading.Event(), "why": ""}


def abort_job(why):
    """Stop the running job: its abort flag, SIGINT to its process group, every arm goal cancelled (the arm freezes)."""
    if not JOB["busy"]:
        return False
    JOB["why"] = why
    JOB["abort"].set()
    p = JOB.get("proc")
    if p is not None and p.poll() is None:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except OSError:
            pass
    gw.cancel_arm()
    return True


def handeye_job(tid, abort):
    """calib.handeye: the hand-eye routine as a process; its result is installed when rms <= HANDEYE_ACCEPT_RMS_MM."""
    import datetime as dt, shutil, yaml
    new = os.path.join(ROOT, "calib", "handeye_new.yaml")
    if os.path.exists(new):
        os.remove(new)
    cmd = ["ros2", "run", "harness_calibration", "handeye", "--tag-id", str(int(tid)), "--accept-mm", "6.0",
           "--settle", str(HANDEYE_SETTLE_S), "--out", new]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True,
                         bufsize=1, env=dict(os.environ, PYTHONUNBUFFERED="1"), start_new_session=True)
    JOB["proc"] = p
    for line in p.stdout:
        line = line.rstrip()
        if line:
            JOB["lines"] = (JOB["lines"] + [line])[-40:]
            print("handeye: " + line, flush=True)
    rc = p.wait()
    if abort.is_set():
        return {"ok": False, "installed": False, "message": JOB["why"] or "aborted"}
    if not os.path.exists(new):
        return {"ok": False, "installed": False,
                "message": "hand-eye did not finish (exit %d): %s" % (rc, JOB["lines"][-1] if JOB["lines"] else "no output")}
    with open(new) as f:
        doc = yaml.safe_load(f) or {}
    os.remove(new)
    rms, mx = float(doc.get("residual_rms_mm") or 1e9), float(doc.get("residual_max_mm") or 1e9)
    x = doc.get("flange_to_camera") or {}
    res = {"rms_mm": round(rms, 2), "max_mm": round(mx, 2), "n_poses": doc.get("n_poses"), "tag_id": tid,
           "xyz_mm": [round(v * 1000, 1) for v in x.get("xyz_m", [])], "rpy_deg": [round(math.degrees(v), 2) for v in x.get("rpy_rad", [])]}
    if rms > HANDEYE_ACCEPT_RMS_MM:
        return dict(res, ok=False, installed=False, message="rms %.2f mm > %.1f mm: the old calibration stays" % (rms, HANDEYE_ACCEPT_RMS_MM))
    os.makedirs(os.path.join(ROOT, "calib", "handeye_backups"), exist_ok=True)
    if os.path.exists(HANDEYE_FILE):
        shutil.copy(HANDEYE_FILE, os.path.join(ROOT, "calib", "handeye_backups",
                                               "handeye_%s.yaml" % dt.datetime.now().strftime("%Y-%m-%dT%H%M%S")))
    doc.update(accepted=True, rms_mm=round(rms, 2), accepted_by="gateway calib.handeye: rms <= %.1f mm" % HANDEYE_ACCEPT_RMS_MM)
    with open(HANDEYE_FILE, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    gw.load_handeye()
    return dict(res, ok=True, installed=True, message="installed (rms %.2f mm, max %.2f mm, %s poses)" % (rms, mx, doc.get("n_poses")))


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
            aborted = abort_job("stopped (stop)")
            r = {"cancelled": gw.cancel_arm(), "job_aborted": aborted}
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
                h = ctl["conn"]
                # a holder that died without closing its socket (half-open link) must not block the
                # lease for ever: the twin's link pings every 0.5 s while its page is open
                stale = h is not None and (not h.alive or time.time() - h.last_rx > HOLDER_STALE_S)
                if h not in (None, self) and not stale:
                    raise PermissionError("control held by %s" % h.client)
                ctl["conn"] = self
            return {"control": True, "taken_over_from": h.client if (h is not None and h is not self) else None}
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
        if cmd == "arm.release":           # operator 2026-10-23: one click; joints 1-6 only, the gripper keeps holding
            self.need("control")
            jog["pose"] = jog["grip"] = None
            abort_job("the arm was released")
            return gw.trigger("/arm/disable", 15)
        if cmd == "calib.handeye":
            self.need("control")
            st = gw.fresh("arm", 1.0)
            if st is None or not st.all_enabled:
                raise RuntimeError("the arm is not holding: press Hold first (the routine moves it around where it is)")
            seen = gw.camera()["tags"]
            tid = a.get("tag_id")
            if tid is None:
                if not seen:
                    raise RuntimeError("no tag in view: point the camera at a fixed tag 150-300 mm away")
                tid = max(seen, key=lambda t: t["size_px"])["id"]
            elif not any(t["id"] == int(tid) for t in seen):
                raise RuntimeError("tag %s is not in view" % tid)
            return self.job("calib.handeye", lambda ev: handeye_job(int(tid), ev), background=True, extra={"tag_id": int(tid)})
        if cmd == "teach.plan_views":
            self.need("key")
            return gw.plan_views(a.get("from_model_deg"), float(a.get("backoff_mm") or 0.0), float(a.get("tilt_deg") or 3.0))
        if cmd == "teach.fine":
            self.need("control")
            tid, black = int(a["tag_id"]), float(a.get("black_mm") or 0.0)
            tilt, views = float(a.get("tilt_deg") or 3.0), a.get("views_model_deg") or None
            return self.job("teach.fine", lambda ev: gw.fine(tid, black, ev, tilt, views))
        if cmd == "arm.follow_joints":
            self.need("control")
            wps, label = a.get("waypoints_model_deg"), str(a.get("label") or "")
            return self.job("arm.follow_joints", lambda ev: gw.follow_joints(self, wps, label, ev))
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

    def job(self, kind, fn, background=False, extra=None):
        """Run fn(abort_event) as THE job (one at a time). This connection is its dead-man: silent DEADMAN_S or
        closed -> abort_job (every arm goal cancelled: the arm freezes, stays enabled) and the lease is dropped.
        background: answer {started} at once; the result goes to state.calib.result."""
        with lock:
            if JOB["busy"]:
                raise RuntimeError("busy: %s is running" % JOB["kind"])
            JOB.update(busy=True, kind=kind, lines=[], result=None, proc=None, abort=threading.Event(), why="")
        ev, done = JOB["abort"], threading.Event()
        ctl["motion"] = {"conn": self, "label": kind}

        def watch():
            while not done.wait(0.1):
                if not self.alive or time.time() - self.last_rx > DEADMAN_S:
                    abort_job("dead-man: controller silent for %.0f s - arm frozen" % DEADMAN_S)
                    with lock:
                        if ctl["conn"] is self:
                            ctl["conn"] = None
                    return

        def body():
            try:
                r = fn(ev)
                JOB["result"] = dict(r, kind=kind) if isinstance(r, dict) else r
                return r
            except Exception as e:     # noqa: BLE001  reported to the caller / in state.calib
                JOB["result"] = {"ok": False, "kind": kind, "message": str(e)}
                raise
            finally:
                done.set()
                ctl["motion"] = None
                JOB["busy"], JOB["proc"] = False, None

        threading.Thread(target=watch, daemon=True).start()
        if background:
            threading.Thread(target=lambda: _quiet(body), daemon=True).start()
            return dict({"started": True, "kind": kind}, **(extra or {}))
        return body()

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


def _quiet(fn):
    try:
        fn()
    except Exception:  # noqa: BLE001  a background job's failure is in state.calib.result
        pass


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
