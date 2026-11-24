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
  {"id": 6, "cmd": "arm.hold"} / "arm.gripper" {gap_mm, force_n, speed_mm_s?: ramped, else full speed} / "bunny.housing" {to} / "bunny.platform" {steps}
      arm.gripper + "end": "contact" (2026-11-22, a close): the ramp stops at contact, the target is set at once, the reply
      after GRIP_CONTACT_SQUEEZE_S with "contact": true|false; an empty close answers only within 1 mm of the target or stopped
  {"id": 7, "cmd": "arm.jog_pose", "args": {"tip_mm_deg": [x, y, z, rx, ry, rz], "speed_percent": 10}}
  {"id": 8, "cmd": "arm.jog_gripper", "args": {"gap_mm": 30, "force_n": 1}}
      LIVE driving from the twin's sliders: answered at once; the LATEST target wins — the gateway moves the
      fingertip there (straight line, /arm/move_pose, the node's IK / floor / workspace checks) as soon as the
      current move ends, then the next latest one. Progress in the state event: control.jog.
  v2 (2026-10-23, spec "Release, hand-eye and tag teaching through the gateway"):
  {"cmd": "arm.release"}                                  joints 1-6 limp (the gripper keeps holding)
  {"cmd": "calib.handeye", "args": {"tag_id": 8}}         job: hand-eye around where the arm holds; state.calib
  {"cmd": "calib.handeye", "args": {"target": "charuco"}}  the same with the ChArUco board as the target (2026-11-16)
  {"cmd": "teach.plan_views", "args": {"from_model_deg": [...6], "backoff_mm": 40, "tilt_deg": 1.5}}   dry run
  {"cmd": "teach.fine", "args": {"tag_id": 7, "black_mm": 37.5, "tilt_deg": 3, "views_model_deg": {...},
                                 "family": "tag16h5"}}   job: the cross (family: only that family's tag, 2026-10-25)
  {"cmd": "arm.follow_joints", "args": {"waypoints_model_deg": [[...6], ...]}}   job: a planner's joint path
                                                          (+ "speed": 1-2 x, "corner_deg": 5-45, 2026-11-18)
                                                          (+ "blend": true = ONE smooth motion through every waypoint,
                                                          at rest only at the ends; "speeds": [K-1] per segment,
                                                          0.25-2 x like "speed" (missing = speed); 2026-11-19)
  {"cmd": "arm.move_linear", "args": {"delta_mm": [0, 0, 100], "speed_percent": 10}}   job: fingertip straight by delta
  v3 (2026-10-24, spec "Gripper release, hold and calibration"):
  {"cmd": "arm.gripper_release"}                          the gripper motor only goes free (the arm keeps holding)
  {"cmd": "arm.gripper_hold"}                             the gripper motor grips where the jaws are (5 N)
  {"cmd": "arm.gripper_cal", "args": {"step": "closed"}}  / {"step": "open", "gap_mm": 35.2} / {"step": "cancel"}
  {"cmd": "teach.tag", "args": {"tag_id": 3}}             the stored fine result of a tag (calib/teach/tags.yaml);
                                                          + "family": "tag36h11" = that family's tag only (2026-11-16)
  {"cmd": "arm.teach_exit"}                               the controller out of teach (drag) mode (enables nothing)
  {"cmd": "arm.move_tip", "args": {"tip_mm_deg": [...6], "linear": true, "speed_percent": 10}}   job: fingertip to a pose
  v4 (2026-10-24, spec "Release = the controller's drag-teach mode"): arm.release stays joints 1-6 limp — the arm's
  drag-teach mode is its wrist button (torque released) and cannot be entered from the PC; arm.hold also leaves
  that teach mode (the controller's resume only when every joint is unpowered already) and holds
  v6 (2026-10-25, the twin's "Run workflow"): {"cmd": "bunny.tare"} / {"cmd": "bunny.weigh_to", "args": {"target_g": 0.02,
  "material": "powder", "powder": "<name or barcode id, '' = measure fresh>"}} / {"cmd": "bunny.dispense_volume",
  "args": {"volume_ml": 10}} — jobs; `stop` aborts a dose (/bunny/abort)
  2026-11-20 (the Bunny's platform height, its server 82cc822): {"cmd": "bunny.platform_set", "args": {"mm": 53.4}}
  (controller) -> /bunny/platform_set: tells the Bunny its platform height in ITS mm (0 = the travel's bottom, + up), as
  the station measured it (tag 3); the Bunny checks the range and refuses while its platform moves -> {platform_mm}; an
  older Bunny server answers its own error ("unknown cmd platform_set"). state.bunny then carries, from a fresh status
  that has them, platform_mm (null = unknown), platform_known, platform_source (the Bunny's word: unknown | told |
  estimated), platform_why (why unknown, else null) and boot_id — absent when the Bunny does not send them
  v5 (2026-10-24, spec "Emergency stop button"): state.arm.estop — true while the e-stop button is pressed (the arm
  node holds the arm where it is and refuses every command; out again, the arm is ready at once, still holding)
  2026-11-16 (spec "the ChArUco board"): state.camera.charuco {seen, corners, corners_total, distance_m, reproj_err_px,
  age_s} from /camera/charuco; {"cmd": "calib.handeye", "args": {"target": "charuco"}} = the hand-eye on the board
  2026-11-16 (spec "the board in the arm base"): state.camera.charuco also pose_base {R, p_mm}, centre_base_mm,
  joints_dt_s, handeye_at (the table calibration's input)
  2026-11-16 (spec "the absolute calibration"): {"cmd": "calib.absolute", "args": {"action": "begin" | "collect"
  (position 1..5: a job) | "solve" (a job) | "install" | "status"}}; state.absolute = the status
  2026-11-17 (spec "the start-posture check"): state.calib_precheck {ok, reachable, of, joints, unreachable, why, board_seen,
  at}; calib.handeye / calib.absolute collect refused up front with its words; 15 viewpoints
  2026-11-18 (spec "the re-pose"): + fixable, needs_repose, words (plain), repose; ok = the routine can go (it re-poses itself)
  2026-11-16 (spec "the nodes' watchdog"): {"cmd": "nodes.status"} -> {nodes: [...], watchdog: {running, at}} (anyone);
  {"cmd": "nodes.restart", "args": {"node": "camera" | "all"}} -> {ok, restarted} (controller; at once, detached
  `harness nodes restart`, the gateway itself last); state.nodes / state.watchdog = the same

The gateway owns no hardware: it is an rclpy client of the module nodes, like harness_cmd.py. Releasing the
arm is the explicit arm.release (operator 2026-10-23; before that it was CLI-only). `stop` is accepted from
anyone. The controller's connection is a dead-man: silent for DEADMAN_S (or closed) while its motion or job
runs -> the goal is cancelled / the job aborted (the arm freezes, stays enabled) and the control lease is dropped.

  python3 tools/gateway_server.py          serve (process `gateway` under `harness nodes`)
  python3 tools/gateway_server.py --key    create the shared key if missing, print it (`harness gateway key`)
"""
import json, math, os, re, secrets, signal, socketserver, subprocess, sys, threading, time

# THE GATEWAY'S ROS LINKS ARE UDP ONLY (2026-11-19): with Fast DDS's shared-memory transport, the gateway's own data port
# (/dev/shm/fastdds_port16417) was unlinked and re-created by another process's "zombie port" check after the 09:52
# restart while the gateway kept listening on the old, deleted segment. The nodes that had opened it before (the arm:
# joints) still reached the gateway; every node started after that (the camera node, restarted by the watchdog and by
# nodes.restart) wrote into a segment nobody reads: camera.online false with age_s None for an hour while the camera
# ran at 13.6 fps and `ros2 topic hz /camera/tags` saw every frame. A UDP socket cannot be orphaned that way (same-host
# traffic goes over loopback). Set before rclpy loads Fast DDS; an explicit FASTDDS_BUILTIN_TRANSPORTS wins.
os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

PORT = int(os.environ.get("GATEWAY_PORT", 7803))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY_FILE = os.path.expanduser("~/.config/railab-harness/gateway.key")
GRIP_RAMP_STEP_M = 0.0015       # arm.gripper speed_mm_s: the ramp's step (within the node's 2 mm tolerance), one every step / speed s
# arm.gripper {"end": "contact"} (2026-11-22, a real run: a close to 0 mm took ~10 s — the long PLA L fingers with thick
# rubber pads BEND, so the jaws kept following the ramp, 17 mm past contact on a 23 mm vial, and the node answered only when
# the motor had finished bending them): the ramp stops AT CONTACT (a step answers stalled, or the jaws, once under way, lag
# the ramp by more than one step), the final target is set at once at the same effort and the reply comes after
# GRIP_CONTACT_SQUEEZE_S (the node keeps that target: the squeeze goes on during the lift). Jaws still closing faster than
# GRIP_FREE_MM_S then = no contact after all (they ran free): waited for like an empty close. AN EMPTY CLOSE answers only
# once the jaws read within GRIP_DONE_TOL_M of the target or have stopped (GRIP_STILL_M in GRIP_STILL_S) — never early
# inside the node's 2 mm tolerance (the station calls a gap <= 1 mm "nothing in the jaws"; an empty close reads 0.0).
GRIP_CONTACT_SQUEEZE_S = 0.8
GRIP_FREE_MM_S = 10.0
GRIP_DONE_TOL_M = 0.001
GRIP_STILL_M, GRIP_STILL_S = 0.0003, 0.4
PIPER_YAML = os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")
# the calibration / planner packages from source, so a new module there needs no colcon rebuild
sys.path[:0] = [os.path.join(ROOT, "src", "harness_calibration"), os.path.join(ROOT, "src", "harness_planner")]
KIN_FILE = os.path.join(ROOT, "calib", "kinematics.yaml")
HANDEYE_FILE = os.path.join(ROOT, "calib", "handeye.yaml")
TEACH_TAGS = os.path.join(ROOT, "calib", "teach", "tags.yaml")      # teach.fine results, by tag id (+ by "family:id")
CV_FAMILIES = ("tag16h5", "tag25h9", "tag36h11")   # the only families stored before 2026-11-16 (records without a family)
VERSION = 5
DEADMAN_S = 10.0                 # (3 until 2026-11-19, relaxed: a network hiccup froze the arm)
PUMP_ERR_LOG_S = 10.0            # a connection's state pump logs a skipped tick at most this often (2026-11-24)
HOLDER_STALE_S = 10.0     # a control holder silent this long (no ping, no command) can be taken over
JOINTS = ["joint%d" % i for i in range(1, 7)]
ARM_ACTIONS = ("/arm/move_j", "/arm/move_pose", "/arm/follow_trajectory")


def tag_words(tid, family=None):
    """'3 (36h11)' / '0 (52h13)' when the family is known, else '3'."""
    m = re.search(r"\d+h\d+$", str(family or ""))
    return "%d (%s)" % (int(tid), m.group(0) if m else family) if family else "%d" % int(tid)


def said(err, detail=""):
    """An error a person reads says at most 10 words (operator 2026-11-21: "所有报错信息不能超过10个单词。捡重要的信息说");
    its long words ride along as err.detail: the reply's / the job result's `detail` and the gateway log, never shown by
    default. Returns err, to raise."""
    err.detail = detail
    return err


def clip(text, n=10):
    """At most n words (2026-11-21): a routine's raw output line cut, "…" where it was (the lines stay in state.calib)."""
    w = str(text).split()
    return " ".join(w[:n - 1]) + " …" if len(w) > n else str(text)
HANDEYE_ACCEPT_RMS_MM = 3.0      # calib.handeye installs a new calibration only at or below this rms
CLI = os.path.join(ROOT, "tools", "harness")
RESTART_LOG = "/tmp/nodes_restart.log"     # nodes.restart's detached shell
NODES_READ_S = 2.0               # the watchdog's status file is read at most this often
WATCHDOG_FRESH_S = 10.0          # ... and counts as running while it is this fresh (it writes every 3 s)
ABS_DIR = os.path.join(ROOT, "calib", "absolute")    # calib.absolute: one folder per session, CURRENT names the open one
ABS_MIN_CORNERS = 20
BACKUP_DIR = os.path.join(ROOT, "calib", "handeye_backups")
ABS_POSITIONS = 5
PRECHECK_S = 1.0                 # state.calib_precheck: recomputed this often while the arm holds (the joints moved
PRECHECK_SAME_DEG = 0.2          # ... more than this, or the target moved and the result is older than PRECHECK_MAX_AGE_S)
PRECHECK_MAX_AGE_S = 30.0
PRECHECK_SAME_TARGET_MM, PRECHECK_SAME_TARGET_DEG = 5.0, 2.0     # the target (board / tag) in the camera: "not moved"
# THE CHECK STEPS ASIDE (2026-11-22, a real run: the next path started ~5 s late — the start-posture check's re-pose search,
# 24 rolls x 9 orbits, ran in this process beside arm.follow_joints' plan_smooth_path on a busy 4-core i5): no round while a
# job / a gateway motion / a live jog / a RUN COMMAND runs, nor within PRECHECK_HOLDOFF_S after the last run command began or
# ended (a station run = a chain of these with gaps of ~0.1-3 s: arrival checks, the Bunny's state reads; a dose is a job);
# a round under way stops between two candidates when one starts (precheck.Interrupted; the state keeps the last result).
# And an idle arm is not re-assessed every PRECHECK_MAX_AGE_S any more: only when the joints or the target moved.
PRECHECK_HOLDOFF_S = 10.0
RUN_CMDS = frozenset(("arm.move_j", "arm.follow_joints", "arm.move_linear", "arm.move_tip", "arm.gripper", "teach.fine",
                      "bunny.housing", "bunny.platform", "bunny.platform_set", "bunny.tare", "bunny.weigh_to",
                      "bunny.dispense_volume"))
CHARUCO_JOINTS_DT_S = 0.2        # camera.charuco.pose_base: the joints sample used was received at most this far from the
                                 # board's frame (the joints arrive at 100 Hz; a moving arm would skew the pose)
HANDEYE_SETTLE_S = 3.0           # wait after every hand-eye move before the images (operator: the arm shakes ~3 s)
TEACH_VEL_RAD_S, TEACH_ACC_RAD_S2 = 0.35, 0.7     # arm.follow_joints: joint speed / acceleration of the path
# arm.follow_joints {speed, corner_deg} (2026-11-18, operator: "在非装载线的步骤，把机械臂的运行速度提升一倍，且大幅提高waypoint节点的
# 速度，不要有卡顿感"): `speed` scales TEACH_VEL / TEACH_ACC (1 … FOLLOW_SPEED_MAX); `corner_deg` = the turn below which a
# waypoint is passed without stopping (trajectory.CORNER_DEG 5 … FOLLOW_CORNER_MAX): a dense planner path turns a few
# degrees at many knots and stopped at each
FOLLOW_SPEED_MAX, FOLLOW_CORNER_MAX = 2.0, 45.0
# ... but never past the arm node's last gate (piper.yaml traj_max_joint_vel_rad_s, refused above + 10 %: 2026-11-18
# "point 329: joint 5 would turn at 0.70 rad/s > 0.66" when it was 0.6; the operator then raised it to 0.75): the path is
# planned to at most this = 2 x TEACH_VEL_RAD_S
FOLLOW_VEL_CAP_RAD_S = 0.70
# THE PER-JOINT SPEED FACTORS (2026-11-23; THE BASE JOINT TWICE AS FAST of 2026-11-19 folded in): arm.follow_joints
# plans every joint at its factor x the speed AND acceleration above (the cap too) — piper.yaml traj_joint_vel_factors,
# read by harness_arm.kinematics.joint_vel_factors (the arm node's gate reads the same list; no copy here):
# [2, 1, 1, 1, 3, 3] = 1.40 / 0.70 / 0.70 / 0.70 / 2.10 / 2.10 rad/s at speed 2. The controller's ceiling while it
# streams: piper.yaml traj_stream_speed_percent
FOLLOW_START_TOL_RAD = 0.35      # its first waypoint: this close to where the arm is (0.1 until 2026-11-19, relaxed)
FOLLOW_CLAMP_RAD = math.radians(10.0)  # waypoints beyond the controller limits by up to this are clamped inside (3 until 2026-11-19)
FOLLOW_RECOVER_RAD = math.radians(20.0) # an arm held past a limit by up to this: the path first brings it back inside (6 until 2026-11-19)
VIEW_CONE_DEG = 22.0             # teach.plan_views: an approximate centre view keeps the tag this close to the camera axis
VIEW_ROOM_RAD = math.radians(4.0)      # ... and this much more room from the joint limits (so the tilts fit)


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


BUNNY_KEYS = ("unit", "housing", "weight_g", "dispensing", "busy", "load_open")
# the housing as read from the motor (2026-11-19, the Bunny's new status fields)
BUNNY_HOUSING_KEYS = ("housing_pos", "housing_powered", "housing_enabled", "housing_frac", "housing_age_s", "housing_error")


def bunny_view(b):
    """state.bunny from the Bunny's fresh /bunny/status (None = not fresh). The housing and platform fields are
    forwarded only when the Bunny sends them: an older Bunny's reply has none of them, and none is added.
    THE PLATFORM HEIGHT (2026-11-20, the Bunny's server 82cc822 — zstage.py): its platform_mm (+ up, null = unknown),
    platform_source (unknown | told | estimated) and platform_why (why unknown, else null) as they are, platform_known =
    a height it knows (source not unknown and a number), and its boot_id (new at every start of its server)."""
    bunny = {"online": bool(b and b.get("online"))}
    if not b:
        return bunny
    bunny.update({k: b.get(k) for k in BUNNY_KEYS})
    bunny.update({k: b[k] for k in BUNNY_HOUSING_KEYS if k in b})
    if "platform_source" in b or "platform_mm" in b:
        mm, src = b.get("platform_mm"), b.get("platform_source")
        bunny.update(platform_mm=mm, platform_source=src, platform_why=b.get("platform_why"),
                     platform_known=bool(mm is not None and src != "unknown"))
    if "boot_id" in b:
        bunny["boot_id"] = b["boot_id"]
    return bunny


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
        from harness_msgs.action import FollowTrajectory, MoveJ, MovePose
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
        import collections
        self.joint_hist = collections.deque(maxlen=300)      # (received, joint_states): ~3 s at 100 Hz

        def on_joints(m):
            now = time.time()
            self.last["joints"] = (m, now)
            self.joint_hist.append((now, m))
        n.create_subscription(JointState, "/joint_states", on_joints, 10)
        n.create_subscription(String, "/bunny/status", lambda m: self.last.__setitem__("bunny", (json.loads(m.data), time.time())), 10)
        n.create_subscription(TagArray, "/camera/tags", put("tags"), 10)
        try:                                 # the ChArUco board (harness_msgs of 2026-11-16 or later)
            from harness_msgs.msg import CharucoDetection

            def on_board(m):
                now = time.time()
                self.last["charuco"] = (m, now)
                if m.seen:
                    self.last["charuco_seen"] = (m, now)
            n.create_subscription(CharucoDetection, "/camera/charuco", on_board, 10)
        except ImportError:
            print("gateway: harness_msgs has no CharucoDetection (built before 2026-11-16): camera.charuco stays empty",
                  flush=True)
        self.MoveJ, self.FollowTrajectory, self.MovePose = MoveJ, FollowTrajectory, MovePose
        self.ac_j = ActionClient(n, MoveJ, "/arm/move_j")
        self.ac_t = ActionClient(n, FollowTrajectory, "/arm/follow_trajectory")
        self.ac_p = ActionClient(n, MovePose, "/arm/move_pose")
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

    def gripper_calib(self):
        """The gripper's mm calibration in use ({date, scale, max_gap_mm}), from gripper_calib_file; None without one."""
        import yaml
        path = os.path.expanduser(str(self.P.get("gripper_calib_file") or ""))
        try:
            mt = os.path.getmtime(path)
        except OSError:
            return None
        if getattr(self, "_gc", (None, None))[0] != mt:
            try:
                with open(path) as f:
                    d = yaml.safe_load(f) or {}
                self._gc = (mt, {"date": d.get("date"), "scale": d.get("scale"),
                                 "max_gap_mm": round(float(d["max_gap_m"]) * 1000, 1)})
            except (OSError, KeyError, TypeError, ValueError):
                self._gc = (mt, None)
        return self._gc[1]

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
        last = self.last.get("tags")
        out = {"online": m is not None, "handeye": self.he_info, "tags": [],
               "age_s": round(time.time() - last[1], 1) if last else None,      # None = no camera frame since the gateway started
               "charuco": self.charuco_state()}
        if m is None or q is None or self.Tfc is None:
            return out
        Tbc = self.kin.fk(q) @ self.Tfc
        out["cam_mm"] = [round(float(v) * 1000, 2) for v in Tbc[:3, 3]]
        for t in m.tags:
            Tbt = Tbc @ T_from_pose_msg(t.pose)
            out["tags"].append({"id": int(t.id), "family": str(t.family), "size_px": round(t.size_px, 1),
                                "center_px": [round(v, 1) for v in t.center_px],
                                "distance_mm": round(t.distance_m * 1000, 1), "assumed_mm": round(t.size_m * 1000, 2),
                                "base_mm": [round(float(v) * 1000, 2) for v in Tbt[:3, 3]],
                                "base_R": [[round(float(v), 6) for v in row] for row in Tbt[:3, :3]],
                                "reproj_px": round(t.reproj_err_px, 2)})
        return out

    def nodes_view(self):
        """The watchdog's status (tools/node_watchdog.py, ~/.cache/harness_watchdog.json), read at most every
        NODES_READ_S: {nodes: [{name, running, healthy, why, restarts, last_restart_at, auto, ...}], watchdog:
        {running, at}} - running = its file is fresh and its process alive."""
        now = time.time()
        c = getattr(self, "_nodes_cache", None)
        if c is not None and now - c[0] < NODES_READ_S:
            return c[1]
        import node_watchdog
        try:
            with open(node_watchdog.STATUS_FILE) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            doc = {}
        at, pid = doc.get("at"), doc.get("pid")
        alive = False
        if pid:
            try:
                os.kill(int(pid), 0)
                alive = True
            except (OSError, ValueError):
                pass
        v = {"nodes": doc.get("nodes") or [],
             "watchdog": {"running": bool(alive and at and now - float(at) < WATCHDOG_FRESH_S), "at": at}}
        self._nodes_cache = (now, v)
        return v

    def joints_near(self, t):
        """(joints rad, |received - t| s) of the /joint_states sample received nearest to time t, or (None, None).
        THREAD SAFETY (2026-11-24): the joint-state callback appends to joint_hist while this runs (the state pump, the
        precheck worker): min() with a Python key over the live deque raised "deque mutated during iteration" and killed
        the connection's state pump. A snapshot first (list() = one C-level copy under the GIL; retried if it ever races)."""
        snap = []
        for _ in range(5):
            try:
                snap = list(self.joint_hist)
                break
            except RuntimeError:
                continue
        best = min(snap, key=lambda e: abs(e[0] - t), default=None)
        if best is None or not all(j in best[1].name for j in JOINTS):
            return None, None
        return [float(best[1].position[best[1].name.index(j)]) for j in JOINTS], abs(best[0] - t)

    def grip_to_contact(self, gap, force, speed, g):
        """arm.gripper {end: "contact"}, a close (see GRIP_CONTACT_SQUEEZE_S; g = the jaws' last reading or None) ->
        {gap_mm, stalled, contact, message}: gap_mm = the jaws' reading when answering; contact = something holds them
        short of the target (stalled = contact)."""
        from harness_msgs.srv import SetGripper
        t0 = time.time()
        timeout = float(self.P.get("gripper_timeout_s", 10.0))
        at = None                                        # the jaws' reading at contact (m)
        if speed > 0 and g is not None and float(g.width_m) - gap > GRIP_RAMP_STEP_M:      # a ramped close
            w0 = w = float(g.width_m)
            t_last = time.time()
            while w - gap > GRIP_RAMP_STEP_M:
                w -= GRIP_RAMP_STEP_M
                r = self.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=w, force=force, timeout_s=2.0), 10)
                t_last += GRIP_RAMP_STEP_M / speed
                time.sleep(max(0.0, t_last - time.time()))
                gn = self.fresh("grip", 2.0)
                now_w = float(gn.width_m) if gn is not None else float(r.width_m)
                # (lag counts once the jaws are under way: a slow start is not a contact; held at the start = stalled)
                if r.stalled or (gn is not None and w0 - now_w >= GRIP_RAMP_STEP_M and now_w > w + GRIP_RAMP_STEP_M):
                    at = now_w
                    break
        if at is not None:
            # the final target at once, at the same effort (the node's answer not awaited: 0.1 s); the node keeps it
            # after this reply, so the squeeze goes on during the lift
            t_sq = time.time()
            self.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=gap, force=force, timeout_s=0.1), 10)
            got = self.grip_wait(gap, t_sq + timeout, t_min=GRIP_CONTACT_SQUEEZE_S, since=t_sq,
                                 still_m=GRIP_FREE_MM_S / 1000.0 * 0.1, still_s=0.1)
            if got is not None and got - gap > GRIP_DONE_TOL_M:
                msg = "contact at %.1f mm, held at %.1f mm" % (at * 1000, got * 1000)
                print("arm.gripper: %s (%.2f s)" % (msg, time.time() - t0), flush=True)
                return {"gap_mm": round(got * 1000, 2), "stalled": True, "contact": True, "message": msg}
            print("arm.gripper: lag at %.1f mm, then closed to the target: no contact" % (at * 1000), flush=True)
        # no contact on the way (an empty close, no ramp, an opening): the final target, the node's own answer, then
        # never early inside its tolerance
        r = self.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=gap, force=force), timeout + 10)
        if not r.success:
            raise RuntimeError(r.message)
        if r.stalled:
            got = float(r.width_m)
        else:
            got = self.grip_wait(gap, time.time() + 2.0, still_m=GRIP_STILL_M, still_s=GRIP_STILL_S)
            got = float(r.width_m) if got is None else got
        contact = bool(r.stalled) and got - gap > GRIP_DONE_TOL_M
        msg = r.message if r.stalled else "width %.1f mm" % (got * 1000)
        print("arm.gripper: %s (%.2f s, contact %s)" % (msg, time.time() - t0, contact), flush=True)
        return {"gap_mm": round(got * 1000, 2), "stalled": bool(r.stalled), "contact": contact, "message": msg}

    def grip_wait(self, gap, deadline, still_m, still_s, t_min=0.0, since=None):
        """The jaws' reading (m) once t_min has passed (from `since`, default now) and they read within GRIP_DONE_TOL_M of
        gap or moved less than still_m over the last still_s; at the deadline in any case (None: no reading at all)."""
        t0 = time.time() if since is None else since
        hist, w = [], None
        while True:
            gn, now = self.fresh("grip", 2.0), time.time()
            if gn is not None:
                w = float(gn.width_m)
                hist.append((now, w))
                while len(hist) > 2 and now - hist[1][0] >= still_s:
                    hist.pop(0)                          # (hist[0] = the newest sample at least still_s old)
                if now - t0 >= t_min:
                    if abs(w - gap) <= GRIP_DONE_TOL_M:
                        return w
                    if now - hist[0][0] >= still_s and abs(w - hist[0][1]) < still_m:
                        return w
            if now >= deadline:
                return w
            time.sleep(0.02)

    def precheck_target(self):
        """The calibration target in the camera (4 x 4, its centre): the ChArUco board when seen with >= 20 corners,
        else the largest tag in view, else None."""
        import numpy as np
        from harness_vision.geom import T_from_pose_msg
        b = self.fresh("charuco", 1.0)
        if b is not None and b.seen and b.corners >= ABS_MIN_CORNERS:
            C = np.eye(4)
            C[:3, 3] = [b.squares_x * b.square_m / 2.0, b.squares_y * b.square_m / 2.0, 0.0]
            return T_from_pose_msg(b.pose) @ C
        m = self.fresh("tags", 1.0)
        if m is not None and m.tags:
            return T_from_pose_msg(max(m.tags, key=lambda t: t.size_px).pose)
        return None

    def update_precheck(self, force=False, stop=None):
        """state.calib_precheck (2026-11-17 / 18): the calibrations' start-posture check + the re-pose search
        (harness_calibration/precheck.py assess, the same the routines run before any motion) for where the arm holds
        now: the joints' margins always; the 15 views and, when they fail, the search for a posture the arm can turn to
        by itself (keeping the camera on the target) while the arm holds (recomputed when the joints moved). ok = the
        routine can go (as is or after its own re-pose); words = plain, for the page; why = technical, for the logs.
        stop (precheck_worker's gate): polled between candidates -> precheck.Interrupted, nothing updated."""
        from harness_calibration import precheck as PC
        from harness_calibration.handeye import VIEW_SET
        now, q = time.time(), self.joints_rad()
        st = self.fresh("arm", 1.0)
        b = self.charuco_state()
        base = {"board_seen": b["seen"], "board_corners": b["corners"], "at": round(now, 2)}
        if q is None:
            pc = {"ok": False, "reachable": None, "of": len(VIEW_SET), "joints": [], "unreachable": [], "why": "no arm feedback",
                  "fixable": False, "needs_repose": False, "words": "the arm does not answer: check that it is on"}
        elif st is None or not st.all_enabled:
            pc = {"ok": False, "reachable": None, "of": len(VIEW_SET), "joints": PC.joint_margins(self.kin, q), "unreachable": [],
                  "why": "the arm is not holding", "fixable": False, "needs_repose": False,
                  "words": "aim the camera at the board, then press Hold"}
        else:
            last = getattr(self, "_pc_last", None)
            T = self.precheck_target()
            if not force and last is not None and precheck_reusable(self, last, q, T, now):
                pc = last[2]
                self._pc_last = (now,) + tuple(last[1:])        # still the result for what the arm holds and sees now
            else:
                pc = PC.assess(self.kin, q, self.P, self.Tfc, T, stop=stop)
                rp = pc.pop("repose")
                pc["repose"] = None if rp is None else {k: rp[k] for k in ("roll_deg", "orbit", "flip", "margin_deg",
                                                                           "reachable", "of", "motion_deg")}
                # (checked at, joints, result, target, model, hand-eye, computed at); precheck_cached reads the first three
                self._pc_last = (now, list(q), pc, T, self.kin, self.Tfc, now)
        self.precheck = dict(pc, **base)
        return self.precheck

    def precheck_cached(self):
        """The last start-posture assessment when it is for the joints the arm holds now (the 1 Hz worker keeps it), else
        None - the commands never wait for the re-pose search (seconds); the routine runs its own."""
        last, q = getattr(self, "_pc_last", None), self.joints_rad()
        if last is None or q is None or time.time() - last[0] > PRECHECK_MAX_AGE_S:
            return None
        if max(abs(a - c) for a, c in zip(q, last[1])) >= math.radians(PRECHECK_SAME_DEG):
            return None
        return last[2]

    def reload_model(self):
        """calib/kinematics.yaml + calib/handeye.yaml read again (after calib.absolute install)."""
        from harness_arm.kinematics import PiperKinematics
        self.kin = PiperKinematics.from_file(KIN_FILE, str(self.P.get("kinematics_model", "piper")))
        self.load_handeye()

    def absolute_view(self):
        """state.absolute: absolute_status(), read from disk at most every NODES_READ_S."""
        now = time.time()
        c = getattr(self, "_abs_cache", None)
        if c is None or now - c[0] >= NODES_READ_S:
            c = self._abs_cache = (now, absolute_status())
        return c[1]

    def charuco_state(self):
        """camera.charuco (2026-11-16): seen = the board's pose solved on a frame at most 1 s old; corners = inner corners
        found on that frame (of corners_total); distance_m (camera -> the board's centre) / reproj_err_px / age_s = the
        newest frame the board was seen on and its age (None: not seen since the gateway started).
        pose_base {R (3 x 3 rows), p_mm} = the board frame (OpenCV's: origin the sheet's top-left outer corner, x along
        the squares_x squares, y down the squares_y, z into the board) in the ARM BASE, and centre_base_mm = the board's
        centre there: the corrected FK of the joints x calib/handeye.yaml (handeye_at = its date) x the camera -> board
        pose, as for the tags; null unless the board is seen now, a hand-eye is loaded and the /joint_states sample
        nearest to the board's frame was received within CHARUCO_JOINTS_DT_S (joints_dt_s) of it."""
        b, sn = self.fresh("charuco", 1.0), self.last.get("charuco_seen")
        out = {"seen": bool(b is not None and b.seen), "corners": int(b.corners) if b is not None else 0,
               "corners_total": int(b.corners_total) if b is not None else (int(sn[0].corners_total) if sn else None),
               "distance_m": round(float(sn[0].distance_m), 4) if sn else None,
               "reproj_err_px": round(float(sn[0].reproj_err_px), 3) if sn else None,
               "age_s": round(time.time() - sn[1], 1) if sn else None,
               "pose_base": None, "centre_base_mm": None, "joints_dt_s": None, "handeye_at": self.he_info.get("date")}
        if out["seen"] and self.Tfc is not None:
            import numpy as np
            from harness_vision.geom import T_from_pose_msg
            q, dt = self.joints_near(self.last["charuco"][1])
            if q is not None and dt <= CHARUCO_JOINTS_DT_S:
                Tbb = self.kin.fk(q) @ self.Tfc @ T_from_pose_msg(b.pose)
                c = Tbb @ np.array([b.squares_x * b.square_m / 2.0, b.squares_y * b.square_m / 2.0, 0.0, 1.0])
                out.update(pose_base={"R": [[round(float(v), 6) for v in row] for row in Tbb[:3, :3]],
                                      "p_mm": [round(float(v) * 1000, 2) for v in Tbb[:3, 3]]},
                           centre_base_mm=[round(float(v) * 1000, 2) for v in c[:3]], joints_dt_s=round(dt, 3))
        return out

    def teach_client(self):
        if self.hc is None:
            from harness_calibration.client import HarnessClient
            self.hc = HarnessClient("gateway_teach")
        return self.hc

    def plan_views(self, q_model_deg, backoff_mm, tilt_deg, tag_base_mm=None):
        """teach.plan_views, a dry run: the five-view cross for a viewing posture (corrected-model degrees) with the
        camera backoff_mm further back along its optical axis and +-tilt_deg tilts. Every view solved by IK inside
        the joint limits (seeded from the posture: same wrist / elbow branch), its fingertip checked against the
        floor and the workspace box; nothing moves. Obstacles are the caller's (the twin's) to check.
        A posture held past a joint limit (the arm was limp) cannot be reproduced exactly: with tag_base_mm (where
        the tag is, arm base) the centre becomes the nearest view inside the limits with VIEW_ROOM_RAD to spare,
        if the tag stays within VIEW_CONE_DEG of the camera axis; the tilts are then around that centre."""
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
        seed = np.clip(q, lo + 1e-6, hi - 1e-6)
        held = "; ".join("joint %d was held at %.1f deg (limit %.1f)" % (j + 1, math.degrees(q[j]), math.degrees(lo[j] - margin if q[j] < lo[j] else hi[j] + margin))
                         for j in range(6) if q[j] < lo[j] - margin or q[j] > hi[j] + margin)
        note, centre = "", None
        qs, pe, ae = self.kin.ik(T0, seed, margin=margin)
        if pe <= 1e-3 and ae <= 5e-3:
            centre = qs
        elif tag_base_mm is not None:                      # the nearest view inside the limits, room for the tilts
            qa, pa, aa = self.kin.ik(T0, seed, margin=margin + VIEW_ROOM_RAD)
            Tc = self.kin.fk(qa) @ self.Tfc
            v = np.array(tag_base_mm, float) / 1000.0 - Tc[:3, 3]
            off_axis = math.degrees(math.acos(max(-1.0, min(1.0, float(Tc[:3, 2] @ v) / max(1e-9, float(np.linalg.norm(v)))))))
            if off_axis <= VIEW_CONE_DEG:
                centre = qa
                note = ("the nearest view inside the joint limits: %.0f mm / %.1f deg from the one asked for%s; the tag %.0f deg off the camera axis"
                        % (pa * 1000, math.degrees(aa), " (" + held + ")" if held else "", off_axis))
            else:
                pe, ae = pa, aa
        out, reasons, q_c = [], [], None
        T_c = self.kin.fk(centre) if centre is not None else T0
        for name, T in cross_targets(T_c, self.Tfc, float(tilt_deg)):
            if name == "centre":
                qs = centre if centre is not None else qs
            else:
                qs, pe, ae = self.kin.ik(T, q_c, margin=margin)
            why = detail = None
            if name == "centre" and centre is None:
                why = "out of reach (best %.1f mm / %.2f deg)" % (pe * 1000, math.degrees(ae))
                detail = ("out of reach inside the joint limits (best %.1f mm / %.2f deg)" % (pe * 1000, math.degrees(ae))
                          + ("; " + held if held else "") + ("; the nearest reachable view loses the tag" if tag_base_mm is not None else ""))
            elif name != "centre" and (pe > 1e-3 or ae > 5e-3):
                why = "out of reach (best %.1f mm / %.2f deg)" % (pe * 1000, math.degrees(ae))
            else:
                Tq, _, clear = self.tip_and_clearance(list(qs))
                if self.P.get("floor_enabled", False) and clear < fl_margin:
                    why = "fingertip %.0f mm above the floor (margin %.0f mm)" % (clear * 1000, fl_margin * 1000)
                elif wlo and whi and not all(wlo[i] <= Tq[i, 3] <= whi[i] for i in range(3)):
                    why = "gripper base would leave the workspace box"
                elif q_c is not None and float(np.abs(qs - q_c).max()) > float(self.P.get("max_joint_delta_rad", 1.2)):
                    why = "a joint would jump %.0f deg (another wrist branch)" % math.degrees(float(np.abs(qs - q_c).max()))
            out.append({"view": name, "ok": why is None, "reason": why,
                        "joints_model_deg": [round(math.degrees(float(v + o)), 4) for v, o in zip(qs, off)]})
            if detail:
                out[-1]["detail"] = detail
            if why:
                reasons.append("%s: %s" % (name, why))
                if name == "centre":
                    break                                     # no centre view, no cross
            elif name == "centre":
                q_c = qs
        return {"ok": not reasons, "reasons": reasons, "note": note, "backoff_mm": float(backoff_mm), "tilt_deg": float(tilt_deg),
                "centre_model_deg": out[0]["joints_model_deg"], "views": out[1:]}

    def camera_frames(self):
        """Raise when the gripper camera sends no frames (2026-10-25: the D405 dropped off USB as the arm arrived at
        tag 3, and the silence read as "tag 3 is not in view from here")."""
        if self.fresh("tags", 1.0) is None:
            last = self.last.get("tags")
            raise said(RuntimeError("gripper camera sends no frames%s: check USB"
                                    % (" for %.0f s" % (time.time() - last[1]) if last else "")),
                       "the gripper camera sends no frames%s: check its USB cable (the camera node reconnects by itself)"
                       % (" (none for %.0f s)" % (time.time() - last[1]) if last else " since the gateway started"))

    def fine(self, tid, black_mm, abort, tilt_deg=3.0, views_model_deg=None, family=None):
        """teach.fine: the five-view cross on tag tid from where the camera looks now (harness_calibration.fine);
        views_model_deg {view: 6 corrected-model degrees} = go to exactly these (teach.plan_views) postures;
        family (tag16h5 / tag36h11, what the twin's model says the tag is; 2026-10-25) = only that family's tag tid."""
        import datetime as dt, numpy as np, yaml
        from harness_calibration.fine import Aborted, cross_locate
        from harness_arm.kinematics import xyzrpy_from_T
        if self.Tfc is None:
            raise RuntimeError("no hand-eye calibration (calib/handeye.yaml)")
        c = self.teach_client()
        c.wait_ready()
        if not (c.status and c.status.all_enabled):
            raise RuntimeError("the arm is not holding: press Hold first")
        self.camera_frames()
        t = c.tag(tid, family=family)
        if t is None:
            raise RuntimeError("tag %s is not in view from here" % tag_words(tid, family))
        scale = (black_mm / 1000.0) / t.size_m if black_mm > 0 else 1.0
        vj = None
        if views_model_deg:
            off = list(self.kin.offsets)
            vj = {k: [math.radians(float(v)) - o for v, o in zip(q, off)] for k, q in views_model_deg.items()}
        try:
            r = cross_locate(c, tid, self.Tfc, scale=scale, abort=abort.is_set, tilt_deg=float(tilt_deg), view_joints=vj,
                             family=family)
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
            rec = {
                "base_to_tag_matrix": T.tolist(), "base_to_tag_xyzrpy": xyzrpy_from_T(T), "spread_mm": r["spread_mm"],
                "spread_deg": r["spread_deg"], "black_mm": res["black_mm"], "size_px": r["size_px"], "joints_rad": q,
                "handeye_date": self.he_info.get("date"), "date": dt.datetime.now().isoformat(timespec="seconds"),
                "family": family}
            tags = doc.setdefault("tags", {})
            tags[int(tid)] = rec                  # by id, as before: teach.tag without a family = the last one located
            if family:                            # and by family + id: one number in two families (2026-11-16: 0 is
                tags["%s:%d" % (family, int(tid))] = dict(rec)   # the Bunny's 52h13 main tag and the capper's 36h11 tag)
            os.makedirs(os.path.dirname(TEACH_TAGS), exist_ok=True)
            with open(TEACH_TAGS, "w") as f:
                yaml.safe_dump(doc, f, sort_keys=False)
        return res

    def follow_joints(self, conn, wps_deg, label, abort, speed: float = 1.0, corner_deg=None, blend=False, speeds=None,
                      times_s=None):
        """arm.follow_joints: a planner's joint path (corrected-model degrees) -> validated, streamed trajectory
        (`speed` x the teach speed / acceleration, `corner_deg` the stop-at-corner turn: FOLLOW_SPEED_MAX / _CORNER_MAX).
        blend (2026-11-19, THE SMOOTH PATH, the workflow repo's docs/plans/no_pause_motion.md): ONE motion through all
        waypoints, at rest only at the first and the last (trajectory.plan_smooth_path; corner_deg unused); `speeds` =
        one factor per segment (K - 1), each clamped and capped like `speed` (missing = `speed` for every segment).
        times_s (2026-11-23, THE TIMED PATH): the caller's own timing, one time per waypoint — streamed as given
        (trajectory.plan_timed_path; speed / corner_deg unused), every sample within timed_caps().
        Without blend / speeds / times_s: exactly as before."""
        import numpy as np
        from harness_arm.kinematics import joint_vel_factors
        from harness_planner.demo import load_model
        from harness_planner.trajectory import PlanError, plan_joint_path, plan_smooth_path, plan_timed_path
        q = self.joints_rad()
        if q is None:
            raise RuntimeError("no arm feedback")
        st = self.fresh("arm", 1.0)
        if st is None or not st.all_enabled:
            raise RuntimeError("the arm is not holding: press Hold first")
        if not isinstance(wps_deg, list) or len(wps_deg) < 1 or any(not isinstance(w, list) or len(w) != 6 for w in wps_deg):
            raise ValueError("waypoints_model_deg needs a list of 6-joint waypoints")
        if speeds is not None:
            if (not isinstance(speeds, list) or len(speeds) != len(wps_deg) - 1
                    or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in speeds)):
                raise ValueError("speeds needs %d numbers (one per segment)" % (len(wps_deg) - 1))
        if times_s is not None:
            if (not isinstance(times_s, list) or len(times_s) != len(wps_deg)
                    or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in times_s)):
                raise ValueError("times_s needs %d numbers (one per waypoint)" % len(wps_deg))
        off = np.array(self.kin.offsets, float)
        W = np.radians(np.array(wps_deg, float)) - off             # corrected-model angles -> controller joints
        lo = np.array([l[0] for l in self.limits]) + 0.02
        hi = np.array([l[1] for l in self.limits]) - 0.02
        over = np.maximum(lo - W, W - hi).max(axis=1)
        over[0] = 0.0                                    # the first waypoint becomes where the arm is (below)
        if (over > FOLLOW_CLAMP_RAD).any():
            k = int(np.argmax(over))
            raise RuntimeError("refused: waypoint %d is %.1f deg beyond a joint limit" % (k + 1, math.degrees(over[k])))
        W = np.clip(W, lo, hi)
        qa = np.array(q)
        qin = np.clip(qa, lo, hi)                        # the arm's posture brought inside the limits
        out = np.abs(qa - qin)
        if out.max() > FOLLOW_RECOVER_RAD:
            j = int(np.argmax(out))
            raise said(RuntimeError("refused: joint %d %.1f deg past limit: Release, move back" % (j + 1, math.degrees(out[j]))),
                       "joint %d is held %.1f deg past its limit (more than %.0f): Release the arm and move it back by hand"
                       % (j + 1, math.degrees(out[j]), math.degrees(FOLLOW_RECOVER_RAD)))
        d0 = float(np.abs(W[0] - qin).max())
        if d0 > FOLLOW_START_TOL_RAD:
            raise said(RuntimeError("refused: the path starts %.1f deg from the arm; re-plan" % math.degrees(d0)),
                       "the path starts %.1f deg from where the arm is; plan it from the current posture" % math.degrees(d0))
        if times_s is not None:                          # THE TIMED PATH (2026-11-23): as given, from where the arm is
            kin, lim = load_model(ROOT)
            lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2 = self.timed_caps()
            try:
                tr = plan_timed_path(self.kin, W, times_s, lim, rate_hz=float(self.P.get("traj_rate_hz", 100.0)),
                                     start_q=qa, from_outside=True, max_points=int(self.P.get("traj_max_points", 20000)))
            except PlanError as e:
                raise said(RuntimeError("refused: %s" % e), getattr(e, "detail", ""))
            return self.stream_plan(tr, label, abort)
        W[0] = qin
        seg_k = None
        if blend:                                        # each segment's factor, like `speed` (THE SMOOTH PATH)
            base = float(speed or 1.0)
            seg_k = [float(x) for x in speeds] if speeds is not None else [base] * (len(W) - 1)
        if out.max() > 1e-6:                             # held past a limit (2026-10-24): first straight back inside
            W = np.vstack([qa, W])
            if seg_k is not None:
                seg_k = [float(speed or 1.0)] + seg_k        # ... at `speed`
        kin, lim = load_model(ROOT)
        from harness_planner.trajectory import CORNER_DEG
        k = max(0.25, min(FOLLOW_SPEED_MAX, float(speed or 1.0)))
        v = min(TEACH_VEL_RAD_S * k, FOLLOW_VEL_CAP_RAD_S)
        f = joint_vel_factors(self.P)                    # THE PER-JOINT SPEED FACTORS (piper.yaml)
        lim.max_joint_vel_rad_s = v * f
        lim.max_joint_acc_rad_s2 = TEACH_ACC_RAD_S2 * k * f
        corner = max(CORNER_DEG, min(FOLLOW_CORNER_MAX, float(corner_deg or CORNER_DEG)))
        try:
            if seg_k is not None:
                ks = np.clip(np.array(seg_k, float), 0.25, FOLLOW_SPEED_MAX)
                sv = np.minimum(TEACH_VEL_RAD_S * ks, FOLLOW_VEL_CAP_RAD_S)[:, None] * f
                sa = (TEACH_ACC_RAD_S2 * ks)[:, None] * f
                tr = plan_smooth_path(self.kin, W, lim, sv, sa, from_outside=True)
            else:
                tr = plan_joint_path(self.kin, W, lim, from_outside=True, corner_deg=corner)
        except PlanError as e:
            raise said(RuntimeError("refused: %s" % e), getattr(e, "detail", ""))
        return self.stream_plan(tr, label, abort)

    def timed_caps(self):
        """THE TIMED PATH's caps (2026-11-23), (speed rad/s, acceleration rad/s^2) per joint, each x its factor of THE
        PER-JOINT SPEED FACTORS (piper.yaml traj_joint_vel_factors): the speed = what the gateway plans at most
        (FOLLOW_VEL_CAP_RAD_S), never above the arm node's gate (traj_max_joint_vel_rad_s; the node refuses past it
        + 10 %); the acceleration = the smooth path's at the top speed factor (TEACH_ACC_RAD_S2 x FOLLOW_SPEED_MAX).
        Today [1.40, 0.70, 0.70, 0.70, 2.10, 2.10] rad/s and [2.8, 1.4, 1.4, 1.4, 4.2, 4.2] rad/s^2."""
        import numpy as np
        from harness_arm.kinematics import joint_vel_factors
        f = joint_vel_factors(self.P)
        gate = float(self.P.get("traj_max_joint_vel_rad_s", 0.6)) * f
        return np.minimum(FOLLOW_VEL_CAP_RAD_S * f, gate), TEACH_ACC_RAD_S2 * FOLLOW_SPEED_MAX * f

    def stream_plan(self, tr, label, abort):
        """A planned trajectory through /arm/follow_trajectory as this job; the abort event cancels it (the arm freezes)."""
        from harness_planner.demo import to_msg
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
            raise RuntimeError("rejected by the arm node (busy, fault, teach, disabled)")
        rf = gh.get_result_async()
        while not rf.done():
            if abort.is_set():
                gh.cancel_goal_async()
                raise RuntimeError(JOB.get("why") or "aborted - arm frozen")
            if time.time() - t0 > tr.duration + 60:
                gh.cancel_goal_async()
                raise RuntimeError("no result after the path's end - arm frozen")
            time.sleep(0.05)
        r = rf.result().result
        if not r.success:
            raise RuntimeError(r.message)
        return {"success": True, "message": r.message, "seconds": round(r.seconds, 1),
                "max_tracking_error_deg": round(math.degrees(r.max_tracking_error_rad), 2), "plan": tr.report}

    def move_linear(self, delta_mm, speed, abort):
        """arm.move_linear: the fingertip (and the gripper base with it: the orientation stays) on a straight line by
        delta_mm in the arm base frame — the node's /arm/move_pose linear (its IK, floor, workspace, jump checks)."""
        import numpy as np
        from harness_arm.kinematics import xyzrpy_from_T
        q = self.joints_rad()
        if q is None:
            raise RuntimeError("no arm feedback")
        T = self.kin.fk(q)
        T[:3, 3] += np.array(delta_mm, float) / 1000.0
        r = self.move_pose_goal([float(v) for v in xyzrpy_from_T(T)], True, speed, abort)
        return dict(r, delta_mm=[float(v) for v in delta_mm])

    def move_tip(self, tip_mm_deg, linear, speed, abort):
        """arm.move_tip (2026-10-24): the fingertip to an absolute pose [x y z mm, rx ry rz deg] in the arm base (flange
        orientation, fixed-axis XYZ), solved and drawn in the harness's calibrated model — the twin's precise steps
        (onto a grip, onto a mount) land where the tags were measured, not where the twin's own model would put them."""
        import numpy as np
        q = self.joints_rad()
        if q is None:
            raise RuntimeError("no arm feedback")
        flange = self.flange_from_tip(tip_mm_deg)
        far = float(np.linalg.norm(np.array(flange[:3]) - self.kin.fk(q)[:3, 3]))
        if far > 1.0:                                     # (0.4 until 2026-11-19, relaxed)
            raise said(RuntimeError("refused: target %.0f mm away (> 1000): free move first" % (far * 1000)),
                       "the target is %.0f mm away (> 1000 mm): plan a free move there first" % (far * 1000))
        return self.move_pose_goal(flange, bool(linear), speed, abort)

    def move_pose_goal(self, pose, linear, speed, abort):
        """One /arm/move_pose goal (flange pose, m / rad) as a job step; the abort event cancels it (the arm freezes)."""
        st = self.fresh("arm", 1.0)
        if st is None or not st.all_enabled:
            raise RuntimeError("the arm is not holding: press Hold first")
        if not self.ac_p.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("/arm/move_pose not available")
        g = self.MovePose.Goal(pose=[float(v) for v in pose], linear=bool(linear), speed_percent=int(speed), timeout_s=60.0)
        gf = self.ac_p.send_goal_async(g)
        t0 = time.time()
        while not gf.done():
            if time.time() - t0 > 5:
                raise RuntimeError("goal not accepted in time")
            time.sleep(0.01)
        gh = gf.result()
        if not gh.accepted:
            raise RuntimeError("rejected by the arm node (busy, fault, teach, disabled)")
        rf = gh.get_result_async()
        while not rf.done():
            if abort.is_set():
                gh.cancel_goal_async()
                raise RuntimeError(JOB.get("why") or "aborted - arm frozen")
            if time.time() - t0 > 90:
                gh.cancel_goal_async()
                raise RuntimeError("straight move timed out - arm frozen")
            time.sleep(0.05)
        r = rf.result().result
        if not r.success:
            raise RuntimeError(r.message)
        return {"success": True, "message": r.message}

    def stored_tag(self, tid, family=None):
        """teach.tag: a tag's last accepted fine result (calib/teach/tags.yaml, written by teach.fine): where it is
        (arm base) and the posture it was looked at from (the corrected model's degrees). family (2026-11-16): that
        family's tag tid only — its "family:id" record, else the id's record when it was located as that family (a
        record from before 2026-11-16 names no family: it counts for the OpenCV families only)."""
        import yaml
        try:
            with open(TEACH_TAGS) as f:
                doc = yaml.safe_load(f) or {}
        except OSError:
            doc = {}
        tags = doc.get("tags") or {}
        rec = tags.get("%s:%d" % (family, int(tid))) if family else None
        if rec is None:
            rec = tags.get(int(tid))
            if rec and family and rec.get("family", "") != family and (rec.get("family") or family not in CV_FAMILIES):
                rec = None                        # the id's last record is another family's tag
        if not rec:
            raise said(RuntimeError("tag %s not located accurately yet: calibrate first" % tag_words(tid, family)),
                       "tag %s has not been located accurately yet (calibrate its module: teach.fine)" % tag_words(tid, family))
        T = rec["base_to_tag_matrix"]
        q = rec.get("joints_rad")
        return {"tag_id": int(tid), "base_mm": [round(float(T[i][3]) * 1000, 2) for i in range(3)],
                "base_R": [[round(float(T[i][j]), 6) for j in range(3)] for i in range(3)],
                "joints_model_deg": [round(math.degrees(float(v) + o), 4) for v, o in zip(q, self.kin.offsets)] if q else None,
                "black_mm": rec.get("black_mm"), "spread_mm": rec.get("spread_mm"), "date": rec.get("date")}

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
                reasons.append("joint %d at %.1f deg is outside [%.1f, %.1f]" % (i + 1, math.degrees(qi), math.degrees(lo), math.degrees(hi)))
        now = self.joints_rad()
        jump = None
        if now is None:
            reasons.append("no arm feedback (/joint_states)")
        else:
            d = [abs(a - b) for a, b in zip(q, now)]
            k = max(range(6), key=lambda i: d[i])
            jump = {"joint": k + 1, "deg": round(math.degrees(d[k]), 2)}
            if d[k] > float(self.P.get("max_joint_delta_rad", 1.2)):
                reasons.append("joint %d jumps %.1f deg (> %.1f): smaller steps"
                               % (k + 1, math.degrees(d[k]), math.degrees(float(self.P.get("max_joint_delta_rad", 1.2)))))
        T, tip, clear = self.tip_and_clearance(q)
        path_clear = clear
        if now is not None:     # move_j is joint-interpolated: sample the straight joint-space path
            for s in range(1, 20):
                f = s / 20.0
                path_clear = min(path_clear, self.tip_and_clearance([a + (b - a) * f for a, b in zip(now, q)])[2])
        margin = float(self.P.get("floor_margin_m", 0.015))
        if self.P.get("floor_enabled", False) and path_clear < margin:
            reasons.append("fingertip %.1f mm above the floor (margin %.0f mm)" % (path_clear * 1000, margin * 1000))
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
                        "gripper_max_mm": (self.gripper_calib() or {}).get("max_gap_mm") or round(float(self.P.get("gripper_max_m", 0.07)) * 1000, 1),
                        "max_speed_percent": int(self.P.get("max_speed_percent", 30)),
                        "default_speed_percent": int(self.P.get("default_speed_percent", 10)),
                        "max_joint_delta_deg": round(math.degrees(float(self.P.get("max_joint_delta_rad", 1.2))), 1),
                        # the same limits in the corrected model's angles (what joints_model_deg reports): the twin's
                        # planners keep inside these (2026-10-24)
                        "limits_model_deg": [[round(math.degrees(lo + o), 2), round(math.degrees(hi + o), 2)]
                                             for (lo, hi), o in zip(self.limits, self.kin.offsets)]},
                # housing = the bunny.housing targets; housing_states = the words state.bunny.housing may report
                # (2026-11-19: read from the motor, never "unknown"; an older Bunny still says unknown)
                # platform_set = bunny.platform_set's argument (2026-11-20: the Bunny's own mm, its range check)
                "bunny": {"housing": ["open", "half", "close"], "platform": "steps", "platform_set": "mm",
                          "housing_states": ["closed", "open", "half", "between", "moving", "no_feedback", "untaught"]}}

    def state(self):
        a, g, q = self.fresh("arm", 1.0), self.fresh("grip", 1.0), self.joints_rad()
        arm = {"online": bool(a and a.connected and q is not None)}
        if a is not None:
            arm.update(enabled=list(a.joints_enabled), all_enabled=a.all_enabled, ctrl_mode=a.ctrl_mode,
                       arm_status=a.arm_status, busy=a.busy, fault=a.fault, errors=list(a.errors),
                       teach=a.teach_mode, feedback_age_s=round(a.feedback_age_s, 3), speed_percent=a.speed_percent,
                       estop=a.fault.startswith("e-stop pressed"))    # piper_node.ESTOP_PRESSED: the button is pressed
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
        arm["gripper_calib"] = self.gripper_calib()
        bunny = bunny_view(self.fresh("bunny", 3.0))
        calib = {"busy": JOB["busy"], "kind": JOB["kind"], "lines": JOB["lines"][-8:], "result": JOB["result"]}
        nv = self.nodes_view()
        return {"arm": arm, "bunny": bunny, "camera": self.camera(), "calib": calib, "nodes": nv["nodes"],
                "watchdog": nv["watchdog"], "absolute": self.absolute_view(), "calib_precheck": getattr(self, "precheck", None)}


# ---------------------------------------------------------------------------- server
gw = None
lock = threading.Lock()
ctl = {"conn": None, "motion": None}     # control lease (a Conn) and the gateway-started motion {conn, label}
# live slider driving: the latest target of each kind wins (arm.jog_pose / arm.jog_gripper)
jog = {"pose": None, "grip": None, "busy": False, "error": "", "last": None, "ev": threading.Event()}
# the one running job (calib.handeye / teach.fine / arm.follow_joints) and the last result
JOB = {"busy": False, "kind": "", "lines": [], "result": None, "proc": None, "abort": threading.Event(), "why": ""}
# run commands (RUN_CMDS) in flight and when the last one began / ended (THE CHECK STEPS ASIDE); 0.0 = none since the start
ACT, ACT_LOCK = {"n": 0, "t": 0.0}, threading.Lock()


def abort_job(why, caller=""):
    """Stop the running job: its abort flag, SIGINT to its process group, every arm goal cancelled (the arm freezes).
    caller (2026-11-24) = who asked (stop / arm.release / nodes.restart / the dead-man and why): logged with the reason
    in /tmp/gateway.log, so an aborted job's cause is on disk, not only in state.calib."""
    if not JOB["busy"]:
        return False
    print("%s job aborted: %s - %s (by %s)" % (time.strftime("%H:%M:%S"), JOB["kind"], why, caller or "?"), flush=True)
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


# ---------------------------------------------------------------------------- calib.absolute (2026-11-16)
def _read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _write_json(path, doc):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f)
    os.replace(tmp, path)


def abs_current():
    """The open session's folder, or None."""
    try:
        with open(os.path.join(ABS_DIR, "CURRENT")) as f:
            sid = f.read().strip()
    except OSError:
        return None
    d = os.path.join(ABS_DIR, sid)
    return d if sid and os.path.isdir(d) else None


SOLVE_KEYS = ("at", "ok", "n_samples", "positions", "params", "rms_before_mm", "rms_after_mm", "leave_one_out_mm",
              "leave_one_out_before_mm", "loo", "offsets", "joint_offsets_deg", "alpha_offsets_deg", "handeye_change",
              "improved", "why")


def solve_view(res):
    """The solve's result without its matrices (handeye = {xyz_mm, rpy_deg})."""
    if not res:
        return None
    v = {k: res[k] for k in SOLVE_KEYS if k in res}
    if res.get("handeye"):
        v["handeye"] = {k: res["handeye"][k] for k in ("xyz_mm", "rpy_deg")}
    return v


def absolute_status():
    """{session, positions [{position, samples, rejected, complete, at}], solve | None, installed_at} (times: ISO local)."""
    d = abs_current()
    if d is None:
        return {"session": None, "positions": [], "solve": None, "installed_at": None}
    pos = []
    for k in range(1, ABS_POSITIONS + 1):
        p = _read_json(os.path.join(d, "position_%d.json" % k))
        if p:
            pos.append({"position": k, "samples": len(p.get("samples") or []), "rejected": len(p.get("rejected") or []),
                        "complete": bool(p.get("complete")), "at": p.get("at")})
    sess = _read_json(os.path.join(d, "session.json")) or {}
    return {"session": os.path.basename(d), "positions": pos, "solve": solve_view(_read_json(os.path.join(d, "solve.json"))),
            "installed_at": sess.get("installed_at")}


def absolute_begin():
    import datetime as dt
    sid = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    d = os.path.join(ABS_DIR, sid)
    os.makedirs(d, exist_ok=True)
    _write_json(os.path.join(d, "session.json"), {"session": sid, "created": dt.datetime.now().isoformat(timespec="seconds")})
    with open(os.path.join(ABS_DIR, "CURRENT.tmp"), "w") as f:
        f.write(sid + "\n")
    os.replace(os.path.join(ABS_DIR, "CURRENT.tmp"), os.path.join(ABS_DIR, "CURRENT"))
    gw._abs_cache = None
    return {"session": sid}


def absolute_job(action, abort, d, position=None):
    """calib.absolute collect / solve: harness_calibration/absolute.py as a process (from source, like the gateway's other
    calibration code); its output lines go to state.calib.lines; stop / the dead-man interrupt it (SIGINT)."""
    cmd = [sys.executable, "-m", "harness_calibration.absolute", action, "--dir", d, "--kin", KIN_FILE, "--handeye", HANDEYE_FILE,
           "--piper-yaml", PIPER_YAML]
    if action == "collect":
        cmd += ["--position", str(int(position)), "--settle", str(HANDEYE_SETTLE_S)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, bufsize=1,
                         env=dict(os.environ, PYTHONUNBUFFERED="1"), start_new_session=True,
                         cwd=os.path.join(ROOT, "src", "harness_calibration"))
    JOB["proc"] = p
    for line in p.stdout:
        line = line.rstrip()
        if line:
            JOB["lines"] = (JOB["lines"] + [line])[-40:]
            print("absolute: " + line, flush=True)
    rc = p.wait()
    gw._abs_cache = None
    if action == "collect":
        doc = _read_json(os.path.join(d, "position_%d.json" % int(position))) or {}
        r = {"position": int(position), "samples": len(doc.get("samples") or []), "rejected": len(doc.get("rejected") or []),
             "complete": bool(doc.get("complete"))}
        if abort.is_set() or rc != 0:
            r.update(ok=False, message=JOB["why"] or last_error(JOB["lines"]))
        return r
    res = _read_json(os.path.join(d, "solve.json")) if rc == 0 else None
    if res is None:
        return {"ok": False, "message": JOB["why"] or last_error(JOB["lines"])}
    return solve_view(res)


def absolute_install():
    """Write calib/kinematics.yaml + calib/handeye.yaml from the open session's last solve (the old files first to
    calib/handeye_backups/), refused unless it improved on the files installed when it was solved; the gateway reads
    them again at once; the arm node (which reads kinematics.yaml only at start) is restarted detached - a start never
    enables or moves the arm, and the PIPER holds its last target meanwhile."""
    import datetime as dt, shutil, yaml
    from scipy.spatial.transform import Rotation as Rot
    d = abs_current()
    res = _read_json(os.path.join(d, "solve.json")) if d else None
    if not res or not res.get("ok"):
        raise RuntimeError("no solve to install (calib.absolute solve first)")
    if not res.get("improved"):
        raise RuntimeError("not installed: " + (res.get("why") or "the left-out error does not drop"))
    if JOB["busy"] or ctl["motion"]:
        raise RuntimeError("busy: %s is running" % (JOB["kind"] or "a motion"))
    with open(KIN_FILE) as f:
        kin_old = yaml.safe_load(f) or {}
    fr = res.get("from") or {}
    if fr.get("kinematics_date") != kin_old.get("date") or fr.get("handeye_date") != gw.he_info.get("date"):
        raise RuntimeError("the installed calibration changed since this solve: solve again")
    now = dt.datetime.now()
    ts, iso = now.strftime("%Y-%m-%dT%H%M%S"), now.isoformat(timespec="seconds")
    os.makedirs(BACKUP_DIR, exist_ok=True)
    backups = []
    for src, name in ((KIN_FILE, "kinematics_%s.yaml" % ts), (HANDEYE_FILE, "handeye_%s.yaml" % ts)):
        if os.path.exists(src):
            shutil.copy(src, os.path.join(BACKUP_DIR, name))
            backups.append(os.path.join(BACKUP_DIR, name))
    th, al = res["joint_offsets_rad"], res["alpha_offsets_rad"]
    note = ("calib.absolute %s: %d samples over board positions %s; fitted %s with the hand-eye; left-out-position "
            "prediction %.2f mm (installed model %.2f mm); rms %.2f -> %.2f mm"
            % (res.get("session"), res["n_samples"], res["positions"], "+".join(res["params"]),
               sum(res["leave_one_out_mm"]) / len(res["leave_one_out_mm"]),
               sum(res["leave_one_out_before_mm"]) / len(res["leave_one_out_before_mm"]), res["rms_before_mm"], res["rms_after_mm"]))
    kin = {"model": kin_old.get("model", 'piper MDH (pyAgxArm ROBOT_MDH_PRESET["piper"])'),
           "joint_zero_offsets_rad": [float(v) for v in th], "joint_zero_offsets_deg": [round(math.degrees(v), 4) for v in th],
           "alpha_offsets_rad": [float(v) for v in al], "alpha_offsets_deg": [round(math.degrees(v), 4) for v in al],
           "note": note, "date": iso}
    X = res["handeye"]["matrix"]
    import numpy as np
    Xm = np.array(X, float)
    he = {"flange_to_camera": {"xyz_m": [float(v) for v in Xm[:3, 3]], "quat_xyzw": [float(v) for v in Rot.from_matrix(Xm[:3, :3]).as_quat()],
                               "rpy_rad": [math.radians(v) for v in res["handeye"]["rpy_deg"]], "matrix": Xm.tolist()},
          "method": "calib.absolute: joint fit with the kinematic corrections over %d ChArUco board positions" % len(res["positions"]),
          "n_poses": res["n_samples"], "accepted": True, "rms_mm": res["rms_after_mm"], "residual_rms_mm": res["rms_after_mm"],
          "residual_max_mm": res.get("max_after_mm"),
          "predict_left_out_group_rms_mm": round(sum(res["leave_one_out_mm"]) / len(res["leave_one_out_mm"]), 3),
          "accepted_by": "gateway calib.absolute install (left-out error dropped)", "date": iso,
          "note": "valid only together with calib/kinematics.yaml of the same date; " + note}
    for path, doc in ((KIN_FILE, kin), (HANDEYE_FILE, he)):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        os.replace(tmp, path)
    gw.reload_model()
    sess_path = os.path.join(d, "session.json")
    sess = _read_json(sess_path) or {}
    sess["installed_at"] = iso
    _write_json(sess_path, sess)
    gw._abs_cache = None
    nodes_restart("piper")
    return {"ok": True, "installed_at": iso, "backups": backups,
            "arm_node": "restarting (harness nodes restart piper): it reads calib/kinematics.yaml only at start"}


def nodes_restart(want):
    """nodes.restart: `harness nodes restart <name>` for the node (or every installed node: "all"), one after the
    other in ONE detached shell (its own session: it outlives this gateway), the gateway itself last, after the
    reply has gone out. A running job / arm goal is stopped first when the arm, the camera or the gateway restarts
    (the arm freezes where it is, stays enabled; the PIPER holds its last target while its node is down)."""
    import node_watchdog
    listed = node_watchdog.node_list(CLI)
    names = [n for n, _, _ in listed]
    if want == "all":
        todo = [n for n, _, c in listed if not node_watchdog.installed(c, ROOT)]
        todo = [n for n in todo if n not in ("watchdog", "gateway")] + [n for n in ("watchdog", "gateway") if n in todo]
    elif want in names:
        todo = [want]
    else:
        raise said(ValueError("nodes.restart: unknown node %r" % want), "one of %s or \"all\"" % ", ".join(names))
    if set(todo) & {"piper", "camera", "gateway"}:
        jog["pose"] = jog["grip"] = None
        abort_job("nodes.restart %s" % want, "nodes.restart")
        if "piper" in todo and ctl["motion"]:      # a motion the gateway started (no job): its goal cancelled
            gw.cancel_arm()
    script = "sleep 0.5; " + "; ".join("%s nodes restart %s" % (CLI, n) for n in todo)
    with open(RESTART_LOG, "a") as out:
        out.write("%s nodes.restart %s: %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), want, " ".join(todo)))
        out.flush()
        subprocess.Popen(["bash", "-c", script], stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         cwd=ROOT, start_new_session=True)
    return {"ok": True, "restarted": todo}


def last_error(lines):
    """A routine's own words for why it stopped: its last "refused: ..." line, else its last line — at most 10 words
    (2026-11-21; every line stays in state.calib.lines)."""
    ref = [ln for ln in lines if ln.startswith("refused:")]
    if ref:
        return clip(ref[-1][len("refused:"):].strip())
    return clip(lines[-1]) if lines else "no output"


def bunny_failed(what, message, outcome_words):
    """A failed Bunny dose in at most 10 words (2026-11-21): the Bunny's own error when it raised one (bunny_node passes
    it as it is), else (its reply, JSON) what it did; the reply itself goes to the detail."""
    try:
        json.loads(message)
    except ValueError:
        return "%s: %s" % (what, message)
    return "%s %s" % (what, outcome_words)


def precheck_reusable(g, last, q, T, now):
    """The last assessment still holds (update_precheck): the same joints (PRECHECK_SAME_DEG), the same model and hand-eye,
    and the target where it was (PRECHECK_SAME_TARGET_MM / _DEG; seen / not seen alike) — a moved target only once the
    result is PRECHECK_MAX_AGE_S old (a flickering tag must not re-run the search every second)."""
    import numpy as np
    if max(abs(a - c) for a, c in zip(q, last[1])) >= math.radians(PRECHECK_SAME_DEG):
        return False
    if len(last) < 7 or last[4] is not g.kin or last[5] is not g.Tfc:
        return False
    T0 = last[3]
    if T0 is None and T is None:
        return True
    if T0 is not None and T is not None:
        R = np.asarray(T0)[:3, :3].T @ np.asarray(T)[:3, :3]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))))
        if (float(np.linalg.norm(np.asarray(T)[:3, 3] - np.asarray(T0)[:3, 3])) * 1000.0 < PRECHECK_SAME_TARGET_MM
                and ang < PRECHECK_SAME_TARGET_DEG):
            return True
    return now - last[6] < PRECHECK_MAX_AGE_S


def run_activity(delta):
    """A run command (RUN_CMDS) began (+1) / ended (-1): precheck_quiet's count and clock."""
    with ACT_LOCK:
        ACT["n"] += delta
        ACT["t"] = time.time()


def precheck_quiet():
    """The start-posture check may run (THE CHECK STEPS ASIDE): no job, no gateway motion, no live jog, no run command
    in flight, the last one PRECHECK_HOLDOFF_S ago."""
    if JOB["busy"] or ctl["motion"] or jog["busy"] or jog["pose"] or jog["grip"]:
        return False
    with ACT_LOCK:
        return ACT["n"] <= 0 and time.time() - ACT["t"] >= PRECHECK_HOLDOFF_S


def precheck_worker():
    """state.calib_precheck at ~1 Hz while the harness is quiet (precheck_quiet); a round stops between two candidates
    when a job or a run command starts (precheck.Interrupted: the state keeps the last full result)."""
    busy = lambda: not precheck_quiet()   # noqa: E731
    while True:
        time.sleep(PRECHECK_S)
        if busy():
            continue
        try:
            gw.update_precheck(stop=busy)
        except Exception as e:  # noqa: BLE001  the state says it, the worker goes on
            if type(e).__name__ == "Interrupted":       # (harness_calibration.precheck.Interrupted)
                continue
            gw.precheck = {"ok": False, "reachable": None, "why": "start-posture check failed: %s" % e, "at": round(time.time(), 2)}


def handeye_job(tid, abort, target="tag"):
    """calib.handeye: the hand-eye routine as a process; its result is installed when rms <= HANDEYE_ACCEPT_RMS_MM.
    target "charuco" (2026-11-16) = the ChArUco board as the fixed target instead of tag tid."""
    import datetime as dt, shutil, yaml
    new = os.path.join(ROOT, "calib", "handeye_new.yaml")
    if os.path.exists(new):
        os.remove(new)
    what = ["--target", "charuco"] if target == "charuco" else ["--tag-id", str(int(tid))]
    cmd = ["ros2", "run", "harness_calibration", "handeye"] + what + ["--accept-mm", "6.0",
                                                                       "--settle", str(HANDEYE_SETTLE_S), "--out", new,
                                                                       "--kin", KIN_FILE, "--piper-yaml", PIPER_YAML,
                                                                       "--handeye-file", HANDEYE_FILE]
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
        return {"ok": False, "installed": False, "message": last_error(JOB["lines"])}
    with open(new) as f:
        doc = yaml.safe_load(f) or {}
    os.remove(new)
    rms, mx = float(doc.get("residual_rms_mm") or 1e9), float(doc.get("residual_max_mm") or 1e9)
    x = doc.get("flange_to_camera") or {}
    res = {"rms_mm": round(rms, 2), "max_mm": round(mx, 2), "n_poses": doc.get("n_poses"), "tag_id": tid, "target": target,
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
                    raise RuntimeError("rejected by the arm node (busy, fault, teach, disabled)")
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
        # 2026-11-24: one failing tick (gw.state() / json.dumps raising) is skipped, never the end of the pump - the
        # connection stayed open with no state, the station looked frozen, a reconnect closed the controller and the
        # dead-man aborted the hand-eye job. One log line per PUMP_ERR_LOG_S at most, with the count since the last one.
        errs, err_logged = 0, 0.0
        while self.alive:
            rate = self.sub_rate                # read once: a subscribe may set it to 0 meanwhile
            if rate > 0:
                try:
                    st = gw.state()
                    c = ctl["conn"]
                    st["control"] = {"held_by": c.client if c else None, "you": c is self,
                                     "jog": {"busy": jog["busy"] or bool(jog["pose"]), "error": jog["error"]}}
                    st["event"], st["t"] = "state", round(time.time(), 3)
                    self.send(st)
                except Exception as e:     # noqa: BLE001  this tick skipped, the pump goes on
                    errs += 1
                    if time.time() - err_logged >= PUMP_ERR_LOG_S:
                        import traceback
                        tb = traceback.extract_tb(e.__traceback__)
                        at = "%s:%d" % (os.path.basename(tb[-1].filename), tb[-1].lineno) if tb else "?"
                        print("%s state pump %s %s: tick skipped (%d since last line): %s: %s at %s"
                              % (time.strftime("%H:%M:%S"), self.client, self.peer, errs, type(e).__name__, e, at),
                              flush=True)
                        errs, err_logged = 0, time.time()
                time.sleep(1.0 / rate)
            else:
                time.sleep(0.2)

    def absolute(self, a):
        """calib.absolute {action}: status (anyone) | begin | collect {position} (a job) | solve (a job) | install."""
        act = str(a.get("action") or "status")
        if act == "status":
            return absolute_status()
        self.need("control")
        if act == "begin":
            return absolute_begin()
        d = abs_current()
        if d is None:
            raise RuntimeError("no calibration session: calib.absolute begin first")
        if act == "collect":
            k = int(a.get("position") or 0)
            if not 1 <= k <= ABS_POSITIONS:
                raise ValueError("position: 1..%d" % ABS_POSITIONS)
            st = gw.fresh("arm", 1.0)
            if st is None or not st.all_enabled:
                raise said(RuntimeError("the arm is not holding: press Hold first"), "the routine moves it around where it is")
            b = gw.charuco_state()
            if not b["seen"] or b["corners"] < ABS_MIN_CORNERS:
                raise RuntimeError("ChArUco board not in view: %d corners, needs %d" % (b["corners"], ABS_MIN_CORNERS))
            pc = gw.precheck_cached()
            if pc is not None and not pc["ok"]:
                raise RuntimeError(pc.get("words") or pc["why"])
            return self.job("calib.absolute", lambda ev: absolute_job("collect", ev, d, k), background=True,
                            extra={"action": "collect", "position": k})
        if act == "solve":
            if not absolute_status()["positions"]:
                raise RuntimeError("no position collected yet")
            return self.job("calib.absolute", lambda ev: absolute_job("solve", ev, d), background=True, extra={"action": "solve"})
        if act == "install":
            return absolute_install()
        raise ValueError("calib.absolute action: begin, collect, solve, install or status")

    def dispatch(self, req):
        rid, cmd, args = req.get("id"), req.get("cmd"), req.get("args") or {}
        run_cmd = cmd in RUN_CMDS                 # (the start-posture check steps aside: precheck_quiet)
        if run_cmd:
            run_activity(+1)
        try:
            res = self.run(cmd, args, rid)
            self.send({"id": rid, "ok": True, "result": res})
        except Exception as e:     # noqa: BLE001  every failure goes back to the client
            rep = {"id": rid, "ok": False, "error": str(e)}
            if getattr(e, "detail", ""):          # the long words (2026-11-21, said()): the reply's detail + the log
                rep["detail"] = e.detail
                print("%s: %s (%s)" % (cmd, e, e.detail), flush=True)
            self.send(rep)
        finally:
            if run_cmd:
                run_activity(-1)

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
            aborted = abort_job("stopped (stop)", "stop from %s %s" % (self.client, self.peer))
            r = {"cancelled": gw.cancel_arm(), "job_aborted": aborted}
            b = gw.fresh("bunny", 3.0)
            if b and (b.get("dispensing") or b.get("busy")):
                try:
                    gw.trigger("/bunny/abort", 15); r["bunny"] = "abort sent"
                except RuntimeError as e:
                    r["bunny"] = str(e)
            return r
        if cmd == "calib.absolute":
            return self.absolute(a)
        if cmd == "nodes.status":          # anyone: the watchdog's view of the harness nodes
            return gw.nodes_view()
        if cmd == "nodes.restart":
            self.need("control")
            return nodes_restart(str(a.get("node") or ""))
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
                raise said(RuntimeError("refused: " + chk["reasons"][0]),
                           "; ".join(chk["reasons"]) if len(chk["reasons"]) > 1 else "")
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
        if cmd == "arm.teach_exit":        # 2026-10-24: the controller out of teach (drag) mode; enables nothing
            self.need("control")
            jog["pose"] = jog["grip"] = None
            return gw.trigger("/arm/teach_exit", 15)
        if cmd == "arm.release":           # operator 2026-10-23: one click; joints 1-6 only, the gripper keeps holding
            self.need("control")
            jog["pose"] = jog["grip"] = None
            abort_job("the arm was released", "arm.release from %s %s" % (self.client, self.peer))
            return gw.trigger("/arm/disable", 15)
        if cmd == "calib.handeye":
            self.need("control")
            st = gw.fresh("arm", 1.0)
            if st is None or not st.all_enabled:
                raise said(RuntimeError("the arm is not holding: press Hold first"), "the routine moves it around where it is")
            if str(a.get("target") or "tag") == "charuco":
                pc = gw.precheck_cached()
                if pc is not None and not pc["ok"]:
                    raise RuntimeError(pc.get("words") or pc["why"])
                if not gw.charuco_state()["seen"]:
                    raise RuntimeError("ChArUco board not in view: aim camera 150-300 mm away")
                return self.job("calib.handeye", lambda ev: handeye_job(None, ev, "charuco"), background=True,
                                extra={"target": "charuco"})
            if str(a.get("target") or "tag") != "tag":
                raise ValueError("calib.handeye target: \"tag\" (tag_id) or \"charuco\"")
            pc = gw.precheck_cached()
            if pc is not None and not pc["ok"]:
                raise RuntimeError(pc.get("words") or pc["why"])
            seen = gw.camera()["tags"]
            tid = a.get("tag_id")
            if tid is None:
                if not seen:
                    raise said(RuntimeError("no tag in view: aim at a tag, 150-300 mm"),
                               "point the camera at a FIXED tag 150-300 mm away")
                tid = max(seen, key=lambda t: t["size_px"])["id"]
            elif not any(t["id"] == int(tid) for t in seen):
                raise RuntimeError("tag %s is not in view" % tid)
            return self.job("calib.handeye", lambda ev: handeye_job(int(tid), ev), background=True, extra={"tag_id": int(tid)})
        if cmd == "teach.plan_views":
            self.need("key")
            return gw.plan_views(a.get("from_model_deg"), float(a.get("backoff_mm") or 0.0), float(a.get("tilt_deg") or 3.0),
                                 a.get("tag_base_mm"))
        if cmd == "teach.fine":
            self.need("control")
            tid, black = int(a["tag_id"]), float(a.get("black_mm") or 0.0)
            tilt, views = float(a.get("tilt_deg") or 3.0), a.get("views_model_deg") or None
            fam = str(a.get("family") or "") or None
            return self.job("teach.fine", lambda ev: gw.fine(tid, black, ev, tilt, views, fam))
        if cmd == "arm.move_linear":
            self.need("control")
            d = [float(v) for v in a.get("delta_mm") or []]
            if len(d) != 3 or math.sqrt(sum(v * v for v in d)) > 1000.0:      # (300 until 2026-11-19, relaxed)
                raise ValueError("delta_mm needs x y z, at most 1000 mm")
            speed = max(1, min(int(gw.P.get("max_speed_percent", 30)), int(a.get("speed_percent") or 10)))
            return self.job("arm.move_linear", lambda ev: gw.move_linear(d, speed, ev))
        if cmd == "teach.tag":
            self.need("key")
            return gw.stored_tag(int(a["tag_id"]), str(a.get("family") or "") or None)
        if cmd == "arm.move_tip":
            self.need("control")
            tip = [float(v) for v in a.get("tip_mm_deg") or []]
            if len(tip) != 6:
                raise ValueError("tip_mm_deg needs x y z rx ry rz")
            speed = max(1, min(int(gw.P.get("max_speed_percent", 30)), int(a.get("speed_percent") or 10)))
            return self.job("arm.move_tip", lambda ev: gw.move_tip(tip, a.get("linear", True), speed, ev))
        if cmd == "arm.follow_joints":
            self.need("control")
            wps, label = a.get("waypoints_model_deg"), str(a.get("label") or "")
            speed, corner = float(a.get("speed") or 1.0), a.get("corner_deg")
            blend, speeds = bool(a.get("blend")), a.get("speeds")          # THE SMOOTH PATH (2026-11-19)
            times = a.get("times_s")                                       # THE TIMED PATH (2026-11-23)
            if speeds is not None and not blend:
                raise ValueError("speeds needs blend: true")
            if times is not None and (blend or speeds is not None):
                raise ValueError("times_s excludes blend and speeds")
            return self.job("arm.follow_joints",
                            lambda ev: gw.follow_joints(self, wps, label, ev, speed, corner, blend, speeds, times))
        if cmd == "arm.gripper":
            self.need("control")
            from harness_msgs.srv import SetGripper
            gap, force = float(a["gap_mm"]) / 1000.0, float(a.get("force_n") or 0.0)
            speed = float(a.get("speed_mm_s") or 0.0) / 1000.0
            end = a.get("end")
            if end not in (None, "", "contact"):
                raise ValueError('arm.gripper end: "contact" or none')
            g = gw.fresh("grip", 2.0)
            if end == "contact" and (g is None or gap < float(g.width_m)):     # a close: stop the ramp at contact
                return gw.grip_to_contact(gap, force, speed, g)
            if speed > 0 and g is not None:
                # a moderate closing / opening speed (twin 2026-10-25: closed at full speed the vial was gripped wrong):
                # the gripper has no speed setting, so the target gap is RAMPED from where the jaws are at `speed`,
                # a GRIP_RAMP_STEP_M step every step / speed seconds (each step's service call returns at once: within the node's
                # tolerance); the last call is the real target, with the node's stall detection as before
                w, t_last = float(g.width_m), time.time()
                while abs(gap - w) > GRIP_RAMP_STEP_M:
                    w += (GRIP_RAMP_STEP_M if gap > w else -GRIP_RAMP_STEP_M)
                    gw.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=w, force=force, timeout_s=2.0), 10)
                    t_last += GRIP_RAMP_STEP_M / speed
                    time.sleep(max(0.0, t_last - time.time()))
                    gn = gw.fresh("grip", 2.0)
                    if gn is not None and gap < w and float(gn.width_m) > w + 4 * GRIP_RAMP_STEP_M:
                        break                                    # closing: the jaws stopped on something — let the final call judge
            r = gw.call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=gap, force=force), 20)
            if not r.success:
                raise RuntimeError(r.message)
            out = {"gap_mm": round(r.width_m * 1000, 2), "stalled": r.stalled, "message": r.message}
            if end:                                      # (an opening asked with end "contact")
                out["contact"] = bool(r.stalled)
            return out
        if cmd == "arm.gripper_release":   # the gripper motor only: the jaws go free, the arm keeps holding
            self.need("control")
            jog["grip"] = None
            return gw.trigger("/arm/gripper_release", 15)
        if cmd == "arm.gripper_hold":      # the gripper motor grips where the jaws are, at gripper_hold_force
            self.need("control")
            return gw.trigger("/arm/gripper_enable", 15)
        if cmd == "arm.gripper_cal":
            self.need("control")
            from harness_msgs.srv import GripperCalibrate
            step = str(a.get("step") or "")
            r = gw.call(GripperCalibrate, "/arm/gripper_calibrate",
                        GripperCalibrate.Request(step=step, gap_m=float(a.get("gap_mm") or 0.0) / 1000.0), 15)
            if not r.success:
                raise RuntimeError(r.message)
            out = {"step": step, "message": r.message, "raw_mm": round(r.raw_m * 1000, 2)}
            if step == "open":
                out.update(scale=round(r.scale, 4), max_gap_mm=round(r.max_gap_m * 1000, 1))
            return out
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
        if cmd == "bunny.platform_set":
            # 2026-11-20: tell the Bunny its platform height (the station measured tag 3), in the Bunny's mm; the Bunny's
            # own range check, its errors passed through (an older server: "unknown cmd platform_set")
            self.need("control")
            from harness_msgs.srv import MoveZ               # borrowed, as bunny.platform borrows LiquidPush
            mm = a.get("mm")
            if isinstance(mm, bool) or not isinstance(mm, (int, float)) or not math.isfinite(mm):
                raise said(ValueError("mm must be a finite number (got %r)" % (mm,)), "mm = the Bunny's platform height, its mm")
            try:
                r = gw.call(MoveZ, "/bunny/platform_set", MoveZ.Request(mm=float(mm), absolute=True), 10)
            except RuntimeError as e:
                if "not answering" in str(e) and gw.fresh("bunny", 3.0) is not None:
                    raise said(RuntimeError("bunny_node too old: harness nodes restart bunny"),
                               "/bunny/platform_set is not offered: bunny_node is older than the gateway")
                raise
            if not r.success:
                raise RuntimeError(r.message)
            return {"platform_mm": round(r.z, 4) if math.isfinite(r.z) else None}
        # v6 (2026-10-25, the twin's "Run workflow"): the Bunny's dosing, each THE job (one at a time, the dead-man
        # watch on this connection; `stop` aborts a dose through /bunny/abort)
        if cmd == "bunny.tare":
            self.need("control")
            return self.job("bunny.tare", lambda ev: gw.trigger("/bunny/tare", 60))
        if cmd == "bunny.weigh_to":
            self.need("control")
            from harness_msgs.srv import WeighTo
            g, mat, powder = float(a.get("target_g") or 0.0), str(a.get("material") or "powder"), str(a.get("powder") or "")
            if not 0.0 < g <= 100.0 or mat not in ("powder", "liquid"):
                raise ValueError("target_g must be in (0, 100], material powder or liquid")

            def weigh(ev):
                r = gw.call(WeighTo, "/bunny/weigh_to", WeighTo.Request(target_g=g, material=mat, powder_id=powder), 1800)
                if not r.success:
                    raise said(RuntimeError(bunny_failed("weigh_to", r.message, "%s at %.4f g" % (r.outcome, r.actual_g))),
                               r.message)
                return {"outcome": r.outcome, "actual_g": round(r.actual_g, 5)}
            return self.job("bunny.weigh_to", weigh)
        if cmd == "bunny.dispense_volume":
            self.need("control")
            from harness_msgs.srv import DispenseVolume
            ml = float(a.get("volume_ml") or 0.0)
            if not 0.0 < ml <= 100.0:
                raise ValueError("volume_ml must be in (0, 100]")

            def dispense(ev):
                r = gw.call(DispenseVolume, "/bunny/dispense_volume", DispenseVolume.Request(
                    volume_ml=ml, density_g_per_ml=float(a.get("density_g_per_ml") or 0.0), rate=0), 600)
                if not r.success:
                    raise said(RuntimeError(bunny_failed("dispense_volume", r.message, "stopped at %d of %d pump steps"
                                                         % (r.steps, r.steps_planned))), r.message)
                return {"actual_ml": round(r.actual_ml, 3), "steps": r.steps}
            return self.job("bunny.dispense_volume", dispense)
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
                    abort_job("dead-man: controller silent for %.0f s - arm frozen" % DEADMAN_S,
                              "dead-man: %s, %s %s" % ("connection closed" if not self.alive else
                                                       "silent %.1f s" % (time.time() - self.last_rx),
                                                       self.client, self.peer))
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
                res = {"ok": False, "kind": kind, "message": str(e)}
                if getattr(e, "detail", ""):
                    res["detail"] = e.detail
                JOB["result"] = res        # whole, never changed after: the state pump's json.dumps may read it now
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
            raise RuntimeError("rejected by the arm node (busy, fault, teach, no feedback)")
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
    threading.Thread(target=precheck_worker, daemon=True).start()
    print("gateway: listening on 0.0.0.0:%d (key %s)" % (PORT, "set" if read_key() else "MISSING: view-only"), flush=True)
    Server(("0.0.0.0", PORT), Conn).serve_forever()


if __name__ == "__main__":
    main()
