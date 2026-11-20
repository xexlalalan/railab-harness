"""THE ARM'S ADAPTER NODE (module interface, 2026-11-20; the workflow repo's module_interface/HARNESS_PORT.md): the PiPER
as one module instance (arm_1). Above: the contract (railab_harness.module.Module). Below: a LINK that is a ROS client of
the EXISTING arm node (harness_arm/piper_node.py), which keeps every check of its own. Side by side with the old
gateway (TCP 7803), which stays as it is.

  do    follow_path {waypoints_deg, speeds?}   the twin's joint path in its MODEL degrees -> a timed trajectory HERE
                                               (harness_planner.trajectory.plan_smooth_path) -> /arm/follow_trajectory
        grip_to {gap_mm, force_n?}             /arm/gripper
        hold                                   /arm/hold
        release                                /arm/disable = the drag-teach release (the old gateway's arm.release:
                                               joints 1-6 limp, the gripper keeps holding). No other disable /
                                               power-off path exists here, and none may be added.
        calibrate_handeye, calibrate_absolute  declared, refused NOT_READY: not ported yet (NOT_PORTED says why)
        stop, clear_fault                      the base's. A `stop` REQUEST (scope "device", the default) stops the
                                               arm whoever moves it (on_stop_request): every goal on the arm node's
                                               actions cancelled (it freezes the arm where it is, enabled; the old
                                               gateway's stop does the same) and a released arm held - that stop never
                                               leaves it limp. Going safe BY ITSELF (no harness beat, closing, a latched
                                               fault, the gateway's own stop of scope "own" when its client fell silent
                                               or hung up: safe_state only) touches only an action of THIS node: its own
                                               goal (follow_path), its own grip (grip_to) - nothing at all when none
                                               runs, and it never holds a released arm (a drag-teach release, by this
                                               node or by the old gateway, stays released: someone may be guiding the
                                               arm by hand). So running this node beside the old gateway never cancels
                                               the old path's motions nor re-holds its release by itself. clear_fault
                                               is refused while the arm node still has the cause (check_clear_fault).
  read  joints_deg (model degrees), gap_mm, held, handeye_error_mm, absolute_error_mm - sensed, "unknown" while no
        fresh status (the two calibration errors stay "unknown": nothing here calibrates yet)
  zones none of its own: the zones a path passes are the twin's `claims`

EXECUTE OR REFUSE - where the old gateway (tools/gateway_server.py, Gateway.follow_joints / arm.gripper) changed a
command silently, this node refuses it with the joint, the value and the limit:
  - a path starting up to 0.35 rad (20 deg) from the arm: the gateway replaced its first waypoint with the arm's
    posture. Here: more than START_TOL_DEG away -> OUT_OF_RANGE; within it the path starts exactly where the arm is.
  - a waypoint up to 10 deg past a joint limit: the gateway clamped it inside. Here: any waypoint outside the
    controller's range less the arm node's own margin -> OUT_OF_RANGE.
  - an arm held up to 20 deg past a limit: the gateway prepended a move back inside. Here nothing is prepended: the
    path starts where the arm is (the node allows a start past a limit) and every later waypoint must be inside.
  - speed / speeds outside 0.25 ... 2: clamped there. Here: OUT_OF_RANGE. (Its 0.70 rad/s cap never binds inside
    that range, so no accepted path is slowed behind the asker's back.)
  - corner_deg (clamped 5 ... 45) and the non-blend plan: not offered. Every path is THE SMOOTH PATH (one motion, at
    rest only at its ends, within 0.3 deg / 1 mm of the waypoints' polyline).
  - grip_to wider than the gripper's widest gap: the arm node clamps it. Here: OUT_OF_RANGE. force_n 0: the node
    reads it as "its default"; here the range starts at GRIP_FORCE_MIN_N (leave force_n out for the default).
    The gateway's speed_mm_s ramp is not part of the contract.

Run (nothing starts it by itself; not in `harness nodes`):
  ros2 run harness_modules arm_module --ros-args -p instance:=arm_1 -p harness_root:=~/bunny-harness-dev

UNTESTED ON ROS (written without ROS, 2026-11-20): Arm + ArmModel were checked on a LocalBus with a fake link and the
real planner on a synthetic arm model (tools/module_check_arm.py); RosArmLink and main() never ran.
"""

import math
import os
import re
import signal
import threading
import time

from railab_harness import messages as M
from railab_harness.messages import Refused
from railab_harness.module import Failed, Fault, Module
from railab_harness.timing import DEFAULT

# ---------------------------------------------------------------------------------------------------- this node's own
FRESH_S = 1.0             # a status / joints / gripper reading older than this is not used (the arm node publishes at
                          # 100 Hz; the old gateway's fresh(..., 1.0))
SILENT_FAULT_S = 2.0      # no /arm/status this long, after one was heard: a latched fault (the arm node is gone)
SENSE_PERIOD_S = 0.1      # joints / gap / held copied into the state at most this often (10 Hz, the twin's rate)
START_TOL_DEG = 1.0       # follow_path's first waypoint: at most this far (per joint) from where the arm is; the path
                          # then starts exactly there. Above the arm node's settle tolerance (joint_tol_rad 0.01 rad =
                          # 0.57 deg), so a path chained from the end of the last one is taken.
GRIP_FORCE_MIN_N = 0.001  # the SDK's effort unit (1 mN); 0 would be read by the arm node as "its default"
GRIP_FORCE_MAX_N = 5.0    # the SDK's effort range 0 ... 5000 mN (the twin's PICK_FORCE_N 5 = its maximum)
POLL_S = 0.05             # how often a running action looks at its goal / call
STOP_CONFIRM_S = 3.0      # a cancelled goal must end within this, else the stop is a fault (the arm may still move)
GOAL_SLACK_S = 60.0       # no result this long after a path should have ended: cancelled (the old gateway's + 60 s)
SETTLE_WAIT_S = 3.0       # hold / release: the status must show the result within this
SERVICE_WAIT_S = 3.0      # a service / action server must be there within this (the old gateway's 3 s)
ACCEPT_WAIT_S = 5.0       # a goal must be accepted or rejected within this (the old gateway's 5 s)
TRIGGER_WAIT_S = 15.0     # /arm/hold, /arm/disable (the old gateway's 15 s)
GRIP_WAIT_S = 20.0        # /arm/gripper (the node's own gripper_timeout_s 10 s inside it; the old gateway's 20 s)
JOINT_FAULT = re.compile(r"^j\d+:")   # ArmStatus.errors entries that are a joint driver's flags (the node does not
                                      # latch its fault on them; only /arm/enable clears them)

FAULT_ARM = "ARM_FAULT"               # the arm node latched a fault (its own words in the reason)
FAULT_SILENT = "ARM_NODE_SILENT"      # no status from the arm node
FAULT_STOP = "STOP_UNCONFIRMED"       # a cancel the arm node did not confirm
FAULT_INTERNAL = "INTERNAL"

NOT_PORTED = {
    "calibrate_handeye": (
        "not ported yet: the hand-eye routine runs today as a job of the old gateway (tools/gateway_server.py "
        "handeye_job: `ros2 run harness_calibration handeye`, its target a tag in view or the ChArUco board, gated by "
        "the gateway's start-posture check, installed into calib/handeye.yaml only at rms <= 3 mm and then held in the "
        "gateway's memory); use calib.handeye on TCP 7803 meanwhile"),
    "calibrate_absolute": (
        "not ported yet: the absolute calibration is a session, not one action (begin, collect at up to five board "
        "positions placed by hand, solve, install into calib/kinematics.yaml + calib/handeye.yaml, then the arm node "
        "restarts); use calib.absolute on TCP 7803 meanwhile"),
}

# ------------------------------------------------- the timed trajectory: COPIED from tools/gateway_server.py (7e2e7d5)
# lines 116-129 (the constants) and Gateway.follow_joints lines 628-648 (the blend branch) - THE COPY TO KEEP WHEN THE
# OLD GATEWAY RETIRES. Changed on purpose: a factor outside SPEED_MIN ... SPEED_MAX is refused (the gateway clamped it).
TEACH_VEL_RAD_S, TEACH_ACC_RAD_S2 = 0.35, 0.7   # joint speed / acceleration at factor 1
SPEED_MIN, SPEED_MAX = 0.25, 2.0                # a segment's factor (the gateway: max(0.25, min(FOLLOW_SPEED_MAX, k)))
FOLLOW_VEL_CAP_RAD_S = 0.70                     # under the arm node's gate piper.yaml traj_max_joint_vel_rad_s 0.75
FOLLOW_J1_FACTOR = 2.0                          # the base joint at twice the others' speed and acceleration (the node's
                                                # gate: traj_j1_vel_factor)


class PathRefused(Exception):
    """The planner cannot time the path within the limits (harness_planner.trajectory.PlanError's words)."""


class LinkError(Exception):
    """The arm node did not take a request (not running, rejected, no answer); its own words when it gave any."""


def _number(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _mtime(path):
    try:
        return os.path.getmtime(path) if path else None
    except OSError:
        return None


# =============================================================================================================== model
class ArmModel:
    """The arm's numbers this node needs, from the files the old gateway reads (calib/kinematics.yaml, piper.yaml): the
    corrected model (joint zero offsets: the twin's model degrees <-> the controller's joints), the controller's joint
    limits and the arm node's own margin inside them, the gripper's widest gap, and the timed-trajectory planner."""

    def __init__(self, kin, lim, P=None, kin_file=None, load_kin=None):
        self.kin, self.lim, self.P = kin, lim, dict(P or {})
        self._kin_file, self._load_kin = kin_file, load_kin
        self._kin_mtime = _mtime(kin_file)
        self._grip = (None, None)   # (gripper calibration file's mtime, its widest gap mm)

    @classmethod
    def load(cls, root: str) -> "ArmModel":
        """From a harness checkout (the old gateway's ROOT): the same model, limits and piper.yaml it plans with."""
        import yaml
        from harness_arm.kinematics import PiperKinematics
        from harness_planner.demo import load_model

        piper_yaml = os.path.join(root, "src", "harness_arm", "config", "piper.yaml")
        kin_file = os.path.join(root, "calib", "kinematics.yaml")
        with open(piper_yaml) as f:
            P = yaml.safe_load(f)["piper"]["ros__parameters"]
        _, lim = load_model(root)
        name = str(P.get("kinematics_model", "piper"))

        def load_kin():
            return PiperKinematics.from_file(kin_file, name)
        return cls(load_kin(), lim, P, kin_file, load_kin)

    def refresh(self) -> bool:
        """calib/kinematics.yaml read again when it changed (an absolute calibration installed by the old gateway changes
        the joint offsets, i.e. what a model degree is). A file caught half-written is tried again next time."""
        mtime = _mtime(self._kin_file)
        if self._load_kin is None or mtime == self._kin_mtime:
            return False
        try:
            self.kin = self._load_kin()
        except Exception:   # noqa: BLE001  keep the model in use; the next look reads it again
            return False
        self._kin_mtime = mtime
        return True

    @property
    def margin_rad(self) -> float:
        """The arm node's gate inside the controller's limits (piper_node._check_trajectory: ik_limit_margin_rad / 2;
        the planner's validate the same)."""
        return float(self.lim.ik_margin_rad) * 0.5

    def offsets(self):
        return [float(v) for v in self.kin.offsets]

    def to_model_deg(self, q_rad):
        return [math.degrees(q + o) for q, o in zip(q_rad, self.offsets())]

    def to_controller_rad(self, w_deg):
        return [math.radians(w) - o for w, o in zip(w_deg, self.offsets())]

    def limits_model_deg(self):
        """Per joint (lo, hi) in model degrees: the controller's range less the arm node's margin."""
        m = self.margin_rad
        return [(math.degrees(lo + m + o), math.degrees(hi - m + o))
                for (lo, hi), o in zip(self.kin.limits, self.offsets())]

    def gripper_max_mm(self) -> float:
        """The widest gap the arm node takes: its gripper calibration file's max_gap_m, else piper.yaml gripper_max_m
        (re-read when the file changes, like the old gateway's gripper_calib())."""
        path = os.path.expanduser(str(self.P.get("gripper_calib_file") or ""))
        mtime = _mtime(path)
        if self._grip[0] != mtime or mtime is None:
            value = None
            if mtime is not None:
                try:
                    import yaml
                    with open(path) as f:
                        value = float((yaml.safe_load(f) or {})["max_gap_m"]) * 1000.0
                except (OSError, KeyError, TypeError, ValueError):
                    value = None
            self._grip = (mtime, value)
        if self._grip[1] is not None:
            return self._grip[1]
        return float(self.P.get("gripper_max_m", 0.07)) * 1000.0

    def plan(self, waypoints_rad, speeds):
        """The timed trajectory (controller joints, rad) through every waypoint, each segment at its own factor:
        THE COPY of the old gateway's blend branch (see the constants above). Raises PathRefused."""
        import numpy as np
        from harness_planner.trajectory import PlanError, plan_smooth_path

        ks = np.array(speeds, float)
        f = np.array([FOLLOW_J1_FACTOR, 1.0, 1.0, 1.0, 1.0, 1.0])     # j1 first (THE BASE JOINT TWICE AS FAST)
        sv = np.minimum(TEACH_VEL_RAD_S * ks, FOLLOW_VEL_CAP_RAD_S)[:, None] * f
        sa = (TEACH_ACC_RAD_S2 * ks)[:, None] * f
        try:
            return plan_smooth_path(self.kin, np.array(waypoints_rad, float), self.lim, sv, sa, from_outside=True)
        except PlanError as error:
            raise PathRefused(str(error)) from None


# ================================================================================================================ link
class ArmLink:
    """What the Arm needs of the arm node: RosArmLink below, or a fake (tools/module_check_arm.py).

    snapshot() -> {"status":  None | {connected, all_enabled, teach_mode, errors [str], fault str, busy, age_s},
                   "joints":  None | {"q": [6 controller joints, rad], "age_s"},
                   "gripper": None | {"present", "width_m", "age_s"}}           (None = never heard)
    follow(trajectory, label) -> a goal {done(), progress() 0..1 | None, result() (ok, words), cancel()}; LinkError
    hold() / release() -> (ok, words); grip(gap_m, force_n, timeout_s) -> {success, stalled, width_m, message}:
        blocking (bounded), called only from an action's own thread; LinkError
    halt() / halt_all() / hold_soon() / grip_here(force_n): return at once (called from the command door)
    """

    def snapshot(self) -> dict:
        raise NotImplementedError

    def follow(self, trajectory, label: str):
        raise NotImplementedError

    def halt(self) -> None:
        """Cancel the goal THIS link sent (the arm node freezes the arm where it is, enabled)."""
        raise NotImplementedError

    def halt_all(self) -> None:
        """Cancel every goal on the arm node's actions, whoever sent it (an explicit stop)."""
        raise NotImplementedError

    def hold(self):
        raise NotImplementedError

    def hold_soon(self) -> None:
        raise NotImplementedError

    def release(self):
        raise NotImplementedError

    def grip(self, gap_m: float, force_n: float, timeout_s: float) -> dict:
        raise NotImplementedError

    def grip_here(self, force_n: float) -> None:
        """The jaws re-targeted where they are now (a grip in progress stops)."""
        raise NotImplementedError

    def close(self) -> None:
        pass


# ============================================================================================================== module
def _actions(gripper_max_mm: float) -> dict:
    return {
        "follow_path": {"args": {
            "waypoints_deg": {"unit": "deg: >= 2 postures of joints 1-6 in the twin's model degrees, the first = where "
                                      f"the arm is (within {START_TOL_DEG} deg per joint)"},
            "speeds": {"unit": f"x the teach speed per segment (len(waypoints_deg) - 1 numbers, {SPEED_MIN} ... "
                               f"{SPEED_MAX}); missing = 1 for every segment", "optional": True}},
            "uses": []},
        "grip_to": {"args": {
            "gap_mm": {"unit": "mm", "range": [0.0, gripper_max_mm]},
            "force_n": {"unit": "N (missing = the arm node's gripper_force)",
                        "range": [GRIP_FORCE_MIN_N, GRIP_FORCE_MAX_N], "optional": True}},
            "uses": []},
        "hold": {"args": {}, "uses": []},
        "release": {"args": {}, "uses": []},              # drag teach: joints 1-6 limp, the gripper keeps holding
        "calibrate_handeye": {"args": {}, "uses": []},    # declared, refused: NOT_PORTED
        "calibrate_absolute": {"args": {}, "uses": []},
    }


def _fields(gripper_max_mm: float) -> dict:
    return {
        "joints_deg": {"unit": "deg (joints 1-6, the twin's model degrees)", "writable": False},
        "gap_mm": {"unit": "mm", "range": [0.0, gripper_max_mm], "writable": False},
        "held": {"values": [True, False], "writable": False},   # False: released (drag teach) or the controller's teach
        "handeye_error_mm": {"unit": "mm", "writable": False},  # "unknown" until calibrated here
        "absolute_error_mm": {"unit": "mm", "writable": False},
    }


class Arm(Module):
    TYPE, VERSION = "piper", "0.1.0"
    ZONES = []

    def __init__(self, name, bus, link: ArmLink, model: ArmModel, timing=DEFAULT, serial: str = ""):
        self.link, self.model = link, model
        widest = round(model.gripper_max_mm(), 1)
        self.ACTIONS, self.FIELDS = _actions(widest), _fields(widest)   # the describe is read from these
        super().__init__(name, bus, timing, serial)
        self._acked_fault = ""     # the arm node's fault words a clear_fault was taken for (not latched again)
        self._heard = False        # an /arm/status has arrived at least once
        self._is_ready = None
        self._grip_force = 0.0
        self._watch_end = threading.Event()
        self._watcher = threading.Thread(target=self._watch, name=f"{name}-watch", daemon=True)

    # ------------------------------------------------------------ life
    def start(self):
        super().start()
        self._watcher.start()
        return self

    def close(self):
        self._watch_end.set()
        if self._watcher.is_alive():
            self._watcher.join(2)
        super().close()

    def safe_state(self):
        """Stop NOW the action THIS node runs, and only that: its goal cancelled (the arm node freezes the arm where it
        is, enabled), its grip in progress re-targeted where the jaws are. Nothing at all when none runs (the goal this
        link sent last may be the old path's turn now: never cancelled by itself), and never a hold: a released arm
        stays released (the operator may be guiding it by hand). Fire-and-forget: the command door never waits. The rest
        of an explicit stop request (every goal, a released arm held) is on_stop_request."""
        action = self.running()
        if action == "follow_path":
            self.link.halt()
        elif action == "grip_to":
            self.link.grip_here(self._grip_force)

    def on_stop_request(self):
        """A `stop` request of scope "device" (the twin's stop) stops the arm whoever moves it (after safe_state: this
        node's action ends "stopped"): every goal on the arm node's actions cancelled, and a released arm held, whoever
        released it. Never on the automatic safe state."""
        self.link.halt_all()
        if self._is_ready and self.value("held") is False:
            self.link.hold_soon()

    def check_clear_fault(self, args):
        """Refused while the arm node still has the cause (no fresh status, no CAN feedback, error bits)."""
        st = self._status()
        why = self._cause_persists(st)
        if why:
            raise Refused(M.FAULT, f"{why}: the fault cannot be cleared yet")
        self._acked_fault = st["fault"]   # the node keeps these words until a hold: not latched again

    @staticmethod
    def _cause_persists(st):
        if st is None:
            return "no fresh status from the arm node"
        if not st["connected"]:
            return "the arm node has no CAN feedback (" + (st["fault"] or "feedback lost") + ")"
        bits = [e for e in st["errors"] if not JOINT_FAULT.match(e)]
        if bits:
            return "the arm node still reports error bits: " + ",".join(bits)
        return None

    # ------------------------------------------------------------ the watch: status -> readiness, faults, values
    def _watch(self):
        while not self._watch_end.wait(SENSE_PERIOD_S):
            try:
                self._sense()
            except Exception as error:   # noqa: BLE001  a bug here must not end the watch silently
                self.latch_fault(FAULT_INTERNAL, f"the arm adapter's watch failed: {type(error).__name__}: {error}")

    def _sense(self):
        self.model.refresh()
        snap = self.link.snapshot()
        st = snap.get("status")
        if st is not None:
            self._heard = True
        if st is None or st["age_s"] > FRESH_S:
            self._set_ready(False)
            for field in ("joints_deg", "gap_mm", "held"):
                self.unknown(field)
            if self._heard and (st is None or st["age_s"] > SILENT_FAULT_S):
                since = "" if st is None else f" for {st['age_s']:.1f} s"
                self.latch_fault(FAULT_SILENT, f"no /arm/status from the arm node{since} (the node is not running or "
                                               "not reachable)")
            return
        self._set_ready(True)
        if not st["fault"]:
            self._acked_fault = ""
        if not st["connected"]:
            self.latch_fault(FAULT_ARM, "arm node: " + (st["fault"] or "feedback lost"))
        elif st["fault"] and st["fault"] != self._acked_fault:
            self.latch_fault(FAULT_ARM, "arm node: " + st["fault"])
        held = bool(st["all_enabled"] and not st["teach_mode"])
        self.sensed("held", held)
        joints = snap.get("joints")
        if joints is not None and joints["age_s"] <= FRESH_S:
            self.sensed("joints_deg", [round(v, 3) for v in self.model.to_model_deg(joints["q"])])
        else:
            self.unknown("joints_deg")
        grip = snap.get("gripper")
        if grip is not None and grip["present"] and grip["age_s"] <= FRESH_S:
            self.sensed("gap_mm", round(grip["width_m"] * 1000.0, 2))
        else:
            self.unknown("gap_mm")

    def _set_ready(self, ready: bool):
        if ready != self._is_ready:
            self._is_ready = ready
            self.set_ready(ready)

    def _status(self):
        st = self.link.snapshot().get("status")
        return st if st is not None and st["age_s"] <= FRESH_S else None

    def _here_rad(self):
        joints = self.link.snapshot().get("joints")
        if joints is None or joints["age_s"] > FRESH_S:
            raise Refused(M.UNKNOWN_VALUE, "joints_deg is unknown: no fresh joint feedback from the arm node")
        return [float(v) for v in joints["q"]]

    def _check_motion(self):
        """What every arm motion needs: the arm held, the arm node's fault cleared, the arm node idle."""
        held = self.value("held")
        if held is False:
            raise Refused(M.NOT_READY, "the arm is released (drag teach) or the controller is in teach mode: send hold "
                                       "first")
        if held is not True:
            raise Refused(M.NOT_READY, "whether the arm holds is unknown (no fresh status from the arm node)")
        st = self._status()
        if st is not None and st["fault"]:
            raise Refused(M.NOT_READY, f"the arm node keeps its fault latched ({st['fault']}): send hold to clear it")
        self._check_node_idle(st)

    def _check_node_idle(self, st=None):
        st = st if st is not None else self._status()
        if st is not None and st["busy"]:
            raise Refused(M.BUSY, "the arm node is running a motion another client gave it")

    # ------------------------------------------------------------ follow_path
    def check_follow_path(self, args):
        self._check_motion()
        waypoints = _postures(args["waypoints_deg"])
        _speeds(args.get("speeds"), len(waypoints))
        here = self.model.to_model_deg(self._here_rad())
        _start_gap(waypoints[0], here)
        _inside(waypoints, self.model.limits_model_deg())

    def do_follow_path(self, args, run):
        waypoints = _postures(args["waypoints_deg"])
        speeds = args.get("speeds") or [1.0] * (len(waypoints) - 1)
        self.model.refresh()
        try:
            here = self._here_rad()
            _start_gap(waypoints[0], self.model.to_model_deg(here))   # the arm moved since the accept?
        except Refused as refusal:
            raise Failed(f"not started: {refusal.reason}") from None
        path = [here] + [self.model.to_controller_rad(w) for w in waypoints[1:]]
        try:
            trajectory = self.model.plan(path, speeds)
        except PathRefused as error:
            raise Failed(f"not started: the planner cannot time this path: {error}") from None
        if run.stopped:
            return
        try:
            goal = self.link.follow(trajectory, f"{self.name} follow_path #{run.id}")
        except LinkError as error:
            raise Failed(f"not started: {error}") from None
        deadline = time.monotonic() + float(trajectory.duration) + GOAL_SLACK_S
        while not goal.done():
            if run.stopped:
                goal.cancel()
                self._confirm_stop(goal)
                return
            if time.monotonic() > deadline:
                goal.cancel()
                self._confirm_stop(goal)
                raise Failed(f"no result {GOAL_SLACK_S:.0f} s after the path should have ended: cancelled")
            progress = goal.progress()
            if progress is not None:
                run.progress(progress)
            run.sleep(POLL_S)
        ok, words = goal.result()
        if not ok and not run.stopped:
            raise Failed(f"arm node: {words}")

    def _confirm_stop(self, goal):
        end = time.monotonic() + STOP_CONFIRM_S
        while not goal.done() and time.monotonic() < end:
            time.sleep(POLL_S)
        if not goal.done():
            raise Fault(FAULT_STOP, f"the arm node did not confirm the stop within {STOP_CONFIRM_S:.0f} s: the arm may "
                                    "still be moving")

    # ------------------------------------------------------------ grip_to
    def check_grip_to(self, args):
        grip = self.link.snapshot().get("gripper")
        if grip is None or grip["age_s"] > FRESH_S or not grip["present"]:
            raise Refused(M.NOT_READY, "the arm node reports no gripper")
        widest = self.model.gripper_max_mm()
        if args["gap_mm"] > widest:
            raise Refused(M.OUT_OF_RANGE, f"gap_mm = {args['gap_mm']} is beyond the gripper's widest gap "
                                          f"{widest:.1f} mm (its calibration now)")

    def do_grip_to(self, args, run):
        self._grip_force = float(args.get("force_n") or 0.0)   # 0 = the arm node's own default (SetGripper.srv)
        try:
            reply = self.link.grip(float(args["gap_mm"]) / 1000.0, self._grip_force, 0.0)
        except LinkError as error:
            raise Failed(str(error)) from None
        if _number(reply.get("width_m")):
            self.sensed("gap_mm", round(reply["width_m"] * 1000.0, 2))
        if not reply.get("success") and not run.stopped:
            raise Failed(f"arm node: {reply.get('message', '')}")

    # ------------------------------------------------------------ hold / release
    def check_hold(self, args):
        self._check_node_idle()

    def do_hold(self, args, run):
        self._trigger(self.link.hold, run,
                      lambda st: st["all_enabled"] and not st["teach_mode"] and not st["fault"], "the arm held")
        self.sensed("held", True)

    def check_release(self, args):
        self._check_node_idle()

    def do_release(self, args, run):
        """The drag-teach release, as the old gateway's arm.release: /arm/disable (joints 1-6 limp, the gripper keeps
        holding). The one way to make the arm limp, on purpose; never called by anything but this action."""
        self._trigger(self.link.release, run, lambda st: not st["all_enabled"], "the arm released")
        self.sensed("held", False)

    def _trigger(self, call, run, shown, what):
        try:
            ok, words = call()
        except LinkError as error:
            raise Failed(str(error)) from None
        if not ok:
            raise Failed(f"arm node: {words}")
        end = time.monotonic() + SETTLE_WAIT_S
        while True:
            st = self._status()
            if st is not None and shown(st):
                return
            if time.monotonic() > end:
                raise Failed(f"arm node said '{words}' but its status does not show {what} after {SETTLE_WAIT_S:.0f} s")
            if not run.sleep(POLL_S):
                return

    # ------------------------------------------------------------ the two calibrations: not ported yet
    def check_calibrate_handeye(self, args):
        raise Refused(M.NOT_READY, NOT_PORTED["calibrate_handeye"])

    def do_calibrate_handeye(self, args, run):
        raise Failed(NOT_PORTED["calibrate_handeye"])

    def check_calibrate_absolute(self, args):
        raise Refused(M.NOT_READY, NOT_PORTED["calibrate_absolute"])

    def do_calibrate_absolute(self, args, run):
        raise Failed(NOT_PORTED["calibrate_absolute"])


# ------------------------------------------------------------ follow_path's checks (what a describe cannot say)
def _postures(waypoints):
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise Refused(M.BAD_ARGS, "waypoints_deg needs at least 2 postures (the first = where the arm is)")
    out = []
    for k, w in enumerate(waypoints, 1):
        if not isinstance(w, list) or len(w) != 6 or not all(_number(x) for x in w):
            raise Refused(M.BAD_ARGS, f"waypoint {k} is not 6 finite joint angles")
        out.append([float(x) for x in w])
    return out


def _speeds(speeds, n_waypoints):
    if speeds is None:
        return
    if not isinstance(speeds, list) or len(speeds) != n_waypoints - 1 or not all(_number(x) for x in speeds):
        raise Refused(M.BAD_ARGS, f"speeds needs {n_waypoints - 1} numbers, one per segment")
    for k, s in enumerate(speeds, 1):
        if not SPEED_MIN <= s <= SPEED_MAX:
            raise Refused(M.OUT_OF_RANGE, f"speeds: segment {k} (waypoint {k} to {k + 1}) = {s} is outside "
                                          f"{SPEED_MIN} ... {SPEED_MAX} x the teach speed")


def _start_gap(first, here):
    gaps = [abs(a - b) for a, b in zip(first, here)]
    j = max(range(6), key=gaps.__getitem__)
    if gaps[j] > START_TOL_DEG:
        raise Refused(M.OUT_OF_RANGE, f"waypoint 1, joint {j + 1} = {first[j]:.2f} deg is {gaps[j]:.2f} deg from where "
                                      f"the arm is ({here[j]:.2f} deg), more than {START_TOL_DEG} deg: plan the path "
                                      "from the arm's current posture")


def _inside(waypoints, limits):
    """Every waypoint after the first (which becomes where the arm is) within the limits, in model degrees."""
    for k, w in enumerate(waypoints[1:], 2):
        for j, (q, (lo, hi)) in enumerate(zip(w, limits)):
            if not lo <= q <= hi:
                raise Refused(M.OUT_OF_RANGE, f"waypoint {k}, joint {j + 1} = {q:.2f} deg is outside its limit "
                                              f"{lo:.2f} ... {hi:.2f} deg (the controller's range less the arm node's "
                                              "margin, in model degrees)")


# ============================================================================================================ ROS link
def _wait(future, timeout_s: float) -> bool:
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    return done.wait(timeout_s) or future.done()


class _RosGoal:
    def __init__(self):
        self._handle, self._result, self._progress = None, None, None

    def feedback(self, message):
        self._progress = float(message.feedback.progress)

    def attach(self, handle):
        self._handle, self._result = handle, handle.get_result_async()

    def done(self):
        return self._result.done()

    def progress(self):
        return self._progress

    def result(self):
        wrapped = self._result.result()
        if wrapped is None:
            return False, "no result from the arm node"
        return bool(wrapped.result.success), str(wrapped.result.message)

    def cancel(self):
        self._handle.cancel_goal_async()


class RosArmLink(ArmLink):
    """The arm node's EXISTING interface as a ROS client; nothing new on the node's side.

      heard    /arm/status (ArmStatus), /arm/gripper_status (GripperStatus), /joint_states (JointState)
      action   /arm/follow_trajectory (FollowTrajectory)
      services /arm/hold, /arm/disable (std_srvs/Trigger; /arm/disable = the drag-teach release), /arm/gripper
               (SetGripper), <action>/_action/cancel_goal (action_msgs/CancelGoal) for halt
    """

    ARM_ACTIONS = ("/arm/move_j", "/arm/move_pose", "/arm/follow_trajectory")   # = tools/gateway_server.py ARM_ACTIONS
    JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

    def __init__(self, node):
        from action_msgs.srv import CancelGoal
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from harness_msgs.action import FollowTrajectory
        from harness_msgs.msg import ArmStatus, GripperStatus
        from harness_msgs.srv import SetGripper

        self.node = node
        self.FollowTrajectory, self.Trigger, self.SetGripper, self.CancelGoal = (FollowTrajectory, Trigger, SetGripper,
                                                                                 CancelGoal)
        self._guard = threading.Lock()
        self._last = {}    # "status" | "joints" | "gripper" -> (plain dict, time.monotonic() received)
        self._handle = None   # the goal this link sent last (halt cancels it)
        group = ReentrantCallbackGroup()
        node.create_subscription(ArmStatus, "/arm/status", self._on_status, 10, callback_group=group)
        node.create_subscription(GripperStatus, "/arm/gripper_status", self._on_gripper, 10, callback_group=group)
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10, callback_group=group)
        self._follow = ActionClient(node, FollowTrajectory, "/arm/follow_trajectory", callback_group=group)
        self._hold = node.create_client(Trigger, "/arm/hold", callback_group=group)
        self._disable = node.create_client(Trigger, "/arm/disable", callback_group=group)
        self._gripper = node.create_client(SetGripper, "/arm/gripper", callback_group=group)
        self._cancel = [node.create_client(CancelGoal, a + "/_action/cancel_goal", callback_group=group)
                        for a in self.ARM_ACTIONS]

    # ---- heard
    def _keep(self, key, value):
        with self._guard:
            self._last[key] = (value, time.monotonic())

    def _on_status(self, m):
        self._keep("status", {"connected": bool(m.connected), "all_enabled": bool(m.all_enabled),
                              "teach_mode": bool(m.teach_mode), "errors": [str(e) for e in m.errors],
                              "fault": str(m.fault), "busy": bool(m.busy)})

    def _on_joints(self, m):
        names = list(m.name)
        if all(j in names for j in self.JOINT_NAMES):
            self._keep("joints", {"q": [float(m.position[names.index(j)]) for j in self.JOINT_NAMES]})

    def _on_gripper(self, m):
        self._keep("gripper", {"present": bool(m.present), "width_m": float(m.width_m)})

    def snapshot(self):
        now = time.monotonic()
        with self._guard:
            last = dict(self._last)
        out = {}
        for key in ("status", "joints", "gripper"):
            if key in last:
                value, at = last[key]
                out[key] = dict(value, age_s=now - at)
            else:
                out[key] = None
        return out

    # ---- asked
    def follow(self, trajectory, label):
        from harness_planner.demo import to_msg   # the old gateway's own Trajectory -> JointTrajectory
        if not self._follow.wait_for_server(timeout_sec=SERVICE_WAIT_S):
            raise LinkError("/arm/follow_trajectory is not available (is the arm node running?)")
        goal = _RosGoal()
        sent = self._follow.send_goal_async(self.FollowTrajectory.Goal(trajectory=to_msg(trajectory), label=label),
                                            feedback_callback=goal.feedback)
        if not _wait(sent, ACCEPT_WAIT_S):
            # accepted later, it would move unwatched: cancel it the moment it is
            sent.add_done_callback(lambda f: f.result() is not None and f.result().accepted
                                   and f.result().cancel_goal_async())
            raise LinkError(f"the arm node did not answer the goal within {ACCEPT_WAIT_S:.0f} s")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise LinkError("rejected by the arm node (busy, fault, teach mode or not enabled)")
        goal.attach(handle)
        self._handle = handle
        return goal

    def halt(self):
        handle = self._handle
        if handle is not None:
            handle.cancel_goal_async()

    def halt_all(self):
        for client in self._cancel:   # a zero goal id and stamp = every goal (the old gateway's cancel_arm)
            client.call_async(self.CancelGoal.Request())

    def _call(self, client, request, name, timeout_s):
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            raise LinkError(f"{name} is not answering (is the arm node running?)")
        future = client.call_async(request)
        if not _wait(future, timeout_s):
            forget = getattr(client, "remove_pending_request", None)
            if forget is not None:
                forget(future)
            raise LinkError(f"{name} did not answer within {timeout_s:.0f} s")
        reply = future.result()
        if reply is None:
            raise LinkError(f"{name}: no reply")
        return reply

    def hold(self):
        reply = self._call(self._hold, self.Trigger.Request(), "/arm/hold", TRIGGER_WAIT_S)
        return bool(reply.success), str(reply.message)

    def hold_soon(self):
        self._hold.call_async(self.Trigger.Request())

    def release(self):
        reply = self._call(self._disable, self.Trigger.Request(), "/arm/disable", TRIGGER_WAIT_S)
        return bool(reply.success), str(reply.message)

    def grip(self, gap_m, force_n, timeout_s):
        reply = self._call(self._gripper, self.SetGripper.Request(width_m=float(gap_m), force=float(force_n),
                                                                  timeout_s=float(timeout_s)), "/arm/gripper",
                           GRIP_WAIT_S)
        return {"success": bool(reply.success), "stalled": bool(reply.stalled), "width_m": float(reply.width_m),
                "message": str(reply.message)}

    def grip_here(self, force_n):
        grip = (self.snapshot().get("gripper") or {})
        if grip.get("present"):
            self._gripper.call_async(self.SetGripper.Request(width_m=float(grip["width_m"]), force=float(force_n)))


# ================================================================================================================ main
PARAMETERS = {
    "instance": "arm_1",                    # the module instance = its ROS namespace (/arm_1/command, /arm_1/state)
    "harness_root": "~/bunny-harness-dev",  # the harness checkout: calib/kinematics.yaml, src/harness_arm/config/piper.yaml
    "serial": "",                           # describe.identity.serial
}


def main(args=None):
    from railab_harness.ros_binding import RosBus, ros_environment
    ros_environment()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    try:   # this node's own signal handling: safe_state must still reach the arm node on Ctrl-C / SIGTERM
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    probe = rclpy.create_node("arm_module_parameters")
    params = {name: probe.declare_parameter(name, default).value for name, default in PARAMETERS.items()}
    probe.destroy_node()
    node = rclpy.create_node(f"{params['instance']}_module")
    for name, value in params.items():
        node.declare_parameter(name, value)
    log = node.get_logger()
    root = os.path.expanduser(str(params["harness_root"]))
    try:
        model = ArmModel.load(root)
    except Exception as error:   # noqa: BLE001  no model = no joint conversion, no planning: do not start
        log.error(f"cannot load the arm's model from {root}: {type(error).__name__}: {error}")
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    link = RosArmLink(node)
    bus = RosBus(node=node)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    threading.Thread(target=executor.spin, name="arm-module-spin", daemon=True).start()
    arm = Arm(str(params["instance"]), bus, link, model, serial=str(params["serial"])).start()
    log.info(f"{arm.name}: the arm's adapter on /{arm.name}/command and /{arm.name}/state (model from {root})")
    leave = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: leave.set())
    while not leave.wait(0.5):
        pass
    try:
        arm.close()   # this node's running action stopped and its own goal cancelled - while ROS still runs
    finally:
        bus.close()
        link.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
