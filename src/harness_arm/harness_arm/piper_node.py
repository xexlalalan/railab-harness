#!/usr/bin/env python3
"""PIPER driver node (harness_arm).

Sole owner of the CAN channel. Everything the rest of the graph knows about the
arm comes from here, and every motion goes through here.

Publishes (rate_hz):
  /joint_states            sensor_msgs/JointState   joint1..joint6 (rad) + gripper (m)
  /arm/tcp_pose            geometry_msgs/PoseStamped  frame arm_base
  /arm/status              harness_msgs/ArmStatus
  /arm/gripper_status      harness_msgs/GripperStatus
Services:
  /arm/enable   std_srvs/Trigger   enable all joints (refused if error bits are set)
  /arm/disable  std_srvs/Trigger   disable all joints -> arm goes limp = drag-teach
  /arm/hold     std_srvs/Trigger   enable + move_j(current) = hold here (tools/piper_hold.py logic)
  /arm/set_speed  harness_msgs/SetSpeed
  /arm/set_joints harness_msgs/SetJoints   power or release SELECTED joints (1..6, 7 = gripper)
  /arm/gripper    harness_msgs/SetGripper
  /arm/gripper_release  std_srvs/Trigger   un-power the gripper motor only (jaws free); the ARM keeps holding
  /arm/gripper_enable   std_srvs/Trigger   re-power the gripper at its current width
Actions:
  /arm/move_j     harness_msgs/MoveJ
  /arm/move_pose  harness_msgs/MovePose   (linear=false -> move_p, true -> move_l)

Rules: enable state comes from joint feedback, never from an SDK return value.
A watchdog disables the arm when feedback stops; any error bit cancels the
running goal and disables. One motion goal at a time.
"""
import math
import os
import threading
import time

import numpy as np
import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from harness_msgs.action import MoveJ, MovePose
from harness_arm.kinematics import PiperKinematics, T_from_xyzrpy, xyzrpy_from_T, pose_error
from harness_msgs.msg import ArmStatus, GripperStatus
from harness_msgs.srv import SetGripper, SetJoints, SetSpeed

from pyAgxArm import AgxArmFactory, PiperFW, create_agx_arm_config
from scipy.spatial.transform import Rotation as _Rot

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _name(x):
    """Enum-ish -> short upper-case name ('CAN_CTRL(0x1)' -> 'CAN_CTRL')."""
    n = getattr(x, "name", None)
    if n:
        return str(n)
    s = str(x)
    return s.split("(")[0]


def _rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _ang_err(a, b):
    """Angle between two fixed-axis XYZ (roll, pitch, yaw) orientations, rad."""
    qa = _rpy_to_quat(*a)
    qb = _rpy_to_quat(*b)
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return 2.0 * math.acos(min(1.0, dot))


class PiperNode(Node):
    def __init__(self):
        super().__init__("piper")
        p = self.declare_parameters("", [
            ("can_channel", "can0"), ("rate_hz", 100.0), ("default_speed_percent", 10),
            ("max_speed_percent", 30), ("watchdog_s", 0.25), ("move_timeout_s", 30.0),
            ("joint_tol_rad", 0.01), ("pos_tol_m", 0.003), ("ang_tol_rad", 0.03),
            ("settle_samples", 5), ("workspace_min", [-0.7, -0.7, -0.05]),
            ("workspace_max", [0.7, 0.7, 0.9]), ("gripper_enabled", True),
            ("gripper_max_m", 0.07), ("gripper_force", 1.0), ("gripper_tol_m", 0.002),
            ("gripper_timeout_s", 4.0), ("gripper_inverted", False), ("gripper_travel_m", 0.0967), ("gripper_scale", 1.0),
            ("max_joint_delta_rad", 1.2), ("pose_max_joint_delta_rad", 0.9),
            ("floor_enabled", False), ("floor_plane", [0.0, 0.0, 0.0]), ("tcp_offset_m", 0.0), ("floor_margin_m", 0.01),
            ("kinematics_file", ""), ("kinematics_model", "piper"), ("ik_limit_margin_rad", 0.02),
            ("linear_step_m", 0.02), ("linear_step_rad", 0.1), ("stall_s", 2.0), ("pose_total_jump_rad", 2.5),
        ])
        self.P = {x.name: x.value for x in p}
        self.cb = ReentrantCallbackGroup()
        self.sdk = threading.Lock()      # serialises SDK *commands*
        self.motion = threading.Lock()   # one motion goal at a time
        self.speed = int(self.P["default_speed_percent"])
        self.fault = ""
        self.busy = False
        self.cancel_requested = False
        self.last_fb_t = 0.0
        self.last_fb_stamp = None
        self.snap = {}                   # latest feedback snapshot, filled by _tick

        self._connect()
        self._load_kinematics()

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/arm/tcp_pose", 10)          # corrected model (harness FK)
        self.pub_tcp_raw = self.create_publisher(PoseStamped, "/arm/tcp_pose_raw", 10)  # controller's own report
        self.pub_status = self.create_publisher(ArmStatus, "/arm/status", 10)
        self.pub_grip = self.create_publisher(GripperStatus, "/arm/gripper_status", 10)

        for name, cb in (("enable", self.srv_enable), ("disable", self.srv_disable),
                         ("hold", self.srv_hold), ("gripper_release", self.srv_gripper_release),
                         ("gripper_enable", self.srv_gripper_enable)):
            self.create_service(Trigger, f"/arm/{name}", cb, callback_group=self.cb)
        self.create_service(SetSpeed, "/arm/set_speed", self.srv_set_speed, callback_group=self.cb)
        self.create_service(SetJoints, "/arm/set_joints", self.srv_set_joints, callback_group=self.cb)
        self.create_service(SetGripper, "/arm/gripper", self.srv_gripper, callback_group=self.cb)

        self.as_j = ActionServer(self, MoveJ, "/arm/move_j", self.exec_move_j,
                                 goal_callback=self.goal_cb, cancel_callback=self.cancel_cb,
                                 callback_group=self.cb)
        self.as_p = ActionServer(self, MovePose, "/arm/move_pose", self.exec_move_pose,
                                 goal_callback=self.goal_cb, cancel_callback=self.cancel_cb,
                                 callback_group=self.cb)

        self.create_timer(1.0 / float(self.P["rate_hz"]), self._tick, callback_group=self.cb)
        self.get_logger().info(f"piper node up on {self.P['can_channel']}, speed cap "
                               f"{self.P['max_speed_percent']} %, gripper="
                               f"{'yes' if self.ee else 'no'}")

    # ------------------------------------------------------------------ gripper mapping
    # Fingers mounted the other way round (2026-09-18): the motor's "open" is the jaws' closed.
    # physical gap = (travel - raw) * scale when inverted; raw = travel - gap / scale.
    def _grip_phys(self, raw):
        if not self.P["gripper_inverted"]:
            return raw
        return max(0.0, (float(self.P["gripper_travel_m"]) - raw) * float(self.P["gripper_scale"]))

    def _grip_raw(self, gap):
        if not self.P["gripper_inverted"]:
            return gap
        return max(0.0, float(self.P["gripper_travel_m"]) - gap / float(self.P["gripper_scale"]))

    # ------------------------------------------------------------------ kinematics
    def _load_kinematics(self):
        """Corrected arm model: SDK MDH table + joint zero offsets from calib/kinematics.yaml.
        /arm/tcp_pose and /arm/move_pose use this model; the controller only ever gets joint targets."""
        offsets = [0.0] * 6
        alphas = [0.0] * 6
        path = os.path.expanduser(str(self.P["kinematics_file"] or ""))
        if path:
            try:
                with open(path) as f:
                    doc = yaml.safe_load(f) or {}
                offsets = [float(v) for v in doc.get("joint_zero_offsets_rad", offsets)]
                alphas = [float(v) for v in doc.get("alpha_offsets_rad", [0.0] * 6)]
                self.get_logger().info(f"kinematics: joint zero offsets {[round(math.degrees(v), 2) for v in offsets]} deg, "
                                       f"axis-angle offsets {[round(math.degrees(v), 2) for v in alphas]} deg from {path}")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"kinematics: cannot read {path} ({e}); using ZERO offsets (controller model)")
        else:
            self.get_logger().warning("kinematics: no kinematics_file set; using the controller model (known j5 zero error)")
        self.kin = PiperKinematics.from_sdk(str(self.P["kinematics_model"]), offsets, alpha_offsets=alphas)

    def _publish_pose(self, pub, pose, now):
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "arm_base"
        x, y, z, r, p, yw = pose
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        qx, qy, qz, qw = _rpy_to_quat(r, p, yw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        pub.publish(ps)

    # ------------------------------------------------------------------ SDK
    def _connect(self):
        ch = self.P["can_channel"]
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot="piper", firmeware_version=PiperFW.V189, interface="socketcan", channel=ch))
        self.robot.connect()
        deadline = time.time() + 5
        while self.robot.get_joint_angles() is None or self.robot.get_arm_status() is None:
            if time.time() > deadline:
                raise RuntimeError(f"no PIPER feedback on {ch}")
            time.sleep(0.05)
        fw = None
        for _ in range(10):                      # the first query can race the feedback stream
            fw = self.robot.get_firmware()
            if fw:
                break
            time.sleep(0.1)
        self.get_logger().info(f"PIPER connected on {ch}: {fw}")
        self.ee = None
        if self.P["gripper_enabled"]:
            try:
                self.ee = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
                time.sleep(0.2)
                if self.ee.get_gripper_status() is None:
                    self.get_logger().warning("AgxGripper initialised but sends no status")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"AgxGripper not available: {e}")
                self.ee = None

    def _errors(self, st):
        if st is None:
            return ["no status feedback"]
        return [k for k, v in vars(st.msg.err_status).items() if v]

    def _read(self):
        """Read feedback into self.snap; returns the snapshot."""
        ja = self.robot.get_joint_angles()
        st = self.robot.get_arm_status()
        tcp = self.robot.get_tcp_pose()
        en = self.robot.get_joints_enable_status_list() or [False] * 6
        now = time.time()
        stamp = getattr(ja, "timestamp", None) if ja is not None else None
        if stamp is not None and stamp != self.last_fb_stamp:
            self.last_fb_stamp = stamp
            self.last_fb_t = now
        elif stamp is None and ja is not None and self.last_fb_t == 0.0:
            self.last_fb_t = now
        jfaults = []
        for j in range(1, 7):
            try:
                ds = self.robot.get_driver_states(j)
            except Exception:  # noqa: BLE001
                ds = None
            if ds is not None:
                f = ds.msg.foc_status
                for k in ("driver_overcurrent", "driver_error_status", "stall_status", "motor_overheating",
                          "driver_overheating", "voltage_too_low", "collision_status"):
                    if getattr(f, k, False):
                        jfaults.append(f"j{j}:{k}")
        s = {
            "t": now,
            "joint_faults": jfaults,
            "joints": list(ja.msg) if ja is not None else None,
            "tcp": self.kin.fk_xyzrpy(list(ja.msg)) if ja is not None else None,   # corrected model
            "tcp_raw": list(tcp.msg) if tcp is not None else None,                  # controller report
            "enabled": list(bool(x) for x in en),
            "errors": self._errors(st),
            "ctrl_mode": _name(st.msg.ctrl_mode) if st else "UNKNOWN",
            "arm_status": _name(st.msg.arm_status) if st else "UNKNOWN",
            "motion_status": _name(st.msg.motion_status) if st else "UNKNOWN",
            "teach": (_name(st.msg.teach_status) not in ("DISABLED", "0", "OFF")) if st else False,
            "age": (now - self.last_fb_t) if self.last_fb_t else 1e9,
        }
        try:
            s["fps"] = float(self.robot.get_fps() or 0.0)
        except Exception:  # noqa: BLE001
            s["fps"] = 0.0
        if self.ee is not None:
            g = self.ee.get_gripper_status()
            if g is not None:
                foc = g.msg.foc_status
                s["grip"] = {
                    "width": self._grip_phys(float(g.msg.value)), "width_raw": float(g.msg.value), "force": float(g.msg.force),
                    "enabled": bool(getattr(foc, "driver_enable_status", False)),
                    "errors": [k for k, v in vars(foc).items()
                               if v and k not in ("driver_enable_status", "homing_status", "sensor_status")],
                }
        self.snap = s
        return s

    # ------------------------------------------------------------------ periodic
    def _tick(self):
        s = self._read()
        now = self.get_clock().now().to_msg()
        connected = s["age"] < float(self.P["watchdog_s"])

        # watchdog / error bits: NEVER disable (a limp arm falls; the PIPER holds its last
        # target by itself when CAN goes quiet). Cancel the running goal, latch a fault,
        # refuse new goals until /arm/enable clears it.
        if not connected or s["errors"]:
            why = "feedback lost" if not connected else "error bits: " + ",".join(s["errors"])
            if self.fault != why:
                self.get_logger().error(f"FAULT: {why} (arm left enabled, goals cancelled)")
            self.fault = why
            self.cancel_requested = True

        if s["joints"] is not None:
            js = JointState()
            js.header.stamp = now
            js.name = list(JOINT_NAMES)
            js.position = [float(x) for x in s["joints"]]
            if "grip" in s:
                js.name.append("gripper")
                js.position.append(s["grip"]["width"])
            self.pub_js.publish(js)
        if s["tcp"] is not None:
            self._publish_pose(self.pub_tcp, s["tcp"], now)
        if s.get("tcp_raw") is not None:
            self._publish_pose(self.pub_tcp_raw, s["tcp_raw"], now)

        m = ArmStatus()
        m.header.stamp = now
        m.connected = connected
        m.ctrl_mode, m.arm_status, m.motion_status = s["ctrl_mode"], s["arm_status"], s["motion_status"]
        m.teach_mode = s["teach"]
        m.joints_enabled = s["enabled"]
        m.all_enabled = all(s["enabled"])
        m.errors = s["errors"] + s.get("joint_faults", [])
        m.ok = connected and not s["errors"] and not s["teach"]
        m.feedback_age_s = min(s["age"], 1e6)
        m.feedback_hz = s["fps"]
        m.speed_percent = self.speed
        m.busy = self.busy
        m.fault = self.fault
        self.pub_status.publish(m)

        g = GripperStatus()
        g.header.stamp = now
        g.present = "grip" in s
        if g.present:
            g.width_m = s["grip"]["width"]
            g.force_n = s["grip"]["force"]
            g.enabled = s["grip"]["enabled"]
            g.errors = s["grip"]["errors"]
        self.pub_grip.publish(g)

    # ------------------------------------------------------------------ helpers
    def _wait_enabled(self, want, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            en = self.robot.get_joints_enable_status_list() or []
            if len(en) == 6 and all(bool(x) == want for x in en):
                return True
            time.sleep(0.05)
        return False

    def _refuse(self):
        """Reason a motion/enable must be refused, or None."""
        s = self.snap or self._read()
        if s["age"] >= float(self.P["watchdog_s"]):
            return "no CAN feedback"
        if s["errors"]:
            return "error bits set: " + ",".join(s["errors"])
        if s["teach"]:
            return "arm is in teach mode"
        if self.fault:
            return f"fault latched: {self.fault} (call /arm/enable to clear)"
        return None

    def _clamp_speed(self, pct):
        pct = int(pct) if pct else self.speed
        return max(1, min(int(self.P["max_speed_percent"]), pct))

    def _in_workspace(self, xyz):
        lo, hi = self.P["workspace_min"], self.P["workspace_max"]
        return all(lo[i] <= xyz[i] <= hi[i] for i in range(3))

    def _floor_violation(self, pose):
        """Reason string if the gripper TIP at this flange pose would come within floor_margin_m
        of the calibrated floor plane (calib/floor.yaml -> params), else None."""
        if not self.P["floor_enabled"]:
            return None
        a, b, c = self.P["floor_plane"]
        r3 = _Rot.from_euler("xyz", pose[3:]).as_matrix()[:, 2]
        tip = np.array(pose[:3]) + float(self.P["tcp_offset_m"]) * r3
        floor_z = a * tip[0] + b * tip[1] + c
        clearance = tip[2] - floor_z
        if clearance < float(self.P["floor_margin_m"]):
            return f"tip would be {clearance*1000:.1f} mm above the floor (margin {self.P['floor_margin_m']*1000:.0f} mm)"
        return None

    # ------------------------------------------------------------------ services
    def srv_enable(self, req, res):
        s = self._read()
        if s["errors"] or s["age"] >= float(self.P["watchdog_s"]):
            res.success, res.message = False, "refused: " + ("errors: " + ",".join(s["errors"]) if s["errors"] else "no CAN feedback")
            return res
        self.fault = ""
        cleared = []
        for jf in s.get("joint_faults", []):
            j = int(jf[1])
            with self.sdk:
                self.robot.clear_joint_error(j)
            cleared.append(jf)
        if cleared:
            time.sleep(0.3)
        with self.sdk:
            self._enable_joints()
        ok = self._wait_enabled(True)
        if ok:
            self.fault = ""
        res.success = ok
        res.message = ("all joints enabled" if ok else "joints did not report enabled within 3 s: " +
                       str([i + 1 for i, e in enumerate(self.snap.get("enabled", [])) if not e]))
        if cleared:
            res.message += f" (cleared driver faults: {', '.join(cleared)})"
        return res

    def _enable_joints(self):
        """Power joints 1..6 only; the gripper (joint 7) keeps whatever state it has."""
        for j in range(1, 7):
            self.robot.enable(j)

    def _disable_joints(self):
        """Free joints 1..6 only. robot.disable() with no index would also free the gripper (joint 7);
        the operator rule is that releasing the arm never touches the gripper (2026-09-27)."""
        for j in range(1, 7):
            self.robot.disable(j)

    def srv_disable(self, req, res):
        self.cancel_requested = True
        with self.sdk:
            self._disable_joints()
        ok = self._wait_enabled(False)
        res.success = ok
        res.message = "all joints disabled (arm is limp)" if ok else "joints still report enabled"
        return res

    def srv_hold(self, req, res):
        why = self._refuse()
        if why:
            res.success, res.message = False, "refused: " + why
            return res
        if not self.motion.acquire(blocking=False):
            res.success, res.message = False, "refused: a motion goal is running"
            return res
        try:
            with self.sdk:
                self.robot.set_speed_percent(self.speed)
                self._enable_joints()
            if not self._wait_enabled(True):
                res.success, res.message = False, "joints did not enable"
                return res
            time.sleep(0.1)
            target = list(self._read()["joints"])   # capture AFTER enable: no sag counted as drift
            with self.sdk:
                self.robot.move_j(target)
            # reply at once; a 1 s drift watch runs in the background and only WARNS (never releases:
            # a falling arm is not helped by disabling it) - operator asked for a fast hold 2026-09-27
            threading.Thread(target=self._drift_watch, args=(target,), daemon=True).start()
            self.fault = ""
            res.success, res.message = True, "holding"
            return res
        finally:
            self.motion.release()

    def _drift_watch(self, target):
        worst = 0.0
        for _ in range(20):
            time.sleep(0.05)
            s = self.snap or {}
            if s.get("errors"):
                self.get_logger().error("hold: joint errors " + ",".join(s["errors"])); return
            if s.get("joints"):
                worst = max(worst, max(abs(a - b) for a, b in zip(s["joints"], target)))
        if worst > 0.1:
            self.get_logger().warning(f"hold: drift {worst:.3f} rad in the first second")

    def srv_set_joints(self, req, res):
        """Power (hold) or release individual joints. Releasing a joint lets gravity act on
        everything beyond it: the caller supports the arm. 7 = gripper."""
        js = [int(j) for j in req.joints] or [1, 2, 3, 4, 5, 6]
        bad = [j for j in js if j < 1 or j > 7]
        if bad:
            res.success, res.message = False, f"invalid joint numbers {bad} (1..6, 7 = gripper)"
            res.joints_enabled = self.snap.get("enabled", [False] * 6)
            return res
        if req.enable:
            why = self._refuse()
            if why and "fault latched" not in why:
                res.success, res.message = False, "refused: " + why
                res.joints_enabled = self.snap.get("enabled", [False] * 6)
                return res
            for jf in self.snap.get("joint_faults", []):
                if int(jf[1]) in js:
                    with self.sdk:
                        self.robot.clear_joint_error(int(jf[1]))
        self.cancel_requested = not req.enable
        with self.sdk:
            for j in js:
                if j == 7:
                    if self.ee is not None:
                        if req.enable:
                            self.ee.move_gripper(float(self.snap.get("grip", {}).get("width", 0.0)), float(self.P["gripper_force"]))
                        else:
                            self.ee.disable_gripper()
                elif req.enable:
                    self.robot.enable(j)
                else:
                    self.robot.disable(j)
        time.sleep(0.5)
        s = self._read()
        arm_js = [j for j in js if j <= 6]
        ok = all(s["enabled"][j - 1] == req.enable for j in arm_js)
        res.success = ok
        res.joints_enabled = s["enabled"]
        verb = "powered" if req.enable else "released"
        res.message = (f"joints {js} {verb}" if ok else f"joints {[j for j in arm_js if s['enabled'][j-1] != req.enable]} did not change") + \
                      f"; enabled now {s['enabled']}"
        if req.enable and ok and all(s["enabled"]):
            self.fault = ""
        return res

    def srv_set_speed(self, req, res):
        if not 1 <= req.percent <= 100:
            res.success, res.message = False, "percent must be 1..100"
            return res
        capped = self._clamp_speed(req.percent)
        self.speed = capped
        with self.sdk:
            self.robot.set_speed_percent(capped)
        res.success = True
        res.message = f"speed {capped} %" + (" (capped)" if capped != req.percent else "")
        return res

    def srv_gripper_release(self, req, res):
        """Un-power the GRIPPER motor only (jaws go free); the arm keeps holding."""
        if self.ee is None:
            res.success, res.message = False, "no gripper"
            return res
        with self.sdk:
            ok = self.ee.disable_gripper()
        time.sleep(0.3)
        g = self.snap.get("grip", {})
        res.success = True
        res.message = f"gripper motor released (SDK returned {ok}); width {g.get('width', float('nan')):.4f} m, driver enabled={g.get('enabled')}"
        return res

    def srv_gripper_enable(self, req, res):
        """Re-power the gripper motor (re-enables on the next move command; sends a hold at the current width)."""
        if self.ee is None:
            res.success, res.message = False, "no gripper"
            return res
        g = self.snap.get("grip", {})
        w = float(g.get("width", 0.0))
        with self.sdk:
            self.ee.move_gripper(w, float(self.P["gripper_force"]))
        time.sleep(0.3)
        g = self.snap.get("grip", {})
        res.success = True
        res.message = f"gripper powered at width {g.get('width', float('nan')):.4f} m, driver enabled={g.get('enabled')}"
        return res

    def srv_gripper(self, req, res):
        if self.ee is None:
            res.success, res.message = False, "no gripper"
            return res
        width = max(0.0, min(float(self.P["gripper_max_m"]), req.width_m))
        force = req.force if req.force > 0 else float(self.P["gripper_force"])
        timeout = req.timeout_s if req.timeout_s > 0 else float(self.P["gripper_timeout_s"])
        tol = float(self.P["gripper_tol_m"])
        with self.sdk:
            self.ee.move_gripper(self._grip_raw(width), force)
        t0 = time.time()
        last_w, last_change = None, time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            g = self.snap.get("grip")
            if g is None:
                continue
            w = g["width"]
            if abs(w - width) <= tol:
                res.success, res.stalled, res.width_m = True, False, w
                res.message = f"width {w:.4f} m"
                return res
            if last_w is None or abs(w - last_w) > 0.0005:
                last_w, last_change = w, time.time()
            elif time.time() - last_change > 0.6 and time.time() - t0 > 0.5:
                res.success, res.stalled, res.width_m = True, True, w
                res.message = f"stalled at {w:.4f} m (target {width:.4f})"
                return res
        w = self.snap.get("grip", {}).get("width", float("nan"))
        res.success, res.stalled, res.width_m = False, False, w
        res.message = f"timeout at {w:.4f} m"
        return res

    # ------------------------------------------------------------------ actions
    def goal_cb(self, goal):
        why = self._refuse()
        if why:
            self.get_logger().warning("goal rejected: " + why)
            return GoalResponse.REJECT
        if self.busy:
            self.get_logger().warning("goal rejected: busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal):
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def _stop_here(self):
        """Freeze the arm where it is: re-target the current joints."""
        s = self._read()
        if s["joints"] is not None and all(s["enabled"]):
            with self.sdk:
                self.robot.move_j(list(s["joints"]))

    def _run_motion(self, gh, send, done_fn, feedback_fn, timeout, max_jump=None, need=None):
        """Common execute loop. send(): issue the SDK command. done_fn(snap)->(done, err_value).
        feedback_fn(snap)->feedback msg. Returns (success, message)."""
        if not self.motion.acquire(blocking=False):
            return False, "another motion is running"
        self.busy = True
        self.cancel_requested = False
        try:
            s = self._read()
            if not all(s["enabled"]):
                return False, "arm is not enabled (call /arm/enable or /arm/hold first)"
            q_start = list(s["joints"]) if s["joints"] is not None else None
            with self.sdk:
                send()
            t0 = time.time()
            settled = 0
            bad = 0
            q_last, t_moved = (list(s["joints"]) if s["joints"] is not None else None), time.time()
            need = int(self.P["settle_samples"]) if need is None else int(need)
            while True:
                time.sleep(0.05)
                s = self._read()
                if self.cancel_requested or gh.is_cancel_requested:
                    self._stop_here()
                    return False, "cancelled"
                if s["errors"]:
                    return False, "errors: " + ",".join(s["errors"])
                bad = bad + 1 if s["arm_status"] != "NORMAL" else 0
                if bad >= 3:                      # controller refused the target (no IK solution, limit, singularity)
                    self._stop_here()
                    return False, f"controller reports {s['arm_status']}"
                if s["age"] >= float(self.P["watchdog_s"]):
                    return False, "feedback lost"
                if max_jump is not None and q_start is not None:
                    # A Cartesian target lets the controller pick any IK branch; near a wrist limit it
                    # silently flips j4/j5/j6 by ~180 deg (2026-09-18). Freeze as soon as a joint runs away.
                    dq = [abs(a - b) for a, b in zip(s["joints"], q_start)]
                    k = max(range(6), key=lambda i: dq[i])
                    if dq[k] > max_jump:
                        self._stop_here()
                        return False, (f"joint-jump guard: joint {k + 1} moved {dq[k]:.2f} rad "
                                       f"(> {max_jump:.2f}) during a Cartesian move - wrist flip / other IK branch; stopped")
                done, err = done_fn(s)
                gh.publish_feedback(feedback_fn(s))
                settled = settled + 1 if done else 0
                # stall: joints stopped moving but the target is not reached -> the controller has
                # clamped a joint at ITS limit (it does so silently, e.g. j4 at 100 deg). Fail fast.
                if q_last is not None and s["joints"] is not None:
                    if max(abs(a - b) for a, b in zip(s["joints"], q_last)) > 0.002:
                        q_last, t_moved = list(s["joints"]), time.time()
                    elif not done and time.time() - t_moved > float(self.P["stall_s"]):
                        self._stop_here()
                        return False, (f"stalled short of the target for {self.P['stall_s']:.1f} s (residual {err:.4f}): "
                                       f"a joint is clamped at a controller limit or the target is unreachable")
                if settled >= need:
                    return True, f"reached (residual {err:.4f})"
                if time.time() - t0 > timeout:
                    self._stop_here()
                    return False, f"timeout after {timeout:.0f} s (residual {err:.4f})"
        finally:
            self.busy = False
            self.motion.release()

    def exec_move_j(self, gh):
        g = gh.request
        target = [float(x) for x in g.joints_rad]
        speed = self._clamp_speed(g.speed_percent)
        timeout = g.timeout_s if g.timeout_s > 0 else float(self.P["move_timeout_s"])
        tol = float(self.P["joint_tol_rad"])
        cur = self.snap.get("joints") or self._read()["joints"]
        jump = max(abs(a - b) for a, b in zip(cur, target))
        if jump > float(self.P["max_joint_delta_rad"]):
            res = MoveJ.Result()
            res.success = False
            res.message = (f"refused: joint jump {jump:.2f} rad exceeds max_joint_delta_rad "
                           f"{self.P['max_joint_delta_rad']}; split the move into via points")
            res.final_joints_rad = [float(x) for x in cur]
            gh.abort()
            return res

        def send():
            self.robot.set_speed_percent(speed)
            self.speed = speed
            self.robot.move_j(target)

        def done(s):
            e = max(abs(a - b) for a, b in zip(s["joints"], target))
            return e <= tol, e

        def fb(s):
            f = MoveJ.Feedback()
            f.max_joint_error_rad = done(s)[1]
            return f

        ok, msg = self._run_motion(gh, send, done, fb, timeout)
        res = MoveJ.Result()
        res.success, res.message = ok, msg
        res.final_joints_rad = [float(x) for x in (self.snap.get("joints") or [0.0] * 6)]
        (gh.succeed if ok else gh.abort)()
        return res

    def exec_move_pose(self, gh):
        """Cartesian goal in the CORRECTED model: solve IK here (seeded from the current joints,
        so the wrist/elbow branch cannot change), then drive the controller with joint targets.
        linear=True: straight line as a chain of short joint moves through IK'd waypoints."""
        g = gh.request
        pose = [float(x) for x in g.pose]
        speed = self._clamp_speed(g.speed_percent)
        timeout = g.timeout_s if g.timeout_s > 0 else float(self.P["move_timeout_s"])
        ptol, atol = float(self.P["pos_tol_m"]), float(self.P["ang_tol_rad"])
        jtol = float(self.P["joint_tol_rad"])
        max_jump = float(self.P["max_joint_delta_rad"])
        res = MovePose.Result()

        def fail(msg):
            res.success, res.message = False, msg
            res.final_pose = [float(x) for x in (self.snap.get("tcp") or [0.0] * 6)]
            gh.abort()
            return res

        if not self._in_workspace(pose[:3]):
            return fail("target outside workspace box")
        fv = self._floor_violation(pose)
        if fv:
            return fail("refused: " + fv)
        cur = self.snap.get("joints") or self._read()["joints"]
        if cur is None:
            return fail("no joint feedback")
        T_goal = T_from_xyzrpy(pose)
        T_now = self.kin.fk(cur)
        if g.linear:
            wps = self.kin.interpolate(T_now, T_goal, float(self.P["linear_step_m"]), float(self.P["linear_step_rad"]))
        else:
            wps = [T_goal]

        # solve the whole chain first; refuse before moving if any waypoint is unreachable
        margin = float(self.P["ik_limit_margin_rad"])
        targets, seed = [], list(cur)
        for k, T in enumerate(wps):
            q, pe, ae = self.kin.ik(T, seed, margin=margin)
            if pe > 1e-3 or ae > 5e-3:
                return fail(f"refused: no IK solution in the current branch for waypoint {k + 1}/{len(wps)} "
                            f"(best {pe*1000:.1f} mm / {math.degrees(ae):.2f} deg); joints near a limit or pose out of reach")
            jump = max(abs(a - b) for a, b in zip(seed, q))
            if jump > max_jump:
                if g.linear or len(wps) > 1:
                    return fail(f"refused: waypoint {k + 1} needs a joint jump of {jump:.2f} rad (> max_joint_delta_rad {max_jump}); "
                                f"split the move")
                # point-to-point: a long but legitimate move. Split it into joint-space via points
                # (what the controller's own move_p does internally), each checked against floor/workspace.
                cap = float(self.P["pose_total_jump_rad"])
                if jump > cap:
                    return fail(f"refused: target needs a joint jump of {jump:.2f} rad (> pose_total_jump_rad {cap}); "
                                f"is this the pose you meant?")
                n = int(math.ceil(jump / (0.8 * max_jump)))
                qs, qe = np.array(seed), np.array(q)
                for i in range(1, n):
                    qi = qs + (qe - qs) * i / n
                    wp_pose = self.kin.fk_xyzrpy(qi)
                    fv = self._floor_violation(wp_pose)
                    if fv or not self._in_workspace(wp_pose[:3]):
                        return fail(f"refused: via point {i}/{n} of the split move " + (fv or "outside workspace box"))
                    targets.append([float(v) for v in qi])
            if g.linear and k < len(wps) - 1:
                wp_pose = xyzrpy_from_T(T)
                fv = self._floor_violation(wp_pose)
                if fv or not self._in_workspace(wp_pose[:3]):
                    return fail(f"refused: waypoint {k + 1} " + (fv or "outside workspace box"))
            targets.append([float(v) for v in q])
            seed = list(q)

        def errs(s):
            return pose_error(self.kin.fk(s["joints"]), T_goal)

        ok, msg = True, ""
        need_final = int(self.P["settle_samples"])
        for k, target in enumerate(targets):
            last = k == len(targets) - 1

            def send(target=target):
                self.robot.set_speed_percent(speed)
                self.speed = speed
                self.robot.move_j(target)

            def done(s, target=target, last=last):
                je = max(abs(a - b) for a, b in zip(s["joints"], target))
                pe, ae = errs(s)
                if last:
                    return (je <= jtol and pe <= ptol and ae <= atol), pe + ae
                return je <= 3 * jtol, pe + ae      # intermediate waypoint: pass through

            def fb(s):
                f = MovePose.Feedback()
                f.position_error_m, f.orientation_error_rad = errs(s)
                return f

            ok, msg = self._run_motion(gh, send, done, fb, timeout,
                                       max_jump=float(self.P["pose_max_joint_delta_rad"]),
                                       need=need_final if last else 1)
            if not ok:
                msg = f"waypoint {k + 1}/{len(targets)}: {msg}"
                break
        res.success, res.message = ok, msg
        res.final_pose = [float(x) for x in (self.snap.get("tcp") or [0.0] * 6)]
        (gh.succeed if ok else gh.abort)()
        return res

    def shutdown(self):
        try:
            self.robot.disconnect()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = PiperNode()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
