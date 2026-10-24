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
  /arm/gripper_enable   std_srvs/Trigger   re-power the gripper where the jaws are now, at gripper_hold_force
  /arm/teach_exit std_srvs/Trigger   take the controller out of teach (drag) mode: end drag teaching, else the
                  controller's resume (2026-10-24); the joints stay as the controller leaves them (Hold powers them)
  /arm/teach_enter std_srvs/Trigger  release the arm into the controller's drag-teach mode (the twin's Release,
                  operator 2026-10-24; was /arm/disable); /arm/hold takes it out and holds
  /arm/gripper_calibrate harness_msgs/GripperCalibrate   two-point calibration of the mm reading (jaws set by
                  hand, motor released): saved to gripper_calib_file, loaded at start (PROJECT_SPEC.md
                  "2026-10-24 — Gripper release, hold and calibration")
Actions:
  /arm/move_j     harness_msgs/MoveJ      a smooth joint move (minimum-jerk, streamed; 2026-10-24)
  /arm/move_pose  harness_msgs/MovePose   IK here; linear=false: a smooth joint move to it, true: the flange on a
                  straight line with a smooth speed profile (streamed; 2026-10-24 — was 20 mm stop-and-go hops)
  /arm/follow_trajectory  harness_msgs/FollowTrajectory   stream a validated joint trajectory
                  (move_j re-targeted at traj_rate_hz; PROJECT_SPEC.md "2026-10-22 — Planner bedrock")

Rules: enable state comes from joint feedback, never from an SDK return value.
Lost feedback or any error bit cancels the running goal and latches a fault; the
node NEVER disables the arm on its own (it keeps holding its last target; INCIDENT
2026-09-17 in PROJECT_SPEC.md). One motion goal at a time.
"""
import math
import os
import threading
import time
import warnings

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

from harness_msgs.action import FollowTrajectory, MoveJ, MovePose
from harness_arm.kinematics import PiperKinematics, T_from_xyzrpy, xyzrpy_from_T, pose_error
from harness_msgs.msg import ArmStatus, GripperStatus
from harness_msgs.srv import GripperCalibrate, SetGripper, SetJoints, SetSpeed

from pyAgxArm import AgxArmFactory, PiperFW, create_agx_arm_config
from scipy.spatial.transform import Rotation as _Rot

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


# teach status values that are a running teach record / replay (the controller's teach_status byte). "End teaching
# record" (STOP_RECORDING) is what "end drag teaching" leaves behind, not a mode: reading it as teach mode made Hold
# fire the controller's resume, which unpowered the arm (2026-10-24)
TEACH_ACTIVE = ("START_RECORDING", "EXECUTE_TRAJECTORY", "PAUSE_EXECUTION", "RESUME_EXECUTION", "MOVE_TO_START")


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


def _min_jerk(u):
    """0..1 -> 0..1 with zero speed AND zero acceleration at both ends: the smooth start and stop."""
    u = min(1.0, max(0.0, float(u)))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _min_jerk_time(dist, vmax, amax, t_min=0.5):
    """Duration of a minimum-jerk move over `dist` whose peak speed (1.875 dist / T) and peak acceleration
    (5.77 dist / T²) stay within vmax / amax."""
    if dist <= 0.0:
        return t_min
    return max(t_min, 1.875 * dist / vmax, math.sqrt(5.7735 * dist / amax))


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
            ("gripper_zero_m", 0.0), ("gripper_hold_force", 5.0), ("gripper_calib_file", ""),
            ("max_joint_delta_rad", 1.2), ("pose_max_joint_delta_rad", 1.2),
            ("floor_enabled", False), ("floor_plane", [0.0, 0.0, 0.0]), ("tcp_offset_m", 0.0), ("floor_margin_m", 0.01),
            ("kinematics_file", ""), ("kinematics_model", "piper"), ("ik_limit_margin_rad", 0.02),
            ("linear_step_m", 0.02), ("linear_step_rad", 0.1), ("stall_s", 2.0), ("pose_total_jump_rad", 2.5),
            ("traj_rate_hz", 100.0), ("traj_max_joint_vel_rad_s", 0.6), ("traj_tracking_error_rad", 0.15),
            ("traj_start_tol_rad", 0.05), ("traj_max_points", 20000),
            ("smooth_joint_vel_rad_s", 3.0), ("smooth_tool_vel_m_s", 0.5), ("smooth_tool_vel_rad_s", 3.0),
            ("smooth_acc_per_s", 2.0), ("smooth_knot_s", 0.04), ("smooth_knot_jump_rad", 0.15),
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
        self.grip_cal_closed = None      # raw reading recorded with the jaws shut (gripper calibration, step 1)

        self._connect()
        self._load_kinematics()
        self._load_gripper_calib()

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/arm/tcp_pose", 10)          # corrected model (harness FK)
        self.pub_tcp_raw = self.create_publisher(PoseStamped, "/arm/tcp_pose_raw", 10)  # controller's own report
        self.pub_status = self.create_publisher(ArmStatus, "/arm/status", 10)
        self.pub_grip = self.create_publisher(GripperStatus, "/arm/gripper_status", 10)

        for name, cb in (("enable", self.srv_enable), ("disable", self.srv_disable),
                         ("hold", self.srv_hold), ("gripper_release", self.srv_gripper_release),
                         ("gripper_enable", self.srv_gripper_enable), ("teach_exit", self.srv_teach_exit),
                         ("teach_enter", self.srv_teach_enter)):
            self.create_service(Trigger, f"/arm/{name}", cb, callback_group=self.cb)
        self.create_service(SetSpeed, "/arm/set_speed", self.srv_set_speed, callback_group=self.cb)
        self.create_service(SetJoints, "/arm/set_joints", self.srv_set_joints, callback_group=self.cb)
        self.create_service(SetGripper, "/arm/gripper", self.srv_gripper, callback_group=self.cb)
        self.create_service(GripperCalibrate, "/arm/gripper_calibrate", self.srv_gripper_calibrate, callback_group=self.cb)

        self.as_j = ActionServer(self, MoveJ, "/arm/move_j", self.exec_move_j,
                                 goal_callback=self.goal_cb, cancel_callback=self.cancel_cb,
                                 callback_group=self.cb)
        self.as_p = ActionServer(self, MovePose, "/arm/move_pose", self.exec_move_pose,
                                 goal_callback=self.goal_cb, cancel_callback=self.cancel_cb,
                                 callback_group=self.cb)
        self.as_t = ActionServer(self, FollowTrajectory, "/arm/follow_trajectory", self.exec_follow_trajectory,
                                 goal_callback=self.goal_cb, cancel_callback=self.cancel_cb,
                                 callback_group=self.cb)

        self.create_timer(1.0 / float(self.P["rate_hz"]), self._tick, callback_group=self.cb)
        self.get_logger().info(f"piper node up on {self.P['can_channel']}, speed cap "
                               f"{self.P['max_speed_percent']} %, gripper="
                               f"{'yes' if self.ee else 'no'}")

    # ------------------------------------------------------------------ gripper mapping
    # Fingers mounted the other way round (2026-09-18): the motor's "open" is the jaws' closed.
    # inverted:  physical gap = (travel - raw) * scale;  raw = travel - gap / scale
    # otherwise: physical gap = (raw - zero) * scale;    raw = zero + gap / scale   (zero 0, scale 1 = the SDK's own)
    def _grip_phys(self, raw):
        k = float(self.P["gripper_scale"])
        if not self.P["gripper_inverted"]:
            return max(0.0, (raw - float(self.P["gripper_zero_m"])) * k)
        return max(0.0, (float(self.P["gripper_travel_m"]) - raw) * k)

    def _grip_raw(self, gap):
        k = float(self.P["gripper_scale"])
        if not self.P["gripper_inverted"]:
            return max(0.0, float(self.P["gripper_zero_m"]) + gap / k)
        return max(0.0, float(self.P["gripper_travel_m"]) - gap / k)

    def _load_gripper_calib(self):
        """The gripper's mm mapping from gripper_calib_file (written by /arm/gripper_calibrate), over piper.yaml."""
        path = os.path.expanduser(str(self.P["gripper_calib_file"] or ""))
        if not path or not os.path.exists(path):
            return
        try:
            with open(path) as f:
                d = yaml.safe_load(f) or {}
            if bool(d["inverted"]) != bool(self.P["gripper_inverted"]):
                raise ValueError("it was made with gripper_inverted=%s, piper.yaml says %s: calibrate again"
                                 % (d["inverted"], self.P["gripper_inverted"]))
            self.P["gripper_travel_m" if self.P["gripper_inverted"] else "gripper_zero_m"] = float(d["raw_closed_m"])
            self.P["gripper_scale"] = float(d["scale"])
            self.P["gripper_max_m"] = float(d["max_gap_m"])
            self.get_logger().info(f"gripper: mm mapping from {path} ({d.get('date')}): gap 0 at raw {d['raw_closed_m']:.4f} m, "
                                   f"scale {d['scale']:.4f}, max gap {d['max_gap_m'] * 1000:.1f} mm")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"gripper: cannot use {path} ({e}); keeping piper.yaml's mapping")

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
            "teach_status": _name(st.msg.teach_status) if st else "UNKNOWN",
            # the controller's own teach (drag) mode — it then ignores every command — or a teach record / replay
            "drag": _name(st.msg.ctrl_mode) == "TEACHING_MODE" if st else False,
            "teach": (_name(st.msg.ctrl_mode) == "TEACHING_MODE" or _name(st.msg.teach_status) in TEACH_ACTIVE) if st else False,
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
        s = self._read()
        if s["teach"] and s["age"] < float(self.P["watchdog_s"]):
            return self._hold_from_teach(res)
        why = self._refuse()
        # a fault latched by error bits that are gone now (e.g. a moment's joint communication loss when the controller
        # left teach mode, 2026-10-24) does not block Hold — holding is the safe answer; it clears it, like /arm/enable
        if why and "fault latched" not in why:
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

    def _leave_teach(self, wait_s, resume=True):
        """Out of the controller's teach (drag) mode: "end drag teaching" (0x150 byte 2 = 0x02); if it still reports
        teach mode after wait_s and `resume`, the controller's resume (0x150 byte 0 = 0x02, the SDK's reset — it
        UNPOWERS every joint: never from Hold). Returns what was sent."""
        from pyAgxArm.protocols.can_protocol.msgs.piper.default import ArmMsgMotionCtrl
        tried = []
        steps = [("end drag teaching", lambda: self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=0x02)))]
        if resume:
            steps.append(("controller resume", lambda: self.robot.reset()))
        for name, send in steps:
            with self.sdk:
                send()
            tried.append(name)
            t_end = time.time() + wait_s
            while time.time() < t_end and self._read()["teach"]:
                time.sleep(0.01)
            if not self._read()["teach"]:
                break
        return tried

    def srv_teach_exit(self, req, res):
        """Take the controller out of teach (drag) mode (2026-10-24: found in TEACHING_MODE, every command refused,
        no button found): _leave_teach. Nothing is enabled or moved here: the joints stay as the controller leaves
        them (support the arm); /arm/hold powers them."""
        if not self._read()["teach"]:
            res.success, res.message = True, "not in teach mode"
            return res
        tried = self._leave_teach(2.0)
        s = self._read()
        res.success = not s["teach"]
        res.message = (f"{'left' if res.success else 'STILL in'} teach mode after: {', '.join(tried)}; control mode "
                       f"{s['ctrl_mode']}, joints enabled {s['enabled']}" + ("" if res.success else
                       " — try the teach button on the arm or power-cycle the arm"))
        self.get_logger().warning("teach_exit: " + res.message)
        return res

    def srv_teach_enter(self, req, res):
        """Release into the controller's drag-teach mode (operator 2026-10-24: the twin's Release — it was
        /arm/disable, joints limp): "start drag teaching" (0x150 byte 2 = 0x01), the arm is moved by hand;
        /arm/hold takes it out again. A running goal is stopped first. If the controller does not report teach
        mode within 2 s nothing else is sent: the arm stays as it was (never disabled instead)."""
        from pyAgxArm.protocols.can_protocol.msgs.piper.default import ArmMsgMotionCtrl
        s = self._read()
        if s["drag"]:
            res.success, res.message = True, "already in drag mode"
            return res
        if s["age"] >= float(self.P["watchdog_s"]):
            res.success, res.message = False, "refused: no CAN feedback"
            return res
        self.cancel_requested = True
        if not self.motion.acquire(timeout=2.0):
            res.success, res.message = False, "refused: the running motion did not stop"
            return res
        try:
            with self.sdk:
                self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=0x01))
            t_end = time.time() + 2.0
            while time.time() < t_end and not self._read()["drag"]:
                time.sleep(0.02)
            s = self._read()
            cleared = False
            if not s["drag"] and s["teach"]:
                # 2026-10-24 on the arm: "start drag teaching" from the PC only starts the teach RECORD (control mode
                # CAN_CTRL, joints still powered and holding) — it is not the drag mode: end that record again
                with self.sdk:
                    self.robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=0x02))
                cleared = True
                time.sleep(0.2)
        finally:
            self.motion.release()
        s = self._read()
        res.success = s["drag"]
        res.message = (f"drag mode: move the arm by hand (control mode {s['ctrl_mode']}, arm {s['arm_status']}, "
                       f"joints enabled {s['enabled']})" if res.success else
                       "the controller does not go into drag mode from the PC" +
                       (": it only started its teach recording (ended again)" if cleared else "") +
                       f" — control mode {s['ctrl_mode']}, joints enabled {s['enabled']}; the arm is as it was")
        self.get_logger().warning("teach_enter: " + res.message)
        return res

    def _hold_from_teach(self, res):
        """Hold from teach mode (or a teach record): "end drag teaching"; if the controller is still in teach mode and
        every joint is ALREADY unpowered — the arm's own drag-teach mode, the button at the wrist between J5 and J6 —
        the controller's resume, the only exit that works for it (2026-10-24): it unpowers every joint, which changes
        nothing then. Never the resume with a joint powered (2026-10-24: it dropped the arm). Then at once power
        joints 1-6 and hold where the arm is, with no error-bit check in between (an arm left unpowered falls)."""
        if not self.motion.acquire(blocking=False):
            res.success, res.message = False, "refused: a motion goal is running"
            return res
        try:
            tried = self._leave_teach(1.0, resume=False)     # the controller's resume only below: it unpowers the arm
            s = self._read()
            if s["teach"] and not any(s["enabled"]):
                with self.sdk:
                    self.robot.reset()
                tried.append("controller resume (every joint was unpowered already)")
                t_end = time.time() + 2.0
                while time.time() < t_end and self._read()["teach"]:
                    time.sleep(0.01)
                s = self._read()
            if s["teach"]:
                res.success, res.message = False, (f"the controller stays in teach mode (control mode {s['ctrl_mode']}, "
                                                   f"teach {s['teach_status']}, joints enabled {s['enabled']}) after: "
                                                   f"{', '.join(tried)}; nothing else sent — click the teach button at the "
                                                   "wrist (between J5 and J6) so its light goes off, then Hold")
                return res
            with self.sdk:
                self.robot.set_speed_percent(self.speed)
                self._enable_joints()
            if not self._wait_enabled(True):
                res.success, res.message = False, f"left drag mode ({', '.join(tried)}) but the joints did not enable: support the arm"
                return res
            time.sleep(0.1)
            target = list(self._read()["joints"])   # capture AFTER enable: no sag counted as drift
            with self.sdk:
                self.robot.move_j(target)
            t_end = time.time() + 1.0                # the moment's error bits, if any, clear before the fault does
            while time.time() < t_end and self._read()["errors"]:
                time.sleep(0.05)
            errs = self._read()["errors"]
            if not errs:
                self.fault = ""
            threading.Thread(target=self._drift_watch, args=(target,), daemon=True).start()
            res.success = True
            res.message = f"holding (left drag mode: {', '.join(tried)})" + (f"; error bits still set: {','.join(errs)}" if errs else "")
            self.get_logger().warning("hold: " + res.message)
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
                            try:
                                self._grip_hold_here()
                            except RuntimeError as e:
                                self.get_logger().error(f"set_joints: gripper not powered ({e})")
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

    def _grip_hold_here(self):
        """Power the gripper where the jaws are now, at gripper_hold_force (caller holds self.sdk). The target is
        the RAW reading: sending the physical gap as a motor target moved the reversed jaws (2026-10-24)."""
        g = self.snap.get("grip")
        if g is None:
            raise RuntimeError("no gripper feedback")
        self.ee.move_gripper(float(g["width_raw"]), float(self.P["gripper_hold_force"]))

    def srv_gripper_enable(self, req, res):
        """Re-power the gripper motor where the jaws are now (an item between them stays held)."""
        if self.ee is None:
            res.success, res.message = False, "no gripper"
            return res
        try:
            with self.sdk:
                self._grip_hold_here()
        except RuntimeError as e:
            res.success, res.message = False, str(e)
            return res
        time.sleep(0.3)
        g = self.snap.get("grip", {})
        res.success = True
        res.message = (f"gripper holding at {g.get('width', float('nan')) * 1000:.1f} mm, {float(self.P['gripper_hold_force']):g} N, "
                       f"driver enabled={g.get('enabled')}")
        return res

    def _raw_now(self, n=10, dt=0.03):
        """The gripper's raw reading averaged over n feedback samples (the jaws held still by hand)."""
        vals = []
        for _ in range(n):
            g = self.snap.get("grip")
            if g is not None:
                vals.append(float(g["width_raw"]))
            time.sleep(dt)
        if not vals:
            raise RuntimeError("no gripper feedback")
        if max(vals) - min(vals) > 0.0005:
            raise RuntimeError("the reading is not still (%.1f mm spread): hold the jaws still" % ((max(vals) - min(vals)) * 1000))
        return sum(vals) / len(vals)

    def srv_gripper_calibrate(self, req, res):
        """Two-point calibration of the mm reading: "closed" (jaws pushed shut by hand = gap 0), then "open" at a
        caliper-measured gap; the new mapping is applied at once and saved (GripperCalibrate.srv)."""
        res.success = False
        if self.ee is None:
            res.message = "no gripper"
            return res
        step = str(req.step).strip().lower()
        if step == "cancel":
            self.grip_cal_closed = None
            res.success, res.message = True, "calibration points forgotten"
            return res
        if step not in ("closed", "open"):
            res.message = "step must be closed | open | cancel"
            return res
        g = self.snap.get("grip")
        if g is None:
            res.message = "no gripper feedback"
            return res
        if g.get("enabled"):
            res.message = "release the gripper first (Release gripper), then set the jaws by hand"
            return res
        try:
            raw = self._raw_now()
        except RuntimeError as e:
            res.message = str(e)
            return res
        res.raw_m = raw
        if step == "closed":
            self.grip_cal_closed = raw
            res.success, res.message = True, f"shut recorded (raw {raw * 1000:.2f} mm): now open the jaws, measure the gap"
            return res
        gap = float(req.gap_m)
        c = self.grip_cal_closed
        if c is None:
            res.message = "record the shut jaws first (step closed)"
            return res
        if not 0.005 <= gap <= 0.2:
            res.message = f"the measured gap {gap * 1000:.1f} mm is not plausible (5-200 mm)"
            return res
        inv = bool(self.P["gripper_inverted"])
        d = (c - raw) if inv else (raw - c)
        if d < 0.003:
            res.message = (f"the reading moved {d * 1000:+.1f} mm from shut to open: "
                           + ("too little - open the jaws wider" if d >= 0 else f"the wrong way for gripper_inverted={inv}"))
            return res
        scale = gap / d
        if not 0.5 <= scale <= 2.0:
            res.message = f"the reading and the gap disagree by x{scale:.2f}: measure again (shut, then open)"
            return res
        stroke_raw = self._grip_raw(float(self.P["gripper_max_m"]))      # the motor's stroke end, in the old mapping
        self.P["gripper_travel_m" if inv else "gripper_zero_m"] = c
        self.P["gripper_scale"] = scale
        self.P["gripper_max_m"] = round(self._grip_phys(stroke_raw), 5)
        self.grip_cal_closed = None
        res.travel_m, res.scale, res.max_gap_m = c, scale, float(self.P["gripper_max_m"])
        doc = {"date": time.strftime("%Y-%m-%dT%H:%M:%S"), "inverted": inv, "raw_closed_m": round(c, 6),
               "scale": round(scale, 6), "max_gap_m": float(self.P["gripper_max_m"]),
               "points": {"closed_raw_m": round(c, 6), "open_raw_m": round(raw, 6), "open_gap_m": round(gap, 6)},
               "note": "two-point calibration, jaws set by hand (motor released), open gap by calipers"}
        path = os.path.expanduser(str(self.P["gripper_calib_file"] or ""))
        saved = ""
        if path:
            try:
                if os.path.exists(path):
                    bdir = os.path.join(os.path.dirname(path), "gripper_backups")
                    os.makedirs(bdir, exist_ok=True)
                    os.replace(path, os.path.join(bdir, "gripper_%s.yaml" % time.strftime("%Y-%m-%dT%H%M%S")))
                with open(path + ".tmp", "w") as f:
                    yaml.safe_dump(doc, f, sort_keys=False)
                os.replace(path + ".tmp", path)
                saved = f"; saved to {path}"
            except OSError as e:
                saved = f"; NOT saved ({e}) - applied until the node restarts"
        else:
            saved = "; no gripper_calib_file set - applied until the node restarts"
        res.success = True
        res.message = f"calibrated: scale {scale:.3f}, max gap {self.P['gripper_max_m'] * 1000:.1f} mm{saved}"
        self.get_logger().info("gripper: " + res.message)
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

    # ------------------------------------------------------------------ smooth moves (2026-10-24)
    # Every motion is a timed trajectory streamed at traj_rate_hz (_stream): minimum-jerk profiles, so the arm
    # starts and stops with zero speed AND zero acceleration — no rush, no stop-and-go (operator 2026-10-24:
    # "as smooth as possible, with smooth acceleration control"). Before, move_j handed the whole move to the
    # controller's own profile and a linear move_pose was a chain of 20 mm hops, each stopping.
    def _smooth_caps(self, speed):
        """speed_percent -> (joint rad/s, rad/s², tool m/s, m/s², tool rad/s, rad/s²): the profile's peaks."""
        f = max(1, min(100, int(speed))) / 100.0
        k = float(self.P["smooth_acc_per_s"])
        vj = min(float(self.P["traj_max_joint_vel_rad_s"]), f * float(self.P["smooth_joint_vel_rad_s"]))
        vl = f * float(self.P["smooth_tool_vel_m_s"])
        va = f * float(self.P["smooth_tool_vel_rad_s"])
        return vj, k * vj, vl, k * vl, va, k * va

    def _joint_line(self, q0, q1, speed):
        """A straight joint move as (t, q): minimum-jerk, its peak joint speed / acceleration within the caps."""
        vj, aj = self._smooth_caps(speed)[:2]
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        T = _min_jerk_time(float(np.abs(q1 - q0).max()), vj, aj)
        t = np.linspace(0.0, T, max(2, int(math.ceil(T * float(self.P["traj_rate_hz"]))) + 1))
        return t, q0 + np.outer([_min_jerk(x / T) for x in t], q1 - q0)

    def _tool_line(self, q0, T0, T1, speed):
        """A straight flange line T0 -> T1 (orientation slerped) as (t, q): minimum-jerk ALONG the line (tool speed
        and acceleration within the caps), IK at knots every smooth_knot_s seeded from the previous one (same
        wrist / elbow branch), joints interpolated at the stream rate; stretched in time until every joint stays
        within its speed cap. Raises ValueError (words) when the line cannot be followed in this branch."""
        vj, aj, vl, al, va, aa = self._smooth_caps(speed)
        pe, ae = pose_error(T0, T1)
        T = max(_min_jerk_time(pe, vl, al), _min_jerk_time(ae, va, aa))
        dt_k, margin, jump = float(self.P["smooth_knot_s"]), float(self.P["ik_limit_margin_rad"]), float(self.P["smooth_knot_jump_rad"])
        for _ in range(5):
            tk = np.linspace(0.0, T, max(2, int(math.ceil(T / dt_k)) + 1))
            qk, seed = [np.asarray(q0, float)], np.asarray(q0, float)
            for ti in tk[1:]:
                f = _min_jerk(ti / T)
                q, perr, aerr = self.kin.ik(self.kin.pose_at(T0, T1, f), seed, margin=margin)
                if perr > 1e-3 or aerr > 5e-3:
                    raise ValueError(f"no IK solution in the current branch {100 * f:.0f} % along the line "
                                     f"(best {perr * 1000:.1f} mm / {math.degrees(aerr):.2f} deg); joints near a limit or out of reach")
                if float(np.abs(q - seed).max()) > jump:
                    raise ValueError(f"{100 * f:.0f} % along the line a joint would jump "
                                     f"{math.degrees(float(np.abs(q - seed).max())):.0f} deg (wrist flip / another branch)")
                qk.append(q)
                seed = q
            qk = np.array(qk)
            vmax = float((np.abs(np.diff(qk, axis=0)) / np.diff(tk)[:, None]).max())
            if vmax <= vj * 1.001:
                break
            T *= vmax / vj * 1.05                               # a joint too fast: the same line, slower
        t = np.linspace(0.0, T, max(2, int(math.ceil(T * float(self.P["traj_rate_hz"]))) + 1))
        return t, np.column_stack([np.interp(t, tk, qk[:, j]) for j in range(6)])

    def exec_move_j(self, gh):
        g = gh.request
        target = [float(x) for x in g.joints_rad]
        speed = self._clamp_speed(g.speed_percent)
        res = MoveJ.Result()

        def finish(ok, msg):
            res.success, res.message = ok, msg
            res.final_joints_rad = [float(x) for x in (self.snap.get("joints") or [0.0] * 6)]
            (gh.succeed if ok else gh.abort)()
            return res
        cur = self.snap.get("joints") or self._read()["joints"]
        if cur is None:
            return finish(False, "no joint feedback")
        jump = max(abs(a - b) for a, b in zip(cur, target))
        if jump > float(self.P["max_joint_delta_rad"]):
            return finish(False, f"refused: joint jump {jump:.2f} rad exceeds max_joint_delta_rad "
                                 f"{self.P['max_joint_delta_rad']}; split the move into via points")
        t, q = self._joint_line(cur, target, speed)
        why = self._check_trajectory(t, q, geometry=False)     # joint moves: limits + speed (not the floor, as before)
        if why:
            return finish(False, "refused: " + why)

        def fb(tt, worst, s):
            f = MoveJ.Feedback()
            f.max_joint_error_rad = max(abs(a - b) for a, b in zip(s["joints"], target)) if s["joints"] is not None else 0.0
            return f
        ok, msg, _, _ = self._stream(gh, t, q, float(self.P["traj_tracking_error_rad"]), "move_j", feedback=fb)
        return finish(ok, msg)

    def exec_move_pose(self, gh):
        """Cartesian goal in the CORRECTED model: IK here (seeded from the current joints, so the wrist / elbow
        branch cannot change). linear=False: one smooth joint move to the IK solution; linear=True: the flange on
        a straight line with a smooth speed profile (_tool_line). Both streamed (_stream)."""
        g = gh.request
        pose = [float(x) for x in g.pose]
        speed = self._clamp_speed(g.speed_percent)
        res = MovePose.Result()

        def finish(ok, msg):
            res.success, res.message = ok, msg
            res.final_pose = [float(x) for x in (self.snap.get("tcp") or [0.0] * 6)]
            (gh.succeed if ok else gh.abort)()
            return res
        if not self._in_workspace(pose[:3]):
            return finish(False, "target outside workspace box")
        fv = self._floor_violation(pose)
        if fv:
            return finish(False, "refused: " + fv)
        cur = self.snap.get("joints") or self._read()["joints"]
        if cur is None:
            return finish(False, "no joint feedback")
        T_goal = T_from_xyzrpy(pose)
        T_now = self.kin.fk(cur)
        try:
            if g.linear:
                t, q = self._tool_line(cur, T_now, T_goal, speed)
            else:
                qg, pe, ae = self.kin.ik(T_goal, list(cur), margin=float(self.P["ik_limit_margin_rad"]))
                if pe > 1e-3 or ae > 5e-3:
                    return finish(False, f"refused: no IK solution in the current branch (best {pe * 1000:.1f} mm / "
                                         f"{math.degrees(ae):.2f} deg); joints near a limit or pose out of reach")
                jump = float(np.abs(np.asarray(qg) - np.asarray(cur)).max())
                if jump > float(self.P["pose_total_jump_rad"]):
                    return finish(False, f"refused: target needs a joint jump of {jump:.2f} rad (> pose_total_jump_rad "
                                         f"{self.P['pose_total_jump_rad']}); is this the pose you meant?")
                t, q = self._joint_line(cur, qg, speed)
        except ValueError as e:
            return finish(False, f"refused: {e}")
        why = self._check_trajectory(t, q)                     # limits, speed, workspace box, floor: every sample
        if why:
            return finish(False, "refused: " + why)

        def fb(tt, worst, s):
            f = MovePose.Feedback()
            if s["joints"] is not None:
                f.position_error_m, f.orientation_error_rad = pose_error(self.kin.fk(s["joints"]), T_goal)
            return f
        ok, msg, _, _ = self._stream(gh, t, q, float(self.P["traj_tracking_error_rad"]),
                                     "move_pose" + (" linear" if g.linear else ""), feedback=fb)
        return finish(ok, msg)

    # ------------------------------------------------------------------ trajectories
    def _check_trajectory(self, t, q, geometry=True):
        """Reason a joint trajectory must be refused, or None: every point within the joint limits,
        the workspace box and the floor margin (geometry=False: limits and speed only — move_j, whose
        targets were never floor-filtered); joint speed between points within
        traj_max_joint_vel_rad_s (+10 %). The planner checked the same; this node is the last gate."""
        m = float(self.P["ik_limit_margin_rad"]) * 0.5
        lo = np.array([l[0] + m for l in self.kin.limits])
        hi = np.array([l[1] - m for l in self.kin.limits])
        # the arm may START past a limit (held there while limp; 2026-10-24): that joint may stay between its
        # start and the limit, never further out (the start itself is checked against the arm below)
        lo, hi = np.minimum(lo, q[0]), np.maximum(hi, q[0])
        bad = np.argwhere((q < lo) | (q > hi))
        if len(bad):
            i, j = bad[0]
            return f"point {i}: joint {j + 1} at {math.degrees(q[i, j]):.1f} deg is outside its limits"
        v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
        vmax = float(self.P["traj_max_joint_vel_rad_s"]) * 1.1
        if (v > vmax).any():
            i, j = np.unravel_index(v.argmax(), v.shape)
            return f"point {i + 1}: joint {j + 1} would turn at {v[i, j]:.2f} rad/s > {vmax:.2f} (traj_max_joint_vel_rad_s + 10 %)"
        for i in (range(len(t)) if geometry else ()):
            with warnings.catch_warnings():          # a level gripper is exactly at the rpy gimbal lock; the rotation is still right
                warnings.simplefilter("ignore", UserWarning)
                pose = self.kin.fk_xyzrpy(q[i])
            if not self._in_workspace(pose[:3]):
                return f"point {i} (t = {t[i]:.2f} s): flange outside the workspace box"
            fv = self._floor_violation(pose)
            if fv:
                return f"point {i} (t = {t[i]:.2f} s): {fv}"
        return None

    def exec_follow_trajectory(self, gh):
        """Validate, then stream: every 1 / traj_rate_hz the target interpolated at the current time
        goes to the controller with move_j (its speed set to max_speed_percent so it can keep up — the
        trajectory's own timing limits the motion). Cancel / errors / lost feedback / tracking error
        -> freeze where it is. At the end, hold the last point until settled."""
        g = gh.request
        res = FollowTrajectory.Result()

        def fail(msg):
            res.success, res.message = False, msg
            gh.abort()
            return res

        jt = g.trajectory
        names = list(jt.joint_names)
        if names and sorted(names) != sorted(JOINT_NAMES):
            return fail(f"refused: joint_names must be {JOINT_NAMES}")
        order = [names.index(n) for n in JOINT_NAMES] if names else list(range(6))
        pts = list(jt.points)
        if len(pts) < 2:
            return fail("refused: a trajectory needs at least 2 points")
        if len(pts) > int(self.P["traj_max_points"]):
            return fail(f"refused: {len(pts)} points > traj_max_points {self.P['traj_max_points']}")
        try:
            t = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts], dtype=float)
            q = np.array([[float(p.positions[i]) for i in order] for p in pts], dtype=float)
        except (IndexError, ValueError):
            return fail("refused: every point needs 6 joint positions")
        if (np.diff(t) <= 0).any():
            return fail("refused: time_from_start must increase strictly")
        why = self._check_trajectory(t, q)
        if why:
            return fail("refused: " + why)
        cur = self.snap.get("joints") or self._read()["joints"]
        if cur is None:
            return fail("no joint feedback")
        d0 = float(np.abs(q[0] - np.array(cur, dtype=float)).max())
        if d0 > float(self.P["traj_start_tol_rad"]):
            return fail(f"refused: the trajectory starts {d0:.3f} rad away from the current joints "
                        f"(> traj_start_tol_rad {self.P['traj_start_tol_rad']}); plan it from where the arm is")
        tol = float(g.max_tracking_error_rad) if g.max_tracking_error_rad > 0 else float(self.P["traj_tracking_error_rad"])
        ok, msg, worst, secs = self._stream(gh, t, q, tol, g.label or "trajectory")
        res.success, res.message = ok, msg
        res.max_tracking_error_rad, res.seconds = float(worst), float(secs)
        (gh.succeed if ok else gh.abort)()
        return res

    def _stream(self, gh, t, q, tol, label, feedback=None):
        """The streaming loop of every motion (follow_trajectory, move_j, move_pose): (ok, message, worst
        tracking error, seconds). feedback(tt, worst, snap) -> the goal's feedback message (default: FollowTrajectory's)."""
        if not self.motion.acquire(blocking=False):
            return False, "another motion is running", 0.0, 0.0
        self.busy = True
        self.cancel_requested = False
        worst, tt, bad = 0.0, 0.0, 0
        try:
            s = self._read()
            if not all(s["enabled"]):
                return False, "arm is not enabled (harness arm hold first)", 0.0, 0.0
            speed = int(self.P["max_speed_percent"])
            with self.sdk:
                self.robot.set_speed_percent(speed)
            self.speed = speed
            T_end, period = float(t[-1]), 1.0 / float(self.P["traj_rate_hz"])
            self.get_logger().info(f"{label}: streaming {len(t)} points over {T_end:.1f} s at "
                                   f"{self.P['traj_rate_hz']:.0f} Hz, speed {speed} %, tracking limit {tol:.3f} rad")
            m = float(self.P["ik_limit_margin_rad"]) * 0.5
            lo = [l[0] + m for l in self.kin.limits]
            hi = [l[1] - m for l in self.kin.limits]
            t0 = time.monotonic()
            nxt, k = t0, 0
            while True:
                tt = time.monotonic() - t0
                if tt >= T_end:
                    break
                # targets inside the controller limits: a start past a limit is pulled back in, never commanded out
                cmd = [min(hi[j], max(lo[j], float(np.interp(tt, t, q[:, j])))) for j in range(6)]
                with self.sdk:
                    self.robot.move_j(cmd)
                s = self._read()
                if self.cancel_requested or gh.is_cancel_requested:
                    self._stop_here()
                    return False, f"cancelled at {tt:.1f} s of {T_end:.1f} s; stopped where it was", worst, tt
                if s["errors"]:
                    self._stop_here()
                    return False, f"errors at {tt:.1f} s: " + ",".join(s["errors"]), worst, tt
                if s["age"] >= float(self.P["watchdog_s"]):
                    return False, f"feedback lost at {tt:.1f} s", worst, tt
                bad = bad + 1 if s.get("arm_status", "NORMAL") != "NORMAL" else 0
                if bad >= 3:                          # the controller refused the targets (limit, singularity)
                    self._stop_here()
                    return False, f"controller reports {s['arm_status']} at {tt:.1f} s; stopped", worst, tt
                if s["joints"] is not None:
                    e = [abs(a - b) for a, b in zip(s["joints"], cmd)]
                    j = max(range(6), key=lambda i: e[i])
                    worst = max(worst, e[j])
                    if e[j] > tol:
                        self._stop_here()
                        return False, (f"tracking error {e[j]:.3f} rad on joint {j + 1} > {tol:.3f} at {tt:.1f} s "
                                       f"(the arm cannot follow: blocked, clamped at a limit, or too fast); stopped"), worst, tt
                if k % 10 == 0:
                    if feedback is not None:
                        gh.publish_feedback(feedback(tt, worst, s))
                    else:
                        f = FollowTrajectory.Feedback()
                        f.progress, f.tracking_error_rad = tt / T_end, worst
                        f.point_index = int(np.searchsorted(t, tt))
                        gh.publish_feedback(f)
                k += 1
                nxt += period
                delay = nxt - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    nxt = time.monotonic()               # overrun: re-sync, never burst
            final = [float(x) for x in q[-1]]
            with self.sdk:
                self.robot.move_j(final)
            jtol, need, settled, t_end = float(self.P["joint_tol_rad"]), int(self.P["settle_samples"]), 0, time.time() + 5.0
            while time.time() < t_end:
                time.sleep(0.05)
                s = self._read()
                if self.cancel_requested or gh.is_cancel_requested:
                    self._stop_here()
                    return False, "cancelled while settling", worst, T_end
                if s["joints"] is not None:
                    e = max(abs(a - b) for a, b in zip(s["joints"], final))
                    settled = settled + 1 if e <= jtol else 0
                    if settled >= need:
                        return True, f"{label}: done in {T_end:.1f} s (worst tracking error {worst:.3f} rad)", worst, T_end
            return False, f"{label}: streamed, but the last point did not settle within 5 s", worst, T_end
        finally:
            self.busy = False
            self.motion.release()

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
