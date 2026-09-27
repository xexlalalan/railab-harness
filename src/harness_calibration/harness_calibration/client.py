"""Synchronous convenience client for the arm node and the tag node.

Spins its own executor in a background thread so the latest /arm/status,
/joint_states, /arm/tcp_pose and /camera/tags are always available to a plain
procedural script (hand-eye routine, teach CLI, cycle sequencer).
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from harness_msgs.action import MoveJ, MovePose
from harness_msgs.msg import ArmStatus, TagArray
from harness_msgs.srv import SetGripper
from harness_vision.geom import T_from_pose_msg, xyzrpy_from_T


class HarnessClient:
    def __init__(self, name="harness_client"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(name)
        self.status = None
        self.joints = None
        self.tcp_T = None
        self.tags = None
        self._tags_t = 0.0
        n = self.node
        n.create_subscription(ArmStatus, "/arm/status", self._on_status, 10)
        n.create_subscription(JointState, "/joint_states", self._on_js, 10)
        n.create_subscription(PoseStamped, "/arm/tcp_pose", self._on_tcp, 10)
        n.create_subscription(TagArray, "/camera/tags", self._on_tags, 10)
        self.ac_pose = ActionClient(n, MovePose, "/arm/move_pose")
        self.ac_j = ActionClient(n, MoveJ, "/arm/move_j")
        self.ex = MultiThreadedExecutor(num_threads=2)
        self.ex.add_node(n)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    # ------------------------------------------------------------ callbacks
    def _on_status(self, m):
        self.status = m

    def _on_js(self, m):
        self.joints = [float(x) for x in m.position[:6]]

    def _on_tcp(self, m):
        self.tcp_T = T_from_pose_msg(m.pose)

    def _on_tags(self, m):
        self.tags = m
        self._tags_t = time.time()

    # ------------------------------------------------------------ waiting
    def wait_ready(self, camera=True, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.status is not None and self.tcp_T is not None and self.joints is not None \
                    and (not camera or self.tags is not None):
                return True
            time.sleep(0.1)
        missing = [k for k, v in (("arm status", self.status), ("tcp pose", self.tcp_T),
                                  ("joint states", self.joints), ("camera tags", self.tags if camera else 1)) if v is None]
        raise RuntimeError("not ready, missing: " + ", ".join(missing))

    def wait_still(self, tol_m=5e-5, window_s=0.5, timeout=5.0, min_s=0.5):
        """Block until the reported flange pose has moved < tol_m over window_s (the PIPER
        keeps creeping ~1 s after it reports the target reached). Returns seconds waited."""
        t0 = time.time()
        hist = []
        while time.time() - t0 < timeout:
            hist.append((time.time(), self.tcp_T[:3, 3].copy()))
            hist = [h for h in hist if h[0] >= time.time() - window_s]
            if time.time() - t0 >= min_s and len(hist) >= 5:
                d = max(float(np.linalg.norm(h[1] - hist[0][1])) for h in hist)
                if d < tol_m:
                    return time.time() - t0
            time.sleep(0.05)
        return time.time() - t0

    def tag(self, tag_id, max_age=0.5):
        """Latest detection of tag_id, or None."""
        if self.tags is None or time.time() - self._tags_t > max_age:
            return None
        for t in self.tags.tags:
            if t.id == tag_id:
                return t
        return None

    def collect_tag(self, tag_id, n=10, timeout=3.0):
        """Collect n distinct detections of tag_id (as 4x4 cam->tag) plus the flange poses seen with them."""
        Ts, Tfs, last = [], [], None
        t0 = time.time()
        while len(Ts) < n and time.time() - t0 < timeout:
            m = self.tags
            if m is not None and m is not last:
                last = m
                for t in m.tags:
                    if t.id == tag_id:
                        Ts.append(T_from_pose_msg(t.pose))
                        Tfs.append(self.tcp_T.copy())
            time.sleep(0.01)
        return Ts, Tfs

    # ------------------------------------------------------------ commands
    def trigger(self, name, timeout=10.0):
        cli = self.node.create_client(Trigger, f"/arm/{name}")
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service /arm/{name} unavailable")
        fut = cli.call_async(Trigger.Request())
        self._wait(fut, timeout)
        self.node.destroy_client(cli)
        return fut.result()

    def gripper(self, width_m, force=0.0, timeout=10.0):
        cli = self.node.create_client(SetGripper, "/arm/gripper")
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service /arm/gripper unavailable")
        req = SetGripper.Request()
        req.width_m, req.force = float(width_m), float(force)
        fut = cli.call_async(req)
        self._wait(fut, timeout)
        self.node.destroy_client(cli)
        return fut.result()

    def move_pose(self, T_or_xyzrpy, linear=False, speed=10, timeout=30.0):
        pose = xyzrpy_from_T(T_or_xyzrpy) if isinstance(T_or_xyzrpy, np.ndarray) else list(T_or_xyzrpy)
        g = MovePose.Goal()
        g.pose = [float(v) for v in pose]
        g.linear, g.speed_percent, g.timeout_s = bool(linear), int(speed), float(timeout)
        return self._send(self.ac_pose, g, timeout + 5)

    def move_j(self, joints, speed=10, timeout=30.0):
        g = MoveJ.Goal()
        g.joints_rad = [float(v) for v in joints]
        g.speed_percent, g.timeout_s = int(speed), float(timeout)
        return self._send(self.ac_j, g, timeout + 5)

    def _send(self, ac, goal, timeout):
        if not ac.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("action server unavailable")
        gf = ac.send_goal_async(goal)
        self._wait(gf, 5.0)
        gh = gf.result()
        if not gh.accepted:
            class R:  # noqa: D401
                success, message = False, "goal rejected by the arm node (check /arm/status)"
            return R()
        rf = gh.get_result_async()
        self._wait(rf, timeout)
        return rf.result().result

    @staticmethod
    def _wait(fut, timeout):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise RuntimeError("timeout waiting for the arm node")
            time.sleep(0.01)

    def close(self):
        try:
            self.ex.shutdown(timeout_sec=1.0)
            self.th.join(timeout=2.0)
            self.ex.remove_node(self.node)
            self.node.destroy_node()
        finally:
            rclpy.try_shutdown()

    # PIPER joint limits (datasheet, rad). A hand-dragged arm can sit outside them; the
    # controller then refuses every Cartesian target with TARGET_POS_EXCEEDS_LIMIT.
    JOINT_LIMITS = [(-2.687, 2.687), (0.0, 3.403), (-3.054, 0.0), (-1.745, 1.745), (-1.222, 1.222), (-2.094, 2.094)]   # j5 70.0 deg measured 2026-09-17; j4 100.0 deg measured 2026-09-18 (controller clamps silently)

    def joints_outside_limits(self, margin=0.03):
        out = []
        for i, (q, (lo, hi)) in enumerate(zip(self.joints, self.JOINT_LIMITS)):
            if q < lo + margin or q > hi - margin:
                out.append((i + 1, q, lo, hi))
        return out
