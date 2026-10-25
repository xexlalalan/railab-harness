"""The e-stop button in the arm node (PROJECT_SPEC.md "2026-10-24 — Emergency stop button"), offline: the USB watch
on a fake sysfs tree, and the node's trip / hold / refuse / release logic with ROS and the arm SDK stubbed out.

    bunny-harness-venv/bin/python -m pytest src/harness_arm/test -q
"""
import os
import sys
import threading
import types

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


class _Any:
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, n):
        return _Any()

    def __call__(self, *a, **k):
        return _Any()


for _m in ("rclpy", "rclpy.action", "rclpy.callback_groups", "rclpy.executors", "rclpy.node", "geometry_msgs",
           "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "std_srvs", "std_srvs.srv", "harness_msgs",
           "harness_msgs.action", "harness_msgs.msg", "harness_msgs.srv", "pyAgxArm"):
    if _m not in sys.modules or not hasattr(sys.modules[_m], "__file__"):
        mod = types.ModuleType(_m)
        mod.__getattr__ = lambda n: _Any
        sys.modules[_m] = mod
sys.modules["rclpy.node"].Node = object
sys.path.insert(0, os.path.join(ROOT, "src", "harness_arm"))
from harness_arm import piper_node as pn                     # noqa: E402

Q = [0.2, 1.0, -1.2, 0.0, 0.6, 0.0]


def _device(root, name, vendor, product):
    d = root / name
    d.mkdir()
    (d / "idVendor").write_text(vendor + "\n")
    (d / "idProduct").write_text(product + "\n")
    (root / (name + ":1.0")).mkdir()                       # its interface: no ids, never a match


class _Robot:
    def __init__(self):
        self.moves, self.disabled = [], []

    def move_j(self, q):
        self.moves.append(list(q))

    def disable(self, j):
        self.disabled.append(j)

    def disable_gripper(self):
        self.disabled.append(7)


class _Log:
    def __init__(self, lines):
        self.lines = lines

    def info(self, m):
        self.lines.append(("info", m))

    def warning(self, m):
        self.lines.append(("warning", m))

    def error(self, m):
        self.lines.append(("error", m))


class _Node:
    def __init__(self, joints=Q, enabled=True, age=0.0):
        self.P = {"watchdog_s": 0.25, "estop_usb_id": "04f2:1459", "estop_poll_s": 0.02}
        self.sdk, self.motion = threading.Lock(), threading.Lock()
        self.fault, self.estop, self.estop_hold, self.cancel_requested = "", "", None, False
        self.snap = {"joints": list(joints) if joints else None, "enabled": [enabled] * 6, "age": age, "errors": [],
                     "teach": False}
        self.robot, self.ee, self.lines = _Robot(), _Robot(), []

    def _read(self):
        return self.snap

    def get_logger(self):
        return _Log(self.lines)


for _n in ("_estop_step", "_estop_trip", "_estop_release", "_refuse", "_refused_estop", "_stop_here",
           "srv_hold", "srv_disable", "srv_gripper", "srv_gripper_release", "srv_enable"):
    setattr(_Node, _n, getattr(pn.PiperNode, _n))


def _res():
    return types.SimpleNamespace(success=None, message="", stalled=False, width_m=0.0)


def test_the_usb_watch_sees_the_mouse_go_and_come_back(tmp_path):
    _device(tmp_path, "1-8.1", "1d50", "606f")             # the CAN adapter
    _device(tmp_path, "1-8.2", "04F2", "1459")             # the mouse (ids in any case)
    w = pn.UsbWatch("04f2:1459", root=str(tmp_path))
    assert w.present() and w.path.endswith("1-8.2")
    for f in ("idVendor", "idProduct"):
        (tmp_path / "1-8.2" / f).unlink()
    (tmp_path / "1-8.2").rmdir()                           # the 5 V cut: the kernel drops the device
    assert not w.present() and w.path is None
    assert pn.UsbWatch("1d50:606f", root=str(tmp_path)).present()   # the adapter is still there
    _device(tmp_path, "1-9", "04f2", "1459")               # back, at another port
    assert w.present() and w.path.endswith("1-9")
    assert not pn.UsbWatch("04f2:1459", root=str(tmp_path / "nowhere")).present()


def test_pressing_the_button_holds_the_arm_where_it_is_and_refuses_every_command():
    n = _Node()
    n._estop_step(present=False)
    assert n.estop.startswith(pn.ESTOP_PRESSED) and n.fault == n.estop and n.cancel_requested
    assert n.robot.moves == [Q] and n.estop_hold == Q
    assert n._refuse() == n.estop
    # the arm keeps the trip-time target while it is pressed, even as its joints still move a little
    n.snap["joints"] = [q + 0.02 for q in Q]
    n._estop_step(present=False)
    n._stop_here()
    assert n.robot.moves == [Q, Q, Q]
    for call in (n.srv_hold, n.srv_disable, n.srv_enable, n.srv_gripper_release):
        r = call(None, _res())
        assert r.success is False and r.message == "refused: " + n.estop
    r = n.srv_gripper(types.SimpleNamespace(width_m=0.03, force=1.0, timeout_s=0.0), _res())
    assert r.success is False and "e-stop" in r.message
    assert n.robot.disabled == [] and n.ee.disabled == []    # Release did not free a joint, nor the gripper
    assert any(lvl == "error" and "E-STOP" in m and "holding joints" in m for lvl, m in n.lines)


def test_twisting_it_out_makes_the_arm_ready_again_without_moving_it():
    n = _Node()
    n._estop_step(present=False)
    n._estop_step(present=True)
    assert n.estop == "" and n.estop_hold is None and n.fault == ""
    assert n.robot.moves == [Q]                             # nothing sent on release: the controller keeps its target
    assert n._refuse() is None and n._refused_estop(_res()) is False   # ready: the operator's next command goes through
    n._estop_step(present=True)
    assert n.robot.moves == [Q] and n.fault == ""


def test_an_unpowered_arm_is_left_in_the_operators_hands():
    n = _Node(enabled=False)
    n._estop_step(present=False)
    assert n.estop and n.estop_hold is None and n.robot.moves == []
    n._estop_step(present=False)
    n._stop_here()
    assert n.robot.moves == []
    assert any("not powered" in m for _, m in n.lines)


def test_no_mouse_at_start_counts_as_pressed():
    n = _Node()
    n._estop_step(present=False, first=True)
    assert "unplugged" in n.estop and n.robot.moves == [Q]


def test_lost_feedback_while_pressed_keeps_the_e_stop_first():
    n = _Node(joints=None, age=9.0)
    n._estop_step(present=False)
    assert n.estop_hold is None and n.robot.moves == [] and any("no CAN feedback" in m for _, m in n.lines)
