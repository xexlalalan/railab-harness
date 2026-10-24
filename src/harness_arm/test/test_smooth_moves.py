"""Smooth moves in the arm node (PROJECT_SPEC.md "2026-10-24 — Smooth motion"), offline: the node's planning
methods with ROS and the arm SDK stubbed out, on the real arm model (SDK MDH table + calib/kinematics.yaml).

    bunny-harness-venv/bin/python -m pytest src/harness_arm/test -q
"""
import math
import os
import sys
import types

import numpy as np
import pytest
import yaml

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
from harness_arm.kinematics import PiperKinematics, pose_error   # noqa: E402

MDH = ((0.123, 0.0, 0.0, 0.0), (0.0, 0.0, -1.5707963267948966, -3.0058060377846343),
       (0.0, 0.28502999999999995, 0.0, -1.793849405199772), (0.25075, -0.02198, 1.5707963267948966, 0.0),
       (0.0, 0.0, -1.5707963267948966, 0.0), (0.091, 0.0, 1.5707963267948966, 0.0))


class _Node:
    def __init__(self):
        with open(os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")) as f:
            self.P = yaml.safe_load(f)["piper"]["ros__parameters"]
        with open(os.path.join(ROOT, "calib", "kinematics.yaml")) as f:
            d = yaml.safe_load(f)
        self.kin = PiperKinematics(MDH, d["joint_zero_offsets_rad"], alpha_offsets=d["alpha_offsets_rad"])

    def _in_workspace(self, p):
        return True

    def _floor_violation(self, pose):
        return None


for _n in ("_smooth_caps", "_joint_line", "_tool_line", "_check_trajectory"):
    setattr(_Node, _n, getattr(pn.PiperNode, _n))

Q = [0.2, 1.0, -1.2, 0.0, 0.6, 0.0]                         # a reaching posture, gripper pointing forward-down


@pytest.fixture(scope="module")
def node():
    return _Node()


def _profile(t, q):
    v = np.diff(q, axis=0) / np.diff(t)[:, None]
    a = np.diff(v, axis=0) / np.diff(t)[1:, None]
    return v, a


def test_joint_move_starts_and_stops_gently(node):
    q1 = list(Q)
    q1[0] += 0.4
    t, q = node._joint_line(Q, q1, 10)
    assert np.allclose(q[0], Q) and np.allclose(q[-1], q1)
    v, a = _profile(t, q)
    vj, aj = node._smooth_caps(10)[:2]
    assert abs(v).max() <= vj * 1.01 and abs(a).max() <= aj * 1.05
    assert abs(v[0]).max() < 0.01 * vj and abs(v[-1]).max() < 0.01 * vj            # at rest at both ends
    assert abs(a[0]).max() < 0.05 * aj and abs(a[-1]).max() < 0.05 * aj            # and no acceleration step
    assert node._check_trajectory(t, q, geometry=False) is None


def test_straight_lift_is_one_continuous_smooth_line(node):
    """The Pick & lift's 100 mm lift: before, five 20 mm hops (the arm stopped at each); now one line."""
    T0 = node.kin.fk(Q)
    T1 = T0.copy()
    T1[2, 3] += 0.100
    t, q = node._tool_line(Q, T0, T1, 10)
    tips = np.array([node.kin.fk(x)[:3, 3] for x in q])
    off = tips - T0[:3, 3]
    assert np.abs(off[:, :2]).max() < 0.0005                                         # straight up, < 0.5 mm off
    assert off[-1, 2] == pytest.approx(0.100, abs=0.0005)
    assert max(pose_error(node.kin.fk(x), T0)[1] for x in q) < math.radians(0.2)    # orientation kept
    z = off[:, 2]
    vz = np.diff(z) / np.diff(t)
    assert (vz >= -1e-6).all()                                                       # never goes back down
    mid = vz[len(vz) // 4: 3 * len(vz) // 4]
    assert mid.min() > 0.3 * vz.max()                                                # no stops on the way
    assert abs(vz[0]) < 0.002 and abs(vz[-1]) < 0.002                                 # gentle start and stop
    vl = node._smooth_caps(10)[2]
    assert vz.max() <= vl * 1.02                                                     # 10 % = 50 mm/s at most
    assert node._check_trajectory(t, q) is None


def test_a_line_that_leaves_the_branch_is_refused(node):
    T0 = node.kin.fk(Q)
    T1 = T0.copy()
    T1[0, 3] += 0.8                                                                  # far out of reach
    with pytest.raises(ValueError, match="no IK solution|would jump"):
        node._tool_line(Q, T0, T1, 10)
