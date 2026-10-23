"""Planner bedrock (PROJECT_SPEC.md "2026-10-22 — Planner bedrock"), offline: no ROS, no arm.

Run from the repo root with numpy, scipy, pyyaml, pytest:
    python -m pytest src/harness_planner/test -q
"""
import copy
import json
import math
import os
import sys

import numpy as np
import pytest
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path[:0] = [os.path.join(ROOT, "src", "harness_arm"), os.path.join(ROOT, "src", "harness_planner")]

from harness_arm.kinematics import PiperKinematics                       # noqa: E402
from harness_planner.trajectory import (CartesianPath, Limits, PlanError, flange_T, min_jerk,   # noqa: E402
                                        plan_path_run, smooth_fraction)

# pyAgxArm ROBOT_MDH_PRESET["piper"] (the arm node loads it from the SDK; copied so the tests need no SDK)
MDH = ((0.123, 0.0, 0.0, 0.0), (0.0, 0.0, -1.5707963267948966, -3.0058060377846343),
       (0.0, 0.28502999999999995, 0.0, -1.793849405199772), (0.25075, -0.02198, 1.5707963267948966, 0.0),
       (0.0, 0.0, -1.5707963267948966, 0.0), (0.091, 0.0, 1.5707963267948966, 0.0))
Q_PARK = [0.0, 0.2, -0.4, 0.0, 0.3, 0.0]          # a folded, parked-like posture


@pytest.fixture(scope="module")
def kin():
    with open(os.path.join(ROOT, "calib", "kinematics.yaml")) as f:
        d = yaml.safe_load(f)
    return PiperKinematics(MDH, d["joint_zero_offsets_rad"], alpha_offsets=d["alpha_offsets_rad"])


@pytest.fixture(scope="module")
def lim():
    with open(os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")) as f:
        p = yaml.safe_load(f)["piper"]["ros__parameters"]
    return Limits(tcp_offset_m=p["tcp_offset_m"], floor_plane=tuple(p["floor_plane"]), floor_margin_m=p["floor_margin_m"],
                  workspace_min=tuple(p["workspace_min"]), workspace_max=tuple(p["workspace_max"]))


@pytest.fixture(scope="module")
def motion():
    with open(os.path.join(ROOT, "motions", "twin_demo.json")) as f:
        return json.load(f)


@pytest.fixture(scope="module")
def plan24(kin, lim, motion):
    return plan_path_run(kin, CartesianPath.from_motion(motion), Q_PARK, 24.0, lim)


def test_motion_file_contract(motion):
    p = CartesianPath.from_motion(motion)
    assert p.closed and len(p.p) == 360 and p.name == "twin-demo-figure-eight"
    assert np.allclose(p.p[0], [0.565, 0.0, 0.300])                       # mm -> m
    assert np.allclose(np.linalg.norm(p.approach, axis=1), 1) and np.allclose((p.approach * p.up).sum(1), 0)
    for key, bad in (("format", "railab-motion/0"), ("frame", "world"), ("units", {"length": "m"}), ("tool", "flange")):
        d = copy.deepcopy(motion)
        d[key] = bad
        with pytest.raises(PlanError):
            CartesianPath.from_motion(d)
    d = copy.deepcopy(motion)
    d["samples"][5]["up"] = d["samples"][5]["approach"]
    with pytest.raises(PlanError, match="parallel"):
        CartesianPath.from_motion(d)


def test_timing_profiles():
    T, ramp = 24.0, 2.0
    t = np.linspace(0, T, 2401)
    u = smooth_fraction(t, T, ramp)
    assert u[0] == 0 and abs(u[-1] - 1) < 1e-12 and (np.diff(u) >= -1e-15).all()
    v = np.diff(u) / np.diff(t)
    assert v[0] < 1e-5 and v[-1] < 1e-5          # starts and stops at rest (first 10 ms; cruise = 0.045 / s)
    assert abs(v[1200] - 1 / (T - ramp)) < 1e-9                            # cruise speed
    tt, qq = min_jerk([0] * 6, [1.0, -0.5, 0, 0, 0, 0], 0.5, 1.0, 100.0)
    vv = np.abs(np.diff(qq, axis=0)) / np.diff(tt)[:, None]
    assert np.allclose(qq[0], 0) and np.allclose(qq[-1], [1.0, -0.5, 0, 0, 0, 0])
    assert vv.max() <= 0.5 + 1e-6 and vv[0].max() < 1e-3 and vv[-1].max() < 1e-3


def test_demo_plans_reachable_level_and_returns(kin, lim, motion, plan24):
    tr, r = plan24, plan24.report
    names = [s[0] for s in tr.segments]
    assert names == ["approach", "loop", "return"]
    assert np.allclose(tr.q[0], Q_PARK) and np.allclose(tr.q[-1], Q_PARK)          # one run, back where it started
    assert abs(r["segments"]["loop"] - 24.0) < 0.05 and len(tr.t) == r["samples"]
    assert max(r["peak_joint_speed_deg_s"]) <= math.degrees(lim.max_joint_vel_rad_s) + 1e-6
    assert r["max_curve_deviation_mm"] < 0.05 and r["min_floor_clearance_mm"] > 150
    # the loop: the fingertip follows the motion file, the gripper level (flange z horizontal, x down)
    _, i0, i1 = next(s for s in tr.segments if s[0] == "loop")
    start = kin.fk(tr.q[i0])
    tip = start[:3, 3] + lim.tcp_offset_m * start[:3, 2]
    assert np.linalg.norm(tip - np.array(motion["samples"][0]["p"]) / 1000) < 5e-4
    for k in range(i0, i1, 97):
        T = kin.fk(tr.q[k])
        assert abs(T[2, 2]) < 2e-3 and T[2, 0] < -0.999
    # the joints move continuously (no branch flip): small steps between 100 Hz samples
    assert np.degrees(np.abs(np.diff(tr.q, axis=0)).max()) < 0.5


def test_too_fast_is_refused_with_a_duration_that_fits(kin, lim, motion):
    path = CartesianPath.from_motion(motion)
    with pytest.raises(PlanError) as e:
        plan_path_run(kin, path, Q_PARK, 12.0, lim)
    msg = str(e.value)
    assert "deg/s" in msg and "--duration" in msg
    hint = int(msg.split("about ")[1].split(" s")[0])
    tr = plan_path_run(kin, path, Q_PARK, float(hint), lim)               # the hint is honest
    assert max(tr.report["peak_joint_speed_deg_s"]) <= math.degrees(lim.max_joint_vel_rad_s) + 1e-6


def test_floor_and_reach_refusals(kin, lim, motion):
    path = CartesianPath.from_motion(motion)
    high_floor = Limits(**{**lim.__dict__, "floor_plane": (0.0, 0.0, 0.25)})   # a floor at 250 mm: the path dips below
    with pytest.raises(PlanError, match="above the floor"):
        plan_path_run(kin, path, Q_PARK, 24.0, high_floor)
    far = copy.deepcopy(motion)
    for s in far["samples"]:
        s["p"] = [v * 1.6 for v in s["p"]]                                  # ~0.9 m out: beyond the arm
    with pytest.raises(PlanError, match="out of reach"):
        plan_path_run(kin, CartesianPath.from_motion(far), Q_PARK, 24.0, lim)


def test_flange_frame_convention():
    T = flange_T([0.5, 0.0, 0.3], [1, 0, 0], [0, 0, 1], 0.1441)
    assert np.allclose(T[:3, 2], [1, 0, 0]) and np.allclose(T[:3, 0], [0, 0, -1])     # z = approach, x = -up
    assert np.allclose(T[:3, 3], [0.5 - 0.1441, 0.0, 0.3])
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)                                  # right-handed
