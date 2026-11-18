#!/usr/bin/env python3
"""Offline proof of the calibrations' re-pose (harness_calibration/precheck.py repose_search / assess / do_repose;
PROJECT_SPEC.md "2026-11-18 — the re-pose"): no ROS, no arm motion. The repo's arm model (calib/kinematics.yaml), the
arm node's parameters (src/harness_arm/config/piper.yaml) and hand-eye (calib/handeye.yaml).

  1. THE PC's CASE (2026-11-18: "turn the wrist (joint 4) toward the middle: 100.0° of ±100°; only 11 of 15 viewpoints
     reachable"): the wrist at +99.9°, the other joints a plausible held posture with the board 250 mm before the camera
     -> the search (no motion): which posture, the board at the same pixel / distance, margins and views before / after,
     the move_j via points, the search time
  2. a posture no re-pose can fix -> the page's plain words
  3. the gateway's state (stubbed): ok / fixable / needs_repose / words / why
  4. do_repose's code path on a stand-in client whose moves are instant (no arm, no planner): it moves, re-checks, passes

    python3 tools/repose_proof.py           (needs pyAgxArm's MDH table: the harness PC's ros-venv has it)
"""
import math
import os
import sys
import time
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "src", "harness_calibration"),
                os.path.join(ROOT, "src", "harness_vision"), os.path.join(ROOT, "src", "harness_arm")]
from harness_calibration import precheck as PC  # noqa: E402

K = np.array([[645.0, 0, 640.0], [0, 645.0, 360.0], [0, 0, 1]])
D = math.radians


def board_in_front(kin, X, q, dist=0.25):
    """The target 250 mm before the camera, facing it (its z away from the camera, as the board's)."""
    Tc = kin.fk(list(q)) @ X
    Tct = np.eye(4)
    Tct[:3, 3] = [0.0, 0.0, dist]
    return Tct, Tc @ Tct


def pixel(T_cam_target):
    p = K @ T_cam_target[:3, 3]
    return p[:2] / p[2]


def main():
    kin, P = PC.load_model(os.path.join(ROOT, "calib", "kinematics.yaml"), os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml"))
    X = PC.load_handeye(os.path.join(ROOT, "calib", "handeye.yaml"))
    rng = np.random.default_rng(7)
    lo = np.array([l[0] for l in kin.limits])
    hi = np.array([l[1] for l in kin.limits])
    ok = True

    def plausible(q):
        Tct, Tbt = board_in_front(kin, X, q)
        Tc = kin.fk(list(q)) @ X
        tip = kin.fk(list(q))[:3, 3] + float(P["tcp_offset_m"]) * kin.fk(list(q))[:3, 2]
        return Tc[2, 2] < -0.4 and 0.0 < Tbt[2, 3] < 0.25 and 0.2 < math.hypot(Tbt[0, 3], Tbt[1, 3]) < 0.5 and tip[2] > 0.08

    # 1. the PC's case: joint 4 at +99.9°
    q = None
    for _ in range(50000):
        c = rng.uniform(lo + 0.1, hi - 0.1)
        c[3] = D(99.9)
        if plausible(c):
            pc0 = PC.precheck(kin, c, P)
            if not pc0["ok"]:
                q = c
                break
    Tct, _ = board_in_front(kin, X, q)
    t0 = time.time()
    pc = PC.assess(kin, list(q), P, X, Tct)
    took = time.time() - t0
    rp = pc["repose"]
    print(f"1. the PC's case: joints {np.round(np.degrees(q), 1).tolist()} deg (the wrist at +99.9°), the board 250 mm before "
          f"the camera")
    print(f"   before: {pc['why']}")
    print(f"   assess -> ok {pc['ok']}, fixable {pc['fixable']}, needs_repose {pc['needs_repose']}; page words: \"{pc['words']}\"")
    ok &= pc["ok"] and pc["needs_repose"] and rp is not None
    if rp:
        q2 = np.array(rp["q"])
        Tc1 = kin.fk(list(q)) @ X
        Tc2 = kin.fk(list(q2)) @ X
        B = (Tc1 @ Tct)[:3, 3]
        T2 = np.linalg.inv(Tc2) @ np.vstack([np.column_stack([np.eye(3), B]), [0, 0, 0, 1]])
        px1, px2 = pixel(Tct), pixel(T2)
        after = PC.precheck(kin, list(q2), P)
        steps = [np.abs(np.array(v) - np.array(w)).max() for v, w in zip([list(q)] + rp["vias"][:-1], rp["vias"])]
        print(f"   the re-pose ({took:.1f} s to find, no motion): roll {rp['roll_deg']}° about the line of sight, orbit "
              f"{rp['orbit'][0]:+.0f}° {rp['orbit'][1] or '-'}, other wrist solution {rp['flip']}")
        print(f"   joints {np.round(np.degrees(q2), 1).tolist()} deg; the smallest joint margin "
              f"{min(j['margin'] for j in PC.joint_margins(kin, q)):.1f}° -> {rp['margin_deg']:.1f}°; views "
              f"{PC.precheck(kin, list(q), P)['reachable']} -> {after['reachable']} of {after['of']}; largest joint motion "
              f"{rp['motion_deg']:.0f}°, the fingertip + camera travel {rp['travel_mm']:.0f} mm, {len(rp['vias'])} move_j goal(s), "
              f"each <= {math.degrees(max(steps)):.0f}°")
        print(f"   the board's centre stays at the same pixel: ({px1[0]:.1f}, {px1[1]:.1f}) -> ({px2[0]:.1f}, {px2[1]:.1f}) px, "
              f"distance {np.linalg.norm(Tct[:3, 3]) * 1000:.1f} -> {np.linalg.norm(T2[:3, 3]) * 1000:.1f} mm")
        ok &= after["ok"] and rp["margin_deg"] >= PC.REPOSE_MARGIN_MIN_DEG and np.linalg.norm(px2 - px1) < 2.0

    # 2. a posture no re-pose can fix: the board far out at the edge of the reach
    bad = None
    for _ in range(20000):
        c = rng.uniform(lo + 0.1, hi - 0.1)
        Tct_b, Tbt = board_in_front(kin, X, c)
        Tc = kin.fk(list(c)) @ X
        if Tc[2, 2] < -0.3 and math.hypot(Tbt[0, 3], Tbt[1, 3]) > 0.55 and Tbt[2, 3] > 0.0 and not PC.precheck(kin, c, P)["ok"]:
            pcb = PC.assess(kin, list(c), P, X, Tct_b)
            if not pcb["ok"]:
                bad = (c, pcb)
                break
    if bad:
        print(f"\n2. a board far out ({np.round(np.degrees(bad[0]), 1).tolist()} deg): ok {bad[1]['ok']}, fixable "
              f"{bad[1]['fixable']}; page words: \"{bad[1]['words']}\"  (log: {bad[1]['why']})")
        ok &= bad[1]["words"] == PC.PLAIN_FAIL and "limit" not in bad[1]["words"] and "joint" not in bad[1]["words"]

    # 3. the gateway's state
    import gateway_server as G
    gw = types.SimpleNamespace(kin=kin, P=P, Tfc=X, last={})
    gw.joints_rad = lambda: list(q)
    gw.charuco_state = lambda: {"seen": True, "corners": 64}
    gw.fresh = lambda k, a: types.SimpleNamespace(all_enabled=True) if k == "arm" else None
    gw.precheck_target = lambda: Tct
    st = G.Gateway.update_precheck(gw, force=True)
    print(f"\n3. state.calib_precheck: ok {st['ok']}, fixable {st['fixable']}, needs_repose {st['needs_repose']}, words "
          f"\"{st['words']}\", repose {st['repose']}; why (logs) \"{st['why']}\"")
    ok &= st["ok"] and st["fixable"] and "limit" not in st["words"]

    # 4. do_repose's code path on a stand-in client (instant moves, the board where it is in the base)
    B_base = (kin.fk(list(q)) @ X @ Tct)
    cl = types.SimpleNamespace(joints=list(q), moves=[])

    def move_j(v, speed=10, timeout=40):
        cl.moves.append(list(v))
        cl.joints = list(v)
        return types.SimpleNamespace(success=True, message="")
    cl.move_j, cl.wait_still = move_j, lambda: 0.0

    def target():
        T = np.linalg.inv(kin.fk(cl.joints) @ X) @ B_base
        return T if T[2, 3] > 0.05 and np.all(np.abs(pixel(T) - [640, 360]) < [600, 340]) else None
    lines = []
    done = PC.do_repose(cl, kin, P, X, target, speed=10, settle_s=0.0, log=lines.append)
    print(f"\n4. do_repose (stand-in client): {done}, {len(cl.moves)} move_j goal(s); lines: " + " | ".join(lines))
    ok &= done and lines[0] == PC.PLAIN_FIX_PROGRESS
    print("\nALL OK" if ok else "\nFAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
