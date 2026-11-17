#!/usr/bin/env python3
"""Offline proof of the gateway's camera.charuco.pose_base / centre_base_mm (PROJECT_SPEC.md "2026-11-16 — the board in
the arm base"): no ROS. The real Gateway.charuco_state on a stub gateway with the real arm model (calib/kinematics.yaml,
PiperKinematics) and the hand-eye in calib/handeye.yaml (the repo's copy; the PC's is the one installed there): a board
laid flat on the table at a known pose in the arm base, the arm at known joints, the camera -> board pose the camera
would report (1) exactly, as a message, and (2) through a synthetic image and the board detector; then the timing rule.

    python3 tools/charuco_base_proof.py
"""
import math
import os
import sys
import time
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "src", "harness_vision"), os.path.join(ROOT, "src", "harness_arm"),
                os.path.join(ROOT, "src", "harness_vision", "test")]
import gateway_server as G  # noqa: E402
from harness_arm.kinematics import PiperKinematics  # noqa: E402
from harness_vision.geom import T_from_xyzrpy, inv, pose_msg_from_T  # noqa: E402

JOINTS = [f"joint{i}" for i in range(1, 7)]


def stub():
    gw = types.SimpleNamespace(last={}, joint_hist=[], P={})
    gw.kin = PiperKinematics.from_file(G.KIN_FILE, "piper")
    G.Gateway.load_handeye(gw)
    gw.fresh = lambda k, a: (lambda v: v[0] if v and time.time() - v[1] < a else None)(gw.last.get(k))
    gw.joints_near = lambda t: G.Gateway.joints_near(gw, t)
    return gw


def board_msg(T_cam_board, seen=True):
    pose = types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace())
    pose_msg_from_T(T_cam_board, pose)
    return types.SimpleNamespace(seen=seen, corners=80, corners_total=80, distance_m=0.25, reproj_err_px=0.05, pose=pose,
                                 squares_x=9, squares_y=11, square_m=0.022)


def feed(gw, q, Tcb, joints_dt=0.01, seen=True):
    now = time.time()
    gw.joint_hist = [(now - joints_dt, types.SimpleNamespace(name=JOINTS, position=list(q)))]
    gw.last["joints"] = (gw.joint_hist[0][1], now - joints_dt)
    m = board_msg(Tcb, seen)
    gw.last["charuco"] = (m, now)
    if seen:
        gw.last["charuco_seen"] = (m, now)
    return G.Gateway.charuco_state(gw)


def errors(st, Tbb):
    R = np.array(st["pose_base"]["R"])
    p = np.array(st["pose_base"]["p_mm"])
    c_true = (Tbb @ np.array([0.099, 0.121, 0.0, 1.0]))[:3] * 1000
    ang = 2 * math.asin(min(1.0, np.linalg.norm(R - Tbb[:3, :3]) / (2 * math.sqrt(2))))   # |Ra - Rb|_F = 2 sqrt2 sin(a/2)
    return (float(np.linalg.norm(p - Tbb[:3, 3] * 1000)), math.degrees(ang),
            float(np.linalg.norm(np.array(st["centre_base_mm"]) - c_true)))


def main():
    gw = stub()
    print(f"hand-eye {G.HANDEYE_FILE} ({gw.he_info}): flange -> camera {np.round(gw.Tfc[:3, 3] * 1000, 1).tolist()} mm")
    rng = np.random.default_rng(4)
    C = np.array([0.099, 0.121, 0.0])                                 # the board's centre in its own frame
    rows, det, ok = [], [], True
    import test_charuco as TC
    from harness_vision.detect import Charuco
    ch = Charuco()
    src = TC.board_source(ch)
    for k in range(8):
        # the arm at known joints (a posture over the table, varied); the board 230-300 mm in front of the camera,
        # facing it, tilted <= 20°, rolled at random; its true pose in the arm base follows
        q = list(np.radians([rng.uniform(-60, 60), rng.uniform(40, 80), rng.uniform(-110, -60), rng.uniform(-20, 20),
                             rng.uniform(50, 80), rng.uniform(-90, 90)]))
        Tbc = gw.kin.fk(q) @ gw.Tfc
        Tcb = T_from_xyzrpy([0, 0, 0] + list(np.radians([rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-180, 180)])))
        Tcb[:3, 3] = np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(0.23, 0.30)]) - Tcb[:3, :3] @ C
        Tbb = Tbc @ Tcb                                                 # the truth: the board in the arm base
        e = errors(feed(gw, q, Tcb), Tbb)                               # (1) the camera's report taken exactly
        rows.append(e)
        img = TC.render(src, Tcb, seed=k)                               # (2) through an image and the board detector
        ids, pts, nm = ch.detect(img)
        m = ch.measure(ids, pts, nm, TC.normalise, TC.K[0, 0])
        if m["seen"]:
            st = feed(gw, q, m["T"])
            d = errors(st, Tbb)
            det.append(d)
        print(f"  {k}: joints {np.round(np.degrees(q), 1).tolist()} deg, board centre {np.round((Tbb @ np.append(C, 1))[:3] * 1000, 1).tolist()} mm"
              f" | exact: p {e[0]:.4f} mm, R {e[1]:.5f} deg, centre {e[2]:.4f} mm"
              + (f" | detected ({m['corners']} corners): centre {d[2]:.3f} mm, R {d[1]:.4f} deg" if m["seen"] else " | not seen"))
    ok &= max(r[0] for r in rows) <= 0.1 and max(r[2] for r in rows) <= 0.1 and len(det) >= 6 and max(r[2] for r in det) <= 0.1
    print(f"1. round trip, the camera's pose taken exactly: {len(rows)} postures, max board-origin error "
          f"{max(r[0] for r in rows):.4f} mm, centre {max(r[2] for r in rows):.4f} mm, orientation {max(r[1] for r in rows):.6f} deg "
          f"(the state's rounding: 0.01 mm, 1e-6)")
    print(f"2. through a synthetic image + the board detector: {len(det)} / {len(rows)} seen, max centre error "
          f"{max(r[2] for r in det):.3f} mm, orientation {max(r[1] for r in det):.4f} deg")
    Tcb_true = Tcb

    # 3. the rules: joints 0.15 s from the board's frame -> posed; 0.25 s -> null; not seen -> null; no hand-eye -> null
    s15, s25, sns = feed(gw, q, Tcb_true, 0.15), feed(gw, q, Tcb_true, 0.25), feed(gw, q, Tcb_true, 0.01, seen=False)
    Tfc, gw.Tfc = gw.Tfc, None
    snh = feed(gw, q, Tcb_true)
    gw.Tfc = Tfc
    rule = (s15["pose_base"] is not None and s25["pose_base"] is None and s25["centre_base_mm"] is None
            and sns["pose_base"] is None and snh["pose_base"] is None and s15["handeye_at"] == gw.he_info.get("date"))
    ok &= rule
    print(f"3. rules: joints 0.15 s apart -> posed ({s15['joints_dt_s']} s), 0.25 s -> {s25['pose_base']}, not seen -> "
          f"{sns['pose_base']}, no hand-eye -> {snh['pose_base']}; handeye_at {s15['handeye_at']!r}")
    print("\nALL OK" if ok else "\nFAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
