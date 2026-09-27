#!/usr/bin/env python3
"""Hand-eye calibration: flange -> gripper camera (eye-in-hand).

Preconditions: piper node and tag node running; ONE tag fixed rigidly in the
workspace; the arm HOLDING at a start pose that sees the tag from ~150-250 mm.

    ros2 run harness_calibration handeye --tag-id 101 --tag-size 0.05
    ros2 run harness_calibration handeye --check-convention        # only the rpy round-trip test

The routine visits ~20 viewpoints around the start pose (translations +-trans m
in the base frame, tilts +-tilt deg about the flange x/y axes), records
(base->flange, camera->tag) pairs, solves with OpenCV's four hand-eye methods,
keeps the one whose fixed-tag reconstruction has the smallest spread, and
writes calib/handeye.yaml (+ raw pairs as JSON for re-solving).
"""
import argparse
import datetime as dt
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from harness_calibration.client import HarnessClient
from harness_vision.geom import angle_between, average_T, rot_about, spread, trans, xyzrpy_from_T

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def viewpoints(T0, trans_m, tilt_rad, roll_rad=math.radians(20)):
    """Start pose; small base-frame translations; small tilts about flange x/y; large rolls
    about flange z (the camera looks roughly along flange z, so a roll keeps the tag in view
    and costs almost no camera motion) combined with the tilts. Rotation diversity is what
    makes the hand-eye translation observable."""
    out = [("start", T0)]
    for ax, name in ((0, "x"), (1, "y"), (2, "z")):
        for s in (+1, -1):
            v = np.zeros(3)
            v[ax] = s * trans_m
            out.append((f"t{name}{'+' if s > 0 else '-'}", trans(v) @ T0))
    for ax, name in (([1, 0, 0], "rx"), ([0, 1, 0], "ry")):
        for s in (+1, -1):
            out.append((f"{name}{'+' if s > 0 else '-'}", T0 @ rot_about(ax, s * tilt_rad)))
    for s in (+1, -1):
        out.append((f"rz{s:+d}", T0 @ rot_about([0, 0, 1], s * roll_rad)))
    for sz in (+1, -1):
        for ax, name in (([1, 0, 0], "rx"), ([0, 1, 0], "ry")):
            for s in (+1, -1):
                out.append((f"rz{sz:+d}{name}{s:+d}", T0 @ rot_about([0, 0, 1], sz * roll_rad) @ rot_about(ax, s * tilt_rad)))
    for s in (+1, -1):
        v = np.zeros(3)
        v[2] = s * trans_m
        out.append((f"tz{s:+d}rz{-s:+d}", trans(v) @ T0 @ rot_about([0, 0, 1], -s * roll_rad)))
    return out


def solve(pairs):
    """pairs: list of (T_base_flange, T_cam_tag). Returns dict method -> (T_flange_cam, dt, dr)."""
    Rg = [p[0][:3, :3] for p in pairs]
    tg = [p[0][:3, 3] for p in pairs]
    Rt = [p[1][:3, :3] for p in pairs]
    tt = [p[1][:3, 3] for p in pairs]
    res = {}
    for name, m in METHODS.items():
        try:
            Rfc, tfc = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
        except cv2.error as e:  # noqa: PERF203
            print(f"  {name}: failed ({str(e).splitlines()[0]})")
            continue
        Tfc = np.eye(4)
        Tfc[:3, :3], Tfc[:3, 3] = Rfc, tfc.reshape(3)
        Tbt = [p[0] @ Tfc @ p[1] for p in pairs]          # fixed tag reconstructed from every pose
        d_t, d_r = spread(Tbt)
        res[name] = (Tfc, d_t, d_r)
        print(f"  {name:10s}: tag spread {d_t*1000:6.2f} mm / {math.degrees(d_r):5.2f} deg   "
              f"flange->cam xyz {tfc.reshape(3)*1000} mm")
    return res


def refine(pairs, X0, fit_scale=True):
    """Position-only refinement: tag *centre* positions are far more reliable than tag
    orientations (small/oblique tags flip). Unknowns: X = flange->cam (6), the fixed tag
    position p in base (3), and optionally a scale k on the camera->tag translations
    (k != 1 means the tag size assumed by the tag node is wrong by 1/k)."""
    from scipy.optimize import least_squares
    from harness_vision.geom import T_from_rvec_tvec
    ts = [p_[1][:3, 3] for p_ in pairs]
    Tb = [p_[0] for p_ in pairs]
    x0 = np.concatenate([R.from_matrix(X0[:3, :3]).as_rotvec(), X0[:3, 3],
                         np.mean([T @ X0 @ np.append(t, 1) for T, t in zip(Tb, ts)], axis=0)[:3], [0.0]])

    def resid(x):
        X = T_from_rvec_tvec(x[:3], x[3:6])
        k = math.exp(x[9]) if fit_scale else 1.0
        return np.concatenate([(T @ X @ np.append(k * t, 1))[:3] - x[6:9] for T, t in zip(Tb, ts)])

    r = least_squares(resid, x0, method="lm")
    X = T_from_rvec_tvec(r.x[:3], r.x[3:6])
    k = math.exp(r.x[9]) if fit_scale else 1.0
    e = resid(r.x).reshape(-1, 3)
    per = np.linalg.norm(e, axis=1)
    refine.last_per = per
    return X, k, float(np.sqrt(np.mean(per ** 2))), float(per.max())


def check_convention(c, speed):
    """Round-trip test of the assumed rpy convention: re-command the current pose, then a 10 deg wrist roll."""
    T0 = c.tcp_T.copy()
    print("convention check 1/2: re-command the current pose (should not move)")
    r = c.move_pose(T0, speed=speed, timeout=15)
    print("   ", r.success, r.message)
    if not r.success:
        return False
    T1 = T0 @ rot_about([0, 0, 1], math.radians(10))
    print("convention check 2/2: +10 deg about the flange z axis")
    r = c.move_pose(T1, speed=speed, timeout=20)
    print("   ", r.success, r.message)
    time.sleep(0.5)
    err_t = np.linalg.norm(c.tcp_T[:3, 3] - T1[:3, 3])
    err_r = angle_between(c.tcp_T, T1)
    print(f"    reported vs predicted: {err_t*1000:.2f} mm / {math.degrees(err_r):.2f} deg")
    ok = r.success and err_t < 0.005 and err_r < math.radians(2)
    print("    convention", "OK" if ok else "MISMATCH - do not trust pose moves until geom.py is fixed")
    c.move_pose(T0, speed=speed, timeout=20)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tag-id", type=int, default=None)
    ap.add_argument("--tag-size", type=float, default=None, help="edge length m (informational; the tag node sets the pose scale)")
    ap.add_argument("--trans", type=float, default=0.04, help="translation amplitude m")
    ap.add_argument("--tilt-deg", type=float, default=15.0)
    ap.add_argument("--frames", type=int, default=20, help="detections averaged per viewpoint")
    ap.add_argument("--roll-deg", type=float, default=20.0)
    ap.add_argument("--settle", type=float, default=1.0, help="fixed wait before the stillness check")
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--out", default=os.path.expanduser("~/bunny-harness-dev/calib/handeye.yaml"))
    ap.add_argument("--check-convention", action="store_true")
    ap.add_argument("--accept-mm", type=float, default=1.5)
    ap.add_argument("--accept-deg", type=float, default=0.7)
    ap.add_argument("--resolve", nargs="+", default=None, help="re-solve from saved *_pairs_*.json files (merged) instead of moving the arm")
    ap.add_argument("--rescale", type=float, default=1.0, help="--resolve: multiply the recorded camera->tag translations by this (true_size/recorded_size)")
    a = ap.parse_args()

    if a.resolve:
        pairs, log, d = [], [], {}
        for fn in a.resolve:
            with open(fn) as f:
                d = json.load(f)
            for e in d["pairs"]:
                Tct = np.array(e["cam_tag"], float)
                Tct[:3, 3] *= a.rescale
                pairs.append((np.array(e["base_flange"]), Tct))
            log += [dict(e) for e in d["pairs"]]
        print(f"re-solving {len(pairs)} pairs from {len(a.resolve)} file(s) (tag {d.get('tag_id')}, size {d.get('tag_size_m')} m)")
        res = solve(pairs)
        a.resolve = ",".join(os.path.basename(x) for x in a.resolve)
        finish(a, pairs, res, d.get("tag_id"), d.get("tag_size_m"), log)
        return

    c = HarnessClient("handeye")
    c.wait_ready(camera=not a.check_convention or a.tag_id is not None)
    st = c.status
    if not (st.ok and st.all_enabled):
        print(f"arm must be ok and enabled (holding). status ok={st.ok} enabled={st.all_enabled} errors={list(st.errors)} fault='{st.fault}'")
        sys.exit(2)

    bad = c.joints_outside_limits()
    if bad:
        for j, q, lo, hi in bad:
            print(f"joint {j} = {math.degrees(q):.1f} deg is outside its limit [{math.degrees(lo):.0f}, {math.degrees(hi):.0f}] deg")
        print("Cartesian moves are refused from here: move the arm inside its limits first (move_j), then retry")
        c.close()
        sys.exit(2)

    if a.check_convention:
        ok = check_convention(c, a.speed)
        c.close()
        sys.exit(0 if ok else 1)
    if a.tag_id is None:
        print("--tag-id required")
        sys.exit(2)

    t = c.tag(a.tag_id)
    if t is None:
        print(f"tag {a.tag_id} not visible from the start pose; drag the arm so it is, hold, retry")
        sys.exit(2)
    print(f"start: tag {a.tag_id} at {t.distance_m*1000:.0f} mm, {t.size_px:.0f} px, reproj {t.reproj_err_px:.2f} px")
    if not check_convention(c, a.speed):
        sys.exit(1)

    T0 = c.tcp_T.copy()
    vps = viewpoints(T0, a.trans, math.radians(a.tilt_deg), math.radians(a.roll_deg))
    pairs, log = [], []
    for i, (name, Tv) in enumerate(vps):
        print(f"[{i+1}/{len(vps)}] {name:12s}", end=" ", flush=True)
        r = c.move_pose(Tv, speed=a.speed, timeout=25)
        if not r.success and "EXCEEDS_LIMIT" in r.message:
            # joint limit / no IK there: retry at half amplitude around the start pose
            Th = average_T([T0, Tv])
            print("refused; half amplitude ->", end=" ", flush=True)
            r = c.move_pose(Th, speed=a.speed, timeout=25)
        if not r.success:
            print("move failed:", r.message)
            continue
        time.sleep(a.settle)                   # settle
        ws = c.wait_still()                    # then wait until the arm has really stopped creeping
        print(f"still after {a.settle + ws:.1f}s", end="  ", flush=True)
        Ts, Tfs = c.collect_tag(a.tag_id, n=a.frames)
        if len(Ts) < max(3, a.frames // 2):
            print(f"tag not seen ({len(Ts)} frames) - skipped")
            continue
        d_t, d_r = spread(Ts)
        if d_t > 0.002 or d_r > math.radians(1.0):
            print(f"unstable detection ({d_t*1000:.1f} mm / {math.degrees(d_r):.1f} deg) - skipped")
            continue
        Tct, Tbf = average_T(Ts), average_T(Tfs)
        pairs.append((Tbf, Tct))
        log.append({"name": name, "base_flange": Tbf.tolist(), "cam_tag": Tct.tolist(), "n": len(Ts)})
        print(f"ok  dist {np.linalg.norm(Tct[:3,3])*1000:.0f} mm  jitter {d_t*1000:.2f} mm")

    print("returning to start")
    c.move_pose(T0, speed=a.speed, timeout=25)
    if len(pairs) < 8:
        print(f"only {len(pairs)} usable poses - need >= 8. Increase standoff or reduce --tilt-deg/--trans.")
        sys.exit(1)

    print(f"solving with {len(pairs)} pairs:")
    res = solve(pairs)
    c.close()
    finish(a, pairs, res, a.tag_id, a.tag_size, log)


def finish(a, pairs, res, tag_id, tag_size, log):
    if not res:
        sys.exit(1)
    best = min(res, key=lambda k: res[k][1] + res[k][2] * 0.1)
    X0, d_t, d_r = res[best]
    print(f"closed-form best {best}: tag spread {d_t*1000:.2f} mm / {math.degrees(d_r):.2f} deg")
    X, k, rms, mx = refine(pairs, X0, fit_scale=True)
    Xn, _, rms_n, mx_n = refine(pairs, X0, fit_scale=False)
    print(f"position-only refinement, scale fitted : rms {rms*1000:.2f} mm, max {mx*1000:.2f} mm, k = {k:.4f} "
          f"(implied tag size {tag_size*k*1000 if tag_size else float('nan'):.1f} mm)")
    print(f"position-only refinement, scale fixed 1: rms {rms_n*1000:.2f} mm, max {mx_n*1000:.2f} mm")
    names = [e.get("name", str(i)) for i, e in enumerate(log)]
    X, k, rms, mx = refine(pairs, X0, fit_scale=True)   # recompute so last_per matches the scaled fit
    order = np.argsort(-refine.last_per)
    print("  worst poses (scaled fit):", ", ".join(f"{names[i]} {refine.last_per[i]*1000:.1f}mm" for i in order[:6]))
    use_scaled = abs(k - 1.0) > 0.02 and rms < 0.8 * rms_n
    Tfc, rms_used, mx_used = (X, rms, mx) if use_scaled else (Xn, rms_n, mx_n)
    ok = mx_used <= a.accept_mm / 1000
    ts = dt.datetime.now().isoformat(timespec="seconds")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    raw = a.out.replace(".yaml", f"_pairs_{ts.replace(':', '')}.json")
    if not a.resolve:
        with open(raw, "w") as f:
            json.dump({"tag_id": tag_id, "tag_size_m": tag_size, "pairs": log}, f)
    q = R.from_matrix(Tfc[:3, :3]).as_quat()
    doc = {
        "flange_to_camera": {
            "xyz_m": [float(v) for v in Tfc[:3, 3]],
            "quat_xyzw": [float(v) for v in q],
            "rpy_rad": xyzrpy_from_T(Tfc)[3:],
            "matrix": Tfc.tolist(),
        },
        "method": f"{best}+position_refine" + ("+scale" if use_scaled else ""),
        "n_poses": len(pairs),
        "residual_rms_mm": float(rms_used * 1000), "residual_max_mm": float(mx_used * 1000),
        "closed_form_spread_mm": float(d_t * 1000), "closed_form_spread_deg": float(math.degrees(d_r)),
        "tag_scale_k": float(k), "implied_tag_size_m": float(tag_size * k) if tag_size else None,
        "accepted": bool(ok), "tag_id": tag_id, "tag_size_m": tag_size,
        "date": ts, "raw_pairs": os.path.basename(a.resolve or raw),
        "note": "T_base_cam = T_base_flange @ flange_to_camera.matrix; camera optical frame x right, y down, z forward",
    }
    with open(a.out, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"\nresult: max position residual {mx_used*1000:.2f} mm (rms {rms_used*1000:.2f}) -> {'ACCEPTED' if ok else 'NOT accepted'}")
    print(f"flange->camera xyz {np.round(Tfc[:3,3]*1000,1)} mm, rpy {np.round(np.degrees(xyzrpy_from_T(Tfc)[3:]),1)} deg")
    print("written", a.out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
