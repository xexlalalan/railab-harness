#!/usr/bin/env python3
"""Absolute calibration (2026-11-16, operator: "标定那一块，再加一个5个不同位置的标定板，来做绝对位置标定" · "板子是在3d空间中
随机摆放的" · "每个位置我只粗糙的拖拽到那里，然后你用不同的位姿去精确标定"): the ChArUco board fixed at up to five places
anywhere in 3D; at each, the arm dragged roughly near it and holding; the routine moves the arm through calib.handeye's
own viewpoints around that posture and records synchronised samples (the reported joints + the board's pose and corners
in the camera). Then ONE joint fit over every position: each position's board pose in the arm base, the hand-eye
(gripper base -> camera) and the September model's kinematic corrections (calib/kinematics.yaml: joint-4 / joint-5 zero
offsets, the joint-5 / joint-6 axis twists; PROJECT_SPEC.md 2026-09-18 / 19) — one posture alone cannot separate joint
offsets from camera tilt, five spread postures can. More parameters (shoulder / elbow zeros, other twists) only when
they lower the left-out prediction (the September rule).

THE SCORE = LEAVE ONE POSITION OUT: the model fitted on the other positions, then only the left-out board's pose fitted
to its samples; its residual is that position's prediction error. "Before" = the same with the INSTALLED model
(calib/kinematics.yaml + calib/handeye.yaml, only the board poses fitted). improved = the mean left-out error drops by
at least IMPROVE_REL and IMPROVE_ABS_MM: only then may the result be installed.

Residuals: per sample, five board points (its centre and the four outermost inner corners) where the camera puts them
(corrected FK of the joints x hand-eye x camera -> board) against where the fitted board pose puts them, in the arm base,
mm; rms = over every point of every sample.

    python3 -m harness_calibration.absolute collect --dir calib/absolute/<session> --position 3     (moves the arm)
    python3 -m harness_calibration.absolute solve --dir calib/absolute/<session> [--kin ...] [--handeye ...]
Files in the session folder: position_<k>.json (rewritten after every sample), solve.json.
"""
import argparse
import datetime as dt
import json
import math
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from harness_vision.geom import average_T, spread, xyzrpy_from_T

PARAM_INDEX = {f"th{i + 1}": ("th", i) for i in range(6)} | {f"al{i + 1}": ("al", i) for i in range(6)}
BASE_PARAMS = ("th4", "th5", "al5", "al6")          # the September model (2026-09-19: nothing beat it on left-out groups)
EXTRA_PARAMS = ("th2", "th3", "al2", "al3", "al4")  # tried, adopted only when the left-out prediction drops
EXTRA_GAIN = 0.03                                   # ... by at least this share
MIN_CORNERS = 20
MIN_SAMPLES = 8                                     # per position, for the solve
MIN_POSITIONS = 3
IMPROVE_REL, IMPROVE_ABS_MM = 0.05, 0.05
JITTER_MM, JITTER_DEG = 2.0, 1.0                    # calib.handeye's stillness of the detections at one viewpoint


# ------------------------------------------------------------------------------------------------ the model
def mdh_table(model="piper"):
    from pyAgxArm.api.constants import ROBOT_MDH_PRESET
    return [tuple(float(v) for v in row) for row in ROBOT_MDH_PRESET[model]]


def fk_batch(mdh, Q, th, al):
    """Corrected FK (harness_arm.kinematics.PiperKinematics.fk) of N joint vectors at once -> (N, 4, 4)."""
    Q = np.atleast_2d(np.asarray(Q, float))
    N = len(Q)
    T = np.tile(np.eye(4), (N, 1, 1))
    for i, (d, a, alpha, off) in enumerate(mdh):
        t = Q[:, i] + off + th[i]
        A = alpha + al[i]
        ca, sa, ct, st = math.cos(A), math.sin(A), np.cos(t), np.sin(t)
        L = np.zeros((N, 4, 4))
        L[:, 0, 0], L[:, 0, 1], L[:, 0, 3] = ct, -st, a
        L[:, 1, 0], L[:, 1, 1], L[:, 1, 2], L[:, 1, 3] = ca * st, ca * ct, -sa, -sa * d
        L[:, 2, 0], L[:, 2, 1], L[:, 2, 2], L[:, 2, 3] = sa * st, sa * ct, ca, ca * d
        L[:, 3, 3] = 1.0
        T = T @ L
    return T


def T_vec(v):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:6]
    return T


def vec_T(T):
    return np.concatenate([R.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def board_points(board):
    """The five board points (board frame, m): the centre and the four outermost inner corners."""
    sx, sy = board["squares"]
    s = float(board["square_m"])
    return np.array([[sx * s / 2, sy * s / 2, 0], [s, s, 0], [(sx - 1) * s, s, 0], [s, (sy - 1) * s, 0],
                     [(sx - 1) * s, (sy - 1) * s, 0]], float)


class Data:
    """Every sample of the positions used: joints Q (N, 6), camera -> board M (N, 4, 4), position index (N,)."""

    def __init__(self, positions):
        self.positions = [p["position"] for p in positions]
        self.P = board_points(positions[0]["board"])
        Q, M, idx = [], [], []
        for j, p in enumerate(positions):
            for s in p["samples"]:
                Q.append(s["joints_rad"])
                M.append(np.array(s["cam_board"], float))
                idx.append(j)
        self.Q, self.M, self.idx = np.array(Q), np.array(M), np.array(idx)
        self.MP = np.einsum("nij,kj->nki", self.M[:, :3, :3], self.P) + self.M[:, None, :3, 3]   # (N, 5, 3) in the camera

    def sub(self, js):
        d = object.__new__(Data)
        keep = np.isin(self.idx, js)
        remap = {j: k for k, j in enumerate(js)}
        d.positions = [self.positions[j] for j in js]
        d.P, d.Q, d.M, d.MP = self.P, self.Q[keep], self.M[keep], self.MP[keep]
        d.idx = np.array([remap[j] for j in self.idx[keep]])
        return d


def residuals(data, mdh, X, th, al, B):
    """(N, 5, 3) mm: the five board points where the camera puts them minus where board pose B[idx] puts them (base)."""
    W = fk_batch(mdh, data.Q, th, al) @ X                                          # base -> camera
    meas = np.einsum("nij,nkj->nki", W[:, :3, :3], data.MP) + W[:, None, :3, 3]
    Bs = B[data.idx]
    board = np.einsum("nij,kj->nki", Bs[:, :3, :3], data.P) + Bs[:, None, :3, 3]
    return (meas - board) * 1000.0


def _angle(Ta, Tb):
    return float(np.linalg.norm(R.from_matrix(Ta[:3, :3].T @ Tb[:3, :3]).as_rotvec()))


def rms(r):
    return float(np.sqrt(np.mean(np.sum(np.asarray(r).reshape(-1, 3) ** 2, axis=1)))) if np.size(r) else float("nan")


def fit(data, mdh, X0, th0, al0, params=(), model_free=True, B0=None):
    """Least squares: the board poses (one per position) + (model_free) the hand-eye and the kinematic `params`.
    Returns (X, th, al, B (P, 4, 4), residuals (N, 5, 3))."""
    nP = len(data.positions)
    if B0 is None:                                         # each board where the starting model puts it on average
        W = fk_batch(mdh, data.Q, th0, al0) @ X0 @ data.M
        B0 = np.array([average_T(list(W[data.idx == j])) for j in range(nP)])
    nm = (6 + len(params)) if model_free else 0

    def unpack(x):
        X, th, al = X0, np.array(th0, float), np.array(al0, float)
        if model_free:
            X = T_vec(x[:6])
            for k, name in enumerate(params):
                kind, i = PARAM_INDEX[name]
                (th if kind == "th" else al)[i] = x[6 + k]
        B = np.array([T_vec(x[nm + 6 * j: nm + 6 * j + 6]) for j in range(nP)])
        return X, th, al, B

    x0 = []
    if model_free:
        x0 += list(vec_T(X0)) + [(th0 if PARAM_INDEX[n][0] == "th" else al0)[PARAM_INDEX[n][1]] for n in params]
    for j in range(nP):
        x0 += list(vec_T(B0[j]))
    s = least_squares(lambda x: residuals(data, mdh, *unpack(x)).ravel(), np.array(x0, float), method="lm", x_scale="jac",
                      max_nfev=4000)
    X, th, al, B = unpack(s.x)
    return X, th, al, B, residuals(data, mdh, X, th, al, B)


def per_position_rms(data, r):
    return [rms(r[data.idx == j]) for j in range(len(data.positions))]


def leave_one_out(data, mdh, X0, th0, al0, params):
    """Per position: the model fitted on the others, then only its board pose fitted to its samples -> its rms."""
    out = []
    for j in range(len(data.positions)):
        others = [k for k in range(len(data.positions)) if k != j]
        X, th, al, _, _ = fit(data.sub(others), mdh, X0, th0, al0, params)
        *_, r = fit(data.sub([j]), mdh, X, th, al, model_free=False)
        out.append(rms(r))
    return out


def solve(positions, kin_doc, X0, model="piper", log=print):
    """positions: the position files' contents (complete or not; each needs >= MIN_SAMPLES samples). kin_doc: the
    installed calib/kinematics.yaml; X0: the installed gripper base -> camera. Returns the result dict."""
    mdh = mdh_table(model)
    th0 = np.array(kin_doc.get("joint_zero_offsets_rad") or [0.0] * 6, float)
    al0 = np.array(kin_doc.get("alpha_offsets_rad") or [0.0] * 6, float)
    use = [p for p in positions if len(p.get("samples") or []) >= MIN_SAMPLES]
    res = {"ok": False, "at": dt.datetime.now().isoformat(timespec="seconds"), "n_samples": sum(len(p["samples"]) for p in use),
           "positions": [p["position"] for p in use], "improved": False}
    if len(use) < MIN_POSITIONS:
        res["why"] = (f"{len(use)} position(s) with >= {MIN_SAMPLES} samples: at least {MIN_POSITIONS} needed (5 asked)")
        return res
    data = Data(use)
    log(f"{res['n_samples']} samples over positions {data.positions}")
    *_, rb = fit(data, mdh, X0, th0, al0, model_free=False)
    before = per_position_rms(data, rb)                     # the installed model, each board's pose fitted to its samples
    log(f"installed model: rms {rms(rb):.2f} mm (per position {[round(v, 2) for v in before]})")
    params = list(BASE_PARAMS)
    lo = leave_one_out(data, mdh, X0, th0, al0, params)
    log(f"{'+'.join(params)}: left-out {np.mean(lo):.2f} mm {[round(v, 2) for v in lo]}")
    tested = [{"params": list(params), "leave_one_out_mm": round(float(np.mean(lo)), 3)}]
    best = None
    for c in EXTRA_PARAMS:
        lc = leave_one_out(data, mdh, X0, th0, al0, params + [c])
        tested.append({"params": params + [c], "leave_one_out_mm": round(float(np.mean(lc)), 3)})
        log(f"  + {c}: left-out {np.mean(lc):.2f} mm")
        if np.mean(lc) < np.mean(lo) * (1 - EXTRA_GAIN) and (best is None or np.mean(lc) < np.mean(best[1])):
            best = (c, lc)
    if best is not None:
        params, lo = params + [best[0]], best[1]
        log(f"adopted {best[0]}: left-out {np.mean(lo):.2f} mm")
    X, th, al, B, ra = fit(data, mdh, X0, th0, al0, params)
    after = per_position_rms(data, ra)
    x6 = xyzrpy_from_T(X)
    mb, ml = float(np.mean(before)), float(np.mean(lo))
    improved = bool(ml < mb * (1 - IMPROVE_REL) and mb - ml >= IMPROVE_ABS_MM)
    res.update(ok=True, params=params, rms_before_mm=round(rms(rb), 3), rms_after_mm=round(rms(ra), 3),
               max_after_mm=round(float(np.max(np.linalg.norm(ra.reshape(-1, 3), axis=1))), 3),
               leave_one_out_mm=[round(v, 3) for v in lo], leave_one_out_before_mm=[round(v, 3) for v in before],
               joint_offsets_deg=[round(math.degrees(v), 4) for v in th], alpha_offsets_deg=[round(math.degrees(v), 4) for v in al],
               joint_offsets_rad=[float(v) for v in th], alpha_offsets_rad=[float(v) for v in al],
               handeye={"xyz_mm": [round(v * 1000, 2) for v in x6[:3]], "rpy_deg": [round(math.degrees(v), 3) for v in x6[3:]],
                        "matrix": X.tolist()},
               handeye_change={"d_xyz_mm": [round(float(v) * 1000, 2) for v in X[:3, 3] - X0[:3, 3]],
                               "d_mm": round(float(np.linalg.norm(X[:3, 3] - X0[:3, 3])) * 1000, 2),
                               "d_rot_deg": round(math.degrees(_angle(X0, X)), 3)},
               offsets={"joint_deg": {f"j{i + 1}": round(math.degrees(v), 4) for i, v in enumerate(th)},
                        "alpha_deg": {f"a{i + 1}": round(math.degrees(v), 4) for i, v in enumerate(al)}},
               loo=[{"position": p, "error_mm": round(lo[j], 3), "before_mm": round(before[j], 3)}
                    for j, p in enumerate(data.positions)],
               per_position=[{"position": p, "samples": int(np.sum(data.idx == j)), "rms_before_mm": round(before[j], 3),
                              "rms_after_mm": round(after[j], 3), "leave_one_out_mm": round(lo[j], 3)}
                             for j, p in enumerate(data.positions)],
               board_base=[{"position": p, "matrix": B[j].tolist()} for j, p in enumerate(data.positions)],
               tested=tested, improved=improved)
    if not improved:
        res["why"] = (f"the left-out error does not drop enough ({mb:.2f} -> {ml:.2f} mm; needs >= {IMPROVE_REL:.0%} and "
                      f">= {IMPROVE_ABS_MM} mm): nothing to install")
    log(f"fit {'+'.join(params)} + hand-eye: rms {rms(rb):.2f} -> {rms(ra):.2f} mm; left-out {mb:.2f} -> {ml:.2f} mm; "
        f"improved {improved}")
    return res


# ------------------------------------------------------------------------------------------------ files
def read_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def write_json(path, doc):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f)
    os.replace(tmp, path)


def position_path(sdir, k):
    return os.path.join(sdir, f"position_{int(k)}.json")


def load_positions(sdir):
    out = []
    for k in range(1, 10):
        p = read_json(position_path(sdir, k))
        if p:
            out.append(p)
    return out


# ------------------------------------------------------------------------------------------------ collecting (ROS)
def collect(a):
    """One position: calib.handeye's viewpoints around where the arm holds; per viewpoint the stillness wait, then
    a.frames board detections (>= MIN_CORNERS corners) with the joints read with each; a viewpoint whose detections
    jitter more than JITTER_MM / JITTER_DEG, or whose move the arm node refuses, is skipped. Saved after every sample."""
    from harness_calibration.client import HarnessClient
    from harness_calibration.handeye import viewpoints
    from harness_vision.geom import T_from_pose_msg
    c = HarnessClient("absolute")
    c.wait_ready()
    st = c.status
    if not (st.ok and st.all_enabled):
        print(f"the arm must be holding (ok={st.ok} enabled={st.all_enabled} fault='{st.fault}')")
        sys.exit(2)
    if c.joints_outside_limits():
        print("a joint is outside its limit: move the arm inside its limits first")
        sys.exit(2)
    m = c.board(max_age=1.0)
    if m is None or m.corners < MIN_CORNERS:
        print(f"the ChArUco board is not in view with >= {MIN_CORNERS} corners ({m.corners if m else 0})")
        sys.exit(2)
    os.makedirs(a.dir, exist_ok=True)
    doc = {"position": a.position, "started": dt.datetime.now().isoformat(timespec="seconds"), "complete": False,
           "board": {"dictionary": m.dictionary, "squares": [int(m.squares_x), int(m.squares_y)], "square_m": float(m.square_m),
                     "marker_m": float(m.marker_m)},
           "start_joints_rad": list(c.joints), "samples": [], "rejected": []}
    path = position_path(a.dir, a.position)

    def save():
        doc["at"] = dt.datetime.now().isoformat(timespec="seconds")
        write_json(path, doc)
    save()
    print(f"position {a.position}: board {m.corners}/{m.corners_total} corners at {m.distance_m * 1000:.0f} mm")
    T0 = c.tcp_T.copy()
    vps = viewpoints(T0, a.trans, math.radians(a.tilt_deg), math.radians(a.roll_deg))
    try:
        for i, (name, Tv) in enumerate(vps):
            print(f"[{i + 1}/{len(vps)}] {name:12s}", end=" ", flush=True)
            r = c.move_pose(Tv, speed=a.speed, timeout=25)
            if not r.success and "EXCEEDS_LIMIT" in r.message:
                print("refused; half amplitude ->", end=" ", flush=True)
                r = c.move_pose(average_T([T0, Tv]), speed=a.speed, timeout=25)
            if not r.success:
                print("move refused:", r.message)
                doc["rejected"].append({"name": name, "why": "move: " + r.message})
                save()
                continue
            time.sleep(a.settle)
            ws = c.wait_still()
            frames, last, t0 = [], None, time.time()
            while len(frames) < a.frames and time.time() - t0 < 4.0:
                b = c.charuco
                if b is not None and b is not last:
                    last = b
                    if b.seen and b.corners >= MIN_CORNERS:
                        frames.append((T_from_pose_msg(b.pose), list(c.joints), b))
                time.sleep(0.01)
            if len(frames) < max(3, a.frames // 2):
                print(f"board not seen with >= {MIN_CORNERS} corners ({len(frames)} frames) - skipped")
                doc["rejected"].append({"name": name, "why": f"board seen on {len(frames)} frames"})
                save()
                continue
            Ts = [f[0] for f in frames]
            d_t, d_r = spread(Ts)
            if d_t * 1000 > JITTER_MM or math.degrees(d_r) > JITTER_DEG:
                print(f"unstable detection ({d_t * 1000:.1f} mm / {math.degrees(d_r):.1f} deg) - skipped")
                doc["rejected"].append({"name": name, "why": f"jitter {d_t * 1000:.1f} mm / {math.degrees(d_r):.1f} deg"})
                save()
                continue
            b = frames[-1][2]
            doc["samples"].append({"name": name, "joints_rad": [float(v) for v in np.mean([f[1] for f in frames], axis=0)],
                                   "cam_board": average_T(Ts).tolist(), "n": len(frames), "jitter_mm": round(d_t * 1000, 3),
                                   "corners": int(b.corners), "corner_ids": [int(v) for v in b.corner_ids],
                                   "corners_px": [round(float(v), 3) for v in b.corners_px], "still_s": round(a.settle + ws, 1)})
            save()
            print(f"ok  {b.corners} corners  dist {np.linalg.norm(Ts[-1][:3, 3]) * 1000:.0f} mm  jitter {d_t * 1000:.2f} mm")
        print("returning to the start posture")
        c.move_pose(T0, speed=a.speed, timeout=25)
    except KeyboardInterrupt:
        print("aborted: the samples so far are kept")
        save()
        sys.exit(1)
    doc["complete"] = True
    save()
    c.close()
    print(json.dumps({"position": a.position, "samples": len(doc["samples"]), "rejected": len(doc["rejected"])}))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=("collect", "solve"))
    ap.add_argument("--dir", required=True, help="the session folder (calib/absolute/<session>)")
    ap.add_argument("--position", type=int, default=1)
    ap.add_argument("--trans", type=float, default=0.04)
    ap.add_argument("--tilt-deg", type=float, default=15.0)
    ap.add_argument("--roll-deg", type=float, default=20.0)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--kin", default=os.path.expanduser("~/bunny-harness-dev/calib/kinematics.yaml"))
    ap.add_argument("--handeye", default=os.path.expanduser("~/bunny-harness-dev/calib/handeye.yaml"))
    a = ap.parse_args()
    if a.action == "collect":
        collect(a)
        return
    import yaml
    with open(a.kin) as f:
        kin = yaml.safe_load(f) or {}
    with open(a.handeye) as f:
        X0 = np.array((yaml.safe_load(f) or {})["flange_to_camera"]["matrix"], float)
    with open(a.handeye) as f:
        he_date = (yaml.safe_load(f) or {}).get("date")
    res = solve(load_positions(a.dir), kin, X0, log=lambda s: print(s, flush=True))
    res["session"] = os.path.basename(os.path.normpath(a.dir))
    res["from"] = {"kinematics_date": kin.get("date"), "handeye_date": he_date}    # the installed files it improves on
    write_json(os.path.join(a.dir, "solve.json"), res)
    print(json.dumps({k: res.get(k) for k in ("ok", "improved", "rms_before_mm", "rms_after_mm", "leave_one_out_mm", "why")}))


if __name__ == "__main__":
    main()
