#!/usr/bin/env python3
"""Offline proof of the absolute calibration (harness_calibration/absolute.py; PROJECT_SPEC.md "2026-11-16 — the absolute
calibration"): no ROS, no arm. A synthetic real arm = the PiPER's MDH table with KNOWN corrections (joint 4 -0.2°,
joint 5 -5°, the joint-5 axis twist -0.5°, joint 6's -0.1°) and a known hand-eye; five ChArUco boards spread in 3D
(flat on the table, raised, tilted, near-vertical). At each: the user's rough posture (the camera ~250 mm from the
board), then calib.handeye's 15 viewpoints commanded in the INSTALLED model (as the arm node's IK does), each reached
by the real arm, the 80 corners projected into a 1280 x 720 pinhole camera with 0.2 px noise (corners off the image
dropped), the pose solved by the board detector's own solver, the reported joints with 0.01° noise. Then
absolute.solve: (1) installed = no kinematic corrections + a hand-eye 2 mm / 0.8° off -> the fit recovers the truth,
improved; (2) installed = the truth -> improved false (nothing to install).

    python3 tools/absolute_calib_proof.py          (needs pyAgxArm's MDH table: the harness PC's ros-venv has it)
"""
import math
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "src", "harness_calibration"), os.path.join(ROOT, "src", "harness_vision"),
                os.path.join(ROOT, "src", "harness_arm")]
import yaml  # noqa: E402

from harness_arm.kinematics import PiperKinematics  # noqa: E402
from harness_calibration import absolute as A  # noqa: E402
from harness_calibration.handeye import viewpoints  # noqa: E402
from harness_vision.detect import Charuco  # noqa: E402
from harness_vision.geom import angle_between, inv  # noqa: E402

K = np.array([[645.0, 0, 640.0], [0, 645.0, 360.0], [0, 0, 1]])
W, H = 1280, 720
D = math.radians
TH_TRUE = np.array([0, 0, 0, D(-0.2), D(-5.0), 0])
AL_TRUE = np.array([0, 0, 0, 0, D(-0.5), D(-0.1)])
with open(os.path.join(ROOT, "calib", "handeye.yaml")) as f:
    X_TRUE = np.array(yaml.safe_load(f)["flange_to_camera"]["matrix"], float)
# five boards spread in 3D: found from postures the arm really holds (random joints inside the limits with margin, the
# camera looking downward-ish, the board 250 mm before it, tilted up to 30° to it, at least 12 of the 15 viewpoints
# reachable), the five most different ones (board centre, its normal, joint 1)
def frame_z(z, roll_deg):
    """A rotation whose z is z (normalised), x chosen and turned by roll about z."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x)
    Rm = np.column_stack([x, np.cross(z, x), z])
    return Rm @ R.from_euler("z", roll_deg, degrees=True).as_matrix()


def ik_any(kin, T, seeds):
    for s in seeds:
        q, pe, ae = kin.ik(T, s)
        if pe < 2e-4 and ae < 2e-3 and not kin.joints_outside(q):
            return q
    return None


def pick_postures(kin_true, kin_inst, rng, n=5, tries=4000):
    lo = np.radians([-70, 30, -150, -60, -50, -90])
    hi = np.radians([70, 120, -40, 60, 50, 90])
    cand = []
    for _ in range(tries):
        q = rng.uniform(lo, hi)
        Tc = kin_true.fk(q) @ X_TRUE
        z = Tc[:3, 2]
        c = Tc[:3, 3] + 0.25 * z
        r = math.hypot(c[0], c[1])
        if z[2] > -0.3 or not (0.2 < r < 0.5) or not (-0.05 < c[2] < 0.35):
            continue
        cand.append((q, c, z))
        if len(cand) >= 300:
            break
    picked = []
    while len(picked) < n and cand:
        def score(e):
            return min((np.linalg.norm(e[1] - p[1]) + 0.15 * np.linalg.norm(e[2] - p[2]) for p in picked), default=1.0)
        best = cand.pop(max(range(len(cand)), key=lambda i: score(cand[i])))
        T0 = kin_inst.fk(best[0])
        ok = sum(kin_inst.ik(Tv, best[0], margin=0.02)[1] < 5e-5 for _, Tv in viewpoints(T0, 0.04, D(15), D(20)))
        if ok >= 12:
            picked.append(best)
    return picked


def make_positions(kin_true, kin_inst, rng, ch):
    mdh = A.mdh_table()
    positions = []
    for k, (q0, c, z) in enumerate(pick_postures(kin_true, kin_inst, rng), start=1):
        Rb = frame_z(R.from_rotvec(rng.normal(0, 1, 3) / 3 * D(rng.uniform(5, 30))).as_matrix() @ z, rng.uniform(-180, 180))
        B = np.eye(4)
        B[:3, :3] = Rb
        B[:3, 3] = c - Rb @ ch.centre                                 # the board's centre 250 mm before the camera
        T0 = kin_inst.fk(q0)                                         # what the arm node reports at the user's posture
        samples, refused = [], 0
        for name, Tv in viewpoints(T0, 0.04, D(15), D(20)):
            q, pe, ae = kin_inst.ik(Tv, q0, margin=0.02)            # the arm node's IK (installed model)
            if pe > 5e-5 or ae > 2e-4:
                refused += 1
                continue
            Tcam = A.fk_batch(mdh, [q], TH_TRUE, AL_TRUE)[0] @ X_TRUE  # where the real camera is
            Tcb = inv(Tcam) @ B
            P = (Tcb[:3, :3] @ ch.obj.T + Tcb[:3, 3:4]).T
            uv = (K @ P.T).T
            uv = uv[:, :2] / uv[:, 2:] + rng.normal(0, 0.2, (len(P), 2))
            keep = (P[:, 2] > 0.05) & (uv[:, 0] > 5) & (uv[:, 0] < W - 5) & (uv[:, 1] > 5) & (uv[:, 1] < H - 5)
            ids = np.nonzero(keep)[0]
            m = ch.measure(ids, uv[ids], 49, lambda p: (np.asarray(p) - K[:2, 2]) / np.diag(K)[:2], K[0, 0])
            if not m["seen"] or m["corners"] < A.MIN_CORNERS:
                refused += 1
                continue
            samples.append({"name": name, "joints_rad": list(np.asarray(q) + rng.normal(0, D(0.01), 6)),
                            "cam_board": m["T"].tolist(), "corners": m["corners"]})
        positions.append({"position": k, "board": {"squares": [9, 11], "square_m": 0.022}, "samples": samples})
        print(f"  board {k}: centre {np.round(c * 1000).tolist()} mm, tilt {math.degrees(math.acos(min(1, -Rb[2, 2]))):.0f}° "
              f"from level, joints {np.round(np.degrees(q0)).tolist()}: {len(samples)} samples ({refused} viewpoints "
              f"refused / board short), corners {min(s['corners'] for s in samples)}-{max(s['corners'] for s in samples)}")
    return positions


def report(label, res, truth=True):
    print(f"\n{label}: improved = {res['improved']}" + (f" ({res['why']})" if res.get("why") else ""))
    if not res.get("ok"):
        return None, None, None
    print(f"   {res['n_samples']} samples, positions {res['positions']}, params {'+'.join(res['params'])}")
    print(f"   rms {res['rms_before_mm']:.3f} -> {res['rms_after_mm']:.3f} mm (max {res['max_after_mm']:.3f})")
    print(f"   left out, installed model : {res['leave_one_out_before_mm']} mm (mean {np.mean(res['leave_one_out_before_mm']):.3f})")
    print(f"   left out, fitted model    : {res['leave_one_out_mm']} mm (mean {np.mean(res['leave_one_out_mm']):.3f})")
    print("   tested: " + ", ".join(f"{'+'.join(t['params'][4:]) or 'base'} {t['leave_one_out_mm']}" for t in res["tested"]))
    th, al = np.array(res["joint_offsets_rad"]), np.array(res["alpha_offsets_rad"])
    X = np.array(res["handeye"]["matrix"])
    if truth:
        print(f"   joint offsets j4 {math.degrees(th[3]):+.3f}° (true -0.200), j5 {math.degrees(th[4]):+.3f}° (true -5.000); "
              f"twists a5 {math.degrees(al[4]):+.3f}° (true -0.500), a6 {math.degrees(al[5]):+.3f}° (true -0.100)")
        print(f"   hand-eye off the truth by {np.linalg.norm(X[:3, 3] - X_TRUE[:3, 3]) * 1000:.3f} mm / "
              f"{math.degrees(angle_between(X, X_TRUE)):.3f}°")
    return th, al, X


def gateway_path(pos_err, pos_ok, X0):
    import json
    import shutil
    import tempfile
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import gateway_server as G
    os.environ["PYTHONPATH"] = os.pathsep.join([os.path.join(ROOT, "src", "harness_vision"), os.environ.get("PYTHONPATH", "")])
    T = tempfile.mkdtemp(prefix="absolute_proof_")
    G.ABS_DIR, G.BACKUP_DIR = os.path.join(T, "absolute"), os.path.join(T, "backups")
    G.KIN_FILE, G.HANDEYE_FILE = os.path.join(T, "kinematics.yaml"), os.path.join(T, "handeye.yaml")
    os.makedirs(G.ABS_DIR)
    with open(G.KIN_FILE, "w") as f:
        yaml.safe_dump({"joint_zero_offsets_rad": [0.0] * 6, "alpha_offsets_rad": [0.0] * 6, "date": "K0"}, f)
    with open(G.HANDEYE_FILE, "w") as f:
        yaml.safe_dump({"flange_to_camera": {"matrix": X0.tolist()}, "date": "H0"}, f)

    class GW:
        he_info, _abs_cache, reloads = {"date": "H0"}, None, 0

        def reload_model(self):
            with open(G.HANDEYE_FILE) as f:
                self.he_info = {"date": yaml.safe_load(f).get("date")}
            self.reloads += 1
    G.gw = GW()
    restarted = []
    G.nodes_restart = lambda want: restarted.append(want) or {"ok": True, "restarted": [want]}
    good = True
    try:
        sid = G.absolute_begin()["session"]
        d = G.abs_current()
        for p in pos_err:
            A.write_json(A.position_path(d, p["position"]), dict(p, complete=True, rejected=[], at="2026-11-17T00:00:00"))
        st = G.absolute_status()
        t1 = time.time()
        r = G.absolute_job("solve", type("E", (), {"is_set": lambda self: False})(), d)
        good &= r["improved"] and r["loo"][0].keys() >= {"position", "error_mm"} and set(r["offsets"]) == {"joint_deg", "alpha_deg"}
        print(f"\n3. gateway: begin -> session {sid}; status positions {[(x['position'], x['samples']) for x in st['positions']]}; "
              f"solve job {time.time() - t1:.1f} s -> improved {r['improved']}, rms {r['rms_before_mm']} -> {r['rms_after_mm']} mm, "
              f"loo {[x['error_mm'] for x in r['loo']]}")
        print(f"   offsets {r['offsets']}; handeye {r['handeye']}, change {r['handeye_change']}")
        inst = G.absolute_install()
        with open(G.KIN_FILE) as f:
            kin = yaml.safe_load(f)
        with open(G.HANDEYE_FILE) as f:
            he = yaml.safe_load(f)
        good &= (G.gw.reloads == 1 and restarted == ["piper"] and len(inst["backups"]) == 2 and abs(kin["joint_zero_offsets_deg"][4] + 5) < 0.05
                 and np.allclose(he["flange_to_camera"]["matrix"], json.load(open(os.path.join(d, "solve.json")))["handeye"]["matrix"]))
        print(f"   install -> {inst['installed_at']}: kinematics j5 {kin['joint_zero_offsets_deg'][4]:+.3f}°, a5 {kin['alpha_offsets_deg'][4]:+.3f}°; "
              f"handeye.yaml written; backups {[os.path.basename(b) for b in inst['backups']]}; model re-read {G.gw.reloads}x; "
              f"arm node restart {restarted}; status installed_at {G.absolute_status()['installed_at']}")
        try:
            G.absolute_install()
            good = False
        except RuntimeError as e:
            print(f"   install again -> refused: {e}")
        G.absolute_begin()
        d2 = G.abs_current()
        for p in pos_ok:
            A.write_json(A.position_path(d2, p["position"]), dict(p, complete=True, rejected=[]))
        with open(G.KIN_FILE, "w") as f:
            yaml.safe_dump({"joint_zero_offsets_rad": [float(v) for v in TH_TRUE], "alpha_offsets_rad": [float(v) for v in AL_TRUE], "date": "K1"}, f)
        with open(G.HANDEYE_FILE, "w") as f:
            yaml.safe_dump({"flange_to_camera": {"matrix": X_TRUE.tolist()}, "date": "H1"}, f)
        G.gw.he_info = {"date": "H1"}
        r2 = G.absolute_job("solve", type("E", (), {"is_set": lambda self: False})(), d2)
        try:
            G.absolute_install()
            good = False
        except RuntimeError as e:
            print(f"   a solve that does not improve (the installed model = the truth): improved {r2['improved']}; install -> refused: {e}")
    finally:
        shutil.rmtree(T, ignore_errors=True)
    print(f"   gateway path {'OK' if good else 'FAILED'}")
    return good


def main():
    t0 = time.time()
    ch = Charuco()
    mdh = A.mdh_table()
    kin_true = PiperKinematics(mdh, TH_TRUE, alpha_offsets=AL_TRUE)
    ok = True

    # 1. the installed model has no kinematic corrections and a hand-eye 2 mm / 0.8° off
    X0 = X_TRUE.copy()
    X0[:3, :3] = R.from_euler("xyz", [0.5, -0.5, 0.4], degrees=True).as_matrix() @ X0[:3, :3]
    X0[:3, 3] += [0.0012, -0.0012, 0.001]
    kin_inst = PiperKinematics(mdh, np.zeros(6), alpha_offsets=np.zeros(6))
    print(f"case 1: installed = no corrections, hand-eye {np.linalg.norm(X0[:3, 3] - X_TRUE[:3, 3]) * 1000:.1f} mm / "
          f"{math.degrees(angle_between(X0, X_TRUE)):.2f}° off")
    pos = pos1 = make_positions(kin_true, kin_inst, np.random.default_rng(1), ch)
    res = A.solve(pos, {"joint_zero_offsets_rad": [0.0] * 6, "alpha_offsets_rad": [0.0] * 6}, X0, log=lambda s: None)
    th, al, X = report("case 1", res)
    if not res["ok"]:
        print("FAILED")
        sys.exit(1)
    err_off = max(abs(math.degrees(th[i] - TH_TRUE[i])) for i in (3, 4))
    ok &= res["improved"] and res["rms_after_mm"] < 0.2 * res["rms_before_mm"] and \
        np.mean(res["leave_one_out_mm"]) < 0.5 * np.mean(res["leave_one_out_before_mm"]) and err_off < 0.3 and \
        np.linalg.norm(X[:3, 3] - X_TRUE[:3, 3]) < 1.0e-3

    # 2. the installed model IS the truth: nothing to install
    kin_inst = PiperKinematics(mdh, TH_TRUE, alpha_offsets=AL_TRUE)
    print("\ncase 2: installed = the truth")
    pos = make_positions(kin_true, kin_inst, np.random.default_rng(2), ch)
    res2 = A.solve(pos, {"joint_zero_offsets_rad": list(TH_TRUE), "alpha_offsets_rad": list(AL_TRUE)}, X_TRUE, log=lambda s: None)
    report("case 2", res2)
    ok &= res2["ok"] and not res2["improved"]
    # 3. the gateway's calib.absolute on those samples: begin, the position files, solve (the real job process), status,
    #    install (backups, the files, the model re-read, the arm node restarted - stubbed), the refusals
    ok &= gateway_path(pos1, pos, X0)
    print(f"\n{'ALL OK' if ok else 'FAILED'} in {time.time() - t0:.0f} s")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
