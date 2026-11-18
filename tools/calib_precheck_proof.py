#!/usr/bin/env python3
"""Offline proof of the calibrations' start-posture check and the 15 viewpoints (harness_calibration/precheck.py,
handeye.VIEW_SET; PROJECT_SPEC.md "2026-11-17 — the start-posture check"): no ROS, no arm. The repo's arm model
(calib/kinematics.yaml) and the arm node's parameters (src/harness_arm/config/piper.yaml).

  1. a held posture with the wrist centred, the camera looking down -> 15 of 15 views reachable, ok
  2. the same posture with joint 4 at -98.8° (the PC's runs 2 / 3) -> refused, in plain words
  3. joint 4 at -92° (outside the routines' 1.7° margin, but tight) -> what the views say
  4. over random held postures: how many views are reachable (the 15 vs the 23)
  5. the gateway: the refusal words a job reports (last_error), calib_precheck in the state (stubbed gateway)
  6. the hand-eye solve with the 15 views on synthetic images (test_charuco.handeye_proof)

    python3 tools/calib_precheck_proof.py           (needs pyAgxArm's MDH table: the harness PC's ros-venv has it)
"""
import math
import os
import sys
import types

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "src", "harness_calibration"),
                os.path.join(ROOT, "src", "harness_vision"), os.path.join(ROOT, "src", "harness_arm"),
                os.path.join(ROOT, "src", "harness_vision", "test")]
from harness_arm.kinematics import PiperKinematics  # noqa: E402
from harness_calibration import precheck as PC  # noqa: E402
from harness_calibration.handeye import viewpoints  # noqa: E402

D = math.radians


def main():
    with open(os.path.join(ROOT, "src", "harness_arm", "config", "piper.yaml")) as f:
        P = yaml.safe_load(f)["piper"]["ros__parameters"]
    kin = PiperKinematics.from_file(os.path.join(ROOT, "calib", "kinematics.yaml"))
    with open(os.path.join(ROOT, "calib", "handeye.yaml")) as f:
        X = np.array(yaml.safe_load(f)["flange_to_camera"]["matrix"], float)
    ok = True
    rng = np.random.default_rng(3)
    lo = np.array([l[0] for l in kin.limits])
    hi = np.array([l[1] for l in kin.limits])

    def looks_down(q):
        Tc = kin.fk(q) @ X
        z = Tc[:3, 2]
        c = Tc[:3, 3] + 0.25 * z
        return z[2] < -0.5 and 0.2 < math.hypot(c[0], c[1]) < 0.5 and -0.05 < c[2] < 0.3

    # 1. a centred wrist (joint 4 within ±10°, joint 5 / 6 away from their limits) looking down
    q_c = None
    for _ in range(20000):
        q = rng.uniform(lo + 0.1, hi - 0.1)
        q[3] = rng.uniform(D(-10), D(10))
        q[4] = rng.uniform(D(25), D(55))
        q[5] = rng.uniform(D(-60), D(60))
        if looks_down(q):
            pc = PC.precheck(kin, q, P)
            if pc["reachable"] == pc["of"]:
                q_c = q
                break
    pc = PC.precheck(kin, q_c, P)
    ok &= pc["ok"] and pc["reachable"] == 15 == pc["of"]
    print(f"1. centred wrist, joints {np.round(np.degrees(q_c), 1).tolist()} deg: ok {pc['ok']}, {pc['reachable']} of {pc['of']} "
          f"views reachable; margins " + ", ".join(f"j{j['j']} {j['margin']}°" for j in pc["joints"]))

    # 2. joint 4 at -98.8°
    q = q_c.copy()
    q[3] = D(-98.8)
    pc = PC.precheck(kin, q, P)
    want = "turn the wrist (joint 4) toward the middle: -98.8° of ±100°"
    ok &= (not pc["ok"]) and pc["why"] == want
    print(f"2. joint 4 at -98.8°: ok {pc['ok']}, refused: \"{pc['why']}\" ({pc['reachable']} of {pc['of']} reachable anyway)")

    # 3. joint 4 at -92°
    q[3] = D(-92.0)
    pc = PC.precheck(kin, q, P)
    print(f"3. joint 4 at -92°: ok {pc['ok']}, {pc['reachable']} of {pc['of']} reachable" + (f"; \"{pc['why']}\"" if pc["why"] else "")
          + "; unreachable: " + ", ".join(f"{u['view']} ({u['why']})" for u in pc["unreachable"]))

    # 4. random held postures looking down: reachable views, 15 vs 23
    n, n15, n23, okc = 0, [], [], 0
    while n < 120:
        q = rng.uniform(lo + 0.05, hi - 0.05)
        if not looks_down(q):
            continue
        n += 1
        pc = PC.precheck(kin, q, P)
        n15.append(pc["reachable"])
        okc += pc["ok"]
        n23.append(sum(not PC._view_why(kin, q, Tv, P)[0] for _, Tv in viewpoints(kin.fk(q), 0.04, D(15), D(20), full=True)))
    print(f"4. {n} random held postures looking down: the 15 views {np.mean(n15) / 15:.0%} reachable on average "
          f"(median {int(np.median(n15))} of 15), the 23 {np.mean(n23) / 23:.0%} (median {int(np.median(n23))} of 23); "
          f"the check passes {okc} ({okc / n:.0%}), refuses {n - okc}")

    # 5. the gateway: the words a job reports; calib_precheck in the state
    import gateway_server as G
    run2 = ["joint 4 = -98.8 deg is outside its limit [-100, 100] deg",
            "Cartesian moves are refused from here: move the arm inside its limits first (move_j), then retry"]
    now = ["start posture: 9 of 15 views reachable; joint margins j1 120°, j2 40°, j3 60°, j4 1°, j5 30°, j6 80°",
           "refused: " + want]
    print(f"5. a job's error = the routine's own words: old output -> \"{G.last_error(run2)}\"; now -> \"{G.last_error(now)}\"")
    ok &= G.last_error(now) == want
    gw = types.SimpleNamespace(kin=kin, P=P, last={}, Tfc=X)
    gw.precheck_target = lambda: None
    gw.joints_rad = lambda: list(q_c)
    gw.charuco_state = lambda: {"seen": True, "corners": 64}
    gw.fresh = lambda k, a: types.SimpleNamespace(all_enabled=True) if k == "arm" else None
    st = G.Gateway.update_precheck(gw, force=True)
    print(f"   state.calib_precheck (holding): ok {st['ok']}, {st['reachable']} of {st['of']}, board_seen {st['board_seen']}, "
          f"keys {sorted(st)}")
    gw.fresh = lambda k, a: types.SimpleNamespace(all_enabled=False) if k == "arm" else None
    st2 = G.Gateway.update_precheck(gw, force=True)
    print(f"   state.calib_precheck (not holding): ok {st2['ok']}, why \"{st2['why']}\", joints {len(st2['joints'])}")
    ok &= st["ok"] and st["of"] == 15 and not st2["ok"]

    # 6. the hand-eye with the 15 views (synthetic images, the board detector)
    import test_charuco as TC
    from harness_vision.detect import Charuco
    for label, kw in (("exact flange poses", {}), ("the arm's error 1 mm / 0.1°", {"flange_noise_mm": 1.0, "flange_noise_deg": 0.1})):
        r = TC.handeye_proof(Charuco(), **kw)
        print(f"6. hand-eye with the 15 views, {label}: {r['views']} / {r['of']} posed; flange -> camera off by "
              f"{r['x_mm']:.3f} mm / {r['x_deg']:.3f}°, rms {r['rms_mm']:.2f} mm")
        ok &= r["views"] == 15 and (r["x_mm"] < 0.5 if not kw else r["x_mm"] < 3.0)
    print("\nALL OK" if ok else "\nFAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
