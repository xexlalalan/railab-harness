#!/usr/bin/env python3
"""Floor calibration: drag the gripper TIP over the floor/table while the arm is limp.

    ros2 run harness_calibration floor --seconds 90        # sample for 90 s (or Ctrl-C earlier), then fit
    ros2 run harness_calibration floor --tcp-offset 0.17   # gripper tip is 170 mm along flange z (if known)

Samples the flange pose while the tip stays in contact with the floor. Fits
  floor plane   z = a*x + b*y + c        (base frame; floor assumed near-horizontal)
  tip           tip_i = p_i + L * r3_i   (r3 = flange z axis; L = tip offset along it)
so that every tip lies on the plane. L is only observable if the wrist
orientation varied during the drag (>= ~10 deg); otherwise pass --tcp-offset or
the result is expressed for the flange at the orientation used.

Writes calib/floor.yaml and prints the parameters for the arm node
(floor_plane, tcp_offset_m, floor_margin_m) which refuse any pose target whose
tip would come within floor_margin_m (default 10 mm) of the plane.
"""
import argparse
import datetime as dt
import math
import os
import signal
import sys
import time

import numpy as np
import yaml
from scipy.optimize import least_squares

from harness_calibration.client import HarnessClient

OUT = os.path.expanduser("~/bunny-harness-dev/calib/floor.yaml")


def fit(P, R3, L_fixed=None):
    """P: Nx3 flange positions, R3: Nx3 flange z axes. Returns (a, b, c, L, residuals)."""
    def resid(x):
        a, b, c = x[:3]
        L = L_fixed if L_fixed is not None else x[3]
        tip = P + L * R3
        return tip[:, 2] - (a * tip[:, 0] + b * tip[:, 1] + c)
    x0 = [0.0, 0.0, float(np.median(P[:, 2]))] + ([] if L_fixed is not None else [0.0])
    r = least_squares(resid, x0, method="lm")
    a, b, c = r.x[:3]
    L = L_fixed if L_fixed is not None else float(r.x[3])
    return float(a), float(b), float(c), L, resid(r.x)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--tcp-offset", type=float, default=None, help="known tip offset along flange z (m); fitted if omitted")
    ap.add_argument("--min-step", type=float, default=0.005, help="new sample only after this much flange motion (m)")
    ap.add_argument("--margin", type=float, default=0.010, help="safe space above the floor (m)")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    c = HarnessClient("floor")
    c.wait_ready(camera=False)
    if c.status.all_enabled:
        print("arm is enabled; release it first (/arm/disable) so you can drag the tip over the floor")
        sys.exit(2)
    print(f"sampling for {a.seconds:.0f} s (Ctrl-C to stop early). Keep the gripper tip on the floor; vary the wrist tilt a little.")
    P, R3, JT = [], [], []
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))
    t0 = time.time()
    last = None
    while time.time() - t0 < a.seconds and not stop["now"]:
        T = c.tcp_T
        p = T[:3, 3].copy()
        if last is None or np.linalg.norm(p - last) >= a.min_step:
            P.append(p)
            R3.append(T[:3, 2].copy())
            JT.append(list(c.joints))
            last = p
            span = np.ptp(np.array(P)[:, :2], axis=0) * 1000 if len(P) > 1 else np.zeros(2)
            print(f"\r{len(P):4d} samples  xy span {span[0]:.0f} x {span[1]:.0f} mm  z {p[2]*1000:6.1f} mm   ", end="", flush=True)
        time.sleep(0.05)
    print()
    P, R3 = np.array(P), np.array(R3)
    if len(P) < 15:
        print(f"only {len(P)} samples; move the tip further (need >= 15 points over >= 100 mm)")
        sys.exit(1)
    span = np.ptp(P[:, :2], axis=0)
    tilt_spread = math.degrees(max(math.acos(min(1.0, float(np.dot(r, R3.mean(0) / np.linalg.norm(R3.mean(0)))))) for r in R3))
    print(f"{len(P)} samples, xy span {span[0]*1000:.0f} x {span[1]*1000:.0f} mm, wrist orientation spread {tilt_spread:.1f} deg")

    L_fixed = a.tcp_offset
    if L_fixed is None and tilt_spread < 8.0:
        print("wrist orientation barely varied: tip offset not observable -> fitting the floor for the FLANGE at this orientation (L = 0). "
              "Pass --tcp-offset <m> if you know the gripper length.")
        L_fixed = 0.0
    aa, bb, cc, L, res = fit(P, R3, L_fixed)
    rms, mx = float(np.sqrt(np.mean(res ** 2))), float(np.abs(res).max())
    tilt_deg = math.degrees(math.atan(math.hypot(aa, bb)))
    z_mean = float(np.mean(aa * P[:, 0] + bb * P[:, 1] + cc))
    print(f"floor plane: z = {aa:+.4f} x {bb:+.4f} y + {cc:.4f}   (tilt {tilt_deg:.2f} deg, z at the sampled area {z_mean*1000:.1f} mm)")
    print(f"tip offset along flange z: {L*1000:.1f} mm {'(fitted)' if a.tcp_offset is None and L_fixed is None else '(given/fixed)'}")
    print(f"fit residual: rms {rms*1000:.2f} mm, max {mx*1000:.2f} mm")
    ok = mx < 0.006
    doc = {
        "floor_plane": [aa, bb, cc], "plane_note": "base frame, z = a*x + b*y + c (m)",
        "tcp_offset_m": L, "tcp_offset_fitted": bool(a.tcp_offset is None and L_fixed is None),
        "floor_margin_m": a.margin, "residual_rms_mm": rms * 1000, "residual_max_mm": mx * 1000,
        "n_samples": int(len(P)), "xy_span_mm": [float(v * 1000) for v in span], "orientation_spread_deg": tilt_deg,
        "accepted": ok, "date": dt.datetime.now().isoformat(timespec="seconds"),
        "samples": {"flange_xyz": P.tolist(), "flange_z_axis": R3.tolist(), "joints": JT},
    }
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print("written", a.out, "->", "ACCEPTED" if ok else "NOT accepted (tip slipped off the floor, or the floor is not flat)")
    print("\narm node parameters (config/piper.yaml):")
    print(f"    floor_plane: [{aa:.5f}, {bb:.5f}, {cc:.5f}]\n    tcp_offset_m: {L:.4f}\n    floor_margin_m: {a.margin:.3f}")
    c.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
