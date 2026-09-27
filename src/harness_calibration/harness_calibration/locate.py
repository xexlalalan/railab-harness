#!/usr/bin/env python3
"""Working-stage step 3: rough approach to a module's secondary tag, then fine localisation.

    ros2 run harness_calibration locate --module super_bunny --role platform --dry-run   # numbers only, no motion
    ros2 run harness_calibration locate --module super_bunny --role platform             # move, look, fine-locate

Predicts the tag position from the module's fine-localised MAIN tag and the
descriptor (module frame -> main tag frame -> base), places the camera at
--standoff along the viewing direction used when the main tag was taught,
reports every tag seen with its offset from the prediction, adopts the tag
(descriptor id, or --tag-id, or the only/nearest one seen and writes its id
into the descriptor), then takes --views small viewpoints, fuses base->tag,
and stores it under the role in calib/teach/<module>.yaml.

Safety: every motion is a move_pose through the arm node (joint-jump, workspace
and floor-clearance guards apply). NOTHING here knows about the Bunny's
windshield: the operator confirms it is open before running without --dry-run.
"""
import argparse
import datetime as dt
import math
import os
import sys
import time

import numpy as np
import yaml

from harness_calibration import registry
from harness_calibration.client import HarnessClient
from harness_calibration.teach_cli import HANDEYE, load_handeye, load_teach, save_teach
from harness_vision.geom import T_from_xyzrpy, average_T, inv, rot_about, spread, trans, xyzrpy_from_T


def T_main_module(desc):
    rpy = desc.get("frame", {}).get("from_main_tag_rpy_deg", [0, 0, 0])
    return T_from_xyzrpy([0, 0, 0] + [math.radians(v) for v in rpy])


def predict(desc, teach, role):
    """Predicted base-frame position of the tag with this role (+ its layout entry)."""
    entry = next((e for e in desc["tags"]["layout"] if e.get("role") == role), None)
    if entry is None:
        raise SystemExit(f"role '{role}' not in descriptor")
    if not teach or not teach.get("fine"):
        raise SystemExit("main tag not fine-localised yet (teach --fine)")
    T_base_main = np.array(teach["fine"]["base_to_tag_matrix"], float)
    p_module = np.array(entry["xyz_mm"], float) / 1000.0
    p_base = T_base_main @ T_main_module(desc) @ np.append(p_module, 1.0)
    return p_base[:3], entry, T_base_main


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--module", required=True)
    ap.add_argument("--role", required=True)
    ap.add_argument("--tag-id", type=int, default=None)
    ap.add_argument("--tip-clearance", type=float, default=0.04, help="gripper TIP to tag-plane distance at the view (m); the camera standoff is derived from it")
    ap.add_argument("--iters", type=int, default=2, help="re-approach from what was seen, up to this many times")
    ap.add_argument("--min-px", type=float, default=40.0, help="detections smaller than this are not accepted")
    ap.add_argument("--views", type=int, default=3)
    ap.add_argument("--tilt-deg", type=float, default=3.0)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--accept-mm", type=float, default=3.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--from-descriptor", action="store_true", help="ignore a previous measurement; predict from the descriptor only")
    ap.add_argument("--retreat", type=float, default=0.08, help="back off this far along the viewing axis when done (m)")
    ap.add_argument("--handeye", default=HANDEYE)
    a = ap.parse_args()

    desc, _ = registry.descriptor(a.module)
    teach = load_teach(a.module)
    he = load_handeye(a.handeye)
    if he is None:
        raise SystemExit("no hand-eye calibration")
    Tfc, _ = he
    tcp = 0.0
    fl_path = os.path.expanduser("~/bunny-harness-dev/calib/floor.yaml")
    if os.path.exists(fl_path):
        with open(fl_path) as f:
            tcp = float(yaml.safe_load(f).get("tcp_offset_m", 0.0))
    lead = float((np.array([0, 0, tcp]) - Tfc[:3, 3]) @ Tfc[:3, 2])      # tip ahead of the camera along the optical axis
    a.standoff = a.tip_clearance + max(lead, 0.0)
    print(f"gripper tip leads the camera by {lead*1000:.0f} mm -> camera standoff {a.standoff*1000:.0f} mm for {a.tip_clearance*1000:.0f} mm tip clearance")
    p_pred, entry, T_base_main = predict(desc, teach, a.role)
    prev = (teach.get("tags") or {}).get(a.role)
    if prev and prev.get("base_to_tag_xyzrpy") and not a.from_descriptor:
        p_prev = np.array(prev["base_to_tag_xyzrpy"][:3], float)
        print(f"  previous measurement of this tag ({prev.get('state')}, {prev.get('date')}): {np.round(p_prev*1000,1).tolist()} mm "
              f"-> used as the prediction ({np.linalg.norm(p_prev-p_pred)*1000:.0f} mm from the descriptor's)")
        p_pred = p_prev
    # viewing direction: the camera orientation used when the main tag was taught
    T_base_flange_teach = T_from_xyzrpy(teach["coarse"]["flange_xyzrpy"])
    R_cam = (T_base_flange_teach @ Tfc)[:3, :3]
    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = R_cam
    T_base_cam[:3, 3] = p_pred - a.standoff * R_cam[:, 2]        # back off along the optical axis
    T_flange = T_base_cam @ inv(Tfc)
    fl = xyzrpy_from_T(T_flange)
    print(f"{a.module}/{a.role}: descriptor offset {entry['xyz_mm']} mm (module frame), tag size {entry.get('size_mm')} mm")
    print(f"  predicted tag position (base): {np.round(p_pred*1000, 1).tolist()} mm  (main tag at {np.round(T_base_main[:3,3]*1000,1).tolist()})")
    print(f"  rough-view flange pose (base): xyz {np.round(np.array(fl[:3])*1000, 1).tolist()} mm  rpy {np.round(np.degrees(fl[3:]), 1).tolist()} deg")
    tag_px = entry.get("size_mm", 10) / 1000.0 / a.standoff * 651
    print(f"  expected tag size in the image at {a.standoff*1000:.0f} mm: {tag_px:.0f} px (detector floor ~50 px)")
    if a.dry_run:
        return

    c = HarnessClient("locate")
    c.wait_ready()
    cur = c.tcp_T[:3, 3]
    print(f"  flange now {np.round(cur*1000,1).tolist()} mm -> move of {np.linalg.norm(T_flange[:3,3]-cur)*1000:.0f} mm")
    if not c.status.all_enabled:
        raise SystemExit("arm is not enabled")
    def look(T_target, label):
        r = c.move_pose(T_target, speed=a.speed, timeout=40)
        print(f"  {label}:", r.message)
        if not r.success:
            return None
        time.sleep(1.0)
        c.wait_still()
        time.sleep(0.5)
        seen = {}
        t0 = time.time()
        while time.time() - t0 < 2.0:
            m = c.tags
            if m is not None and time.time() - c._tags_t < 0.5:
                for t in m.tags:
                    seen.setdefault(t.id, t)
            time.sleep(0.05)
        T_bc = c.tcp_T @ Tfc
        out = {}
        for tid_, t in seen.items():
            p_cam = np.array([t.pose.position.x, t.pose.position.y, t.pose.position.z])
            p_b = (T_bc @ np.append(p_cam, 1.0))[:3]
            out[tid_] = (p_b, float(np.linalg.norm(p_b - p_pred)), t)
            ratio = (t.depth_m / t.distance_m) if t.depth_m > 0 else float("nan")
            print(f"    seen tag {tid_}: {t.size_px:.0f} px, PnP {t.distance_m*1000:.0f} mm, depth {t.depth_m*1000:.0f} mm "
                  f"(size implied by depth {t.depth_size_m*1000:.1f} mm) -> base {np.round(p_b*1000,1).tolist()} mm, "
                  f"{out[tid_][1]*1000:.1f} mm from the prediction")
        return out

    tid = a.tag_id if a.tag_id is not None else entry.get("id")
    cands = look(T_flange, "rough approach") or {}
    for it in range(a.iters):
        if tid is None or tid not in cands:
            break
        t = cands[tid][2]
        if abs(t.distance_m - a.standoff) < 0.02 and t.size_px >= a.min_px:
            break
        p_seen = cands[tid][0]
        T_base_cam[:3, 3] = p_seen - a.standoff * R_cam[:, 2]
        T_flange = T_base_cam @ inv(Tfc)
        print(f"  re-approach {it+1}: tag was at {t.distance_m*1000:.0f} mm, moving to {a.standoff*1000:.0f} mm from where it was seen")
        new = look(T_flange, f"re-approach {it+1}")
        if not new:
            break
        cands = new
    if not cands:
        print("  no tag seen; check the descriptor offset / that the tag faces the camera")
        c.close()
        sys.exit(1)
    if tid is None or tid not in cands:
        tid = min(cands, key=lambda k: cands[k][1])
        if entry.get("id") is None:
            registry.set_tag_id(a.module, a.role, tid)
            print(f"  adopted tag {tid} as {a.module}/{a.role} (written to the descriptor)")
    print(f"  using tag {tid}: prediction error {cands[tid][1]*1000:.1f} mm")

    # fine pass: five-view CROSS around the rough view. Tilts are about the CAMERA axes (x = image
    # horizontal, y = image vertical), applied to the flange as T_flange @ Tfc @ Rot @ inv(Tfc), and
    # come in opposite pairs so the tilt-linked bias of a small tag cancels in each pair's mean.
    T0 = c.tcp_T.copy()
    tilt = math.radians(a.tilt_deg)

    def cam_tilt(axis, ang):
        return T0 @ Tfc @ rot_about(axis, ang) @ inv(Tfc)

    vps = [("centre", T0),
           ("cam_x+", cam_tilt([1, 0, 0], tilt)), ("cam_x-", cam_tilt([1, 0, 0], -tilt)),
           ("cam_y+", cam_tilt([0, 1, 0], tilt)), ("cam_y-", cam_tilt([0, 1, 0], -tilt))]
    Tbt = {}
    rays, used = [], []
    for name, Tv in vps:
        print(f"  {name:7s}", end=" ", flush=True)
        if name != "centre":
            r = c.move_pose(Tv, speed=a.speed, timeout=25)
            if not r.success:
                print("move failed:", r.message)
                continue
            time.sleep(1.0)
            c.wait_still()
        Ts, Tfs = c.collect_tag(tid, n=a.frames)
        if len(Ts) < 3:
            print("tag not seen - skipped")
            continue
        j_t, _ = spread(Ts)
        Tct, Tbc = average_T(Ts), average_T(Tfs) @ Tfc
        Tbt[name] = Tbc @ Tct
        rays.append((Tbc[:3, 3].copy(), Tbc[:3, :3] @ Tct[:3, 3]))
        used.append({"view": name, "base_to_tag_xyz_mm": [float(v) for v in np.round(Tbt[name][:3, 3] * 1000, 2)], "n": len(Ts), "jitter_mm": float(j_t * 1000)})
        print(f"ok  base->tag xyz {np.round(Tbt[name][:3,3]*1000,1)} mm  jitter {j_t*1000:.2f} mm")
    Tbt_all = list(Tbt.values())
    # pair means: each opposite pair averaged; estimates = centre + pair means
    estimates = {}
    if "centre" in Tbt:
        estimates["centre"] = Tbt["centre"]
    for ax in ("x", "y"):
        if f"cam_{ax}+" in Tbt and f"cam_{ax}-" in Tbt:
            estimates[f"pair_{ax}"] = average_T([Tbt[f"cam_{ax}+"], Tbt[f"cam_{ax}-"]])
    r = c.move_pose(T0, speed=a.speed, timeout=25)
    print("  back to the rough view:", r.message)
    Tr = T0.copy()
    Tr[:3, 3] = T0[:3, 3] - a.retreat * (T0[:3, :3] @ Tfc[:3, :3])[:, 2]
    r = c.move_pose(Tr, speed=a.speed, timeout=25)
    print(f"  retreat {a.retreat*1000:.0f} mm:", r.message)
    if len(estimates) < 2:
        print("  fine pass failed (fewer than 2 usable estimates)")
        c.close()
        sys.exit(1)
    Tm = average_T(list(estimates.values()))
    d_t, d_r = spread(list(estimates.values()), Tm)            # uncertainty = spread between centre and pair-means
    raw_t, _ = spread(Tbt_all, average_T(Tbt_all))              # diagnostic: single-view spread
    E = np.array([T[:3, 3] for T in estimates.values()]) * 1000
    per_axis = np.ptp(E, axis=0)
    print(f"  estimates: " + ", ".join(f"{k} {np.round(v[:3,3]*1000,1).tolist()}" for k, v in estimates.items()))
    print(f"  spread between estimates {d_t*1000:.2f} mm (per axis {np.round(per_axis,1).tolist()}); single-view spread {raw_t*1000:.2f} mm")
    A = np.vstack([np.hstack([np.eye(3), -v.reshape(3, 1)]) for _, v in rays])
    sol, *_ = np.linalg.lstsq(A, np.concatenate([p for p, _ in rays]), rcond=None)
    k = float(sol[3])
    implied = entry.get("size_mm") * k if entry.get("size_mm") else None
    print(f"  size check (informational, degenerate for pure-rotation views): k = {k:.3f}")
    px = cands[tid][2].size_px
    reasons = []
    if d_t * 1000 > a.accept_mm:
        reasons.append(f"spread {d_t*1000:.1f} mm > {a.accept_mm} mm")
    if len(Tbt_all) < 4:
        reasons.append(f"only {len(Tbt_all)} views")
    if px < a.min_px:
        reasons.append(f"tag only {px:.0f} px (< {a.min_px:.0f})")
    # (the size self-check is informational only: the cross pattern has no translation baseline, so k is degenerate)
    state = "fine" if not reasons else "failed"
    if reasons:
        print("  NOT accepted: " + "; ".join(reasons))
    teach.setdefault("tags", {})[a.role] = {
        "tag_id": int(tid), "base_to_tag_xyzrpy": xyzrpy_from_T(Tm), "base_to_tag_matrix": Tm.tolist(),
        "residual_mm": float(d_t * 1000), "residual_deg": float(math.degrees(d_r)),
        "predicted_xyz_mm": [float(v) for v in np.round(p_pred * 1000, 1)],
        "prediction_error_mm": float(np.linalg.norm(Tm[:3, 3] - p_pred) * 1000),
        "size_scale_k": k, "implied_size_mm": implied, "views": used, "state": state,
        "estimates_xyz_mm": {k_: [float(v) for v in np.round(T[:3, 3] * 1000, 2)] for k_, T in estimates.items()},
        "uncertainty_per_axis_mm": [float(v) for v in np.round(per_axis, 2)], "single_view_spread_mm": float(raw_t * 1000),
        "rough_view_flange_xyzrpy": xyzrpy_from_T(T0), "date": dt.datetime.now().isoformat(timespec="seconds"),
        "size_px": float(px), "depth_m": float(cands[tid][2].depth_m), "depth_size_mm": float(cands[tid][2].depth_size_m * 1000),
        "reject_reasons": reasons,
    }
    save_teach(teach)
    print(f"\n  {a.role}: base->tag xyz {np.round(Tm[:3,3]*1000,1)} mm rpy {np.round(np.degrees(xyzrpy_from_T(Tm)[3:]),1)} deg; "
          f"uncertainty {d_t*1000:.2f} mm / {math.degrees(d_r):.2f} deg (per axis {np.round(per_axis,1).tolist()}) from {len(Tbt_all)} views; "
          f"prediction was off by {np.linalg.norm(Tm[:3,3]-p_pred)*1000:.1f} mm -> {state.upper()}")
    c.close()
    sys.exit(0 if state == "fine" else 1)


if __name__ == "__main__":
    main()
