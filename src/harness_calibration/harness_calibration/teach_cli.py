#!/usr/bin/env python3
"""Stage II teach CLI.

    ros2 run harness_calibration teach --drag-recog      # installation steps 1-3
    ros2 run harness_calibration teach --fine            # installation step 4 (all coarse-taught modules)
    ros2 run harness_calibration teach --fine --module super_bunny
    ros2 run harness_calibration teach --list            # what has been taught so far

--drag-recog: the arm is RELEASED (limp). The harness does not know which module
it will see: whenever a tag becomes SET (inside the central square, at the right
standoff, held --hold-s s, joints inside their limits) it looks the tag id up in
descriptors/*.yaml, records the arm pose + camera-frame tag pose to
calib/teach/<module>.yaml, and prints one line. Keep dragging to the next
module. Ctrl-C = "module setup finish": the arm is enabled and holds where it is.

--fine: for every coarse-taught module (or --module), the arm goes to the stored
joint configuration, takes --views viewpoints around it, chains
base->flange->camera->tag with calib/handeye.yaml, fuses, stores base->tag with
the spread as residual. Needs the arm enabled (it enables/holds first).
"""
import argparse
import datetime as dt
import glob
import math
import os
import sys
import time

import numpy as np
import yaml

from harness_calibration import registry
from harness_calibration.client import HarnessClient
from harness_vision.geom import T_from_xyzrpy, average_T, rot_about, spread, trans, xyzrpy_from_T

TEACH_DIR = os.path.expanduser("~/bunny-harness-dev/calib/teach")
HANDEYE = os.path.expanduser("~/bunny-harness-dev/calib/handeye.yaml")


def load_handeye(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        d = yaml.safe_load(f)
    return np.array(d["flange_to_camera"]["matrix"], float), d


def teach_file(module):
    return os.path.join(TEACH_DIR, f"{module}.yaml")


def load_teach(module):
    fn = teach_file(module)
    if not os.path.exists(fn):
        return None
    with open(fn) as f:
        return yaml.safe_load(f)


def save_teach(doc):
    os.makedirs(TEACH_DIR, exist_ok=True)
    with open(teach_file(doc["module"]), "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)


def limit_message(c, margin_deg=8.0):
    bad = c.joints_outside_limits(margin=math.radians(margin_deg))
    if not bad:
        return ""
    j, q, lo, hi = bad[0]
    return f"JOINT {j} AT {math.degrees(q):.0f} deg (limit {math.degrees(hi if q > 0 else lo):.0f}) - tilt it back"


def live_line(tags, W, H, lm, set_since, reg):
    if not tags:
        s = "no tag in view"
    else:
        t = tags[0]
        who = reg.get(t.id)
        name = f"{who['module']}/{who['role']}" if who else "UNKNOWN module"
        dx, dy = t.center_px[0] - W / 2, t.center_px[1] - H / 2
        s = f"tag {t.id} ({name}): dx {dx:+5.0f} dy {dy:+5.0f} px  size {t.size_px:4.0f} px  dist {t.distance_m*1000:4.0f} mm"
        if lm:
            s += "  " + lm
        elif t.in_center_square:
            s += f"  SET {time.time() - set_since:.1f}s" if set_since else "  SET"
        else:
            s += "  (bring it into the square)"
    return "\r" + s.ljust(110)


# ---------------------------------------------------------------------------- drag-recog
def drag_recog(c, a):
    reg = registry.load()
    W, H = c.tags.image_width, c.tags.image_height
    if not a.no_release:
        input("\n*** SUPPORT THE ARM. Press Enter to release it (it goes limp) *** ")
        r = c.trigger("disable")
        print("release:", r.message)
        if not r.success:
            sys.exit(1)
    print("\nknown tags: " + ", ".join(f"{i}={v['module']}/{v['role']}" for i, v in sorted(reg.items())))
    print("Drag the camera to each module's main tag. Square = central third of the image. Ctrl-C = setup finished (arm holds).\n")
    set_since, set_id, recorded = None, None, {}
    try:
        while True:
            m = c.tags if c.tags is not None and time.time() - c._tags_t < 0.5 else None
            tags = sorted(m.tags, key=lambda t: -t.size_px) if m else []
            lm = limit_message(c)
            t = tags[0] if tags else None
            if t is not None and t.in_center_square and not lm:
                if set_id != t.id:
                    set_since, set_id = time.time(), t.id
            else:
                set_since, set_id = None, None
            print(live_line(tags, W, H, lm, set_since, reg), end="", flush=True)
            if set_since and time.time() - set_since >= a.hold_s and t.id not in recorded:
                who = reg.get(t.id)
                module = who["module"] if who else f"unknown_tag_{t.id}"
                Ts, Tfs = c.collect_tag(t.id, n=a.frames)
                doc = load_teach(module) or {"module": module, "tag_id": t.id}
                doc.update({
                    "tag_id": t.id, "role": who["role"] if who else "unknown",
                    "date": dt.datetime.now().isoformat(timespec="seconds"),
                    "coarse": {
                        "joints_rad": [float(v) for v in c.joints],
                        "flange_xyzrpy": xyzrpy_from_T(average_T(Tfs)),
                        "cam_tag_xyzrpy": xyzrpy_from_T(average_T(Ts)),
                        "distance_mm": float(np.linalg.norm(average_T(Ts)[:3, 3]) * 1000),
                    },
                    "fine": None, "state": "coarse",
                })
                save_teach(doc)
                recorded[t.id] = module
                print(f"\nRECORDED tag {t.id} -> {module}{'' if who else '  (no descriptor: add one in descriptors/)'}: "
                      f"joints {np.round(np.degrees(c.joints), 1).tolist()} deg, tag at {doc['coarse']['distance_mm']:.0f} mm "
                      f"-> {teach_file(module)}")
                set_since, set_id = None, None
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print(f"\n\nsetup finished: {len(recorded)} module(s) recorded: {', '.join(recorded.values()) or 'none'}")
    r = c.trigger("enable")
    print("arm hold:", r.message)


def wait_camera_still(c, tag_id, settle, tol_mm=0.15, timeout=8.0):
    """Fixed settle, encoder stillness, then wait until the TAG stops moving in the camera:
    two consecutive 8-frame windows whose mean positions differ by < tol_mm. Returns (seconds, last shift mm)."""
    t0 = time.time()
    time.sleep(settle)
    c.wait_still()
    prev, shift = None, float("nan")
    while time.time() - t0 < settle + timeout:
        Ts, _ = c.collect_tag(tag_id, n=8, timeout=1.5)
        if len(Ts) < 4:
            prev = None
            continue
        cur = np.mean([T[:3, 3] for T in Ts], axis=0)
        if prev is not None:
            shift = float(np.linalg.norm(cur - prev) * 1000)
            if shift < tol_mm:
                break
        prev = cur
    return time.time() - t0, shift


# ---------------------------------------------------------------------------- fine
def fine(c, a):
    he = load_handeye(a.handeye)
    if he is None:
        print(f"no hand-eye calibration at {a.handeye}: run 'handeye' first")
        sys.exit(2)
    Tfc, hed = he
    files = [teach_file(a.module)] if a.module else sorted(glob.glob(os.path.join(TEACH_DIR, "*.yaml")))
    docs = []
    for fn in files:
        if os.path.exists(fn):
            with open(fn) as f:
                d = yaml.safe_load(f)
            if d.get("coarse"):
                docs.append(d)
    if not docs:
        print("nothing coarse-taught yet (run --drag-recog first)")
        sys.exit(2)
    if not c.status.all_enabled:
        r = c.trigger("enable")
        print("enable:", r.message)
        if not r.success:
            sys.exit(1)
    tilt = math.radians(a.tilt_deg)
    for doc in docs:
        tag_id, module = doc["tag_id"], doc["module"]
        print(f"\n== {module} (tag {tag_id})")
        # go back the way the arm left: a pose move to the taught flange pose (joint-interpolated by the
        # controller, no jump guard), then a small joint move to snap to the exact taught configuration
        cur = np.array(c.joints)
        if np.max(np.abs(cur - np.array(doc["coarse"]["joints_rad"]))) > 1.0:
            r = c.move_pose(T_from_xyzrpy(doc["coarse"]["flange_xyzrpy"]), speed=a.speed, timeout=40)
            print("  to taught flange pose:", r.message)
            if not r.success:
                continue
            time.sleep(0.5)
        r = c.move_j(doc["coarse"]["joints_rad"], speed=a.speed, timeout=40)
        print("  to coarse configuration:", r.message)
        if not r.success:
            continue
        wait_camera_still(c, tag_id, a.settle)
        T0 = c.tcp_T.copy()
        # extra views: lateral offset + counter-tilt, and ALWAYS a.lift higher than the hold pose
        # (the gripper may be just above the table at the taught pose; a tilt swings its tip down)
        cand = [([a.offset, 0, a.lift], [0, 1, 0], -tilt), ([-a.offset, 0, a.lift], [0, 1, 0], tilt),
                ([0, a.offset, a.lift], [1, 0, 0], tilt), ([0, -a.offset, a.lift], [1, 0, 0], -tilt)]
        vps = [("hold", T0)] + [(f"view{k+1}", trans(v) @ T0 @ rot_about(ax, ang))
                                for k, (v, ax, ang) in enumerate(cand[:max(0, a.views - 1)])]
        Tbt_all, used, rays = [], [], []
        for name, Tv in vps:
            print(f"  {name:6s}", end=" ", flush=True)
            if name != "hold":
                r = c.move_pose(Tv, speed=a.speed, timeout=25)
                if not r.success:
                    print("move failed:", r.message)
                    continue
                w, sh = wait_camera_still(c, tag_id, a.settle)
                print(f"(still after {w:.1f}s, last shift {sh:.2f} mm)", end=" ", flush=True)
            Ts, Tfs = c.collect_tag(tag_id, n=a.frames)
            if len(Ts) < 3:
                print("tag not seen - skipped")
                continue
            j_t, _ = spread(Ts)
            Tct = average_T(Ts)
            Tbc = average_T(Tfs) @ Tfc                      # base->camera
            Tbt = Tbc @ Tct
            Tbt_all.append(Tbt)
            rays.append((Tbc[:3, 3].copy(), Tbc[:3, :3] @ Tct[:3, 3]))   # camera position, camera->tag vector in base
            used.append({"view": name, "base_to_tag_xyz_mm": [float(v) for v in np.round(Tbt[:3, 3] * 1000, 2)],
                         "cam_dist_mm": float(np.linalg.norm(Tct[:3, 3]) * 1000), "n": len(Ts), "jitter_mm": float(j_t * 1000)})
            print(f"ok  base->tag xyz {np.round(Tbt[:3,3]*1000,1)} mm  cam dist {np.linalg.norm(Tct[:3,3])*1000:.0f} mm  jitter {j_t*1000:.2f} mm")
        r = c.move_j(doc["coarse"]["joints_rad"], speed=a.speed, timeout=40)
        print("  back to coarse configuration:", r.message)
        if len(Tbt_all) >= 2:
            Tm = average_T(Tbt_all)
            d_t, d_r = spread(Tbt_all, Tm)
            # self-check of the tag size: find the scale k on the camera->tag vectors that makes all
            # views agree (tag_i = cam_i + k * v_i). k != 1 means the descriptor size is wrong by 1/k.
            A = np.vstack([np.hstack([np.eye(3), -v.reshape(3, 1)]) for _, v in rays])
            bvec = np.concatenate([c_ for c_, _ in rays])
            sol, *_ = np.linalg.lstsq(A, bvec, rcond=None)
            k = float(sol[3])
            tag_k = sol[:3]
            resid_k = max(float(np.linalg.norm(c_ + k * v - tag_k)) for c_, v in rays)
            size_mm = registry.load().get(tag_id, {}).get("size_mm")
            implied = size_mm * k if size_mm else None
            print(f"  size check: scale k = {k:.3f} -> implied black square {implied:.1f} mm (descriptor {size_mm} mm); "
                  f"spread with that scale {resid_k*1000:.2f} mm" if implied else f"  size check: k = {k:.3f}, spread {resid_k*1000:.2f} mm")
            # informational only: over 20-25 mm baselines this fit is dominated by the ~2 mm hand-eye
            # error (it once claimed 35 mm for a 28 mm tag). Stereo depth (TagDetection.depth_size_m) is the size reference.
            doc["fine"] = {"base_to_tag_xyzrpy": xyzrpy_from_T(Tm), "base_to_tag_matrix": Tm.tolist(),
                           "residual_mm": float(d_t * 1000), "residual_deg": float(math.degrees(d_r)),
                           "views": used, "handeye_date": hed.get("date"),
                           "size_scale_k": k, "implied_size_mm": implied,
                           "date": dt.datetime.now().isoformat(timespec="seconds")}
            doc["state"] = "fine" if d_t * 1000 <= a.accept_mm else "failed"
            print(f"  fine: base->tag xyz {np.round(Tm[:3,3]*1000,1)} mm rpy {np.round(np.degrees(xyzrpy_from_T(Tm)[3:]),1)} deg; "
                  f"residual {d_t*1000:.2f} mm / {math.degrees(d_r):.2f} deg over {len(Tbt_all)} views -> {doc['state'].upper()}")
        else:
            doc["state"] = "failed"
            print("  fine pass failed (fewer than 2 usable views)")
        save_teach(doc)
        print("  written", teach_file(module))


def list_taught():
    files = sorted(glob.glob(os.path.join(TEACH_DIR, "*.yaml")))
    if not files:
        print("nothing taught yet")
    for fn in files:
        with open(fn) as f:
            d = yaml.safe_load(f)
        fine_s = ""
        if d.get("fine"):
            fine_s = (f"  base->tag {np.round(np.array(d['fine']['base_to_tag_xyzrpy'][:3])*1000).tolist()} mm"
                      f"  residual {d['fine']['residual_mm']:.2f} mm")
        print(f"{d['module']:16s} tag {d.get('tag_id'):3d}  state {d.get('state'):7s}  taught {d.get('date')}{fine_s}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--drag-recog", action="store_true")
    mode.add_argument("--fine", action="store_true")
    mode.add_argument("--list", action="store_true")
    ap.add_argument("--module", default=None, help="--fine: only this module")
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--no-release", action="store_true", help="--drag-recog: arm is already limp")
    ap.add_argument("--views", type=int, default=3)
    ap.add_argument("--offset", type=float, default=0.02)
    ap.add_argument("--lift", type=float, default=0.015, help="--fine: extra views are this much higher than the hold pose (m)")
    ap.add_argument("--tilt-deg", type=float, default=6.0)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--settle", type=float, default=3.0, help="--fine: wait this long after each move before measuring (the wrist-mounted camera keeps shaking after the joints stop)")
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--accept-mm", type=float, default=3.0, help="fine: max spread between views (hand-eye floor is ~2 mm)")
    ap.add_argument("--handeye", default=HANDEYE)
    a = ap.parse_args()
    if a.list:
        list_taught()
        return
    c = HarnessClient("teach")
    c.wait_ready()
    try:
        (drag_recog if a.drag_recog else fine)(c, a)
    finally:
        c.close()


if __name__ == "__main__":
    main()
