"""Fine localisation of one tag from where the camera looks at it now (PROJECT_SPEC.md
"2026-10-23 — Release, hand-eye and tag teaching through the gateway").

The five-view cross of `locate` (2026-09-18): centre, then +-tilt about the CAMERA's
horizontal and vertical axes (applied to the gripper base as T @ Tfc @ Rot @ inv(Tfc), so the
camera turns about its own centre and the tag stays in view). Each opposite pair is averaged,
which cancels the tilt-linked bias of a small tag; the uncertainty is the spread between the
centre view and the two pair means. Ends back at the centre view.

    res = cross_locate(client, tag_id, Tfc, scale=black_true / black_assumed)

`client` is a harness_calibration.client.HarnessClient (arm holding, camera node running).
`scale` corrects a pose solved with the wrong tag size: the camera->tag translation is
multiplied by it (the detector's pose scales linearly with the size it assumes).
"""
import math
import time

import numpy as np

from harness_vision.geom import average_T, inv, rot_about, spread, trans

VIEWS = ("centre", "cam_x+", "cam_x-", "cam_y+", "cam_y-")


class Aborted(RuntimeError):
    """The caller's abort() said stop (dead-man, stop button): the arm was frozen by the caller."""


def backed_off(T_flange, Tfc, backoff_m):
    """The gripper base pose that puts the camera backoff_m further back along its optical axis
    (the same view of the tag, from further away)."""
    return T_flange @ Tfc @ trans([0.0, 0.0, -float(backoff_m)]) @ inv(Tfc)


def cross_targets(T0, Tfc, tilt_deg=3.0):
    """[(view name, gripper base pose)] of the five-view cross around the centre pose T0: the camera
    turned +-tilt about its own horizontal / vertical axis, about its own centre."""
    tilt = math.radians(tilt_deg)

    def cam_tilt(axis, ang):
        return T0 @ Tfc @ rot_about(axis, ang) @ inv(Tfc)
    return [("centre", T0), ("cam_x+", cam_tilt([1, 0, 0], tilt)), ("cam_x-", cam_tilt([1, 0, 0], -tilt)),
            ("cam_y+", cam_tilt([0, 1, 0], tilt)), ("cam_y-", cam_tilt([0, 1, 0], -tilt))]


def cross_locate(c, tag_id, Tfc, scale=1.0, tilt_deg=3.0, frames=20, speed=10, accept_mm=3.0,
                 min_px=40.0, abort=None, log=None, view_joints=None, family=None):
    """Returns {ok, reasons, T (4x4 base->tag, m), spread_mm, spread_deg, single_view_spread_mm,
    views [{view, base_mm, n, jitter_mm}], size_px, estimates}. Raises Aborted, RuntimeError.
    view_joints {view name: 6 joints (rad)}: go to exactly these postures (joint moves) instead of
    solving the tilted poses here — the postures a caller has already checked (limits, floor, obstacles).
    family (tag16h5 / tag36h11): only that family's tag tag_id counts (2026-10-25; None = any family)."""
    abort = abort or (lambda: False)
    say = log or (lambda m: None)

    def check():
        if abort():
            raise Aborted("aborted")

    def settle():
        time.sleep(1.0)
        c.wait_still()
        time.sleep(0.5)

    check()
    settle()
    t = c.tag(tag_id, family=family)
    if t is None:
        raise RuntimeError(f"tag {tag_id}{' (' + family[3:] + ')' if family else ''} is not in view from here")
    size_px = float(t.size_px)
    T0 = c.tcp_T.copy()
    q0 = list(c.joints)
    vps = cross_targets(T0, Tfc, tilt_deg)
    Tbt, views = {}, []
    for name, Tv in vps:
        check()
        if name != "centre":
            if view_joints and name in view_joints:
                r = c.move_j(view_joints[name], speed=speed, timeout=25)
            else:
                r = c.move_pose(Tv, speed=speed, timeout=25)
            check()
            if not r.success:
                say(f"{name}: move failed: {r.message}")
                continue
            settle()
        Ts, Tfs = c.collect_tag(tag_id, n=frames, family=family)
        if len(Ts) < 3:
            say(f"{name}: tag not seen - skipped")
            continue
        for T in Ts:
            T[:3, 3] *= scale
        j_t, _ = spread(Ts)
        Tbt[name] = average_T(Tfs) @ Tfc @ average_T(Ts)
        views.append({"view": name, "base_mm": [round(float(v) * 1000, 2) for v in Tbt[name][:3, 3]],
                      "n": len(Ts), "jitter_mm": round(float(j_t * 1000), 2)})
        say(f"{name}: base->tag {np.round(Tbt[name][:3, 3] * 1000, 1).tolist()} mm, jitter {j_t * 1000:.2f} mm")
    check()
    r = c.move_j(q0, speed=speed, timeout=25) if view_joints else c.move_pose(T0, speed=speed, timeout=25)
    if not r.success:
        say(f"back to the centre view: {r.message}")
    estimates = {}
    if "centre" in Tbt:
        estimates["centre"] = Tbt["centre"]
    for ax in ("x", "y"):
        if f"cam_{ax}+" in Tbt and f"cam_{ax}-" in Tbt:
            estimates[f"pair_{ax}"] = average_T([Tbt[f"cam_{ax}+"], Tbt[f"cam_{ax}-"]])
    if len(estimates) < 2:
        return {"ok": False, "reasons": [f"only {len(Tbt)} usable views"], "T": None, "views": views,
                "size_px": size_px, "spread_mm": None, "spread_deg": None}
    Tm = average_T(list(estimates.values()))
    d_t, d_r = spread(list(estimates.values()), Tm)
    raw_t, _ = spread(list(Tbt.values()), average_T(list(Tbt.values())))
    reasons = []
    if d_t * 1000 > accept_mm:
        reasons.append(f"spread {d_t * 1000:.1f} mm > {accept_mm} mm")
    if len(Tbt) < 4:
        reasons.append(f"only {len(Tbt)} views")
    if size_px < min_px:
        reasons.append(f"tag only {size_px:.0f} px (< {min_px:.0f})")
    return {"ok": not reasons, "reasons": reasons, "T": Tm, "spread_mm": round(float(d_t * 1000), 2),
            "spread_deg": round(float(math.degrees(d_r)), 3), "single_view_spread_mm": round(float(raw_t * 1000), 2),
            "views": views, "size_px": size_px,
            "estimates": {k: [round(float(v) * 1000, 2) for v in T[:3, 3]] for k, T in estimates.items()}}
