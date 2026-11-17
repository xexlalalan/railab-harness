"""The calibrations' START-POSTURE CHECK (2026-11-17, after real runs: one reached 8 of 23 viewpoints, two refused at once
with "joint 4 = -98.8 deg is outside its limit"): before any motion, from where the arm holds, (1) every joint's
margin to its limit - closer than REFUSE_MARGIN_RAD (the routines' own refusal, HarnessClient.joints_outside_limits)
is refused in plain words, e.g. "turn the wrist (joint 4) toward the middle: -98.8° of ±100°"; (2) how many of the
routine's viewpoints (handeye.viewpoints: 15) the arm node would accept from here - the same tests as its
/arm/move_pose: workspace box, the tip's floor margin, IK in the current branch inside the joint limits less
ik_limit_margin_rad (best within 1 mm / 5 mrad), the joint jump within pose_total_jump_rad - fewer than MIN_REACHABLE
is refused, naming the joint the unreachable views run into. Pure (no ROS): the routines (handeye, absolute) and the
gateway (its calib_precheck state, its refusals) call the same function.
"""
import math

import numpy as np

from harness_calibration.handeye import viewpoints

REFUSE_MARGIN_RAD = 0.03          # HarnessClient.joints_outside_limits' margin
MIN_REACHABLE = 10
JOINT_WORDS = {1: ("turn", "the base"), 2: ("move", "the shoulder"), 3: ("move", "the elbow"), 4: ("turn", "the wrist"),
               5: ("bend", "the wrist"), 6: ("turn", "the hand")}


def joint_words(j, value_deg, lo_deg, hi_deg):
    """'turn the wrist (joint 4) toward the middle: -98.8° of ±100°'."""
    verb, name = JOINT_WORDS[j]
    rng = f"±{hi_deg:.0f}°" if abs(lo_deg + hi_deg) < 1.0 else f"{lo_deg:.0f}…{hi_deg:.0f}°"
    return f"{verb} {name} (joint {j}) toward the middle: {value_deg:.1f}° of {rng}"


def _view_why(kin, q, Tv, P):
    """'' when the arm node would take the move to Tv from joints q, else (why, the joint IK ran into or None)."""
    lo, hi = P.get("workspace_min"), P.get("workspace_max")
    if lo and hi and not all(lo[i] <= Tv[i, 3] <= hi[i] for i in range(3)):
        return "outside the workspace box", None
    if P.get("floor_enabled"):
        a, b, c = P["floor_plane"]
        tip = Tv[:3, 3] + float(P.get("tcp_offset_m", 0.0)) * Tv[:3, 2]
        if tip[2] - (a * tip[0] + b * tip[1] + c) < float(P.get("floor_margin_m", 0.0)):
            return "the tip would come too close to the table", None
    m = float(P.get("ik_limit_margin_rad", 0.02))
    qv, pe, ae = kin.ik(Tv, list(q), margin=m)
    if pe > 1e-3 or ae > 5e-3:
        at = [i + 1 for i, (v, (l, h)) in enumerate(zip(qv, kin.limits)) if v <= l + m + 1e-3 or v >= h - m - 1e-3]
        return "no IK solution in the current branch", (at[0] if at else None)
    if float(np.abs(np.asarray(qv) - np.asarray(q)).max()) > float(P.get("pose_total_jump_rad", 2.5)):
        return "too large a joint jump", None
    return "", None


def joint_margins(kin, q):
    """[{j, value, limit [lo, hi], margin}] in degrees (margin = to the nearer limit)."""
    return [{"j": i + 1, "value": round(math.degrees(v), 1), "limit": [round(math.degrees(l), 1), round(math.degrees(h), 1)],
             "margin": round(math.degrees(min(v - l, h - v)), 1)} for i, (v, (l, h)) in enumerate(zip(q, kin.limits))]


def precheck(kin, q, P, trans=0.04, tilt_deg=15.0, roll_deg=20.0, min_reachable=MIN_REACHABLE):
    """{ok, reachable, of, joints [{j, value, limit [lo, hi], margin} deg], unreachable [names], why} for joints q (rad,
    the controller's = /joint_states) with model kin (harness_arm PiperKinematics, the corrected one) and the arm node's
    parameters P (config/piper.yaml)."""
    joints = joint_margins(kin, q)
    close = sorted([j for j in joints if j["margin"] < math.degrees(REFUSE_MARGIN_RAD)], key=lambda j: j["margin"])
    views = viewpoints(kin.fk(list(q)), trans, math.radians(tilt_deg), math.radians(roll_deg))
    bad, hit = [], {}
    for name, Tv in views:
        why, j = _view_why(kin, q, Tv, P)
        if why:
            bad.append({"view": name, "why": why})
            if j:
                hit[j] = hit.get(j, 0) + 1
    n_ok = len(views) - len(bad)
    out = {"ok": False, "reachable": n_ok, "of": len(views), "joints": joints, "unreachable": bad, "why": None}
    if close:
        j = close[0]
        out["why"] = joint_words(j["j"], j["value"], *j["limit"])
    elif n_ok < min_reachable:
        if hit:
            j = joints[max(hit, key=hit.get) - 1]
            hint = joint_words(j["j"], j["value"], *j["limit"])
        else:
            reasons = sorted({b["why"] for b in bad})
            hint = "the views fail with: " + ", ".join(reasons)
        out["why"] = f"only {n_ok} of {len(views)} calibration views can be reached from here: {hint}"
    else:
        out["ok"] = True
    return out


def precheck_from_files(q, kin_file, piper_yaml, **kw):
    """The routines' call: the corrected model from calib/kinematics.yaml, the arm node's parameters from piper.yaml."""
    import yaml
    from harness_arm.kinematics import PiperKinematics
    with open(piper_yaml) as f:
        P = yaml.safe_load(f)["piper"]["ros__parameters"]
    kin = PiperKinematics.from_file(kin_file, str(P.get("kinematics_model", "piper")))
    return precheck(kin, q, P, **kw)


def summary(pc):
    """One line for the routines' output."""
    return (f"start posture: {pc['reachable']} of {pc['of']} views reachable; joint margins "
            + ", ".join(f"j{j['j']} {j['margin']:.0f}°" for j in pc["joints"]))

