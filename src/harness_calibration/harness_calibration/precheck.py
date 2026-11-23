"""The calibrations' START-POSTURE CHECK (2026-11-17, after real runs: one reached 8 of 23 viewpoints, two refused at once
with "joint 4 = -98.8 deg is outside its limit"): before any motion, from where the arm holds, (1) every joint's
margin to its limit - closer than REFUSE_MARGIN_RAD (the routines' own refusal, HarnessClient.joints_outside_limits)
is refused in plain words, e.g. "turn the wrist (joint 4) toward the middle: -98.8° of ±100°"; (2) how many of the
routine's viewpoints (handeye.viewpoints: 15) the arm node would accept from here - the same tests as its
/arm/move_pose: workspace box, the tip's floor margin, IK in the current branch inside the joint limits less
ik_limit_margin_rad (best within 1 mm / 5 mrad), the joint jump within pose_total_jump_rad - fewer than MIN_REACHABLE
is refused, naming the joint the unreachable views run into. Pure (no ROS): the routines (handeye, absolute) and the
gateway (its calib_precheck state, its refusals) call the same function.

THE RE-POSE (2026-11-18, operator: "可以让控制器自己回复到极限吗？user可不知道什么是limit啊" — the controller fixes it, the user
never hears of a joint limit): when the start posture fails, `repose_search` looks - without motion - for a better
posture that keeps the camera on the target: the camera pose turned rigidly about the target's centre (a ROLL about the
line of sight, free for a view of the target: it stays at the same pixel and distance; then orbits of ±12° / ±24° about
two axes across it), each through IK from the current joints AND from the other wrist solution (j4 ± 180°, −j5,
j6 ∓ 180°: the PiPER's same-pose flip); a candidate needs every joint ≥ REPOSE_MARGIN_MIN_DEG from its ends, a
joint-space way there (`_path_eval`: workspace box, the tip's floor margin, the tip and the camera ≥ TARGET_CLEAR_M from
the target's plane, sampled every 2°) and then the precheck's own ≥ MIN_REACHABLE views. COST (mm) = how far the
fingertip and the camera travel on the way + REPOSE_MARGIN_MM_PER_DEG per degree of joint margin short of
REPOSE_MARGIN_GOOD_DEG + REPOSE_WRIST_MM_PER_DEG per degree joints 4 / 6 turn (the camera cable) − REPOSE_VIEW_MM per view
above MIN_REACHABLE; the cheapest wins; nothing that makes the fingertip + camera travel more than REPOSE_TRAVEL_MAX_MM
(the harness knows only the floor and the target, not the other modules). `do_repose` moves there with move_j (via points ≤ 0.9 × max_joint_delta_rad; the
routine's speed), waits for stillness, checks the target is seen again (else the next candidate, 3 tries).
`assess` = precheck + the search: `ok` (the routine can go: as is or after the re-pose), `fixable`, `needs_repose`,
`words` (plain, for the page: never "limit", never a joint number) next to `why` (technical, for the logs).
"""
import math

import numpy as np

from harness_calibration.handeye import viewpoints

REFUSE_MARGIN_RAD = 0.03          # HarnessClient.joints_outside_limits' margin
MIN_REACHABLE = 10
REPOSE_ROLLS_DEG = tuple(range(-180, 180, 15))   # about the line of sight to the target
REPOSE_ORBITS_DEG = (12.0, 24.0)                 # ... then orbits about the target across it, both ways, two axes
REPOSE_KEEP = 10                                 # the cheapest candidates whose 15 views are checked
REPOSE_MARGIN_MIN_DEG = 8.0                      # every joint at least this far from its ends after the re-pose (hard)
REPOSE_MARGIN_GOOD_DEG = 25.0                    # ... below this the cost rises REPOSE_MARGIN_MM_PER_DEG per degree
REPOSE_MARGIN_MM_PER_DEG = 8.0
REPOSE_WRIST_MM_PER_DEG = 1.0                    # the wrist turns (joints 4, 6) cost this too: the camera cable
REPOSE_VIEW_MM = 25.0                            # each reachable view above MIN_REACHABLE is worth this
REPOSE_TRAVEL_MAX_MM = 400.0                     # the fingertip + camera travel at most this on a re-pose (a cluttered
                                                 # cell: the harness knows only the floor and the target)

# THE STOP (2026-11-22): the gateway's background start-posture check runs while the arm is idle; a job / a run command
# that starts meanwhile must not wait for it (the next path's plan_smooth_path was ~5 s late on a round of the re-pose
# search). `stop` (a callable, optional; the routines pass none) is polled before every view and every re-pose candidate:
# True = raise Interrupted at once, no result.
class Interrupted(Exception):
    """The check was stopped by its caller's `stop()` (no result)."""


def _check_stop(stop):
    if stop is not None and stop():
        raise Interrupted()


TARGET_CLEAR_M = 0.06                            # the tip / camera keep this from the target's plane on the way
PLAIN_FIX = "the arm will first turn its wrist to a better position by itself (the camera stays on the board)"
PLAIN_FIX_PROGRESS = "turning the wrist to a better position first (the camera stays on the board)"
PLAIN_FAIL = "move the camera closer, more in front of the board"     # <= 10 words (2026-11-21)
PLAIN_NO_TARGET = "point the camera at the board and press again"
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


def precheck(kin, q, P, trans=0.04, tilt_deg=15.0, roll_deg=20.0, min_reachable=MIN_REACHABLE, stop=None):
    """{ok, reachable, of, joints [{j, value, limit [lo, hi], margin} deg], unreachable [names], why} for joints q (rad,
    the controller's = /joint_states) with model kin (harness_arm PiperKinematics, the corrected one) and the arm node's
    parameters P (config/piper.yaml). stop: see Interrupted."""
    joints = joint_margins(kin, q)
    close = sorted([j for j in joints if j["margin"] < math.degrees(REFUSE_MARGIN_RAD)], key=lambda j: j["margin"])
    views = viewpoints(kin.fk(list(q)), trans, math.radians(tilt_deg), math.radians(roll_deg))
    bad, hit = [], {}
    for name, Tv in views:
        _check_stop(stop)
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



# ------------------------------------------------------------------------------------------------ the re-pose
def load_model(kin_file, piper_yaml):
    """(the corrected arm model, the arm node's parameters)."""
    import yaml
    from harness_arm.kinematics import PiperKinematics
    with open(piper_yaml) as f:
        P = yaml.safe_load(f)["piper"]["ros__parameters"]
    return PiperKinematics.from_file(kin_file, str(P.get("kinematics_model", "piper"))), P


def load_handeye(path):
    """calib/handeye.yaml's gripper base -> camera (4 x 4), or None."""
    import yaml
    try:
        with open(path) as f:
            return np.array((yaml.safe_load(f) or {})["flange_to_camera"]["matrix"], float)
    except (OSError, KeyError, TypeError, ValueError):
        return None


def flipped(kin, q):
    """The other wrist solution of the same gripper-base pose (j4 ± 180°, −j5, j6 ∓ 180°), or None when j4 / j6 have no
    room for it."""
    q2 = np.array(q, float)
    for i, sgn in ((3, 1.0), (5, -1.0)):
        lo, hi = kin.limits[i]
        opts = [q2[i] + sgn * math.pi, q2[i] - sgn * math.pi]
        ok = [v for v in opts if lo <= v <= hi]
        if not ok:
            return None
        q2[i] = min(ok, key=abs)
    q2[4] = -q2[4]
    return q2


def _rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _min_margin_deg(kin, q):
    return min(math.degrees(min(v - l, h - v)) for v, (l, h) in zip(q, kin.limits))


def _path_eval(kin, q0, q1, P, X, B, n):
    """(why or '', how far the fingertip + the camera travel in mm) on the joint-space way q0 -> q1 (move_j's): the
    gripper base in the workspace box, the tip over the floor margin, the tip and the camera >= TARGET_CLEAR_M on the
    camera's side of the target's plane (point B, normal n toward the camera), sampled every 2° of the largest joint."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    steps = max(2, int(math.ceil(math.degrees(np.abs(q1 - q0).max()) / 2.0)))
    lo, hi = P.get("workspace_min"), P.get("workspace_max")
    tcp = float(P.get("tcp_offset_m", 0.0))
    travel, last = 0.0, None
    for k in range(steps + 1):
        T = kin.fk(list(q0 + (q1 - q0) * k / steps))
        if lo and hi and not all(lo[i] <= T[i, 3] <= hi[i] for i in range(3)):
            return "outside the workspace box", travel
        tip = T[:3, 3] + tcp * T[:3, 2]
        if P.get("floor_enabled"):
            a, b, c = P["floor_plane"]
            if tip[2] - (a * tip[0] + b * tip[1] + c) < float(P.get("floor_margin_m", 0.0)):
                return "the tip would come too close to the table", travel
        cam = (T @ X)[:3, 3]
        if min(float(np.dot(tip - B, n)), float(np.dot(cam - B, n))) < TARGET_CLEAR_M:
            return "the hand would come too close to the board", travel
        if last is not None:
            travel += float(np.linalg.norm(tip - last[0]) + np.linalg.norm(cam - last[1])) * 1000.0
        last = (tip, cam)
    return "", travel


def vias(kin, P, q0, q1):
    """move_j goals from q0 to q1, each joint step <= 0.9 x max_joint_delta_rad (the arm node's per-goal cap)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    cap = 0.9 * float(P.get("max_joint_delta_rad", 1.2))
    n = max(1, int(math.ceil(np.abs(q1 - q0).max() / cap)))
    return [list(q0 + (q1 - q0) * k / n) for k in range(1, n + 1)]


def repose_search(kin, q, P, X, T_cam_target, trans=0.04, tilt_deg=15.0, roll_deg=20.0, min_reachable=MIN_REACHABLE,
                  exclude=(), stop=None):
    """The best posture that keeps the camera on the target (see the module's text), or None: {q, roll_deg, orbit,
    flip, margin_deg, reachable, of, motion_deg, vias, score}. No motion. exclude = postures tried already. stop: see
    Interrupted (polled before every candidate)."""
    q = np.asarray(q, float)
    m = float(P.get("ik_limit_margin_rad", 0.02))
    Tc = kin.fk(list(q)) @ X
    Tt = Tc @ T_cam_target
    B = Tt[:3, 3]
    u = B - Tc[:3, 3]
    u /= np.linalg.norm(u)
    a1 = Tc[:3, 0] - np.dot(Tc[:3, 0], u) * u
    a1 /= np.linalg.norm(a1)
    a2 = np.cross(u, a1)
    n = Tt[:3, 2] if np.dot(Tc[:3, 3] - B, Tt[:3, 2]) > 0 else -Tt[:3, 2]
    Xi = np.linalg.inv(X)
    fq = flipped(kin, q)
    seeds = [(q, False)] + ([(fq, True)] if fq is not None else [])
    cands = []
    orbits = [(None, 0.0)] + [(ax, sg * o) for o in REPOSE_ORBITS_DEG for ax in (a1, a2) for sg in (1.0, -1.0)]
    for psi in REPOSE_ROLLS_DEG:
        for ax, th in orbits:
            Rm = _rot(u, math.radians(psi)) @ (_rot(ax, math.radians(th)) if ax is not None else np.eye(3))
            Tc2 = np.eye(4)
            Tc2[:3, :3] = Rm @ Tc[:3, :3]
            Tc2[:3, 3] = B + Rm @ (Tc[:3, 3] - B)
            Tf2 = Tc2 @ Xi
            for seed, is_flip in seeds:
                _check_stop(stop)
                q2, pe, ae = kin.ik(Tf2, list(seed), margin=m)
                if pe > 1e-3 or ae > 5e-3:
                    continue
                mg = _min_margin_deg(kin, q2)
                if mg < REPOSE_MARGIN_MIN_DEG:
                    continue
                if any(np.abs(np.asarray(e) - q2).max() < math.radians(1.0) for e in exclude):
                    continue
                if any(np.abs(c["q"] - q2).max() < math.radians(1.0) for c in cands):
                    continue
                why, travel = _path_eval(kin, q, q2, P, X, B, n)
                if why or travel > REPOSE_TRAVEL_MAX_MM:
                    continue
                dq = np.degrees(np.abs(q2 - q))
                cost = (travel + REPOSE_MARGIN_MM_PER_DEG * max(0.0, REPOSE_MARGIN_GOOD_DEG - mg)
                        + REPOSE_WRIST_MM_PER_DEG * (dq[3] + dq[5]))
                cands.append({"q": q2, "roll_deg": psi, "orbit": (0.0 if ax is None else th, "a1" if ax is a1 else
                              ("a2" if ax is a2 else "")), "flip": is_flip, "margin_deg": mg,
                              "motion_deg": float(dq.max()), "travel_mm": travel, "cost": cost})
    cands.sort(key=lambda c: c["cost"])
    best = None
    for c in cands[:REPOSE_KEEP]:
        pc = precheck(kin, c["q"], P, trans=trans, tilt_deg=tilt_deg, roll_deg=roll_deg, min_reachable=min_reachable,
                      stop=stop)
        if not pc["ok"]:
            continue
        c = dict(c, reachable=pc["reachable"], of=pc["of"], cost=c["cost"] - REPOSE_VIEW_MM * (pc["reachable"] - min_reachable))
        if best is None or c["cost"] < best["cost"]:
            best = c
    if best is None:
        return None
    best = dict(best, vias=vias(kin, P, q, best["q"]))
    best["q"] = [float(v) for v in best["q"]]
    return best


def assess(kin, q, P, X=None, T_cam_target=None, stop=None, **kw):
    """precheck + the re-pose search: adds fixable, needs_repose, words (plain, for the page) and repose (the plan);
    ok = the routine can go (as is, or after the re-pose). stop: see Interrupted."""
    pc = precheck(kin, q, P, stop=stop, **kw)
    pc.update(fixable=pc["ok"], needs_repose=False, words="", repose=None)
    if pc["ok"]:
        return pc
    if X is None or T_cam_target is None:
        pc["words"] = PLAIN_NO_TARGET
        return pc
    rp = repose_search(kin, q, P, X, T_cam_target, stop=stop, **kw)
    if rp is None:
        pc["words"] = PLAIN_FAIL
        return pc
    pc.update(ok=True, fixable=True, needs_repose=True, words=PLAIN_FIX, repose=rp)
    return pc


def do_repose(c, kin, P, X, get_target, speed=10, settle_s=3.0, tries=3, log=print, **kw):
    """The routines' re-pose on the arm (client c: joints, move_j, wait_still): search from where it is, move there by
    move_j via points at the routine's speed, wait for stillness, the target seen again (get_target() -> camera ->
    target 4 x 4, or None) and the precheck passing; else the next candidate. True when the arm is ready."""
    import time
    tried = []
    for _ in range(tries):
        T_ct = get_target()
        if T_ct is None:
            return False
        q = list(c.joints)
        rp = repose_search(kin, q, P, X, T_ct, exclude=tried, **kw)
        if rp is None:
            return False
        log(PLAIN_FIX_PROGRESS)
        log(f"  (re-pose: roll {rp['roll_deg']}°, orbit {rp['orbit'][0]:+.0f}° {rp['orbit'][1]}, other wrist solution "
            f"{rp['flip']}, joint margin {rp['margin_deg']:.0f}°, {rp['reachable']} of {rp['of']} views, "
            f"{len(rp['vias'])} move(s))")
        tried.append(rp["q"])
        moved = True
        for v in rp["vias"]:
            r = c.move_j(v, speed=speed, timeout=40)
            if not r.success:
                log(f"  (re-pose move refused: {r.message})")
                moved = False
                break
        if not moved:
            continue
        time.sleep(settle_s)
        c.wait_still()
        if get_target() is None:
            log("  (the board is out of view after the re-pose: trying another position)")
            continue
        pc = precheck(kin, list(c.joints), P, **kw)
        if pc["ok"]:
            log(f"the wrist is in a better position: {pc['reachable']} of {pc['of']} views reachable")
            return True
    return False
