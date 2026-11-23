"""Planner bedrock (PROJECT_SPEC.md "2026-10-22 — Planner bedrock"): a Cartesian path for the
fingertip -> a validated, time-stamped joint trajectory the arm node streams.

Pure Python (numpy / scipy), no ROS: the CLI daemon calls it, the tests run it offline.

    path (motion file)  ->  timing u(t)  ->  IK at knots (harness corrected model)
                        ->  cubic spline in time, sampled at the streaming rate
                        ->  + approach / return (joint space, minimum jerk)
                        ->  validation of every sample (limits, FK deviation, workspace,
                            floor clearance, joint speed / acceleration)

Units: metres and radians inside; the motion file is in mm (contract `railab-motion/1`).
Tool frame -> flange frame (PIPER): flange z = approach, flange x = -up, y = z x x;
flange origin = fingertip - tcp_offset * approach.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline

from harness_arm.kinematics import pose_error

FORMAT = "railab-motion/1"


class PlanError(RuntimeError):
    """The motion cannot be planned / is refused; the message names the reason in at most ~9 words (operator 2026-11-21:
    the gateway says "refused: " + it), `detail` the sample and the numbers behind it."""

    def __init__(self, words="", detail=""):
        super().__init__(words)
        self.detail = detail


# ---------------------------------------------------------------------------------- motion file
@dataclass
class CartesianPath:
    """Fingertip positions (m) and tool directions (unit), in the arm base frame."""
    p: np.ndarray            # (N, 3)
    approach: np.ndarray     # (N, 3)
    up: np.ndarray           # (N, 3)
    closed: bool = False
    name: str = ""
    meta: dict = field(default_factory=dict)

    @classmethod
    def from_motion(cls, doc: dict) -> "CartesianPath":
        if doc.get("format") != FORMAT:
            raise PlanError(f"not a {FORMAT} motion file (format = {doc.get('format')!r})")
        if doc.get("frame") != "arm_base":
            raise PlanError(f"motion frame must be 'arm_base', not {doc.get('frame')!r}")
        if (doc.get("units") or {}).get("length") != "mm":
            raise PlanError("motion units.length must be 'mm'")
        if doc.get("tool") != "fingertip":
            raise PlanError(f"motion tool must be 'fingertip', not {doc.get('tool')!r}")
        s = doc.get("samples") or []
        if len(s) < 4:
            raise PlanError("a motion needs at least 4 samples")
        try:
            p = np.array([x["p"] for x in s], dtype=float) / 1000.0
            a = np.array([x["approach"] for x in s], dtype=float)
            u = np.array([x["up"] for x in s], dtype=float)
        except (KeyError, TypeError, ValueError) as e:
            raise PlanError(f"bad motion sample: {e}") from e
        if p.shape[1:] != (3,) or a.shape != p.shape or u.shape != p.shape:
            raise PlanError("every sample needs p, approach and up as 3-vectors")
        a /= np.linalg.norm(a, axis=1, keepdims=True)
        u -= (u * a).sum(1, keepdims=True) * a                  # up made perpendicular to approach
        n = np.linalg.norm(u, axis=1)
        if (n < 1e-6).any():
            raise PlanError("a sample's up is parallel to its approach")
        u /= n[:, None]
        meta = {k: v for k, v in doc.items() if k != "samples"}
        return cls(p, a, u, bool(doc.get("closed")), str(doc.get("name") or ""), meta)

    @classmethod
    def load(cls, path: str) -> "CartesianPath":
        try:
            with open(path) as f:
                return cls.from_motion(json.load(f))
        except OSError as e:
            raise PlanError(f"cannot read the motion file: {e}") from e
        except ValueError as e:
            raise PlanError(f"motion file is not valid JSON: {e}") from e

    # ------------------------------------------------------------------ resampling
    def splines(self):
        """Arc-length splines of position and of the two directions; returns (length_m, f(s))
        with s in [0, length] (periodic when closed)."""
        p, a, u = self.p, self.approach, self.up
        if self.closed:
            p, a, u = np.vstack([p, p[:1]]), np.vstack([a, a[:1]]), np.vstack([u, u[:1]])
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        if (seg < 1e-9).any():
            raise PlanError("two consecutive motion samples coincide")
        s = np.concatenate([[0.0], np.cumsum(seg)])
        bc = "periodic" if self.closed else "not-a-knot"
        sp, sa, su = (CubicSpline(s, v, bc_type=bc) for v in (p, a, u))

        def at(x):
            x = np.atleast_1d(np.asarray(x, dtype=float))
            P, A, U = sp(x), sa(x), su(x)
            A /= np.linalg.norm(A, axis=1, keepdims=True)
            U -= (U * A).sum(1, keepdims=True) * A
            U /= np.linalg.norm(U, axis=1, keepdims=True)
            return P, A, U
        return float(s[-1]), at


def flange_T(p, approach, up, tcp_offset_m: float) -> np.ndarray:
    """4x4 flange pose for a fingertip point + tool directions (PIPER flange: z = approach, x = -up)."""
    z = np.asarray(approach, float)
    x = -np.asarray(up, float)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(p, float) - tcp_offset_m * z
    return T


# ---------------------------------------------------------------------------------- timing
def smooth_fraction(t: np.ndarray, T: float, ramp: float) -> np.ndarray:
    """u(t) in [0, 1]: constant speed with smoothstep velocity ramps of length `ramp` at both
    ends (starts and stops at rest, continuous acceleration). Cruise speed V = 1 / (T - ramp)."""
    t = np.clip(np.asarray(t, float), 0.0, T)
    V = 1.0 / (T - ramp)
    S = lambda x: x ** 3 - 0.5 * x ** 4                  # integral of the smoothstep 3x^2 - 2x^3
    u = np.where(t < ramp, V * ramp * S(t / ramp),
                 np.where(t > T - ramp, 1.0 - V * ramp * S((T - t) / ramp), V * (0.5 * ramp + (t - ramp))))
    return np.clip(u, 0.0, 1.0)


def _per_joint(x, n: int = 6) -> np.ndarray:
    """A joint limit given as one number for every joint or one per joint (2026-11-19: the base joint's own) -> (n,)."""
    a = np.asarray(x, float)
    return np.full(n, float(a)) if a.ndim == 0 else a.reshape(-1)[:n]


def min_jerk(q0, q1, vmax, amax, rate_hz: float, t_min: float = 2.0):
    """Joint-space move q0 -> q1 (minimum-jerk profile), duration from the limits: peak speed
    1.875 d / T, peak acceleration 5.774 d / T^2 (d = each joint's distance against its own limit: vmax / amax one
    number for every joint, or one per joint). Returns (t, q)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = np.abs(q1 - q0)
    vm, am = _per_joint(vmax, len(d)), _per_joint(amax, len(d))
    T = max(t_min, float((1.875 * d / vm).max()), math.sqrt(float((5.774 * d / am).max())))
    n = max(2, int(math.ceil(T * rate_hz)) + 1)
    t = np.linspace(0.0, T, n)
    tau = t / T
    f = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    return t, q0 + np.outer(f, q1 - q0)


CORNER_DEG = 5.0      # plan_joint_path: waypoints whose path turns less than this are passed without stopping


def _corner_runs(W, corner_deg: float = CORNER_DEG):
    """Split a joint polyline (no zero-length segments) into runs of waypoint indices whose consecutive
    segments turn by less than corner_deg (default CORNER_DEG): a run is driven as one motion, a real corner stops."""
    runs, cur = [], [0]
    for k in range(1, len(W) - 1):
        a, b = W[k] - W[k - 1], W[k + 1] - W[k]
        c = float(a @ b) / max(1e-12, float(np.linalg.norm(a) * np.linalg.norm(b)))
        cur.append(k)
        if math.degrees(math.acos(max(-1.0, min(1.0, c)))) >= corner_deg:
            runs.append(cur)
            cur = [k]
    cur.append(len(W) - 1)
    runs.append(cur)
    return runs


def _run_motion(P, vmax, amax, rate_hz: float, t_min: float):
    """Nearly-collinear waypoints P (>= 3) as ONE motion (2026-10-24, operator: smooth, no stop-and-go): a C2
    spline through every waypoint (parameter = the largest joint's distance along the polyline, each joint weighted by
    the slowest joint's limit over its own — 2026-11-19: a faster base joint), minimum-jerk along it, stretched in time
    until every joint's speed and acceleration are within vmax / amax (one number, or one per joint). Returns (t, q)."""
    vm, am = _per_joint(vmax, P.shape[1]), _per_joint(amax, P.shape[1])
    w = vm.min() / vm
    u = np.concatenate([[0.0], np.cumsum((np.abs(np.diff(P, axis=0)) * w).max(axis=1))])
    L = float(u[-1])
    La = float(np.abs(np.diff(P, axis=0) * (am.min() / am)).max(axis=1).sum())
    sp = CubicSpline(u, P, axis=0, bc_type="natural")
    T = max(t_min, 1.875 * L / float(vm.min()), math.sqrt(5.774 * La / float(am.min())))
    for _ in range(8):
        t = np.linspace(0.0, T, max(3, int(math.ceil(T * rate_hz)) + 1))
        tau = t / T
        q = sp(L * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5))
        v = np.diff(q, axis=0) / np.diff(t)[:, None]
        a = np.diff(v, axis=0) / np.diff(t)[1:, None]
        r = max(float((np.abs(v) / vm).max()), math.sqrt(float((np.abs(a) / am).max())))
        if r <= 1.0:
            break
        T *= r * 1.02
    q[0], q[-1] = P[0], P[-1]
    return t, q


# ---------------------------------------------------------------------------------- the plan
@dataclass
class Limits:
    """Planner limits + the safety parameters it shares with the arm node (piper.yaml). The joint speed / acceleration:
    one number for every joint, or one per joint (2026-11-19, arm.follow_joints: the base joint twice the others)."""
    max_joint_vel_rad_s: float = 0.5
    max_joint_acc_rad_s2: float = 1.0
    ik_margin_rad: float = 0.02
    max_dev_m: float = 0.001          # FK of a streamed sample vs the intended curve
    max_dev_rad: float = math.radians(0.5)
    tcp_offset_m: float = 0.1441
    floor_enabled: bool = True
    floor_plane: tuple = (0.0, 0.0, 0.0)   # base frame z = a x + b y + c (m)
    floor_margin_m: float = 0.015
    workspace_min: tuple = (-0.7, -0.7, -0.05)
    workspace_max: tuple = (0.7, 0.7, 0.9)


@dataclass
class Trajectory:
    t: np.ndarray                      # (M,) s from start
    q: np.ndarray                      # (M, 6) rad
    segments: list                     # [(name, i0, i1)] sample index ranges, i1 exclusive
    report: dict

    @property
    def duration(self) -> float:
        return float(self.t[-1])


def _tip(kin, q, tcp_offset_m):
    T = kin.fk(q)
    return T, T[:3, 3] + tcp_offset_m * T[:3, 2]


def plan_path_run(kin, path: CartesianPath, q_now, duration_s: float, lim: Limits,
                  rate_hz: float = 100.0, knot_hz: float = 40.0, ret: bool = True) -> Trajectory:
    """One run of `path`: approach from q_now to its start, the path once in `duration_s`, and
    (ret) back to q_now. Raises PlanError with the reason if anything is out of bounds."""
    q_now = np.asarray(q_now, float)
    if q_now.shape != (6,):
        raise PlanError("need the 6 current joint angles")
    L, at = path.splines()
    ramp = min(2.0, duration_s / 4.0)

    # ---- IK at knots, seeded along the path (same branch, continuous)
    tk = np.linspace(0.0, duration_s, max(8, int(math.ceil(duration_s * knot_hz)) + 1))
    P, A, U = at(smooth_fraction(tk, duration_s, ramp) * L)
    T0 = flange_T(P[0], A[0], U[0], lim.tcp_offset_m)
    seeds = [q_now] + [np.array([q_now[0], b, c, 0.0, w, 0.0]) for b in (0.8, 1.4, 1.8) for c in (-0.6, -1.2, -1.8)
                       for w in (-0.6, 0.0, 0.6)]
    sols = []
    for sd in seeds:
        q, pe, ae = kin.ik(T0, sd, margin=lim.ik_margin_rad)
        if pe <= 2e-4 and ae <= 2e-3:
            sols.append(q)
    if not sols:
        raise PlanError("the path's start is out of reach",
                        f"start {np.round(P[0] * 1000, 1).tolist()} mm: no IK solution of the corrected arm model "
                        "within the joint limits")
    q = min(sols, key=lambda s: float(np.abs(s - q_now).max()))
    knots = [q]
    for k in range(1, len(tk)):
        q, pe, ae = kin.ik(flange_T(P[k], A[k], U[k], lim.tcp_offset_m), q, margin=lim.ik_margin_rad)
        if pe > 2e-4 or ae > 2e-3:
            raise PlanError(f"path point at {tk[k]:.2f} s out of reach",
                            f"{np.round(P[k] * 1000, 1).tolist()} mm, in the current arm branch "
                            f"(best {pe * 1000:.1f} mm / {math.degrees(ae):.2f} deg)")
        knots.append(q)
    knots = np.array(knots)

    # ---- spline through the joint knots (clamped: at rest at both ends), sampled at rate_hz
    js = CubicSpline(tk, knots, bc_type="clamped")
    n_loop = int(math.ceil(duration_s * rate_hz)) + 1
    t_loop = np.linspace(0.0, duration_s, n_loop)
    q_loop = js(t_loop)

    # ---- approach / return in joint space
    ta, qa = min_jerk(q_now, q_loop[0], lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2, rate_hz)
    parts = [("approach", ta, qa), ("loop", t_loop, q_loop)]
    if ret:
        tr, qr = min_jerk(q_loop[-1], q_now, lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2, rate_hz)
        parts.append(("return", tr, qr))
    ts, qs, segs, t_off = [], [], [], 0.0
    for name, t, qq in parts:
        if ts:                                    # drop the duplicate joint sample at a segment boundary
            t, qq = t[1:], qq[1:]
        i0 = sum(len(x) for x in ts)
        ts.append(t + t_off)
        qs.append(qq)
        segs.append((name, i0, i0 + len(t)))
        t_off = float(ts[-1][-1])
    traj = Trajectory(np.concatenate(ts), np.vstack(qs), segs, {})
    loop = next(s for s in segs if s[0] == "loop")
    traj.report = validate(kin, traj, lim, at=at, L=L, ramp=ramp, loop=loop,
                           duration_s=duration_s)
    traj.report.update({"path": path.name, "path_length_m": round(L, 3), "loop_s": duration_s, "ramp_s": ramp,
                        "rate_hz": rate_hz, "knots": len(tk), "samples": len(traj.t),
                        "segments": {n: round(float(traj.t[i1 - 1] - traj.t[i0]), 2) for n, i0, i1 in segs}})
    return traj


def plan_joint_path(kin, waypoints, lim: Limits, rate_hz: float = 100.0, t_min: float = 0.8,
                    from_outside: bool = False, corner_deg: float = CORNER_DEG) -> Trajectory:
    """A joint path from a planner (rad, K x 6, the first = where the arm is; PROJECT_SPEC.md
    "2026-10-23 — Release, hand-eye and tag teaching"): each segment a STRAIGHT line in joint space —
    what a collision-checked joint path assumes between its waypoints — timed as a minimum-jerk move
    within lim's joint speed / acceleration, at rest at every real corner (a turn of CORNER_DEG or more;
    nearly collinear waypoints are passed without stopping, on a smooth spline through them). Validated like every other
    trajectory; raises PlanError naming the sample. from_outside: the arm may START past a joint limit
    (held there while limp, 2026-10-24) — that joint may stay between its start and the limit (it must
    never go further out); the caller's path brings it back inside."""
    W = np.asarray(waypoints, float)
    if W.ndim != 2 or W.shape[1] != 6 or len(W) < 2:
        raise PlanError("a joint path needs at least 2 waypoints", "of 6 joints")
    keep = [0] + [k for k in range(1, len(W)) if float(np.abs(W[k] - W[k - 1]).max()) >= 1e-6]
    idx, W = keep, W[keep]                        # zero-length segments dropped; idx = the caller's numbering
    ts, qs, segs, t_off = [], [], [], 0.0
    # 2026-10-24: waypoints on a nearly straight run (a planned straight rise, a curve sampled densely) are driven
    # through without stopping; the arm comes to rest only at real corners (CORNER_DEG) and at the end
    for run in (_corner_runs(W, corner_deg) if len(W) >= 2 else []):   # (corner_deg: 2026-11-18, the caller may
                                                                           # pass small turns of a dense planner path)
        P = W[run]
        if len(P) == 2:
            t, q = min_jerk(P[0], P[1], lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2, rate_hz, t_min=t_min)
            name = f"segment {idx[run[1]]}"
        else:
            t, q = _run_motion(P, lim.max_joint_vel_rad_s, lim.max_joint_acc_rad_s2, rate_hz, t_min)
            name = f"segments {idx[run[1]]}-{idx[run[-1]]}"
        if ts:                                    # the segment boundary sample is the previous segment's last
            t, q = t[1:], q[1:]
        i0 = sum(len(x) for x in ts)
        ts.append(t + t_off)
        qs.append(q)
        segs.append((name, i0, i0 + len(t)))
        t_off = float(ts[-1][-1])
    if not ts:
        raise PlanError("the path does not move")
    traj = Trajectory(np.concatenate(ts), np.vstack(qs), segs, {})
    traj.report = validate(kin, traj, lim, from_outside=from_outside)
    traj.report.update({"segments": len(segs), "samples": len(traj.t), "rate_hz": rate_hz,
                        "duration_s": round(traj.duration, 2)})
    return traj


def validate(kin, traj: Trajectory, lim: Limits, at=None, L=None, ramp=None, loop=None,
             duration_s=None, from_outside: bool = False) -> dict:
    """Check every sample; raise PlanError naming the first violation; return a summary."""
    t, q = traj.t, traj.q
    # the loop's time zero = the last approach sample (the loop's own t = 0 sample was dropped as a duplicate)
    t_loop0 = float(t[loop[1] - 1]) if loop and loop[1] > 0 else (float(t[loop[1]]) if loop else 0.0)
    seg_of = lambda i: next((n for n, i0, i1 in traj.segments if i0 <= i < i1), "?")
    where = lambda i: f"{seg_of(i)}, t = {t[i]:.2f} s"
    lo = np.array([l[0] + lim.ik_margin_rad * 0.5 for l in kin.limits])
    hi = np.array([l[1] - lim.ik_margin_rad * 0.5 for l in kin.limits])
    if from_outside:                              # a start past a limit: allowed between it and the limit
        lo, hi = np.minimum(lo, q[0]), np.maximum(hi, q[0])
    bad = np.argwhere((q < lo) | (q > hi))
    if len(bad):
        i, j = bad[0]
        raise PlanError(f"joint {j + 1} at {math.degrees(q[i, j]):.1f} deg leaves its limits", where(i))
    dt = np.diff(t)
    if (dt <= 0).any():
        raise PlanError("trajectory time does not increase")
    v = np.diff(q, axis=0) / dt[:, None]
    vmax = np.abs(v).max(axis=0)
    vlim, alim = _per_joint(lim.max_joint_vel_rad_s, q.shape[1]), _per_joint(lim.max_joint_acc_rad_s2, q.shape[1])
    i, j = np.unravel_index((np.abs(v) / vlim).argmax(), v.shape)
    if abs(v[i, j]) > vlim[j] * 1.001:
        need = ""
        if duration_s and loop and loop[1] <= i < loop[2]:
            # cruise speed ~ 1 / (T - ramp), ramp = min(2 s, T / 4): the loop time that brings this joint to the limit
            r = ramp if ramp is not None else min(2.0, duration_s / 4.0)
            T_need = (duration_s - r) * abs(v[i, j]) / vlim[j]
            T_need = T_need + 2.0 if T_need >= 6.0 else T_need * 4.0 / 3.0
            need = f"; a loop of about {math.ceil(T_need * 1.05)} s or longer fits (--duration)"
        raise PlanError(f"joint {j + 1} would turn at {math.degrees(abs(v[i, j])):.1f} > {math.degrees(vlim[j]):.1f} deg/s",
                        f"joint {j + 1} would turn at {math.degrees(abs(v[i, j])):.1f} deg/s > the "
                        f"{math.degrees(vlim[j]):.1f} deg/s limit ({where(i)}){need}")
    acc = np.diff(v, axis=0) / dt[1:, None]
    amax = np.abs(acc).max(axis=0) if len(acc) else np.zeros(6)
    if len(acc):
        i, j = np.unravel_index((np.abs(acc) / alim).argmax(), acc.shape)
        if abs(acc[i, j]) > alim[j] * 1.001:
            raise PlanError(f"joint {j + 1} would accelerate at {abs(acc[i, j]):.2f} > {alim[j]:.2f} rad/s^2",
                            f"({where(i + 1)}); slow it down (--duration)")
    a, b, c = lim.floor_plane
    wmin, wmax = np.array(lim.workspace_min), np.array(lim.workspace_max)
    clear_min, dev_m, dev_r = 1e9, 0.0, 0.0
    for k in range(len(t)):
        T, tip = _tip(kin, q[k], lim.tcp_offset_m)
        if not ((wmin <= T[:3, 3]) & (T[:3, 3] <= wmax)).all():
            raise PlanError("flange leaves the workspace box",
                            f"flange at {np.round(T[:3, 3] * 1000, 1).tolist()} mm ({where(k)})")
        if lim.floor_enabled:
            clear = tip[2] - (a * tip[0] + b * tip[1] + c)
            clear_min = min(clear_min, clear)
            if clear < lim.floor_margin_m:
                raise PlanError(f"fingertip {clear * 1000:.1f} mm above the floor (margin "
                                f"{lim.floor_margin_m * 1000:.0f} mm)", where(k))
        if at is not None and loop and loop[1] <= k < loop[2]:      # the streamed sample stays on the curve
            u = smooth_fraction(t[k] - t_loop0, duration_s, ramp)
            Pp, Aa, Uu = at(u * L)
            pe, ae = pose_error(T, flange_T(Pp[0], Aa[0], Uu[0], lim.tcp_offset_m))
            dev_m, dev_r = max(dev_m, pe), max(dev_r, ae)
            if pe > lim.max_dev_m or ae > lim.max_dev_rad:
                raise PlanError(f"streamed sample {pe * 1000:.2f} mm / {math.degrees(ae):.2f} deg off the curve", where(k))
    return {"peak_joint_speed_deg_s": [round(math.degrees(x), 1) for x in vmax],
            "peak_joint_acc_rad_s2": [round(float(x), 2) for x in amax],
            "min_floor_clearance_mm": round(float(clear_min) * 1000, 1) if lim.floor_enabled else None,
            "max_curve_deviation_mm": round(dev_m * 1000, 3), "max_curve_deviation_deg": round(math.degrees(dev_r), 3)}


# ---------------------------------------------------------------------------------- the smooth path
# THE SMOOTH PATH (2026-11-19, operator: "想一想怎么让waypoint之间不停顿"; the workflow repo's docs/plans/no_pause_motion.md):
# arm.follow_joints {blend: true, speeds} drives ALL its waypoints as ONE motion, at rest only at the first and the last.
# Geometry: every vertex of the joint polyline rounded by a Gaussian of its own width (closed form, C-infinity), the width
# shrunk until the curve stays within SMOOTH_DEV_RAD per joint and SMOOTH_DEV_M at the TCP of the polyline (a corner that
# cannot be rounded more is passed slowly, never cut further). Timing: a time-optimal speed profile along the curve
# (forward / backward pass on a dense grid, every joint's speed and acceleration within ITS segment's limits), then
# smoothed in time by a SMOOTH_WINDOW_S Hann window (continuous acceleration, no chatter), re-checked, slowed locally
# where the smoothing overshot. plan_joint_path and everything above are unchanged.
SMOOTH_DEV_RAD = math.radians(0.3)        # the streamed curve within this of the commanded polyline, per joint ...
SMOOTH_DEV_M = 0.001                      # ... and within this at the TCP
SMOOTH_SIGMA_MAX_RAD = math.radians(2.0)  # the widest rounding (the Gaussian's sigma along the path, rad of joint path)
SMOOTH_WINDOW_S = 0.2                     # the time window smoothing the speed profile (Hann, full width)
SMOOTH_GRID_RAD = math.radians(0.25)      # the speed profile's grid along the path (finer at tight roundings)
SMOOTH_LEVER_M = 1.0                      # an upper bound of the TCP's distance from any joint axis (the PiPER: < 0.8 m)
_SQ2PI = 1.0 / math.sqrt(2.0 * math.pi)


def _ndtr(z):
    """The standard normal CDF (vectorised)."""
    from scipy.special import ndtr
    return ndtr(z)


def _pairs(s, centres, radii):
    """(point index, centre index) of every sorted point s within radii of each centre."""
    lo = np.searchsorted(s, centres - radii, "left")
    hi = np.searchsorted(s, centres + radii, "right")
    cnt = np.maximum(hi - lo, 0)
    tot = int(cnt.sum())
    if not tot:
        return np.zeros(0, int), np.zeros(0, int)
    mm = np.repeat(np.arange(len(centres)), cnt)
    ii = np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt) + np.repeat(lo, cnt)
    return ii, mm


class _SmoothCurve:
    """The joint polyline W (K x 6, no zero-length segment) with vertex k (1 … K-2) rounded by a Gaussian of width
    sigma[k - 1] along the path: the polyline is W[0] + d0 s + sum_k Dk ramp(s - sk); each ramp is convolved with its own
    Gaussian (R(x) = sigma (z Phi(z) + phi(z)), z = x / sigma), mirrored at both ends so the curve starts and ends exactly
    at W[0] / W[-1] with zero curvature. Parameter s = the polyline's arc length (rad)."""

    def __init__(self, W, sigma):
        self.W = W
        D = np.diff(W, axis=0)
        self.seg = np.linalg.norm(D, axis=1)
        self.d = D / self.seg[:, None]
        self.sk = np.concatenate([[0.0], np.cumsum(self.seg)])
        self.L = float(self.sk[-1])
        self.jump = self.d[1:] - self.d[:-1]             # (K-2, 6): the direction change at vertex 1 … K-2
        self.sv = self.sk[1:-1]
        self.sigma = np.asarray(sigma, float)

    def polyline(self, s):
        """(segment index, the polyline's point) at s."""
        j = np.clip(np.searchsorted(self.sk, s, "right") - 1, 0, len(self.seg) - 1)
        return j, self.W[j] + self.d[j] * (s - self.sk[j])[:, None]

    def eval(self, s):
        """q, q', q'' at the sorted parameters s (rad of polyline)."""
        s = np.asarray(s, float)
        n = len(s)
        j, q = self.polyline(s)
        dq = self.d[j].copy()
        ddq = np.zeros_like(q)
        nv = len(self.sv)
        if nv:
            sg, sv, L = self.sigma, self.sv, self.L
            r = 6.0 * sg
            acc = [np.zeros((n, 6)) for _ in range(3)]

            def add(ii, mm, val, d1, d2):
                J = self.jump[mm]
                for c in range(6):
                    w = J[:, c]
                    acc[0][:, c] += np.bincount(ii, weights=w * val, minlength=n)
                    acc[1][:, c] += np.bincount(ii, weights=w * d1, minlength=n)
                    acc[2][:, c] += np.bincount(ii, weights=w * d2, minlength=n)

            ii, mm = _pairs(s, sv, r)                     # the rounding of each vertex
            if len(ii):
                x = s[ii] - sv[mm]
                g = sg[mm]
                z = x / g
                Phi, phi = _ndtr(z), _SQ2PI * np.exp(-0.5 * z * z)
                passed = (mm + 1 <= j[ii]).astype(float)  # the polyline already turned there
                add(ii, mm, g * (z * Phi + phi) - passed * x, Phi - passed, phi / g)
            near0 = np.nonzero(sv < r)[0]                 # the mirror at the start (odd: W[0] exact, zero curvature)
            if len(near0):
                ii, k = _pairs(s, -sv[near0], r[near0])
                mm = near0[k]
                y = (s[ii] + sv[mm]) / sg[mm]
                Phi, phi = _ndtr(-y), _SQ2PI * np.exp(-0.5 * y * y)
                add(ii, mm, -sg[mm] * (-y * Phi + phi), Phi, -phi / sg[mm])
            near1 = np.nonzero(L - sv < r)[0]             # the mirror at the end
            if len(near1):
                ii, k = _pairs(s, 2.0 * L - sv[near1], r[near1])
                mm = near1[k]
                y = (s[ii] + sv[mm] - 2.0 * L) / sg[mm]
                Phi, phi = _ndtr(y), _SQ2PI * np.exp(-0.5 * y * y)
                add(ii, mm, -sg[mm] * (y * Phi + phi), -Phi, -phi / sg[mm])
            q, dq, ddq = q + acc[0], dq + acc[1], ddq + acc[2]
        return q, dq, ddq


def _tcp(kin, Q, tcp_offset_m):
    out = np.empty((len(Q), 3))
    for i, qq in enumerate(Q):
        out[i] = _tip(kin, qq, tcp_offset_m)[1]
    return out


def _smooth_grid(cv, h0):
    """The speed profile's grid: every h0, every vertex, finer (sigma / 2) round a tight rounding."""
    L = cv.L
    n = max(2, int(math.ceil(L / h0)) + 1)
    parts = [np.linspace(0.0, L, n), cv.sk]
    tight = np.nonzero(cv.sigma < 3.0 * h0)[0]
    if len(tight):
        parts.append((cv.sv[tight, None] + cv.sigma[tight, None] * np.linspace(-6.0, 6.0, 25)[None, :]).ravel())
    s = np.unique(np.clip(np.concatenate(parts), 0.0, L))
    keep = np.concatenate([[True], np.diff(s) > 1e-9 * max(1.0, L)])
    s = s[keep]
    s[-1] = L
    return s


def _smooth_sigmas(kin, W, cv, lim, dev_rad, dev_m, sigma_max):
    """Each vertex's rounding width: wide where the curve allows it, narrowed until the curve (same parameter) stays
    within 0.8 x the joint / TCP bounds of the polyline everywhere on the grid. Returns the curve."""
    sig = np.full(len(cv.sv), min(sigma_max, cv.L / 12.0))
    if not len(sig):
        return cv
    jmax = np.abs(cv.jump).max(axis=1)
    sig = np.minimum(sig, 0.5 * dev_rad / np.maximum(1e-12, 0.3989 * jmax))
    big = np.nonzero(0.3989 * sig * SMOOTH_LEVER_M * np.abs(cv.jump).sum(axis=1) > 0.5 * dev_m)[0]
    for m in big:                                         # the TCP bound of a single rounding (the arm's Jacobian there)
        q0 = W[m + 1]
        p0 = _tip(kin, q0, lim.tcp_offset_m)[1]
        Jp = np.empty((3, 6))
        for c in range(6):
            dq = np.zeros(6)
            dq[c] = 1e-6
            Jp[:, c] = (_tip(kin, q0 + dq, lim.tcp_offset_m)[1] - p0) / 1e-6
        sig[m] = min(sig[m], 0.5 * dev_m / max(1e-12, 0.3989 * float(np.linalg.norm(Jp @ cv.jump[m]))))
    sig = np.maximum(sig, 1e-5)
    cv.sigma = sig
    tj, tm = 0.8 * dev_rad, 0.8 * dev_m
    for _ in range(16):
        s = _smooth_grid(cv, SMOOTH_GRID_RAD)
        q = cv.eval(s)[0]
        _, qp = cv.polyline(s)
        e = q - qp
        ratio = np.abs(e).max(axis=1) / tj
        chk = np.nonzero(np.abs(e).sum(axis=1) * SMOOTH_LEVER_M > tm)[0]     # the TCP: only where it may matter
        if len(chk):
            dm = np.linalg.norm(_tcp(kin, q[chk], lim.tcp_offset_m) - _tcp(kin, qp[chk], lim.tcp_offset_m), axis=1)
            ratio[chk] = np.maximum(ratio[chk], dm / tm)
        bad = np.nonzero(ratio > 1.0)[0]
        if not len(bad):
            return cv
        ii, mm = _pairs(s[bad], cv.sv, 4.0 * cv.sigma)     # the roundings reaching a point off the bounds
        if not len(ii):
            ii, mm = _pairs(s[bad], cv.sv, 6.0 * cv.sigma)
        f = np.ones(len(sig))
        np.minimum.at(f, mm, np.maximum(0.3, 0.9 / ratio[bad][ii]))
        sig = np.maximum(sig * f, 1e-5)
        cv.sigma = sig
    raise PlanError("smooth path cannot keep within %.2f deg / %.1f mm"
                    % (math.degrees(dev_rad), dev_m * 1000), "of the waypoints")


def _topp(dq, ddq, ds, vlim, alim, cap):
    """Time-optimal x = sdot^2 on the grid: every joint |q' sdot| <= vlim, |q' sddot + q'' sdot^2| <= alim (per point,
    per joint), x <= cap; at rest at both ends (forward / backward pass)."""
    N = len(ds) + 1
    A, B = dq, ddq
    absA = np.abs(A)
    small = absA < 1e-9
    with np.errstate(divide="ignore", invalid="ignore"):
        mvc = np.where(small, np.inf, (vlim / absA) ** 2).min(axis=1)
        mvc = np.minimum(mvc, np.where(small & (np.abs(B) > 1e-12), alim / np.abs(B), np.inf).min(axis=1))
        P = np.where(small, 1e30, alim / absA)                # u in [-P + r x, P + r x] per joint
        R = np.where(small, 0.0, -B / np.where(small, 1.0, A))
        dr = R[:, :, None] - R[:, None, :]                    # pair (j, k): (r_j - r_k) x <= P_j + P_k
        lim2 = np.where(dr > 1e-12, (P[:, :, None] + P[:, None, :]) / np.where(dr > 1e-12, dr, 1.0), np.inf)
    mvc = np.minimum(np.minimum(mvc, lim2.reshape(N, -1).min(axis=1)), cap)
    mvc[0] = mvc[-1] = 0.0
    Pl, Rl, ml, dl = P.tolist(), R.tolist(), mvc.tolist(), ds.tolist()
    x = [0.0] * N
    for i in range(N - 1):                                    # forward: as fast as the acceleration allows
        xi, p, r = x[i], Pl[i], Rl[i]
        um = min(p[0] + r[0] * xi, p[1] + r[1] * xi, p[2] + r[2] * xi, p[3] + r[3] * xi, p[4] + r[4] * xi, p[5] + r[5] * xi)
        xn = xi + 2.0 * dl[i] * um
        c = ml[i + 1]
        x[i + 1] = c if xn > c else (xn if xn > 0.0 else 0.25 * min(xi, c))
    x[-1] = 0.0
    for i in range(N - 2, -1, -1):                            # backward: slow down in time
        xn, p, r = x[i + 1], Pl[i + 1], Rl[i + 1]
        lm = max(r[0] * xn - p[0], r[1] * xn - p[1], r[2] * xn - p[2], r[3] * xn - p[3], r[4] * xn - p[4], r[5] * xn - p[5])
        xb = xn - 2.0 * dl[i] * lm
        if xb < x[i]:
            x[i] = xb if xb > 0.0 else 0.25 * min(x[i], xn)
    x[0] = 0.0
    return np.array(x)


def _time_smooth(s, x, window_s, dt=0.001):
    """The profile x(s) as s(t) on a dt grid (sdot linear in t between grid points), convolved with a Hann window of
    window_s (at rest before and after): (t, s) from 0 to the end, smooth acceleration."""
    v = np.sqrt(np.maximum(x, 0.0))
    dts = 2.0 * np.diff(s) / np.maximum(v[:-1] + v[1:], 1e-12)
    t = np.concatenate([[0.0], np.cumsum(dts)])
    T = float(t[-1])
    tf = np.arange(0.0, T + dt, dt)
    tf[-1] = min(tf[-1], T)
    i = np.clip(np.searchsorted(t, tf, "right") - 1, 0, len(dts) - 1)
    tau = tf - t[i]
    a = (v[i + 1] - v[i]) / np.maximum(dts[i], 1e-12)
    sf = np.minimum(s[i] + v[i] * tau + 0.5 * a * tau * tau, s[-1])
    sf[-1] = s[-1]
    m = max(3, int(round(window_s / dt)) | 1)
    k = np.hanning(m + 2)[1:-1]
    k /= k.sum()
    pad = np.concatenate([np.zeros(m - 1), sf, np.full(m - 1, s[-1])])
    out = np.convolve(pad, k, "valid")
    out[0], out[-1] = 0.0, s[-1]
    return np.arange(len(out)) * dt, np.maximum.accumulate(out)


def plan_smooth_path(kin, waypoints, lim: Limits, seg_vel=None, seg_acc=None, rate_hz: float = 100.0,
                     from_outside: bool = False, dev_rad: float = SMOOTH_DEV_RAD, dev_m: float = SMOOTH_DEV_M,
                     window_s: float = SMOOTH_WINDOW_S) -> Trajectory:
    """THE SMOOTH PATH: a planner's joint path (rad, K x 6, the first = where the arm is) driven as ONE motion through
    every waypoint, at rest only at the first and the last. seg_vel / seg_acc: each segment's joint speed / acceleration
    limits (K - 1 rows of 6, or K - 1 numbers; None = lim's for every segment). The curve stays within dev_rad per joint
    and dev_m at the TCP of the polyline (a tight corner is passed slowly, never cut more); each streamed sample keeps
    every joint within its segment's limits. Validated like every other trajectory (validate, against the segments'
    largest limits, + every sample against its own segment's); raises PlanError naming the reason."""
    import time as _time
    t_start = _time.perf_counter()
    W = np.asarray(waypoints, float)
    if W.ndim != 2 or W.shape[1] != 6 or len(W) < 2:
        raise PlanError("a joint path needs at least 2 waypoints", "of 6 joints")
    K = len(W)

    def per_seg(v, default):
        if v is None:
            return np.tile(_per_joint(default, 6), (K - 1, 1))
        a = np.asarray(v, float)
        if a.ndim == 1 and len(a) == K - 1:
            a = np.repeat(a[:, None], 6, axis=1)
        if a.shape != (K - 1, 6) or not np.isfinite(a).all() or (a <= 0).any():
            raise PlanError("the segment limits need %d positive rows" % (K - 1), "one per segment")
        return a
    sv_, sa_ = per_seg(seg_vel, lim.max_joint_vel_rad_s), per_seg(seg_acc, lim.max_joint_acc_rad_s2)
    keep = [0] + [k for k in range(1, K) if float(np.abs(W[k] - W[k - 1]).max()) >= 1e-6]
    if len(keep) < 2:
        raise PlanError("the path does not move")
    # a kept segment spans the caller's segments keep[i-1] … keep[i]-1 (the dropped ones have no length): their slowest
    seg_v = np.array([sv_[a:b].min(axis=0) for a, b in zip(keep[:-1], keep[1:])])
    seg_a = np.array([sa_[a:b].min(axis=0) for a, b in zip(keep[:-1], keep[1:])])
    W = W[keep]
    cv = _smooth_sigmas(kin, W, _SmoothCurve(W, np.zeros(len(W) - 2)), lim, dev_rad, dev_m, SMOOTH_SIGMA_MAX_RAD)

    # ---- the speed profile on the grid; each point's limits = the slower of the segments its two grid steps touch
    s = _smooth_grid(cv, SMOOTH_GRID_RAD)
    _, dq, ddq = cv.eval(s)
    jseg = np.clip(np.searchsorted(cv.sk, 0.5 * (s[:-1] + s[1:]), "right") - 1, 0, len(cv.seg) - 1)
    iv, ia = seg_v[jseg], seg_a[jseg]                     # per grid step
    vpt = np.minimum(np.vstack([iv[:1], iv]), np.vstack([iv, iv[-1:]]))
    apt = np.minimum(np.vstack([ia[:1], ia]), np.vstack([ia, ia[-1:]]))
    ds = np.diff(s)
    cap = np.full(len(s), np.inf)
    ascale = np.full(len(s), 0.85)                        # the speed profile's margin for the time smoothing
    vscale = 0.98
    n_fix = 0
    for it in range(10):
        x = _topp(dq, ddq, ds, vpt * vscale, apt * ascale[:, None], cap)
        tf, sf = _time_smooth(s, x, window_s)
        T = float(tf[-1])
        n = max(3, int(math.ceil(T * rate_hz)) + 1)
        t = np.linspace(0.0, T, n)
        su = np.interp(t, tf, sf)
        su[0], su[-1] = 0.0, cv.L
        q = cv.eval(su)[0]
        q[0], q[-1] = W[0], W[-1]
        v = np.diff(q, axis=0) / np.diff(t)[:, None]
        a = np.diff(v, axis=0) / np.diff(t)[1:, None]
        # every sample against the slowest segment it touches (speed: its step; acceleration: its two steps)
        j0 = np.clip(np.searchsorted(cv.sk, su, "right") - 1, 0, len(cv.seg) - 1)
        jl = np.clip(np.searchsorted(cv.sk, su, "left") - 1, 0, len(cv.seg) - 1)
        lv = _span_min(seg_v, j0[:-1], jl[1:])
        la = _span_min(seg_a, j0[:-2], jl[2:])
        rv = (np.abs(v) / lv).max(axis=1)
        ra = (np.abs(a) / la).max(axis=1)
        worst = max(float(rv.max()), float(ra.max()))
        if worst <= 0.999:
            break
        n_fix += 1
        sd = np.abs(np.diff(su)) / np.diff(t)              # path speed per step
        for idx, r, kind in [(i, rv[i], "v") for i in np.nonzero(rv > 0.999)[0]] + \
                            [(i + 1, ra[i], "a") for i in np.nonzero(ra > 0.999)[0]]:
            sc = float(su[idx])
            reach = max(3.0 * SMOOTH_GRID_RAD, float(sd[max(0, idx - 1):idx + 2].max()) * window_s * 0.75)
            lo, hi = np.searchsorted(s, [sc - reach, sc + reach])
            sl = slice(max(0, lo - 1), min(len(s), hi + 1))
            f = r * r if kind == "v" else r
            cap[sl] = np.minimum(cap[sl], x[sl] / (f * 1.05))
            if kind == "a":
                ascale[sl] = np.minimum(ascale[sl], ascale[sl] / (r * 1.05))
    else:
        # still over after the local fixes: the whole motion slowed evenly (speed / k, acceleration / k^2)
        k = max(float(rv.max()), math.sqrt(float(ra.max()))) * 1.01
        T2 = T * k
        n = max(3, int(math.ceil(T2 * rate_hz)) + 1)
        t = np.linspace(0.0, T2, n)
        su = np.interp(t / k, tf, sf)
        su[0], su[-1] = 0.0, cv.L
        q = cv.eval(su)[0]
        q[0], q[-1] = W[0], W[-1]
        v = np.diff(q, axis=0) / np.diff(t)[:, None]
        a = np.diff(v, axis=0) / np.diff(t)[1:, None]
        j0 = np.clip(np.searchsorted(cv.sk, su, "right") - 1, 0, len(cv.seg) - 1)
        jl = np.clip(np.searchsorted(cv.sk, su, "left") - 1, 0, len(cv.seg) - 1)
        rv = (np.abs(v) / _span_min(seg_v, j0[:-1], jl[1:])).max(axis=1)
        ra = (np.abs(a) / _span_min(seg_a, j0[:-2], jl[2:])).max(axis=1)
        n_fix = -n_fix
    i_v, i_a = int(rv.argmax()), int(ra.argmax())
    if rv[i_v] > 1.001:
        j = int(np.argmax(np.abs(v[i_v]) / _span_min(seg_v, j0[i_v:i_v + 1], jl[i_v + 1:i_v + 2])[0]))
        raise PlanError("joint %d would turn at %.1f deg/s, over its limit" % (j + 1, math.degrees(abs(v[i_v, j]))),
                        "over its segment's limit (t = %.2f s)" % t[i_v])
    if ra[i_a] > 1.001:
        raise PlanError("a joint would accelerate past its segment's limit", "t = %.2f s" % t[i_a + 1])

    # ---- the streamed samples on the curve: within the bounds of the polyline (same parameter: an upper bound)
    _, qp = cv.polyline(su)
    e = np.abs(q - qp)
    dev_j = float(e.max())
    if dev_j > dev_rad:
        i = int(e.max(axis=1).argmax())
        raise PlanError("streamed sample %.2f deg off the waypoints' path" % math.degrees(dev_j), "t = %.2f s" % t[i])
    chk = np.nonzero(e.sum(axis=1) * SMOOTH_LEVER_M > 0.05 * dev_m)[0]
    dev_t = 0.0
    if len(chk):
        dm = np.linalg.norm(_tcp(kin, q[chk], lim.tcp_offset_m) - _tcp(kin, qp[chk], lim.tcp_offset_m), axis=1)
        dev_t = float(dm.max())
        if dev_t > dev_m:
            raise PlanError("streamed sample %.2f mm off the path at the TCP" % (dev_t * 1000),
                            "t = %.2f s" % t[chk[int(dm.argmax())]])
    traj = Trajectory(t, q, [("smooth path", 0, len(t))], {})
    env = Limits(**{f: getattr(lim, f) for f in lim.__dataclass_fields__})
    env.max_joint_vel_rad_s, env.max_joint_acc_rad_s2 = seg_v.max(axis=0), seg_a.max(axis=0)
    traj.report = validate(kin, traj, env, from_outside=from_outside)
    speed = np.abs(v).max(axis=1)                         # the fastest joint per step; inside = past the start's
    inner = (t[1:] > window_s) & (t[:-1] < t[-1] - window_s)   # and before the end's smoothing window
    traj.report.update({
        "smooth": True, "segments": 1, "samples": len(t), "rate_hz": rate_hz, "duration_s": round(traj.duration, 2),
        "waypoints": len(W), "max_path_deviation_deg": round(math.degrees(dev_j), 3),
        "max_path_deviation_mm": round(dev_t * 1000, 3),
        "min_inner_speed_deg_s": (round(math.degrees(float(speed[inner].min())), 2) if inner.any() else None),
        "slowed_locally": abs(n_fix), "slowed_evenly": n_fix < 0,
        "plan_s": round(_time.perf_counter() - t_start, 3)})
    return traj


def _span_min(lim_seg, ja, jb):
    """Per sample step: the elementwise minimum of the segment limits from segment ja to jb (inclusive)."""
    lo, hi = np.minimum(ja, jb), np.maximum(ja, jb)
    out = np.minimum(lim_seg[lo], lim_seg[hi])
    wide = np.nonzero(hi - lo > 1)[0]
    for i in wide:
        out[i] = lim_seg[lo[i]:hi[i] + 1].min(axis=0)
    return out


# ---------------------------------------------------------------------------------- the timed path
# THE TIMED PATH (2026-11-23, the station's hand SHAKE: the base joint alone oscillating ~±0.85° at 2 Hz for 60 s; the
# re-timed paths stop at every reversal and the smooth path rounds such a shake flat): arm.follow_joints {times_s} = the
# caller's own timing, one time per waypoint. NO re-timing, NO corner rounding: a C2 cubic spline through every knot
# (clamped: zero velocity at both ends; position, velocity and acceleration continuous), sampled at the arm node's stream
# rate. Checked, never changed: at rest at both ends (the first / last segment's speed), the spline within dev_rad of the
# knots' straight lines (knots dense enough that what the twin checked is what streams), every sample's joint limits,
# speed and acceleration within lim's (one number or one per joint), the point count. The arm's offset from the first
# waypoint (within the caller's start tolerance) is faded out over the first fade_s (minimum jerk; refused when the fade
# alone would break a cap), so the stream starts exactly where the arm is.
TIMED_REST_DEG_S = 0.5            # the first / last segment of a timed path: at rest = no joint faster than this
TIMED_DEV_RAD = SMOOTH_DEV_RAD    # the streamed curve within this of the knots' straight lines, per joint
TIMED_FADE_S = 0.5                # the arm's offset from the first waypoint faded out over this (at most half the path)


def plan_timed_path(kin, waypoints, times_s, lim: Limits, rate_hz: float = 100.0, start_q=None,
                    from_outside: bool = False, max_points=None, rest_deg_s: float = TIMED_REST_DEG_S,
                    dev_rad: float = TIMED_DEV_RAD, fade_s: float = TIMED_FADE_S) -> Trajectory:
    """THE TIMED PATH: waypoints (rad, K x 6) at times_s (K seconds from the path's start: the first 0, strictly
    increasing) streamed as given — a clamped C2 cubic spline through every knot sampled at rate_hz, no re-timing, no
    rounding. start_q: where the arm is (its offset from waypoint 0 faded out over the first fade_s). lim's joint speed /
    acceleration = the caps every sample keeps (one number or one per joint). Raises PlanError (<= 10 words) naming the
    number; validated like every other trajectory."""
    import time as _time
    t_start = _time.perf_counter()
    W = np.asarray(waypoints, float)
    T = np.asarray(times_s, float)
    if W.ndim != 2 or W.shape[1] != 6 or len(W) < 2:
        raise PlanError("a timed path needs at least 2 waypoints", "of 6 joints")
    if T.shape != (len(W),) or not np.isfinite(T).all() or not np.isfinite(W).all():
        raise PlanError("times_s needs one number per waypoint", "%d waypoints" % len(W))
    if abs(T[0]) > 1e-9:
        raise PlanError("times_s must start at 0", "it starts at %.6f s" % T[0])
    dT = np.diff(T)
    if (dT <= 0).any():
        k = int(np.argmax(dT <= 0))
        raise PlanError("times_s must increase strictly (waypoint %d)" % (k + 2), "%.6f s after %.6f s" % (T[k + 1], T[k]))
    rest = math.radians(rest_deg_s)
    v_in, v_out = (float(np.abs(W[1] - W[0]).max() / dT[0]), float(np.abs(W[-1] - W[-2]).max() / dT[-1]))
    if v_in > rest:
        raise PlanError("path not at rest at its start (%.1f deg/s)" % math.degrees(v_in),
                        "the first segment's fastest joint; at most %.1f deg/s" % rest_deg_s)
    if v_out > rest:
        raise PlanError("path not at rest at its end (%.1f deg/s)" % math.degrees(v_out),
                        "the last segment's fastest joint; at most %.1f deg/s" % rest_deg_s)
    D = float(T[-1])
    n = max(2, int(math.ceil(D * rate_hz)) + 1)
    if max_points is not None and n > int(max_points):
        raise PlanError("path too long: %d samples > %d" % (n, int(max_points)),
                        "%.1f s at %.0f Hz" % (D, rate_hz))
    t = np.linspace(0.0, D, n)
    sp = CubicSpline(T, W, axis=0, bc_type="clamped")
    q = sp(t)
    q[0], q[-1] = W[0], W[-1]
    ql = np.column_stack([np.interp(t, T, W[:, j]) for j in range(6)])
    e = np.abs(q - ql)
    dev = float(e.max())
    if dev > dev_rad:
        i = int(e.max(axis=1).argmax())
        raise PlanError("knots too sparse: %.2f deg off their lines" % math.degrees(dev),
                        "t = %.2f s; send more waypoints" % t[i])
    off = 0.0
    tf = min(fade_s, D / 2.0)
    if start_q is not None:
        d = np.asarray(start_q, float) - W[0]
        off = float(np.abs(d).max())
        if off > 1e-12:
            vlim, alim = _per_joint(lim.max_joint_vel_rad_s), _per_joint(lim.max_joint_acc_rad_s2)
            r = max(float((1.875 * np.abs(d) / tf / vlim).max()), float((5.774 * np.abs(d) / tf ** 2 / alim).max()))
            if r > 0.5:                                   # the fade alone may take at most half a cap
                raise PlanError("the path starts %.1f deg from the arm" % math.degrees(off),
                                "faded out over %.2f s it would take %.0f %% of a joint cap" % (tf, 100 * r))
            tau = np.clip(t / tf, 0.0, 1.0)
            q += np.outer(1.0 - (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5), d)
    traj = Trajectory(t, q, [("timed path", 0, len(t))], {})
    traj.report = validate(kin, traj, lim, from_outside=from_outside)
    traj.report.update({
        "timed": True, "segments": 1, "samples": len(t), "rate_hz": rate_hz, "duration_s": round(D, 3),
        "waypoints": len(W), "interp": "cubic spline C2, clamped (at rest) ends",
        "max_knot_line_deviation_deg": round(math.degrees(dev), 4), "start_offset_deg": round(math.degrees(off), 3),
        "start_fade_s": round(tf, 3) if off > 1e-12 else 0.0,
        "caps_speed_rad_s": [round(float(x), 3) for x in _per_joint(lim.max_joint_vel_rad_s)],
        "caps_acc_rad_s2": [round(float(x), 3) for x in _per_joint(lim.max_joint_acc_rad_s2)],
        "plan_s": round(_time.perf_counter() - t_start, 3)})
    return traj
