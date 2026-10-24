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
    """The motion cannot be planned / is refused; the message names the sample and the reason."""


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


def min_jerk(q0, q1, vmax: float, amax: float, rate_hz: float, t_min: float = 2.0):
    """Joint-space move q0 -> q1 (minimum-jerk profile), duration from the limits: peak speed
    1.875 d / T, peak acceleration 5.774 d / T^2 (d = largest joint distance). Returns (t, q)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.abs(q1 - q0).max())
    T = max(t_min, 1.875 * d / vmax, math.sqrt(5.774 * d / amax))
    n = max(2, int(math.ceil(T * rate_hz)) + 1)
    t = np.linspace(0.0, T, n)
    tau = t / T
    f = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    return t, q0 + np.outer(f, q1 - q0)


CORNER_DEG = 5.0      # plan_joint_path: waypoints whose path turns less than this are passed without stopping


def _corner_runs(W):
    """Split a joint polyline (no zero-length segments) into runs of waypoint indices whose consecutive
    segments turn by less than CORNER_DEG: a run is driven as one motion, a real corner stops."""
    runs, cur = [], [0]
    for k in range(1, len(W) - 1):
        a, b = W[k] - W[k - 1], W[k + 1] - W[k]
        c = float(a @ b) / max(1e-12, float(np.linalg.norm(a) * np.linalg.norm(b)))
        cur.append(k)
        if math.degrees(math.acos(max(-1.0, min(1.0, c)))) >= CORNER_DEG:
            runs.append(cur)
            cur = [k]
    cur.append(len(W) - 1)
    runs.append(cur)
    return runs


def _run_motion(P, vmax: float, amax: float, rate_hz: float, t_min: float):
    """Nearly-collinear waypoints P (>= 3) as ONE motion (2026-10-24, operator: smooth, no stop-and-go): a C2
    spline through every waypoint (parameter = the largest joint's distance along the polyline), minimum-jerk
    along it, stretched in time until joint speed and acceleration are within vmax / amax. Returns (t, q)."""
    u = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(P, axis=0)).max(axis=1))])
    L = float(u[-1])
    sp = CubicSpline(u, P, axis=0, bc_type="natural")
    T = max(t_min, 1.875 * L / vmax, math.sqrt(5.774 * L / amax))
    for _ in range(8):
        t = np.linspace(0.0, T, max(3, int(math.ceil(T * rate_hz)) + 1))
        tau = t / T
        q = sp(L * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5))
        v = np.diff(q, axis=0) / np.diff(t)[:, None]
        a = np.diff(v, axis=0) / np.diff(t)[1:, None]
        r = max(float(np.abs(v).max()) / vmax, math.sqrt(float(np.abs(a).max()) / amax))
        if r <= 1.0:
            break
        T *= r * 1.02
    q[0], q[-1] = P[0], P[-1]
    return t, q


# ---------------------------------------------------------------------------------- the plan
@dataclass
class Limits:
    """Planner limits + the safety parameters it shares with the arm node (piper.yaml)."""
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
        raise PlanError(f"the path's start ({np.round(P[0] * 1000, 1).tolist()} mm) is out of reach "
                        "for the corrected arm model (no IK solution within the joint limits)")
    q = min(sols, key=lambda s: float(np.abs(s - q_now).max()))
    knots = [q]
    for k in range(1, len(tk)):
        q, pe, ae = kin.ik(flange_T(P[k], A[k], U[k], lim.tcp_offset_m), q, margin=lim.ik_margin_rad)
        if pe > 2e-4 or ae > 2e-3:
            raise PlanError(f"path point at t = {tk[k]:.2f} s ({np.round(P[k] * 1000, 1).tolist()} mm) is out of reach "
                            f"in the current arm branch (best {pe * 1000:.1f} mm / {math.degrees(ae):.2f} deg)")
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
                    from_outside: bool = False) -> Trajectory:
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
        raise PlanError("a joint path needs at least 2 waypoints of 6 joints")
    keep = [0] + [k for k in range(1, len(W)) if float(np.abs(W[k] - W[k - 1]).max()) >= 1e-6]
    idx, W = keep, W[keep]                        # zero-length segments dropped; idx = the caller's numbering
    ts, qs, segs, t_off = [], [], [], 0.0
    # 2026-10-24: waypoints on a nearly straight run (a planned straight rise, a curve sampled densely) are driven
    # through without stopping; the arm comes to rest only at real corners (CORNER_DEG) and at the end
    for run in (_corner_runs(W) if len(W) >= 2 else []):
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
        raise PlanError(f"joint {j + 1} at {math.degrees(q[i, j]):.1f} deg leaves its limits ({where(i)})")
    dt = np.diff(t)
    if (dt <= 0).any():
        raise PlanError("trajectory time does not increase")
    v = np.diff(q, axis=0) / dt[:, None]
    vmax = np.abs(v).max(axis=0)
    i, j = np.unravel_index(np.abs(v).argmax(), v.shape)
    if abs(v[i, j]) > lim.max_joint_vel_rad_s * 1.001:
        need = ""
        if duration_s and loop and loop[1] <= i < loop[2]:
            # cruise speed ~ 1 / (T - ramp), ramp = min(2 s, T / 4): the loop time that brings this joint to the limit
            r = ramp if ramp is not None else min(2.0, duration_s / 4.0)
            T_need = (duration_s - r) * abs(v[i, j]) / lim.max_joint_vel_rad_s
            T_need = T_need + 2.0 if T_need >= 6.0 else T_need * 4.0 / 3.0
            need = f"; a loop of about {math.ceil(T_need * 1.05)} s or longer fits (--duration)"
        raise PlanError(f"joint {j + 1} would turn at {math.degrees(abs(v[i, j])):.1f} deg/s > the "
                        f"{math.degrees(lim.max_joint_vel_rad_s):.1f} deg/s limit ({where(i)}){need}")
    acc = np.diff(v, axis=0) / dt[1:, None]
    amax = np.abs(acc).max(axis=0) if len(acc) else np.zeros(6)
    if len(acc):
        i, j = np.unravel_index(np.abs(acc).argmax(), acc.shape)
        if abs(acc[i, j]) > lim.max_joint_acc_rad_s2 * 1.001:
            raise PlanError(f"joint {j + 1} would accelerate at {abs(acc[i, j]):.2f} rad/s^2 > the "
                            f"{lim.max_joint_acc_rad_s2:.2f} limit ({where(i + 1)}); slow it down (--duration)")
    a, b, c = lim.floor_plane
    wmin, wmax = np.array(lim.workspace_min), np.array(lim.workspace_max)
    clear_min, dev_m, dev_r = 1e9, 0.0, 0.0
    for k in range(len(t)):
        T, tip = _tip(kin, q[k], lim.tcp_offset_m)
        if not ((wmin <= T[:3, 3]) & (T[:3, 3] <= wmax)).all():
            raise PlanError(f"flange at {np.round(T[:3, 3] * 1000, 1).tolist()} mm leaves the workspace box ({where(k)})")
        if lim.floor_enabled:
            clear = tip[2] - (a * tip[0] + b * tip[1] + c)
            clear_min = min(clear_min, clear)
            if clear < lim.floor_margin_m:
                raise PlanError(f"fingertip {clear * 1000:.1f} mm above the floor < margin "
                                f"{lim.floor_margin_m * 1000:.0f} mm ({where(k)})")
        if at is not None and loop and loop[1] <= k < loop[2]:      # the streamed sample stays on the curve
            u = smooth_fraction(t[k] - t_loop0, duration_s, ramp)
            Pp, Aa, Uu = at(u * L)
            pe, ae = pose_error(T, flange_T(Pp[0], Aa[0], Uu[0], lim.tcp_offset_m))
            dev_m, dev_r = max(dev_m, pe), max(dev_r, ae)
            if pe > lim.max_dev_m or ae > lim.max_dev_rad:
                raise PlanError(f"streamed sample {pe * 1000:.2f} mm / {math.degrees(ae):.2f} deg off the curve ({where(k)})")
    return {"peak_joint_speed_deg_s": [round(math.degrees(x), 1) for x in vmax],
            "peak_joint_acc_rad_s2": [round(float(x), 2) for x in amax],
            "min_floor_clearance_mm": round(float(clear_min) * 1000, 1) if lim.floor_enabled else None,
            "max_curve_deviation_mm": round(dev_m * 1000, 3), "max_curve_deviation_deg": round(math.degrees(dev_r), 3)}
