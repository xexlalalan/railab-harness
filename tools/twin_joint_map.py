#!/usr/bin/env python3
"""Derive the digital twin's PIPER joint map (railab-workflow modules.yaml `real_joints`) from the two
kinematic models — no motion, read-only. Run on the harness PC (needs pyAgxArm + harness_arm):

    source rosenv.sh && python3 tools/twin_joint_map.py <path to the twin's orchestrator_piper/kinematics.json> [j1,..,j6 deg now]

Result 2026-10-23: j2 +82.22, j3 sign -1 offset -167.22, j5 sign -1, the rest identity (PROJECT_SPEC.md
"Twin control gateway" / the twin spec section 10.7).

SDK side: ROBOT_MDH_PRESET["piper"] (Craig MDH, what the harness's corrected FK is built on).
Twin side: railab-workflow protocol/models/orchestrator_piper/kinematics.json (axes picked from the CAD).
Both describe the same arm in the same base frame; a joint's axis line must coincide, its direction
gives the sign, and the angle that brings the NEXT axis (or the flange 'up') into place gives the
offset. Branches that stay consistent are verified at random postures against the SDK flange frame."""
import json, math, sys, itertools
import numpy as np
from pyAgxArm.api.constants import ROBOT_MDH_PRESET
from harness_arm.kinematics import _link, JOINT_LIMITS_RAD   # noqa: E402 (after the docstring)

mdh = [(float(d), float(a), float(al), float(off)) for d, a, al, off in ROBOT_MDH_PRESET["piper"]]
print("SDK MDH (d m, a m, alpha deg, theta_off deg):")
for i, (d, a, al, off) in enumerate(mdh):
    print("  j%d  d=%.4f a=%.4f alpha=%.2f off=%.2f" % (i + 1, d, a, math.degrees(al), math.degrees(off)))

def sdk_frames(q):
    """T_0..i for i = 1..6 (metres)."""
    T = np.eye(4); out = []
    for (d, a, al, off), qi in zip(mdh, q):
        T = T @ _link(d, a, al, qi + off); out.append(T.copy())
    return out

kin = json.load(open(sys.argv[1]))
links = {l["id"]: l for l in kin["links"]}
J = ["j%d" % i for i in range(1, 7)]
jl = {j: next(l for l in kin["links"] if l.get("joint") and l["joint"]["id"] == j) for j in J}
tool = kin["tool"]

def rot(axis, p, deg):
    a = np.asarray(axis, float); a /= np.linalg.norm(a); th = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(p, float) - R @ np.asarray(p, float)
    return T

def twin_T(vals):
    """link id -> posed transform (mm) for twin joint values (deg); missing = modelled."""
    out = {}
    def of(lid):
        if lid in out: return out[lid]
        l = links[lid]
        base = of(l["parent"]) if l.get("parent") else np.eye(4)
        j = l.get("joint")
        T = base @ rot(j["axis"], j["point_mm"], vals.get(j["id"], j["modelled"]) - j["modelled"]) if j else base
        out[lid] = T; return T
    for lid in links: of(lid)
    return out

def twin_axis(j, vals):
    """(point mm, unit direction) of twin joint j's axis at `vals` (its own value is irrelevant)."""
    l = jl[j]; Tp = twin_T(vals)[l["parent"]] if l.get("parent") else np.eye(4)
    jt = l["joint"]; d = np.asarray(jt["axis"], float); d /= np.linalg.norm(d)
    return Tp[:3, :3] @ np.asarray(jt["point_mm"], float) + Tp[:3, 3], Tp[:3, :3] @ d

def twin_tool(vals):
    T = twin_T(vals)[tool["link"]]
    p = T[:3, :3] @ np.asarray(tool["point_mm"], float) + T[:3, 3]
    return p, T[:3, :3] @ np.asarray(tool["x_axis"], float), T[:3, :3] @ np.asarray(tool["z_axis"], float)

def perp(v, a): v = np.asarray(v, float); return v - np.dot(v, a) * a
def ang_about(v0, v1, a):
    v0, v1 = perp(v0, a), perp(v1, a)
    return math.degrees(math.atan2(np.dot(np.cross(v0, v1), a), np.dot(v0, v1)))
def line_dist(p, p0, d): v = np.asarray(p) - np.asarray(p0); return float(np.linalg.norm(v - np.dot(v, d) * d))

F0 = sdk_frames([0.0] * 6)
sdk_axes = [(T[:3, 3] * 1000.0, T[:3, 2]) for T in F0]           # (point mm, z) of joint i at q = 0
flange_up = -F0[5][:3, 0]                                        # harness convention: up = flange -x

# ---- tree search over the 180-degree ambiguities
branches = [({}, [])]                                            # (twin values at SDK q=0, signs)
for i, j in enumerate(J):
    nxt = []
    for vals, signs in branches:
        p_s, z_s = sdk_axes[i]
        p_t, d_t = twin_axis(j, vals)
        if abs(abs(np.dot(z_s, d_t)) - 1) > 1e-3 or line_dist(p_s, p_t, d_t) > 1.5:
            continue                                             # this branch's axis i is not the SDK's
        sign = 1 if np.dot(z_s, d_t) > 0 else -1
        if i < 5:
            p_s2, z_s2 = sdk_axes[i + 1]
            p_t2, d_t2 = twin_axis(J[i + 1], vals)
            if np.linalg.norm(np.cross(z_s2, d_t)) > 0.05:      # next axis not parallel: match its direction
                base = ang_about(d_t2, z_s2, d_t)
            else:                                                # parallel: match where it sits
                base = ang_about(p_t2 - p_t, p_s2 - p_t, d_t)
        else:
            _, _, up_t = twin_tool(vals)
            base = ang_about(up_t, flange_up, d_t)
        for extra in (0.0, 180.0):
            v = dict(vals); v[j] = jl[j]["joint"]["modelled"] + ((base + extra + 180) % 360 - 180)
            nxt.append((v, signs + [sign]))
    branches = nxt
print("\nconsistent branches:", len(branches))

# ---- verify each branch at random postures against the SDK flange frame
rng = np.random.default_rng(1)
Q = [[rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS_RAD] for _ in range(60)]
results = []
for vals, signs in branches:
    tw0 = [vals[j] for j in J]
    rows = [{"joint": j, "real": i + 1, "sign": s, "offset_deg": round(-s * t, 3)} for i, (j, s, t) in enumerate(zip(J, signs, tw0))]
    worst_ang, Ls = 0.0, []
    for q in Q:
        Tf = sdk_frames(q)[5]; approach, up, pf = Tf[:3, 2], -Tf[:3, 0], Tf[:3, 3] * 1000
        tv = {j: (math.degrees(q[i]) - r["offset_deg"]) * r["sign"] for i, (j, r) in enumerate(zip(J, rows))}
        p, xa, za = twin_tool(tv)
        worst_ang = max(worst_ang, math.degrees(math.acos(max(-1, min(1, np.dot(xa, approach))))), math.degrees(math.acos(max(-1, min(1, np.dot(za, up))))))
        Ls.append(np.dot(p - pf, approach)); Ls.append(np.linalg.norm(perp(p - pf, approach)))
    along = [Ls[k] for k in range(0, len(Ls), 2)]; off = [Ls[k] for k in range(1, len(Ls), 2)]
    results.append((rows, worst_ang, float(np.mean(along)), float(np.std(along)), float(max(off))))
results.sort(key=lambda r: (r[1], sum(abs(x["offset_deg"]) for x in r[0])))
for rows, wa, L, Lsd, off in results:
    print("\n  map:", "  ".join("%s:%s%d%+.2f" % (r["joint"], "+" if r["sign"] > 0 else "-", r["real"], r["offset_deg"]) for r in rows))
    print("  worst orientation error %.3f deg | tool point %.1f +/- %.2f mm beyond the flange along the approach, %.2f mm off-axis" % (wa, L, Lsd, off))
print("\nBEST (as modules.yaml real_joints):")
print(json.dumps(results[0][0]))
# the current real posture, for a sanity line
if len(sys.argv) > 2:
    q = [math.radians(float(x)) for x in sys.argv[2].split(",")]
    rows = results[0][0]
    tv = {j: (math.degrees(q[i]) - r["offset_deg"]) * r["sign"] for i, (j, r) in enumerate(zip(J, rows))}
    print("current real joints -> twin joints:", {k: round(v, 1) for k, v in tv.items()})
