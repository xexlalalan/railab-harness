"""Harness-side kinematic model of the PIPER (correction layer over the controller).

The controller's reported flange pose is FK(reported joints) with ITS model, and that
model's joint-5 zero is ~7.5 deg off the real arm (identified 2026-09-18, see
PROJECT_SPEC.md and calib/kinematics.yaml). This module owns the corrected model:

    theta_i = q_reported_i + mdh_theta_offset_i + joint_zero_offset_i

FK is the same modified-DH chain the SDK ships (ROBOT_MDH_PRESET["piper"], verified to
0.4 mm against the controller with zero offsets). IK is numerical, seeded from the current
joints, so the solution stays in the wrist branch the arm is in (no controller-side
branch flips).

Pose convention: [x, y, z, roll, pitch, yaw], fixed-axis XYZ (R = Rz(yaw) Ry(pitch) Rx(roll)),
the same as the PIPER controller and /arm/tcp_pose.
"""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

# j4 +-100 deg and j5 +-70 deg measured on the controller (it clamps silently); datasheet said 106 / 70.
JOINT_LIMITS_RAD = [(-2.687, 2.687), (0.0, 3.403), (-3.054, 0.0),
                    (-1.745, 1.745), (-1.222, 1.222), (-2.094, 2.094)]


def _link(d, a, alpha, theta):
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0.0, a],
                     [ca * st, ca * ct, -sa, -sa * d],
                     [sa * st, sa * ct, ca, ca * d],
                     [0.0, 0.0, 0.0, 1.0]])


def T_from_xyzrpy(pose):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", pose[3:6]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def xyzrpy_from_T(T):
    r, p, y = Rot.from_matrix(T[:3, :3]).as_euler("xyz")
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), float(r), float(p), float(y)]


def pose_error(Ta, Tb):
    """(position error m, orientation error rad) between two 4x4 poses."""
    pe = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    ae = float(np.linalg.norm(Rot.from_matrix(Ta[:3, :3].T @ Tb[:3, :3]).as_rotvec()))
    return pe, ae


class PiperKinematics:
    def __init__(self, mdh, joint_zero_offsets=None, limits=JOINT_LIMITS_RAD, alpha_offsets=None):
        """joint_zero_offsets: added to each joint angle (rad). alpha_offsets: added to each link's
        MDH twist alpha (rad) - e.g. link 5 = the angle between the joint-4 and joint-5 axes."""
        al = np.array(alpha_offsets if alpha_offsets is not None else np.zeros(6), dtype=float)
        self.mdh = [(float(d), float(a), float(alpha) + float(al[i]), float(off)) for i, (d, a, alpha, off) in enumerate(mdh)]
        self.alpha_offsets = al
        self.offsets = np.array(joint_zero_offsets if joint_zero_offsets is not None else np.zeros(6), dtype=float)
        self.limits = [tuple(l) for l in limits]
        if len(self.mdh) != 6 or len(self.offsets) != 6:
            raise ValueError("need 6 MDH links and 6 joint offsets")

    @classmethod
    def from_sdk(cls, model="piper", joint_zero_offsets=None, alpha_offsets=None):
        from pyAgxArm.api.constants import ROBOT_MDH_PRESET
        return cls(ROBOT_MDH_PRESET[model], joint_zero_offsets, alpha_offsets=alpha_offsets)

    @classmethod
    def from_file(cls, path, model="piper"):
        """Corrected model from calib/kinematics.yaml (joint_zero_offsets_rad, alpha_offsets_rad)."""
        import yaml
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        return cls.from_sdk(model, doc.get("joint_zero_offsets_rad"), doc.get("alpha_offsets_rad"))

    # ------------------------------------------------------------------ FK
    def fk(self, q):
        T = np.eye(4)
        for (d, a, alpha, off), qi, oi in zip(self.mdh, q, self.offsets):
            T = T @ _link(d, a, alpha, float(qi) + off + oi)
        return T

    def fk_xyzrpy(self, q):
        return xyzrpy_from_T(self.fk(q))

    # ------------------------------------------------------------------ IK
    def ik(self, T_target, seed, margin=0.0, pos_tol=5e-5, ang_tol=2e-4):
        """Numerical IK seeded from `seed` (rad). Returns (q, pos_err_m, ang_err_rad).
        Joint limits (minus margin) are hard bounds. Because the solve starts at the
        current joints it converges to the nearest solution, i.e. stays in the current
        wrist/elbow branch. Convergence is the caller's decision via the returned errors."""
        seed = np.array(seed, dtype=float)
        lo = np.array([l[0] + margin for l in self.limits])
        hi = np.array([l[1] - margin for l in self.limits])
        x0 = np.clip(seed, lo + 1e-6, hi - 1e-6)
        Rt = T_target[:3, :3]
        pt = T_target[:3, 3]

        def resid(q):
            T = self.fk(q)
            return np.concatenate([(T[:3, 3] - pt) * 1000.0,
                                   Rot.from_matrix(Rt.T @ T[:3, :3]).as_rotvec() * 100.0])

        s = least_squares(resid, x0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=400)
        q = s.x
        pe, ae = pose_error(self.fk(q), T_target)
        return q, pe, ae

    def joints_outside(self, q, margin=0.0):
        return [i + 1 for i, (qi, (lo, hi)) in enumerate(zip(q, self.limits)) if qi < lo + margin or qi > hi - margin]

    # ------------------------------------------------------------------ paths
    @staticmethod
    def interpolate(T0, T1, step_m=0.02, step_rad=0.1):
        """Waypoints from T0 (exclusive) to T1 (inclusive): linear position, slerp orientation."""
        pe, ae = pose_error(T0, T1)
        n = max(1, int(math.ceil(max(pe / step_m, ae / step_rad))))
        R0, R1 = Rot.from_matrix(T0[:3, :3]), Rot.from_matrix(T1[:3, :3])
        dR = R0.inv() * R1
        rv = dR.as_rotvec()
        out = []
        for k in range(1, n + 1):
            f = k / n
            T = np.eye(4)
            T[:3, :3] = (R0 * Rot.from_rotvec(rv * f)).as_matrix()
            T[:3, 3] = T0[:3, 3] * (1 - f) + T1[:3, 3] * f
            out.append(T)
        return out
