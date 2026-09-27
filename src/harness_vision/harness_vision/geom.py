"""Small rigid-transform helpers shared by the harness nodes.

Conventions
-----------
* A transform is a 4x4 numpy array T such that p_parent = T @ p_child.
* The PIPER reports the flange pose as [x, y, z, roll, pitch, yaw] in metres/radians
  with fixed-axis XYZ angles: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).  (Assumed — verify
  with `handeye --check-convention` before trusting pose moves.)
* Camera optical frame: x right, y down, z forward (OpenCV / REP 104).
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def T_from_xyzrpy(v):
    x, y, z, r, p, yw = v
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", [r, p, yw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def xyzrpy_from_T(T):
    r, p, yw = R.from_matrix(T[:3, :3]).as_euler("xyz")
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), float(r), float(p), float(yw)]


def T_from_rvec_tvec(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(np.asarray(rvec).reshape(3)).as_matrix()
    T[:3, 3] = np.asarray(tvec).reshape(3)
    return T


def T_from_pose_msg(pose):
    q = pose.orientation
    T = np.eye(4)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def pose_msg_from_T(T, pose):
    """Fill a geometry_msgs/Pose in place."""
    q = R.from_matrix(T[:3, :3]).as_quat()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
    return pose


def inv(T):
    Ti = np.eye(4)
    Rt = T[:3, :3].T
    Ti[:3, :3] = Rt
    Ti[:3, 3] = -Rt @ T[:3, 3]
    return Ti


def rot_about(axis, angle):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(np.asarray(axis, float) / np.linalg.norm(axis) * angle).as_matrix()
    return T


def trans(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


def angle_between(Ta, Tb):
    """Rotation angle (rad) between the orientations of two transforms."""
    dR = Ta[:3, :3].T @ Tb[:3, :3]
    c = (np.trace(dR) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, c))))


def average_T(Ts):
    """Mean pose: mean translation + chordal-mean rotation (quaternion averaging)."""
    Ts = list(Ts)
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    Rm = R.from_matrix([T[:3, :3] for T in Ts]).mean().as_matrix()
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T


def spread(Ts, Tmean=None):
    """(max translation deviation m, max rotation deviation rad) from the mean."""
    Tmean = average_T(Ts) if Tmean is None else Tmean
    dt = max(float(np.linalg.norm(T[:3, 3] - Tmean[:3, 3])) for T in Ts)
    dr = max(angle_between(T, Tmean) for T in Ts)
    return dt, dr
