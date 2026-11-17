"""The ChArUco board offline (harness_vision/detect.py Charuco; harness_calibration/handeye.py; PROJECT_SPEC.md
"2026-11-16 — the ChArUco board"): synthetic images of the lab's board (cv2.aruco CharucoBoard.generateImage: DICT_5X5_100,
9 x 11 squares of 22 mm, markers 16 mm) at known poses before a pinhole camera (4x supersampled, then averaged down:
OpenCV's pixel convention), read by the board detector; then the hand-eye solve on synthetic pairs: a known flange ->
camera transform, the board fixed on the bench, the routine's own viewpoints around a start pose, each view rendered,
detected and posed, the closed-form solvers + the position refinement of handeye.py.

    python src/harness_vision/test/test_charuco.py          # the proof's numbers
    python -m pytest src/harness_vision/test/test_charuco.py -q
"""
import contextlib
import io
import math
import os
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..", "harness_calibration")]
from harness_calibration.handeye import refine, solve, viewpoints  # noqa: E402
from harness_vision.detect import Charuco  # noqa: E402
from harness_vision.geom import T_from_xyzrpy, angle_between, inv  # noqa: E402

K = np.array([[645.0, 0.0, 640.0], [0.0, 645.0, 360.0], [0.0, 0.0, 1.0]])   # ~ the D405 colour stream at 1280 x 720
W, H = 1280, 720
PPM = 10000.0                    # board image: px per metre (220 px per 22 mm square)
MARGIN_M = 0.010                 # white paper round the board
# the hand-eye in use (PROJECT_SPEC.md "Hand-eye calibration log"): flange -> camera (-64.8, 11.5, 76.9) mm,
# rpy (0.69, 0.46, -91.33) deg
X_TRUE = T_from_xyzrpy([-0.0648, 0.0115, 0.0769] + list(np.radians([0.69, 0.46, -91.33])))


def board_source(ch):
    img = ch.board.generateImage((int(round(ch.squares[0] * ch.square_m * PPM)), int(round(ch.squares[1] * ch.square_m * PPM))),
                                 marginSize=0, borderBits=1)
    m = int(round(MARGIN_M * PPM))
    return cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255)


def render(src, T_cam_board, ss=4, bg=140, noise=1.5, seed=0):
    """The board (its frame: x right, y down the sheet, z into it) at camera pose T_cam_board -> a grey 1280 x 720 image."""
    A = np.array([[1 / PPM, 0, 0.5 / PPM - MARGIN_M], [0, 1 / PPM, 0.5 / PPM - MARGIN_M], [0, 0, 1]])
    Kh = K.copy()
    Kh[:2, :2] *= ss
    Kh[:2, 2] = ss * K[:2, 2] + (ss - 1) / 2
    R, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
    Hm = Kh @ np.column_stack([R[:, 0], R[:, 1], t]) @ A
    hi = cv2.warpPerspective(src, Hm, (W * ss, H * ss), flags=cv2.INTER_LINEAR, borderValue=bg)
    lo = cv2.resize(hi, (W, H), interpolation=cv2.INTER_AREA).astype(np.float64)
    lo += np.random.default_rng(seed).normal(0.0, noise, lo.shape)
    return np.clip(lo, 0, 255).astype(np.uint8)


def normalise(pts):
    return (np.asarray(pts, np.float64) - K[:2, 2]) / np.diag(K)[:2]


def board_pose(ch, rng, dist, tilt_max=40.0, off=(0.0, 0.0)):
    """The board's centre `dist` m in front of the camera (shifted by off m in the image plane), facing it, rolled
    at random and tilted up to tilt_max."""
    R = (Rot.from_euler("z", rng.uniform(0, 360), degrees=True) * Rot.from_euler("x", rng.uniform(0, tilt_max), degrees=True)
         * Rot.from_euler("z", rng.uniform(-180, 180), degrees=True)).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([off[0], off[1], dist]) - R @ ch.centre
    return T


def detection_proof(ch, n_per=8, seed=1):
    """Board poses at 150-350 mm (centred, and pushed toward the image's edge; below ~300 mm the 198 x 242 mm sheet does
    not fit the 1280 x 720 image anyway): what the detector finds and the pose error."""
    src, rng, rows = board_source(ch), np.random.default_rng(seed), []
    for dist in (0.15, 0.20, 0.25, 0.30, 0.35):
        for j in range(n_per):
            partial = j >= n_per - 2                          # the last two: the board's centre pushed toward the edge
            off = (rng.choice([-1, 1]) * 0.45 * dist, rng.uniform(-0.1, 0.1) * dist) if partial else \
                (rng.uniform(-0.1, 0.1) * dist, rng.uniform(-0.05, 0.05) * dist)
            T = board_pose(ch, rng, dist, off=off)
            img = render(src, T, seed=j)
            ids, pts, nm = ch.detect(img)
            m = ch.measure(ids, pts, nm, normalise, K[0, 0])
            row = {"dist": dist, "partial": partial, "corners": m["corners"], "markers": nm, "seen": m["seen"]}
            if len(ids):
                p = (K @ (T[:3, :3] @ ch.obj[ids].T + T[:3, 3:4])).T
                row["corner_err_px"] = float(np.abs(pts - p[:, :2] / p[:, 2:]).max())
            if m["seen"]:
                Tc_true, Tc = T @ ch.centre_T(), m["T"] @ ch.centre_T()
                row.update(deg=math.degrees(angle_between(m["T"], T)), mm=float(np.linalg.norm(Tc[:3, 3] - Tc_true[:3, 3]) * 1000),
                           dist_err_mm=(m["distance_m"] - float(np.linalg.norm(Tc_true[:3, 3]))) * 1000,
                           reproj=m["reproj_err_px"])
            rows.append(row)
    return rows


def handeye_proof(ch, flange_noise_mm=0.0, flange_noise_deg=0.0, seed=3):
    """The board on the bench, the camera over it; handeye.viewpoints round the start pose; each view rendered, detected,
    posed (camera -> the board's centre); handeye.solve + refine as handeye.finish does. Returns the recovered
    flange -> camera against X_TRUE."""
    rng = np.random.default_rng(seed)
    src = board_source(ch)
    Rb = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])        # face up on the bench: z into it = down
    Tbb = np.eye(4)
    Tbb[:3, :3] = Rb
    Tbb[:3, 3] = np.array([0.30, 0.0, 0.0]) - Rb @ ch.centre          # its centre at (300, 0, 0) mm in the arm base
    Tbc0 = np.eye(4)                                                  # the camera 240 mm over it, looking down, tilted 10°
    Tbc0[:3, :3] = Rb @ Rot.from_euler("x", 10, degrees=True).as_matrix()
    Tbc0[:3, 3] = [0.30, -0.04, 0.24]
    T0 = Tbc0 @ inv(X_TRUE)                                           # the start flange pose
    pairs, names, seen = [], [], 0
    for name, Tbf in viewpoints(T0, 0.04, math.radians(15.0), math.radians(20.0)):
        T_cam_board = inv(Tbf @ X_TRUE) @ Tbb
        img = render(src, T_cam_board, seed=len(names) + 10)
        ids, pts, nm = ch.detect(img)
        m = ch.measure(ids, pts, nm, normalise, K[0, 0])
        if not m["seen"]:
            continue
        seen += 1
        Tm = Tbf.copy()                                               # the arm's reported flange pose, with its error
        if flange_noise_mm or flange_noise_deg:
            Tm[:3, :3] = Rot.from_rotvec(np.radians(rng.normal(0, flange_noise_deg, 3))).as_matrix() @ Tm[:3, :3]
            Tm[:3, 3] += rng.normal(0, flange_noise_mm / 1000, 3)
        pairs.append((Tm, m["T"] @ ch.centre_T()))
        names.append((name, m["corners"]))
    with contextlib.redirect_stdout(io.StringIO()):
        res = solve(pairs)
    best = min(res, key=lambda k: res[k][1] + res[k][2] * 0.1)
    X0, d_t, d_r = res[best]
    X, k, rms, mx = refine(pairs, X0, fit_scale=True)
    Xn, _, rms_n, mx_n = refine(pairs, X0, fit_scale=False)
    use_scaled = abs(k - 1.0) > 0.02 and rms < 0.8 * rms_n                # handeye.finish's rule
    Xu, rms_u, mx_u = (X, rms, mx) if use_scaled else (Xn, rms_n, mx_n)
    return {"views": len(names), "of": len(viewpoints(T0, 0.04, 0.1, 0.1)), "corners_min": min(c for _, c in names),
            "best": best, "closed_mm": float(np.linalg.norm(X0[:3, 3] - X_TRUE[:3, 3]) * 1000),
            "closed_deg": math.degrees(angle_between(X0, X_TRUE)), "spread_mm": d_t * 1000, "spread_deg": math.degrees(d_r),
            "x_mm": float(np.linalg.norm(Xu[:3, 3] - X_TRUE[:3, 3]) * 1000), "x_deg": math.degrees(angle_between(Xu, X_TRUE)),
            "rms_mm": rms_u * 1000, "max_mm": mx_u * 1000, "k": k, "methods": {n: (float(np.linalg.norm(
                v[0][:3, 3] - X_TRUE[:3, 3]) * 1000), math.degrees(angle_between(v[0], X_TRUE))) for n, v in res.items()}}


# ------------------------------------------------------------------------------------------ tests
def test_the_pdf_board_definition():
    ch = Charuco()
    assert ch.total == 80 and len(ch.board.getIds()) == 49 and ch.board.getIds().max() == 48


def test_board_poses():
    rows = detection_proof(Charuco(), n_per=4)
    assert all(r["seen"] for r in rows), [r for r in rows if not r["seen"]]
    assert all(r["corners"] == 80 for r in rows if not r["partial"] and r["dist"] >= 0.3)   # the whole sheet in view
    for r in [r for r in rows if r["seen"]]:
        assert r["deg"] < 0.3 and r["mm"] < 0.5 and r["corner_err_px"] < 0.5, r


def test_handeye_recovers_the_flange_to_camera():
    r = handeye_proof(Charuco())
    assert r["views"] >= 20 and r["x_mm"] < 0.5 and r["x_deg"] < 0.1 and r["rms_mm"] < 0.5, r
    r = handeye_proof(Charuco(), flange_noise_mm=1.0, flange_noise_deg=0.1)
    assert r["x_mm"] < 3.0 and r["rms_mm"] < 3.0, r


# ------------------------------------------------------------------------------------------ the numbers
if __name__ == "__main__":
    ch = Charuco()
    print(f"OpenCV {cv2.__version__}; board {ch.dict_name} {ch.squares[0]} x {ch.squares[1]} squares of "
          f"{ch.square_m * 1000:g} mm, markers {ch.marker_m * 1000:g} mm, {ch.total} corners, "
          f"{len(ch.board.getIds())} markers (ids {ch.board.getIds().min()}-{ch.board.getIds().max()})")
    rows = detection_proof(ch)
    print("\n1. board poses: random roll, tilt <= 40°; per distance 6 centred + 2 pushed toward the image's edge "
          "(the sheet fits the image whole only from ~300 mm)")
    print("   dist  views  seen  corners (min-max)  corner err px (max)  deg (max)  centre mm (max)  distance err mm  reproj px")
    for d in sorted({r["dist"] for r in rows}):
        g = [r for r in rows if r["dist"] == d]
        s = [r for r in g if r["seen"]]
        print(f"   {d * 1000:3.0f}   {len(g)}      {len(s)}     {min(r['corners'] for r in g):2d}-{max(r['corners'] for r in g):2d}"
              f"              {max(r['corner_err_px'] for r in g if 'corner_err_px' in r):.3f}"
              f"                {max(r['deg'] for r in s):.3f}      {max(r['mm'] for r in s):.3f}"
              f"            {max(abs(r['dist_err_mm']) for r in s):.3f}            {np.mean([r['reproj'] for r in s]):.3f}")
    part = [r for r in rows if r["partial"]]
    print(f"   pushed toward the edge: {sum(r['seen'] for r in part)} / {len(part)} posed, corners "
          f"{sorted(r['corners'] for r in part)}")
    for label, kw in (("exact flange poses", {}), ("flange poses with the arm's error (1 mm, 0.1° per axis, random)",
                                                    {"flange_noise_mm": 1.0, "flange_noise_deg": 0.1})):
        r = handeye_proof(ch, **kw)
        print(f"\n2. hand-eye on synthetic pairs, {label}: {r['views']} / {r['of']} views posed (>= {r['corners_min']} corners)")
        print("   closed form: " + ", ".join(f"{n} {v[0]:.3f} mm / {v[1]:.3f}°" for n, v in r["methods"].items())
              + f"; best {r['best']} (board-centre spread {r['spread_mm']:.2f} mm / {r['spread_deg']:.3f}°)")
        print(f"   after the position refinement: flange -> camera off by {r['x_mm']:.3f} mm / {r['x_deg']:.3f}°; "
              f"residual rms {r['rms_mm']:.2f} mm, max {r['max_mm']:.2f} mm (installed at rms <= 3 mm); scale k {r['k']:.4f}")
