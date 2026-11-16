"""Tag detection without ROS or a camera (harness_vision/detect.py; PROJECT_SPEC.md "2026-11-16 — tagStandard52h13"):
synthetic images of tags drawn from the AprilTag library's own family tables (apriltag.c apriltag_to_image), placed at
known poses before a pinhole camera (4x supersampled, then averaged down: OpenCV's pixel convention, centres at
integers), read by OpenCV aruco and AprilTag 3. The TABLE FRAME = the tag as the tables draw it: x right, y up (the
image's top), z out of the face; the digital twin's tag frame is that frame.

    HARNESS_APRILTAG_LIB=<path/libapriltag.so.3> python -m pytest src/harness_vision/test -q
    python src/harness_vision/test/test_detect.py [libapriltag path]      # the proof's numbers

The AprilTag 3 tests skip when no libapriltag is found (detect.load_apriltag's search).
"""
import ctypes
import itertools
import os
import sys

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from harness_vision import detect as D  # noqa: E402

K = np.array([[645.0, 0.0, 640.0], [0.0, 645.0, 360.0], [0.0, 0.0, 1.0]])   # ~ the D405 colour stream at 1280 x 720
W, H = 1280, 720
GATE = ((0, 0, W, H), (0, 10000))
RZ180 = np.diag([-1.0, -1.0, 1.0])


# ------------------------------------------------------------------------------------------ synthetic images
def lib_family(lib, fam):
    """A family's tables straight from the library (its <family>_create struct)."""
    create = getattr(lib, fam + "_create")
    create.restype = ctypes.POINTER(D._Family)
    return D.family_info(create())


def tag_image(info, idx):
    """apriltag.c apriltag_to_image: total_width x total_width cells, 0 black / 255 white, row 0 = the top."""
    tw, wab, rev = info["total_width"], info["width_at_border"], info["reversed_border"]
    code = info["codes"][idx]
    im = np.zeros((tw, tw), np.uint8)
    wbw = wab + (0 if rev else 2)
    s = (tw - wbw) // 2
    for i in range(wbw - 1):
        im[s, s + i] = 255
        im[s + i, tw - 1 - s] = 255
        im[tw - s - 1, s + i + 1] = 255
        im[s + 1 + i, s] = 255
    bs = (tw - wab) // 2
    nb = info["nbits"]
    for i in range(nb):
        if code & (1 << (nb - i - 1)):
            im[info["bit_y"][i] + bs, info["bit_x"][i] + bs] = 255
    return im


def render(timg, print_m, R, t, ss=4, k=48, margin=3, bg=140, noise=1.5, seed=0):
    """The printed tag (print_m = its total_width cells, on white paper `margin` cells wide) with its table frame at
    camera pose (R, t) -> a 1280 x 720 grey image."""
    tw = timg.shape[0]
    canvas = np.full((tw + 2 * margin, tw + 2 * margin), 255, np.uint8)
    canvas[margin:margin + tw, margin:margin + tw] = timg
    src = cv2.resize(canvas, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
    c = print_m / tw                                       # one cell, m; source pixel centre (u, v) -> table x, y:
    A = np.array([[c / k, 0, (0.5 / k - margin - tw / 2) * c], [0, -c / k, (tw / 2 + margin - 0.5 / k) * c], [0, 0, 1]])
    Kh = K.copy()
    Kh[:2, :2] *= ss
    Kh[:2, 2] = ss * K[:2, 2] + (ss - 1) / 2               # high-res pixel x_h = ss x + (ss - 1) / 2
    Hm = Kh @ np.column_stack([R[:, 0], R[:, 1], t]) @ A
    hi = cv2.warpPerspective(src, Hm, (W * ss, H * ss), flags=cv2.INTER_LINEAR, borderValue=bg)
    lo = cv2.resize(hi, (W, H), interpolation=cv2.INTER_AREA).astype(np.float64)
    lo += np.random.default_rng(seed).normal(0.0, noise, lo.shape)
    return np.clip(lo, 0, 255).astype(np.uint8)


def table_corners(info, print_m, R, t):
    """The detector's square (width_at_border cells) in the image, in the TABLE frame's order: (-h, h), (h, h),
    (h, -h), (-h, -h) = top-left, top-right, bottom-right, bottom-left of the drawn tag."""
    p = (K @ (R @ D.object_points(print_m * D.size_share(info)).T + t[:, None])).T
    return p[:, :2] / p[:, 2:]


def pose(rng, tilt_max=40.0, roll=None):
    """A tag facing the camera: in-plane roll, tilted up to tilt_max in any direction, 150-300 mm away."""
    roll = rng.uniform(-180, 180) if roll is None else roll
    R = (Rot.from_euler("z", rng.uniform(0, 360), degrees=True) * Rot.from_euler("x", rng.uniform(0, tilt_max), degrees=True)
         * Rot.from_euler("x", 180, degrees=True) * Rot.from_euler("z", roll, degrees=True)).as_matrix()
    return R, np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.03, 0.03), rng.uniform(0.15, 0.30)])


def normalise(pts):
    return (np.asarray(pts, np.float64) - K[:2, 2]) / np.diag(K)[:2]


def angle_deg(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def roll_of(c, truth):
    """r such that detected corner k = table corner (k + r) % 4 (2 = the table frame turned 180° about z)."""
    return min(range(4), key=lambda r: np.abs(np.asarray(c, float) - np.roll(truth, -r, axis=0)).max())


@pytest.fixture(scope="module")
def lib():
    lb, where = D.load_apriltag()
    if lb is None:
        pytest.skip(f"no AprilTag 3 library ({where}); set {D.AT3_LIB_ENV}")
    return lb


# ------------------------------------------------------------------------------------------ the proof
def corner_permutation(lib, n=24, families=("tag36h11", "tag16h5"), seed=1):
    """Both decoders on the same synthetic images of the families both read: for each image the permutation s with
    OpenCV corner k ~ AprilTag 3 corner s[k], OpenCV's corner order vs the table's, the corner errors (px)."""
    out = {"perm": {}, "cv_roll": {}, "cv_minus_at3": [], "cv_err": [], "at3_err_raw": [], "missed": 0, "n": 0}
    rng = np.random.default_rng(seed)
    for fam in families:
        info = lib_family(lib, fam)
        at3 = D.AprilTag3(lib, [fam], max_hamming=0)
        cvd = D.cv_detector(fam)
        for j in range(n):
            R, t = pose(rng)
            idx = int(rng.integers(0, len(info["codes"])))
            img = render(tag_image(info, idx), 0.05, R, t, seed=j)
            truth = table_corners(info, 0.05, R, t)
            corners, ids, _ = cvd.detectMarkers(img)
            raw = at3.detect_raw(img)
            if ids is None or len(raw) != 1 or len(ids) != 1 or int(ids.flatten()[0]) != idx or raw[0][1] != idx:
                out["missed"] += 1
                continue
            c, p = corners[0].reshape(4, 2).astype(np.float64), raw[0][2]
            s = min(itertools.permutations(range(4)), key=lambda q: np.abs(c - p[list(q)]).max())
            out["perm"][(fam, s)] = out["perm"].get((fam, s), 0) + 1
            r = roll_of(c, truth)
            out["cv_roll"][(fam, r)] = out["cv_roll"].get((fam, r), 0) + 1
            tr = np.roll(truth, -r, axis=0)
            out["cv_minus_at3"].append(c - p[list(s)])
            out["cv_err"].append(c - tr)
            out["at3_err_raw"].append(p[list(s)] - tr)
            out["n"] += 1
        at3.close()
    for k in ("cv_minus_at3", "cv_err", "at3_err_raw"):
        out[k] = np.array(out[k]).reshape(-1, 2)
    return out


TAGS = [("tagStandard52h13", 0, 0.050), ("tagStandard52h13", 4102, 0.050), ("tagStandard52h13", 4103, 0.050),
        ("tagStandard52h13", 4104, 0.050), ("tagStandard52h13", 3970, 0.030), ("tagStandard41h12", 0, 0.050),
        ("tagStandard41h12", 77, 0.040), ("tag36h11", 3, 0.0275), ("tag36h11", 0, 0.024), ("tag16h5", 7, 0.050)]


def poses_of_known_tags(lib, rolls=(0, 90, 180, 270, 33, -150), seed=2):
    """Every tag of TAGS at every in-plane roll (plus a random tilt and place), through the full Detector (both
    decoders) and measure() with size = print x width_at_border / total_width: the reported pose against the table
    frame turned 180° about its normal (OpenCV's convention, the twin's teach.py) and against the table frame itself."""
    det = D.Detector(["tag16h5", "tag36h11", "tagStandard52h13", "tagStandard41h12"])
    infos = {f: lib_family(lib, f) for f in D.FAMILIES if f != "tag25h9"}
    rng = np.random.default_rng(seed)
    rows = []
    for fam, idx, print_m in TAGS:
        info = infos[fam]
        for j, roll in enumerate(rolls):
            R, t = pose(rng, roll=roll)
            img = render(tag_image(info, idx), print_m, R, t, seed=100 + j)
            truth = table_corners(info, print_m, R, t)
            hits = [(f, c, i) for f, c, i in det.detect(img) if f == fam and i == idx]
            if len(hits) != 1:
                rows.append({"fam": fam, "id": idx, "roll": roll, "found": False})
                continue
            f, c, i = hits[0]
            m = D.measure(f, c, i, print_m * D.size_share(info), normalise, K[0, 0], *GATE)
            Rd, td = m["T"][:3, :3], m["T"][:3, 3]
            rows.append({"fam": fam, "id": idx, "roll": roll, "found": True, "print_mm": print_m * 1000,
                         "size_mm": print_m * D.size_share(info) * 1000,
                         "err_deg_vs_turned": angle_deg(Rd, R @ RZ180), "err_deg_vs_table": angle_deg(Rd, R),
                         "err_mm": float(np.linalg.norm(td - t) * 1000), "range_mm": float(np.linalg.norm(t) * 1000),
                         "corner_roll": roll_of(c, truth), "corner_err_px": float(np.abs(c - np.roll(truth, -2, axis=0)).max()),
                         "edge_px": m["size_px"]})
    det.close()
    return rows


# ------------------------------------------------------------------------------------------ tests
def test_corner_permutation_and_pixel_shift(lib):
    r = corner_permutation(lib)
    assert r["n"] >= 40 and r["missed"] <= 2
    assert {s for _, s in r["perm"]} == {D.AT3_TO_CV}                     # one permutation on every image, both families
    shift = r["cv_minus_at3"].mean(axis=0)
    assert np.allclose(shift, D.AT3_PIXEL_SHIFT_PX, atol=0.03)            # AprilTag 3's half-pixel convention
    assert np.abs(r["at3_err_raw"] + D.AT3_PIXEL_SHIFT_PX).max() < 0.2    # its corners, shifted, on the truth
    assert np.abs(r["cv_err"]).max() < 0.6


def test_opencv_holds_the_tables_turned_180(lib):
    """OpenCV's corner 0 is the drawn tag's bottom-right for 36h11 and 16h5 (the twin's 180° turn, teach.py)."""
    r = corner_permutation(lib, n=8)
    assert {roll for _, roll in r["cv_roll"]} == {2}


def test_known_tags_report_opencvs_frame(lib):
    rows = poses_of_known_tags(lib)
    assert all(x["found"] for x in rows), [x for x in rows if not x["found"]]
    for x in rows:
        assert x["corner_roll"] == 2, x
        # the table frame turned 180° about z, to PnP's own orientation noise (a near-frontal 64 px square: ~1-2°);
        # any other convention would be 90° or 180° off
        assert x["err_deg_vs_turned"] < 3.0, x
        assert abs(x["err_deg_vs_table"] - 180.0) < 3.0, x
        assert x["err_mm"] < 2.0, x
        assert x["corner_err_px"] < 0.6, x


def test_size_share_from_the_library(lib):
    assert D.size_share(lib_family(lib, "tagStandard52h13")) == pytest.approx(6 / 10)
    assert D.size_share(lib_family(lib, "tagStandard41h12")) == pytest.approx(5 / 9)
    assert D.size_share(lib_family(lib, "tag36h11")) == pytest.approx(8 / 10)
    assert D.size_share(lib_family(lib, "tag16h5")) == pytest.approx(6 / 8)


def test_decode_table_stays_small(lib):
    at3 = D.AprilTag3(lib, list(D.AT3_FAMILIES), max_hamming=2)
    assert at3.hamming == {"tagStandard52h13": 1, "tagStandard41h12": 2}   # 52h13 at 2 bits = 6.3 GB: lowered
    at3.close()


def test_without_the_library_the_opencv_families_still_work(monkeypatch):
    monkeypatch.delenv(D.AT3_LIB_ENV, raising=False)
    monkeypatch.setattr(D, "AT3_LIB_NAMES", ("libapriltag-absent.so.3",))
    monkeypatch.setattr(D, "AT3_LIB_DIRS", ())
    monkeypatch.setattr(D.ctypes.util, "find_library", lambda name: None)
    det = D.Detector(["tag16h5", "tag36h11", "tagStandard52h13"], apriltag_lib="/nonexistent/libapriltag.so.3")
    assert det.families == ["tag16h5", "tag36h11"] and list(det.missing) == ["tagStandard52h13"]
    img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 3, 200)
    img = cv2.copyMakeBorder(img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    assert [(f, i) for f, _, i in det.detect(img)] == [("tag36h11", 3)]


def test_unknown_family_and_sizes():
    with pytest.raises(ValueError):
        D.Detector(["tag36h10"])
    assert D.parse_tag_sizes(["tagStandard52h13:4102=0.03", "", "tag16h5:7=0.0375"]) == {
        ("tagStandard52h13", 4102): 0.03, ("tag16h5", 7): 0.0375}
    with pytest.raises(ValueError):
        D.parse_tag_sizes(["tagStandard52h13=0.03"])
    assert [D.short_family(f) for f in ("tag36h11", "tagStandard52h13")] == ["36h11", "52h13"]


def test_measure_matches_the_old_node_code():
    """measure() = tag_node's inline code before 2026-11-16 (an OpenCV detection, float32 corners)."""
    rng = np.random.default_rng(5)
    pts = (np.array([[600, 300], [700, 305], [695, 402], [598, 398]]) + rng.normal(0, 0.3, (4, 2))).astype(np.float32)
    depth = np.full((H, W), 210, np.uint16)
    m = D.measure("tag36h11", pts, 3, 0.022, normalise, K[0, 0], (500, 200, 800, 500), (40, 260), depth, 0.001)
    size = 0.022
    h = size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, normalise(pts), np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok
    proj, _ = cv2.projectPoints(obj, rvec, tvec, np.eye(3), None)
    err = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - normalise(pts), axis=1))) * K[0, 0]
    cx, cy = pts.mean(axis=0)
    assert m["center_px"] == [float(cx), float(cy)] and m["corners_px"] == [float(v) for v in pts.flatten()]
    assert m["size_px"] == float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]))
    assert np.allclose(m["T"][:3, 3], tvec.ravel(), atol=0) and m["reproj_err_px"] == err
    assert m["in_center_square"] and m["depth_m"] == pytest.approx(0.21)
    assert m["depth_size_m"] == pytest.approx(size * 0.21 / float(np.linalg.norm(tvec)))


# ------------------------------------------------------------------------------------------ the numbers
if __name__ == "__main__":
    if len(sys.argv) > 1:
        os.environ[D.AT3_LIB_ENV] = sys.argv[1]
    lb, where = D.load_apriltag()
    if lb is None:
        sys.exit(f"no AprilTag 3 library: {where}")
    print(f"libapriltag: {where}; OpenCV {cv2.__version__}")
    r = corner_permutation(lb)
    print(f"\n1. corner permutation, {r['n']} images (tag36h11 + tag16h5, both decoders), {r['missed']} missed")
    for (fam, s), n in sorted(r["perm"].items()):
        print(f"   {fam}: OpenCV corner k = AprilTag 3 corner {s}[k] on {n} images")
    d = r["cv_minus_at3"]
    print(f"   OpenCV - AprilTag 3 (permuted): mean {np.round(d.mean(0), 3).tolist()} px, sd "
          f"{np.round(d.std(0), 3).tolist()} px  -> AT3_PIXEL_SHIFT_PX {D.AT3_PIXEL_SHIFT_PX}")
    a, c = r["at3_err_raw"] + D.AT3_PIXEL_SHIFT_PX, r["cv_err"]
    print(f"   corner error vs the truth: OpenCV rms {np.sqrt((c ** 2).mean()):.3f} max {np.abs(c).max():.3f} px; "
          f"AprilTag 3 shifted rms {np.sqrt((a ** 2).mean()):.3f} max {np.abs(a).max():.3f} px")
    print(f"   OpenCV corner 0 = table corner (0 + r) % 4: {dict(sorted(r['cv_roll'].items()))} (2 = turned 180°)")
    for f in ("tag16h5", "tag36h11", "tagStandard41h12", "tagStandard52h13"):
        info = lib_family(lb, f)
        print(f"   {f}: width_at_border {info['width_at_border']} / total_width {info['total_width']} = "
              f"{D.size_share(info):.4f}{' (reversed border)' if info['reversed_border'] else ''}")
    rows = poses_of_known_tags(lb)
    print("\n2. known tags at rolls 0/90/180/270/33/-150 + tilt <= 40°, 150-300 mm, through Detector + measure():")
    print("   family            id   print  size   found  corner0  deg vs table turned 180 (mean / max)  deg vs table (min)  mm (max)")
    for (fam, idx), grp in itertools.groupby(rows, key=lambda x: (x["fam"], x["id"])):
        g = list(grp)
        ok = [x for x in g if x["found"]]
        print(f"   {fam:17s} {idx:5d}  {g[0].get('print_mm', 0):4.1f}  {ok[0]['size_mm'] if ok else 0:4.1f}  "
              f"{len(ok)}/{len(g)}    {sorted({x['corner_roll'] for x in ok})}      "
              f"{np.mean([x['err_deg_vs_turned'] for x in ok]):.2f} / {max(x['err_deg_vs_turned'] for x in ok):.2f}"
              f"                          {min(x['err_deg_vs_table'] for x in ok):.1f}             "
              f"{max(x['err_mm'] for x in ok):.2f}")
    print("   (deg vs the table frame turned 180° = PnP's orientation noise; deg vs the table frame itself ~ 180)")
