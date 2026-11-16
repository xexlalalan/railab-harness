"""AprilTag detection on one grey image: pure functions, no ROS, no camera. tag_node runs them on every frame;
src/harness_vision/test/test_detect.py runs them on synthetic images.

Two decoders, one report:
  * OpenCV aruco (its DICT_APRILTAG_* dictionaries): tag16h5, tag25h9, tag36h11, as since 2026-09-16.
  * AprilTag 3 (AprilRobotics' C library libapriltag, through ctypes): tagStandard52h13, tagStandard41h12
    (2026-11-16: the lab's new tags; OpenCV has no tagStandard dictionary). The library (3.4.5) comes from apt:
    Ubuntu's `libapriltag3t64` (/usr/lib/x86_64-linux-gnu) or `ros-lyrical-apriltag` (/opt/ros/lyrical/lib...);
    without it the OpenCV families are still decoded and `Detector.missing` says why the others are not.

ONE CONVENTION FOR EVERY FAMILY (the contract with the digital twin, 2026-11-16): a detection is reported in
OpenCV aruco's corner order and tag frame for the same physical tag, so the twin's 180° turn about the tag normal
(its teach.py header) holds for every family. AprilTag 3's corners p[0..3] are re-ordered by AT3_TO_CV
(corner k in OpenCV's order = AprilTag 3's corner AT3_TO_CV[k]) - found and proved on synthetic tag36h11 / tag16h5
images that both decoders read (test_detect.py) - and moved from AprilTag 3's pixel convention to OpenCV's
(AT3_PIXEL_SHIFT_PX, measured the same way). The pose is the same for every family: solvePnP(IPPE_SQUARE) on the
device-normalised corners, object points [-h, h], [h, h], [h, -h], [-h, -h] (x right, y up, z out of the tag).

SIZES: a tag's size for the pose = the edge of the square the detector finds. tag16h5 / 25h9 / 36h11: the black
square = printed size x width_at_border / total_width (6/8, 7/9, 8/10). tagStandard52h13 / 41h12 ("reversed
border"): the outer edge of the white ring between the outer and the inner data bits, the same share of the printed
tag (6/10, 5/9) - the printed tag = total_width cells, its outer data ring included (what apriltag_to_image draws and
the twin prints).
"""
import ctypes
import ctypes.util
import os
import re

import cv2
import numpy as np

from harness_vision.geom import T_from_rvec_tvec

CV_FAMILIES = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}
AT3_FAMILIES = ("tagStandard52h13", "tagStandard41h12")
FAMILIES = tuple(CV_FAMILIES) + AT3_FAMILIES

# AprilTag 3 corner p[i] is the homography's image of tag point ((-1, 1), (1, 1), (1, -1), (-1, -1))[i] in its
# decoding frame (apriltag.c); OpenCV aruco's corner k = AprilTag 3's corner AT3_TO_CV[k] for the same physical tag
# (test_detect.py: the same permutation on all 48 synthetic tag36h11 / tag16h5 images, which both decoders read)
AT3_TO_CV = (1, 0, 3, 2)
# AprilTag 3 puts pixel centres at +0.5, OpenCV at integers: corner_cv = corner_at3 + this (test_detect.py: OpenCV -
# AprilTag 3 = -0.50 px on average; shifted, AprilTag 3's corners are within 0.09 px of the truth, OpenCV's within 0.4)
AT3_PIXEL_SHIFT_PX = -0.5

# libapriltag: the name first (the loader's cache: Ubuntu's package; LD_LIBRARY_PATH: rosenv.sh puts ROS's on it), then
# where apt puts it
AT3_LIB_ENV = "HARNESS_APRILTAG_LIB"
AT3_LIB_NAMES = ("libapriltag.so.3", "libapriltag.so")
AT3_LIB_DIRS = ("/usr/lib/x86_64-linux-gnu", "/opt/ros/lyrical/lib", "/opt/ros/lyrical/lib/x86_64-linux-gnu",
                "/usr/lib/aarch64-linux-gnu", "/usr/local/lib")
AT3_INSTALL = "sudo apt install libapriltag3t64"
# AprilTag 3's decode table per family = ncodes x (1 + nbits [+ nbits (nbits - 1)]) x 3 x 16 bytes for 0 / 1 / 2 bits
# corrected: tagStandard52h13 2.3 MB / 124 MB / 6.3 GB. max_hamming is lowered per family to stay under this.
AT3_TABLE_MAX_BYTES = 300e6


def short_family(fam):
    """'36h11' for tag36h11, '52h13' for tagStandard52h13 (overlay text, messages)."""
    m = re.search(r"(\d+h\d+)$", str(fam))
    return m.group(1) if m else str(fam)


def parse_tag_sizes(entries):
    """tag_sizes ["family:id=size_m", ...] -> {(family, id): size_m}; '' entries are skipped."""
    sizes = {}
    for e in entries or []:
        e = str(e).strip()
        if not e:
            continue
        try:
            fam, rest = e.split(":", 1)
            i, s = rest.split("=", 1)
            sizes[(fam.strip(), int(i))] = float(s)
        except ValueError:
            raise ValueError(f"tag_sizes entry {e!r}: expected family:id=size_m, e.g. tag36h11:3=0.024") from None
    return sizes


# ---------------------------------------------------------------------------------------------- AprilTag 3 (ctypes)
class _Family(ctypes.Structure):          # apriltag_family_t (AprilTag 3.x; the fields before `impl`)
    _fields_ = [("ncodes", ctypes.c_uint32), ("codes", ctypes.POINTER(ctypes.c_uint64)),
                ("width_at_border", ctypes.c_int), ("total_width", ctypes.c_int), ("reversed_border", ctypes.c_bool),
                ("nbits", ctypes.c_uint32), ("bit_x", ctypes.POINTER(ctypes.c_int32)),
                ("bit_y", ctypes.POINTER(ctypes.c_int32)), ("h", ctypes.c_uint32), ("name", ctypes.c_char_p)]


class _Detector(ctypes.Structure):        # apriltag_detector_t: only its leading, user-set fields
    _fields_ = [("nthreads", ctypes.c_int), ("quad_decimate", ctypes.c_float), ("quad_sigma", ctypes.c_float),
                ("refine_edges", ctypes.c_bool), ("decode_sharpening", ctypes.c_double)]


class _Image(ctypes.Structure):           # image_u8_t
    _fields_ = [("width", ctypes.c_int32), ("height", ctypes.c_int32), ("stride", ctypes.c_int32),
                ("buf", ctypes.POINTER(ctypes.c_uint8))]


class _Zarray(ctypes.Structure):          # zarray_t
    _fields_ = [("el_sz", ctypes.c_size_t), ("size", ctypes.c_int), ("alloc", ctypes.c_int),
                ("data", ctypes.c_void_p)]


class _Detection(ctypes.Structure):       # apriltag_detection_t
    _fields_ = [("family", ctypes.POINTER(_Family)), ("id", ctypes.c_int), ("hamming", ctypes.c_int),
                ("decision_margin", ctypes.c_float), ("H", ctypes.c_void_p), ("c", ctypes.c_double * 2),
                ("p", (ctypes.c_double * 2) * 4)]


def load_apriltag(path=None):
    """(the loaded libapriltag, its path) or (None, why): `path` (the node's apriltag_lib), $HARNESS_APRILTAG_LIB,
    the library names on the loader's path, the apt directories, ctypes.util.find_library."""
    tries = [p for p in (path, os.environ.get(AT3_LIB_ENV)) if p]
    tries += list(AT3_LIB_NAMES) + [os.path.join(d, n) for d in AT3_LIB_DIRS for n in AT3_LIB_NAMES]
    found = ctypes.util.find_library("apriltag")
    if found:
        tries.append(found)
    errors = []                               # only files that exist but do not load / lack a function
    for p in dict.fromkeys(tries):
        if os.sep in p and not os.path.exists(p):
            if p in (path, os.environ.get(AT3_LIB_ENV)):
                errors.append(f"{p}: no such file")
            continue
        try:
            lib = ctypes.CDLL(p)
        except OSError as e:
            if os.sep in p:
                errors.append(f"{p}: {e}")
            continue
        missing = [f for f in ("apriltag_detector_create", "apriltag_detector_detect") + tuple(
            f + "_create" for f in AT3_FAMILIES) if not hasattr(lib, f)]
        if missing:
            errors.append(f"{p}: lacks {', '.join(missing)}")
            continue
        return lib, p
    return None, ("libapriltag not found (looked for " + AT3_LIB_NAMES[0] + " on the library path and in "
                  + ", ".join(AT3_LIB_DIRS[:3]) + ")" + ("; " + "; ".join(errors[-2:]) if errors else ""))


def family_info(fam_ptr):
    """The family struct as plain data: name, width_at_border, total_width, reversed_border, nbits, codes, bit_x,
    bit_y (the tables apriltag_to_image draws from)."""
    f = fam_ptr.contents
    n, nb = int(f.ncodes), int(f.nbits)
    return {"name": f.name.decode(), "width_at_border": int(f.width_at_border), "total_width": int(f.total_width),
            "reversed_border": bool(f.reversed_border), "nbits": nb, "h": int(f.h),
            "codes": [int(f.codes[i]) for i in range(n)],
            "bit_x": [int(f.bit_x[i]) for i in range(nb)], "bit_y": [int(f.bit_y[i]) for i in range(nb)]}


def size_share(info):
    """The detector's square / the printed tag (total_width cells): width_at_border / total_width."""
    return info["width_at_border"] / info["total_width"]


def table_bytes(info, max_hamming):
    """AprilTag 3's quick-decode table for this family at max_hamming bits (apriltag.c quick_decode_init)."""
    n, nb = len(info["codes"]), info["nbits"]
    cap = n + (n * nb if max_hamming >= 1 else 0) + (n * nb * (nb - 1) if max_hamming >= 2 else 0)
    return cap * 3 * 16


class AprilTag3:
    """One AprilTag 3 detector over `families` (one quad search per image for all of them)."""

    def __init__(self, lib, families, max_hamming=1, threads=2, decimate=2.0, sharpening=0.25):
        self.lib, self.fams, self.hamming = lib, {}, {}
        lib.apriltag_detector_create.restype = ctypes.POINTER(_Detector)
        lib.apriltag_detector_detect.restype = ctypes.POINTER(_Zarray)
        lib.apriltag_detector_detect.argtypes = [ctypes.POINTER(_Detector), ctypes.POINTER(_Image)]
        lib.apriltag_detector_add_family_bits.argtypes = [ctypes.POINTER(_Detector), ctypes.POINTER(_Family),
                                                          ctypes.c_int]
        lib.apriltag_detections_destroy.argtypes = [ctypes.POINTER(_Zarray)]
        lib.apriltag_detector_destroy.argtypes = [ctypes.POINTER(_Detector)]
        self.td = lib.apriltag_detector_create()
        d = self.td.contents
        d.nthreads, d.quad_decimate, d.quad_sigma = int(threads), float(decimate), 0.0
        d.refine_edges, d.decode_sharpening = True, float(sharpening)
        for fam in families:
            create = getattr(lib, fam + "_create")
            create.restype = ctypes.POINTER(_Family)
            ptr = create()
            info = family_info(ptr)
            h = int(max_hamming)
            while h > 0 and table_bytes(info, h) > AT3_TABLE_MAX_BYTES:
                h -= 1
            lib.apriltag_detector_add_family_bits(self.td, ptr, h)
            self.fams[fam] = (ptr, info)
            self.hamming[fam] = h

    def info(self, fam):
        return self.fams[fam][1]

    def detect_raw(self, gray):
        """[(family, id, corners 4x2 in AprilTag 3's own order and pixel convention, hamming, decision margin)]."""
        g = np.ascontiguousarray(gray, dtype=np.uint8)
        im = _Image(g.shape[1], g.shape[0], g.strides[0], g.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
        za = self.lib.apriltag_detector_detect(self.td, ctypes.byref(im))
        out = []
        try:
            z = za.contents
            ptrs = ctypes.cast(z.data, ctypes.POINTER(ctypes.POINTER(_Detection)))
            for k in range(z.size):
                d = ptrs[k].contents
                p = np.array([[d.p[i][0], d.p[i][1]] for i in range(4)], np.float64)
                out.append((d.family.contents.name.decode(), int(d.id), p, int(d.hamming), float(d.decision_margin)))
        finally:
            self.lib.apriltag_detections_destroy(za)
        return out

    def detect(self, gray):
        """[(family, corners 4x2 float64 in OpenCV's order and pixel convention, id)]."""
        return [(fam, to_cv_corners(p), i) for fam, i, p, _, _ in self.detect_raw(gray)]

    def close(self):
        if self.td:
            self.lib.apriltag_detector_destroy(self.td)
            self.td = None
            for fam, (ptr, _) in self.fams.items():
                destroy = getattr(self.lib, fam + "_destroy")
                destroy.argtypes = [ctypes.POINTER(_Family)]
                destroy(ptr)
            self.fams = {}


def to_cv_corners(p):
    """AprilTag 3 corners (its order, its pixel convention) -> OpenCV aruco's order and pixel convention."""
    return np.asarray(p, np.float64)[list(AT3_TO_CV)] + AT3_PIXEL_SHIFT_PX


# ---------------------------------------------------------------------------------------------- both decoders
def cv_detector(fam, error_correction_rate=0.0):
    """tag_node's OpenCV detector for one family (unchanged settings since 2026-09-16)."""
    d = cv2.aruco.getPredefinedDictionary(CV_FAMILIES[fam])
    dp = cv2.aruco.DetectorParameters()
    dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    dp.adaptiveThreshWinSizeMin = 23          # one threshold scale: 5x faster on cluttered 720p scenes
    dp.adaptiveThreshWinSizeMax = 23
    dp.minMarkerPerimeterRate = 0.04          # ignore candidates smaller than ~13 px edge
    dp.errorCorrectionRate = float(error_correction_rate)  # 0 = exact code match (16h5 mis-decodes otherwise)
    return cv2.aruco.ArucoDetector(d, dp)


class Detector:
    """Every configured family on one grey image. `families` = the decoded ones in config order, `missing`
    {family: why} = configured but not decodable here (no libapriltag). Unknown names raise ValueError."""

    def __init__(self, families, error_correction_rate=0.0, apriltag_lib=None, max_hamming=1, threads=2,
                 decimate=2.0):
        bad = [f for f in families if f not in FAMILIES]
        if bad:
            raise ValueError(f"unknown tag families {bad}: choose from {sorted(FAMILIES)}")
        self.cv = [(f, cv_detector(f, error_correction_rate)) for f in families if f in CV_FAMILIES]
        self.at3, self.at3_path, self.missing = None, None, {}
        want = [f for f in families if f in AT3_FAMILIES]
        if want:
            lib, where = load_apriltag(apriltag_lib)
            if lib is None:
                self.missing = {f: where for f in want}
            else:
                self.at3 = AprilTag3(lib, want, max_hamming=max_hamming, threads=threads, decimate=decimate)
                self.at3_path = where
        self.families = [f for f in families if f not in self.missing]

    def detect(self, gray):
        """[(family, corners 4x2 in OpenCV's order and pixel convention, id)]: OpenCV's corners as it returns them
        (float32), AprilTag 3's as float64."""
        found = []
        for fam, det in self.cv:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None:
                found += [(fam, c.reshape(4, 2), int(i)) for c, i in zip(corners, ids.flatten())]
        if self.at3 is not None:
            found += self.at3.detect(gray)
        return found

    def close(self):
        if self.at3 is not None:
            self.at3.close()


# ---------------------------------------------------------------------------------------------- one detection
def object_points(size_m):
    """The tag's corners in its own frame (OpenCV aruco's: x right, y up, z out of the tag), corner order k."""
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def measure(fam, pts, tag_id, size_m, normalise, fx, square, size_gate, depth=None, depth_scale=0.0):
    """One detection as tag_node reports it (TagDetection's fields; `T` = the tag in the camera optical frame, 4x4)
    or None when the pose cannot be solved. pts = its 4 corners (OpenCV order, px); normalise(pts) = normalised
    image coordinates (the device's distortion model); fx = px per normalised unit (the reprojection error in px);
    square = (x0, y0, x1, y1) the central square; size_gate = (min_px, max_px); depth (m x depth_scale per unit),
    aligned to the image, or None."""
    obj = object_points(size_m)
    nrm = normalise(pts)
    ok, rvec, tvec = cv2.solvePnP(obj, nrm, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, np.eye(3), None)
    err = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - nrm, axis=1))) * fx
    cx, cy = pts.mean(axis=0)
    edges = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
    size_px = float(np.mean(edges))
    lo, hi = size_gate
    r = {"id": int(tag_id), "family": fam, "size_m": float(size_m), "center_px": [float(cx), float(cy)],
         "corners_px": [float(v) for v in pts.flatten()], "size_px": size_px, "T": T_from_rvec_tvec(rvec, tvec),
         "distance_m": float(np.linalg.norm(tvec)), "reproj_err_px": err,
         "in_center_square": bool(square[0] <= cx <= square[2] and square[1] <= cy <= square[3] and lo <= size_px <= hi),
         "depth_m": 0.0, "depth_size_m": 0.0}
    if depth is not None:
        mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, pts.astype(np.int32), 1)
        z = depth[mask == 1].astype(np.float64) * depth_scale
        z = z[(z > 0.05) & (z < 1.5)]
        if len(z) >= 20:
            r["depth_m"] = float(np.median(z))
            r["depth_size_m"] = float(size_m * r["depth_m"] / r["distance_m"])  # S_true = S_assumed * depth / PnP range
    return r
