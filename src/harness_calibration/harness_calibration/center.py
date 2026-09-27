#!/usr/bin/env python3
"""Centre a tag in the camera image by small flange translations (no hand-eye needed).

    ros2 run harness_calibration center --tag-id 5 [--probe 0.02] [--tol-px 30]

Two probe moves (base x, base y) measure how the tag's pixel position responds,
giving a 2x2 image Jacobian; then up to 4 corrective moves bring the tag centre
within --tol-px of the image centre. Also used as the "rough approach" step later.
"""
import argparse
import sys
import time

import numpy as np

from harness_calibration.client import HarnessClient
from harness_vision.geom import trans


def tag_px(c, tag_id, n=5):
    xs = []
    t0 = time.time()
    last = None
    while len(xs) < n and time.time() - t0 < 2.0:
        m = c.tags
        if m is not None and m is not last:
            last = m
            for t in m.tags:
                if t.id == tag_id:
                    xs.append(t.center_px)
        time.sleep(0.02)
    return np.mean(xs, axis=0) if xs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag-id", type=int, required=True)
    ap.add_argument("--probe", type=float, default=0.02)
    ap.add_argument("--tol-px", type=float, default=30.0)
    ap.add_argument("--speed", type=int, default=10)
    a = ap.parse_args()
    c = HarnessClient("center")
    c.wait_ready()
    W, H = c.tags.image_width, c.tags.image_height
    goal = np.array([W / 2, H / 2])
    T0 = c.tcp_T.copy()
    p0 = tag_px(c, a.tag_id)
    if p0 is None:
        print("tag not visible")
        sys.exit(1)
    print(f"tag {a.tag_id} at {p0.round()} px, image centre {goal}")
    if np.linalg.norm(p0 - goal) <= a.tol_px:
        print("already centred")
        c.close()
        return
    # probe base x and base y
    J = np.zeros((2, 2))
    for k, ax in enumerate(("x", "y")):
        v = np.zeros(3)
        v[k] = a.probe
        r = c.move_pose(trans(v) @ T0, speed=a.speed, timeout=20)
        if not r.success:
            print("probe move failed:", r.message)
            c.move_pose(T0, speed=a.speed)
            sys.exit(1)
        time.sleep(0.4)
        p = tag_px(c, a.tag_id)
        if p is None:
            print("tag lost during probe; returning")
            c.move_pose(T0, speed=a.speed)
            sys.exit(1)
        J[:, k] = (p - p0) / a.probe
        print(f"probe {ax}: {J[:, k].round(0)} px/m")
        c.move_pose(T0, speed=a.speed, timeout=20)
    if abs(np.linalg.det(J)) < 1e3:
        print("degenerate Jacobian; camera probably not looking along a flange-ish axis")
        sys.exit(1)
    T = T0.copy()
    for it in range(4):
        time.sleep(0.4)
        p = tag_px(c, a.tag_id)
        if p is None:
            print("tag lost")
            break
        err = goal - p
        print(f"iter {it}: tag at {p.round()} px, error {np.linalg.norm(err):.0f} px")
        if np.linalg.norm(err) <= a.tol_px:
            print("centred")
            break
        d = np.linalg.solve(J, err)
        d = np.clip(d, -0.06, 0.06)
        T = trans([d[0], d[1], 0.0]) @ T
        r = c.move_pose(T, speed=a.speed, timeout=20)
        if not r.success:
            print("move failed:", r.message)
            break
    c.close()


if __name__ == "__main__":
    main()
