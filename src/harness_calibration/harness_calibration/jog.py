#!/usr/bin/env python3
"""Joint jog through the arm node.

    ros2 run harness_calibration jog --joint 5 --to 1.25        # absolute, rad
    ros2 run harness_calibration jog --joint 6 --delta 0.1      # relative, rad
    ros2 run harness_calibration jog --print                    # joints, tcp pose, limits check, tags
"""
import argparse
import math
import sys
import time

import numpy as np

from harness_calibration.client import HarnessClient
from harness_vision.geom import trans, xyzrpy_from_T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, choices=range(1, 7))
    ap.add_argument("--to", type=float)
    ap.add_argument("--delta", type=float)
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--print", action="store_true")
    ap.add_argument("--dx", type=float, default=0.0, help="base-frame translation m (move_pose, orientation kept)")
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.0)
    a = ap.parse_args()
    c = HarnessClient("jog")
    c.wait_ready(camera=False)
    time.sleep(0.3)
    j = list(c.joints)
    print("joints deg:", np.round(np.degrees(j), 1).tolist())
    p = xyzrpy_from_T(c.tcp_T)
    print(f"tcp: xyz {np.round(np.array(p[:3])*1000,1).tolist()} mm  rpy {np.round(np.degrees(p[3:]),1).tolist()} deg")
    for jn, q, lo, hi in c.joints_outside_limits():
        print(f"  !! joint {jn} = {math.degrees(q):.1f} deg outside [{math.degrees(lo):.0f}, {math.degrees(hi):.0f}]")
    if c.tags is not None:
        for t in c.tags.tags:
            print(f"tag {t.id}: {t.size_px:.0f} px, {t.distance_m*1000:.0f} mm, set={t.in_center_square}")
    if a.joint and (a.to is not None or a.delta is not None):
        j[a.joint - 1] = a.to if a.to is not None else j[a.joint - 1] + a.delta
        print(f"move_j -> joint {a.joint} = {math.degrees(j[a.joint-1]):.1f} deg")
        r = c.move_j(j, speed=a.speed)
        print(r.success, r.message)
        c.close()
        sys.exit(0 if r.success else 1)
    if a.dx or a.dy or a.dz:
        print(f"move_pose translate base ({a.dx*1000:.0f}, {a.dy*1000:.0f}, {a.dz*1000:.0f}) mm")
        r = c.move_pose(trans([a.dx, a.dy, a.dz]) @ c.tcp_T, speed=a.speed)
        print(r.success, r.message)
        c.close()
        sys.exit(0 if r.success else 1)
    c.close()


if __name__ == "__main__":
    main()
