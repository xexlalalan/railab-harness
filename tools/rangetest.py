"""Size-scale test: back the camera off along its optical axis in known steps; PnP range must grow by the same amount.
slope = d(PnP range)/d(actual displacement) = S_assumed / S_true."""
import sys, time, numpy as np, yaml
from harness_calibration.client import HarnessClient
TAG = int(sys.argv[1]); STEPS = [0.0, 0.03, 0.06, 0.09, 0.12]
he = yaml.safe_load(open("/home/bunny-harness/bunny-harness-dev/calib/handeye.yaml")); Tfc = np.array(he["flange_to_camera"]["matrix"])
c = HarnessClient("rangetest"); c.wait_ready(); time.sleep(0.3)
T0 = c.tcp_T.copy(); z_cam = (T0[:3,:3] @ Tfc[:3,:3])[:,2]
rows = []
for d in STEPS:
    T = T0.copy(); T[:3,3] = T0[:3,3] - d * z_cam
    r = c.move_pose(T, speed=10, timeout=25)
    if not r.success: print("move failed:", r.message); break
    time.sleep(1.0); c.wait_still(); time.sleep(0.3)
    pnp, dep, px, n = [], [], [], 0; last = None; t0 = time.time()
    while n < 20 and time.time() - t0 < 4:
        m = c.tags
        if m is not None and m is not last:
            last = m
            for t in m.tags:
                if t.id == TAG: pnp.append(t.distance_m); dep.append(t.depth_m); px.append(t.size_px); n += 1
        time.sleep(0.01)
    disp = float(np.linalg.norm(c.tcp_T[:3,3] - T0[:3,3]))
    if n < 5: print(f"step {d*1000:.0f} mm: tag {TAG} not seen"); continue
    rows.append((disp, np.median(pnp), np.median([x for x in dep if x > 0] or [0]), np.median(px)))
    print(f"actual back-off {disp*1000:6.1f} mm | PnP range {np.median(pnp)*1000:6.1f} mm | depth {rows[-1][2]*1000:6.1f} mm | {np.median(px):.0f} px  (n={n})")
c.move_pose(T0, speed=10, timeout=25); print("returned to start")
if len(rows) >= 3:
    A = np.array(rows); size = None
    slope_pnp = np.polyfit(A[:,0], A[:,1], 1)[0]; good = A[A[:,2] > 0.1]
    print(f"PnP slope {slope_pnp:.4f} -> S_true = S_assumed / slope = S_assumed x {1/slope_pnp:.4f}")
    if len(good) >= 3:
        slope_dep = np.polyfit(good[:,0], good[:,2], 1)[0]; print(f"depth slope {slope_dep:.4f} (should be 1.0 if depth is metric; only points > 100 mm used)")
c.close()
