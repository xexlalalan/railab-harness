"""Fill the volumetric flask to its ring mark: the Bunny's liquid pump (/bunny/liquid_push) doses in
small bounded chunks while the dtv camera (/dtv/lines) watches the liquid surface climb to the ring.

Image rows grow DOWNWARD; gap = surface lower edge row - ring row (positive = below the mark).
  surface not in view  continuous flow at far_rate       (/bunny/liquid_run, re-armed every 0.4 s)
  gap > coarse_px      continuous flow at coarse_rate
  gap > fine_px        continuous flow at fine_rate
  else                 bounded pushes of fine_steps, then wait settle_s (drops land, the surface settles)
  gap <= stop_px       STOP, suck back
Aborts (pump stopped, suck back): cancel, ring lost 1 s, max_ml reached, surface not rising over
stall_steps, a module offline. Runs on a node that is being spun elsewhere (multi-threaded executor).
"""
import time
from collections import deque
import numpy as np
from std_srvs.srv import Trigger
from harness_msgs.msg import FlaskLines
from harness_msgs.srv import LiquidPush, LiquidRun

UL_PER_STEP = 0.556                      # Bunny water calibration (0.556 mg/step ~ 0.556 uL)
P = dict(far_rate=600, coarse_px=80, coarse_rate=300, fine_px=25, fine_steps=8, fine_rate=100,
         settle_s=1.2, stop_px=0.0, stall_s=20.0)


def _call(node, cli, req, timeout):
    if not cli.wait_for_service(timeout_sec=3.0): raise RuntimeError("%s not available" % cli.srv_name)
    fut = cli.call_async(req); t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout: raise RuntimeError("%s timed out" % cli.srv_name)
        time.sleep(0.01)
    return fut.result()


def run(node, log, max_ml=5.0, offset_px=0.0, watch=False, cancelled=lambda: False):
    """Returns a result string; raises RuntimeError on abort."""
    latest = {"m": None, "t": 0.0}
    sub = node.create_subscription(FlaskLines, "/dtv/lines", lambda m: latest.update(m=m, t=time.time()), 10)
    push = node.create_client(LiquidPush, "/bunny/liquid_push"); run_cli = node.create_client(LiquidRun, "/bunny/liquid_run")
    stop = node.create_client(Trigger, "/bunny/liquid_stop"); suck = node.create_client(Trigger, "/bunny/liquid_suck")
    hist = deque(maxlen=3); pushed = 0.0; max_steps = max_ml * 1000 / UL_PER_STEP
    t_start = time.time(); ring_seen = t_start; wait_until = 0.0; best_gap = None; t_best = t_start; last_t = 0.0
    flow = {"rate": 0, "t": 0.0}                       # current continuous flow rate (0 = none) and its start

    def set_flow(rate):
        nonlocal pushed
        now = time.time()
        if flow["rate"]: pushed += flow["rate"] * (now - flow["t"])      # steps delivered so far by the flow
        if rate:
            r = _call(node, run_cli, LiquidRun.Request(rate=int(rate), direction=1), 5)
            if not r.success: raise RuntimeError("Bunny refused flow: " + r.message)
        elif flow["rate"]:
            _call(node, stop, Trigger.Request(), 5)
        flow.update(rate=rate, t=now)
    try:
        t0 = time.time()
        while latest["m"] is None:
            if time.time() - t0 > 3: raise RuntimeError("no frames on /dtv/lines (dtv node running?)")
            time.sleep(0.05)
        if not watch:
            _call(node, stop, Trigger.Request(), 5)       # proves the Bunny link before pumping
        while True:
            if cancelled(): raise RuntimeError("cancelled")
            while latest["t"] <= last_t:
                if time.time() - latest["t"] > 2.0: raise RuntimeError("camera frames stopped")
                time.sleep(0.01)
            m = latest["m"]; last_t = latest["t"]; now = time.time()
            if m.ring_found: ring_seen = now
            elif now - ring_seen > 1.0: raise RuntimeError("ring mark lost for 1 s")
            gap = None
            if m.ring_found and m.surface_found:
                hist.append(m.surf_low - m.ring_row - offset_px); gap = float(np.median(hist))
            elif not m.surface_found:
                hist.clear()
            log("%6.1fs  ring %s  surface %s  %s  pumped %.2f mL" % (
                now - t_start, "%.1f" % m.ring_row if m.ring_found else "--",
                "%.1f" % m.surf_low if m.surface_found else "--",
                "gap %6.1f px" % gap if gap is not None else "surface not in view", pushed * UL_PER_STEP / 1000))
            if watch: continue
            if gap is not None and gap <= P["stop_px"] and len(hist) == hist.maxlen:
                set_flow(0)
                return "AT THE MARK: gap %.1f px, pumped %.2f mL" % (gap, pushed * UL_PER_STEP / 1000)
            if gap is not None:
                if best_gap is None or gap < best_gap - 2: best_gap, t_best = gap, now
                elif now - t_best > P["stall_s"]: raise RuntimeError("surface not rising for %.0f s" % (now - t_best))
            if pushed >= max_steps: raise RuntimeError("max %.1f mL reached without reaching the mark" % max_ml)
            if gap is None:                 want = P["far_rate"]
            elif gap > P["coarse_px"]:      want = P["coarse_rate"]
            elif gap > P["fine_px"]:        want = P["fine_rate"]
            else:                           want = 0
            if want:
                set_flow(want)                              # (re)arms the Bunny's 1 s dead-man every frame
            else:
                set_flow(0)
                if now >= wait_until:                       # last millimetres: small bounded pushes, let it settle
                    n = int(min(P["fine_steps"], max_steps - pushed))
                    r = _call(node, push, LiquidPush.Request(steps=n, rate=P["fine_rate"], wait=False), 5)
                    if not r.success:
                        if "busy" in r.message: wait_until = time.time() + 0.2; continue
                        raise RuntimeError("Bunny refused push: " + r.message)
                    pushed += n; wait_until = time.time() + n / P["fine_rate"] + P["settle_s"]
    finally:
        if not watch:
            try: _call(node, stop, Trigger.Request(), 5); _call(node, suck, Trigger.Request(), 10)
            except Exception as e: log("WARNING: pump stop/suck-back failed: %s" % e)
        node.destroy_subscription(sub); node.destroy_client(push); node.destroy_client(run_cli); node.destroy_client(stop); node.destroy_client(suck)
