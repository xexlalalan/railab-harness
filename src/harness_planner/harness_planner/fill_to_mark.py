"""Dilute to volume: the Bunny's liquid doser fills the volumetric flask to its ring mark while the
dtv camera (/dtv/lines) watches the liquid surface climb to the ring.

Pre-condition: the flask arrives at least 80 % full. While no surface is in view the doser runs
continuously with the camera checked every frame (~10 Hz); if 20 % of the flask volume plus the
tube's liquid volume has been pushed and still no surface is seen, the run stops with an ALARM:
the reservoir is empty or the flask was not pre-filled.

Image rows grow DOWNWARD; gap = surface lower edge row - ring row (positive = below the mark).
  surface not in view  continuous flow at far_rate       (/bunny/liquid_run, re-armed every 0.4 s)
  gap > coarse_px      continuous flow at coarse_rate
  gap > fine_px        continuous flow at fine_rate
  else                 bounded pushes of fine_steps, then wait settle_s (drops land, the surface settles)
  gap <= stop_px       STOP, suck back
Aborts (pump stopped, suck back): cancel, ring lost 1 s, ALARM (no surface after 20 % + tube),
max_ml reached, surface not rising for stall_s, a module offline. Runs on a node that is being spun elsewhere (multi-threaded executor).
"""
import time
from collections import deque
import numpy as np
from std_srvs.srv import Trigger
from harness_msgs.msg import FlaskLines
from harness_msgs.srv import LiquidPush, LiquidRun

UL_PER_STEP = 0.556                      # Bunny water calibration (0.556 mg/step ~ 0.556 uL)
P = dict(far_rate=600,            # continuous flow while no surface is in view
         approach=0.8,            # like the Bunny's weigh loop: each move aims at this fraction of the remaining gap
         steps_per_px=10.0,       # seed for the learned scale (2026-09-27 run: ~180 px per mL = 10 steps/px)
         min_steps=4, max_move_steps=1500,
         rate_per_px=4.0, rate_min=60, rate_max=800,   # rate = gap * rate_per_px: the closer, the slower
         settle_s=0.8, done_px=1.0, stall_s=20.0, lock_frames=10)


def _call(node, cli, req, timeout):
    if not cli.wait_for_service(timeout_sec=3.0): raise RuntimeError("%s not available" % cli.srv_name)
    fut = cli.call_async(req); t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout: raise RuntimeError("%s timed out" % cli.srv_name)
        time.sleep(0.01)
    return fut.result()


def run(node, log, flask_ml=25.0, tube_ml=0.06, max_ml=None, offset_px=0.0, ring_row=None, watch=False, cancelled=lambda: False):
    """Returns a result string; raises RuntimeError on abort (an ALARM is a RuntimeError starting with 'ALARM').
    The ring row is measured ONCE before dosing and kept (the flask does not move); dosing is the Bunny's
    weigh-to scheme: move = approach * remaining gap * learned steps/px, settle, re-measure, repeat."""
    unseen_limit_ml = 0.2 * flask_ml + tube_ml          # flask must be >= 80 % full on arrival
    max_ml = max_ml if max_ml is not None else 0.3 * flask_ml
    latest = {"m": None, "t": 0.0}
    sub = node.create_subscription(FlaskLines, "/dtv/lines", lambda m: latest.update(m=m, t=time.time()), 10)
    push = node.create_client(LiquidPush, "/bunny/liquid_push"); run_cli = node.create_client(LiquidRun, "/bunny/liquid_run")
    stop = node.create_client(Trigger, "/bunny/liquid_stop"); suck = node.create_client(Trigger, "/bunny/liquid_suck")
    pushed = 0.0; t_start = time.time(); k = P["steps_per_px"]; sum_steps = 0.0; sum_px = 0.0
    last = {"t": 0.0}

    def frame():
        while latest["t"] <= last["t"]:
            if cancelled(): raise RuntimeError("cancelled")
            if time.time() - latest["t"] > 2.0: raise RuntimeError("camera frames stopped (dtv node?)")
            time.sleep(0.01)
        last["t"] = latest["t"]; return latest["m"]

    def gap_of(m, ring):
        return (m.surf_low - ring - offset_px) if m.surface_found else None

    def settled_gap(ring, n=3):
        """median gap of n fresh frames; None if the surface is not in view"""
        g = [gap_of(frame(), ring) for _ in range(n)]
        g = [v for v in g if v is not None]
        return float(np.median(g)) if len(g) >= 2 else None

    try:
        t0 = time.time()
        while latest["m"] is None:
            if time.time() - t0 > 3: raise RuntimeError("no frames on /dtv/lines (dtv node running?)")
            time.sleep(0.05)
        # 1. lock the ring
        if ring_row is not None:
            ring = float(ring_row); log("ring row given by the operator: %.1f" % ring)
        else:
            rows = [m.ring_row for m in (frame() for _ in range(P["lock_frames"])) if m.ring_found]
            if len(rows) < P["lock_frames"] // 2: raise RuntimeError("ring mark not found reliably before dosing")
            ring = float(np.median(rows)); log("ring locked at row %.1f (spread %.1f px)" % (ring, np.ptp(rows)))
        if not watch: _call(node, stop, Trigger.Request(), 5)       # proves the Bunny link before pumping
        # 2. far phase: continuous flow until the surface is in view
        flowing = False; t_flow = 0.0
        while True:
            m = frame(); g = gap_of(m, ring); now = time.time()
            if flowing: pushed += P["far_rate"] * (now - t_flow); t_flow = now
            log("%6.1fs  surface %s  %s  pumped %.2f mL" % (now - t_start, "%.1f" % m.surf_low if m.surface_found else "--",
                "gap %6.1f px" % g if g is not None else "not in view", pushed * UL_PER_STEP / 1000))
            if g is not None: break
            if watch: continue
            if pushed * UL_PER_STEP / 1000 >= unseen_limit_ml:
                raise RuntimeError("ALARM: pushed %.2f mL (20%% of %.0f mL + %.2f mL tube) and still no liquid surface in view: "
                                   "reservoir empty or flask not pre-filled to 80%%" % (pushed * UL_PER_STEP / 1000, flask_ml, tube_ml))
            r = _call(node, run_cli, LiquidRun.Request(rate=P["far_rate"], direction=1), 5)       # re-arms the 1 s dead-man
            if not r.success: raise RuntimeError("Bunny refused flow: " + r.message)
            if not flowing: flowing = True; t_flow = time.time()
        if flowing: _call(node, stop, Trigger.Request(), 5); time.sleep(P["settle_s"])
        # 3. approach: shrinking moves, slower as it gets closer
        best = None; t_best = time.time()
        while True:
            if watch: g = settled_gap(ring); log("gap %s" % g); continue
            g = settled_gap(ring)
            if g is None: raise RuntimeError("liquid surface lost after it was seen")
            if g <= P["done_px"]:
                return "AT THE MARK: gap %.1f px, pumped %.2f mL, learned %.1f steps/px" % (g, pushed * UL_PER_STEP / 1000, k)
            if best is None or g < best - 1: best, t_best = g, time.time()
            elif time.time() - t_best > P["stall_s"]: raise RuntimeError("surface not rising for %.0f s" % (time.time() - t_best))
            if pushed * UL_PER_STEP / 1000 >= max_ml: raise RuntimeError("max %.1f mL reached without reaching the mark" % max_ml)
            n = int(max(P["min_steps"], min(P["max_move_steps"], P["approach"] * g * k)))
            rate = int(min(P["rate_max"], max(P["rate_min"], g * P["rate_per_px"])))
            r = _call(node, push, LiquidPush.Request(steps=n, rate=rate, wait=True), n / rate + 20)
            if not r.success: raise RuntimeError("Bunny refused push: " + r.message)
            pushed += r.steps_done; time.sleep(P["settle_s"])
            g2 = settled_gap(ring)
            log("%6.1fs  gap %6.1f -> %s px  move %d steps @ %d/s  pumped %.2f mL  k=%.1f steps/px" % (
                time.time() - t_start, g, "%.1f" % g2 if g2 is not None else "--", n, rate, pushed * UL_PER_STEP / 1000, k))
            if g2 is not None and g - g2 > 2:                          # learn the scale as a ratio of sums (like steps/mg)
                sum_steps += r.steps_done; sum_px += (g - g2); k = sum_steps / sum_px
    finally:
        if not watch:
            try: _call(node, stop, Trigger.Request(), 5); _call(node, suck, Trigger.Request(), 10)
            except Exception as e: log("WARNING: pump stop/suck-back failed: %s" % e)
        node.destroy_subscription(sub); node.destroy_client(push); node.destroy_client(run_cli); node.destroy_client(stop); node.destroy_client(suck)
