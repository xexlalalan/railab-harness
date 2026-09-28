#!/usr/bin/env python3
"""The `harness` command's daemon: one rclpy node kept alive with clients to the arm, bunny and dtv
nodes, so every shell command costs only the service call. tools/harness starts it on demand
(--serve) and sends one line over a unix socket; the daemon streams reply lines and closes the
connection; the last line starts with OK or FAILED.
"""
import json, math, os, socket, sys, threading, time

SOCK = "/tmp/harness_cmd.sock"

USAGE = """harness arm    enable | release | hold | grip-release | grip-enable | status
harness arm    grip <mm> [N]                        gripper to a gap (near the Bunny: <= 20 mm, ~1 N)
harness arm    move-pose <x> <y> <z> <roll> <pitch> <yaw> [--joint] [--speed N]   mm / deg, straight line unless --joint
harness arm    move-j <j1..j6 deg> [--speed N]
harness arm    teach ... | locate ...               (runs the calibration programs)
harness bunny  status | weight | tare | abort | barcode [timeout_s]
harness bunny  housing open | close | release
harness bunny  weigh-to <g> [powder|liquid] [powder_id]
harness bunny  liquid push <steps> [rate] | pull <steps> [rate]   one continuous move (default 1200 steps/s)
harness bunny  liquid weigh <g> | suck | stop | pos
harness bunny  load begin | end
harness bunny  gantry park | jog <dx> <dy> <dz_up>   (mm; not on a Bunny without a gantry)
harness dtv    status | lines | frame [file.png]
harness dtv    z rel <mm> | move <mm> | zero | home | stop   (+ = camera DOWN)
harness dtv    v clamp | open <mm> | release | home
harness run    dilute-to-volume [--flask-ml 25] [--tube-ml 0.06] [--max-ml N] [--offset-px 0] [--watch]   (alias: fill-to-mark)
harness daemon-stop"""


def serve():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionClient
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from harness_msgs.msg import ArmStatus, GripperStatus, FlaskLines
    from harness_msgs.srv import SetGripper, MoveZ, MoveV, WeighTo, LiquidPush, LiquidRun, Barcode, GantryJog
    from harness_msgs.action import MovePose, MoveJ
    from harness_planner import fill_to_mark

    rclpy.init(); n = Node("harness_cli"); last = {}
    n.create_subscription(ArmStatus, "/arm/status", lambda m: last.__setitem__("arm", m), 10)
    n.create_subscription(GripperStatus, "/arm/gripper_status", lambda m: last.__setitem__("grip", m), 10)
    n.create_subscription(String, "/bunny/status", lambda m: last.__setitem__("bunny", (json.loads(m.data), time.time())), 10)
    n.create_subscription(String, "/dtv/status", lambda m: last.__setitem__("dtv", (json.loads(m.data), time.time())), 10)
    n.create_subscription(FlaskLines, "/dtv/lines", lambda m: last.__setitem__("lines", (m, time.time())), 10)
    n.create_subscription(Image, "/dtv/image", lambda m: last.__setitem__("image", m), 2)
    ac_pose = ActionClient(n, MovePose, "/arm/move_pose"); ac_j = ActionClient(n, MoveJ, "/arm/move_j")
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    clients = {}

    def call(srv, name, req, timeout=30.0):
        cli = clients.get(name) or clients.setdefault(name, n.create_client(srv, name))
        if not cli.wait_for_service(timeout_sec=3.0): return None, "%s not answering (node not running?)" % name
        fut = cli.call_async(req); t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout: return None, "%s timed out" % name
            time.sleep(0.01)
        return fut.result(), None

    def trig(name, timeout=30.0):
        r, err = call(Trigger, name, Trigger.Request(), timeout)
        if err: return "FAILED " + err
        return ("OK " if r.success else "FAILED ") + r.message

    def action(ac, goal, timeout):
        if not ac.wait_for_server(timeout_sec=3.0): return "FAILED arm node not answering"
        gf = ac.send_goal_async(goal); t0 = time.time()
        while not gf.done():
            if time.time() - t0 > 5: return "FAILED goal not accepted in time"
            time.sleep(0.01)
        gh = gf.result()
        if not gh.accepted: return "FAILED goal rejected by the arm node (check harness arm status)"
        rf = gh.get_result_async()
        while not rf.done():
            if time.time() - t0 > timeout: return "FAILED move timed out"
            time.sleep(0.02)
        r = rf.result().result
        return ("OK " if r.success else "FAILED ") + r.message

    def fresh(key, max_age=2.0):
        v = last.get(key)
        return v[0] if v and time.time() - v[1] < max_age else None

    def opts(w):
        """split '--key value' and '--flag' out of a word list."""
        pos, kv = [], {}; i = 0
        while i < len(w):
            if w[i].startswith("--"):
                k = w[i][2:]
                if i + 1 < len(w) and not w[i + 1].startswith("--"): kv[k] = w[i + 1]; i += 2
                else: kv[k] = True; i += 1
            else: pos.append(w[i]); i += 1
        return pos, kv

    # ------------------------------------------------------------------ arm
    def do_arm(w, out, cancelled):
        w = w or ["?"]; c = w[0]
        if c in ("enable", "release", "hold", "grip-release", "grip-enable"):
            return trig("/arm/" + {"release": "disable", "grip-release": "gripper_release", "grip-enable": "gripper_enable"}.get(c, c), 15)
        if c == "grip" and len(w) >= 2:
            r, err = call(SetGripper, "/arm/gripper", SetGripper.Request(width_m=float(w[1]) / 1000.0, force=float(w[2]) if len(w) > 2 else 0.0), 20)
            if err: return "FAILED " + err
            return ("OK " if r.success else "FAILED ") + "%s (gap %.1f mm%s)" % (r.message, r.width_m * 1000, ", stalled on an object" if r.stalled else "")
        if c == "status":
            a, g = last.get("arm"), last.get("grip")
            return "OK arm: %s | gripper: %s" % ("joints on" if a and a.all_enabled else "released" if a else "no status",
                   "gap %.1f mm, %s" % (g.width_m * 1000, "powered" if g.enabled else "free") if g else "no status")
        if c in ("move-pose", "move-j"):
            pos, kv = opts(w[1:]); v = [float(x) for x in pos]; speed = int(kv.get("speed", 10))
            if c == "move-pose":
                if len(v) != 6: return "FAILED move-pose needs x y z roll pitch yaw (mm, deg)"
                g = MovePose.Goal(); g.pose = [v[0] / 1000, v[1] / 1000, v[2] / 1000] + [math.radians(x) for x in v[3:]]
                g.linear = "joint" not in kv; g.speed_percent = speed; g.timeout_s = 60.0
                return action(ac_pose, g, 65)
            if len(v) != 6: return "FAILED move-j needs 6 joint angles (deg)"
            g = MoveJ.Goal(); g.joints_rad = [math.radians(x) for x in v]; g.speed_percent = speed; g.timeout_s = 60.0
            return action(ac_j, g, 65)
        return "FAILED unknown arm command; see harness --help"

    # ------------------------------------------------------------------ bunny
    def do_bunny(w, out, cancelled):
        w = w or ["?"]; c = w[0]
        if c == "status":
            st = fresh("bunny", 3.0)
            return "OK " + json.dumps(st) if st else "FAILED no /bunny/status (bunny node running?)"
        if c == "weight":
            st = fresh("bunny", 3.0)
            return "OK %.4f g" % st["weight_g"] if st and st.get("weight_g") is not None else "FAILED no scale reading"
        if c in ("tare", "abort"): return trig("/bunny/" + c, 90)
        if c == "housing" and len(w) > 1 and w[1] in ("open", "close", "release"): return trig("/bunny/housing_" + w[1], 90)
        if c == "barcode":
            t = float(w[1]) if len(w) > 1 else 30.0; out("scan a label within %.0f s ..." % t)
            r, err = call(Barcode, "/bunny/barcode_read", Barcode.Request(timeout_s=t), t + 5)
            if err: return "FAILED " + err
            return ("OK code %s name '%s'" % (r.code, r.name)) if r.success else "FAILED " + r.message
        if c == "weigh-to" and len(w) >= 2:
            req = WeighTo.Request(target_g=float(w[1]), material=w[2] if len(w) > 2 else "powder", powder_id=w[3] if len(w) > 3 else "")
            out("dispensing %.3f g of %s ... (harness bunny abort to stop)" % (req.target_g, req.material))
            r, err = call(WeighTo, "/bunny/weigh_to", req, 1800)
            if err: return "FAILED " + err
            return ("OK " if r.success else "FAILED ") + "%s, scale %.4f g" % (r.outcome, r.actual_g)
        if c == "liquid" and len(w) > 1:
            s = w[1]
            if s == "weigh" and len(w) > 2: return do_bunny(["weigh-to", w[2], "liquid"], out, cancelled)
            if s in ("push", "pull") and len(w) > 2:
                steps = int(float(w[2])) * (1 if s == "push" else -1); rate = int(float(w[3])) if len(w) > 3 else 1200
                r, err = call(LiquidPush, "/bunny/liquid_push", LiquidPush.Request(steps=steps, rate=rate, wait=True), abs(steps) / max(rate, 20) + 30)
                if err: return "FAILED " + err
                return ("OK %d steps" % r.steps_done) if r.success else "FAILED " + r.message
            if s in ("suck", "stop"): return trig("/bunny/liquid_" + s, 15)
            if s == "pos":
                st = fresh("bunny", 3.0)
                return "OK " + json.dumps(st.get("pump")) if st else "FAILED no /bunny/status"
        if c == "load" and len(w) > 1 and w[1] in ("begin", "end"): return trig("/bunny/load_" + w[1], 10)
        if c == "gantry" and len(w) > 1:
            if w[1] == "park": return trig("/bunny/gantry_park", 90)
            if w[1] == "jog" and len(w) == 5:
                r, err = call(GantryJog, "/bunny/gantry_jog", GantryJog.Request(dx_mm=float(w[2]), dy_mm=float(w[3]), dz_up_mm=float(w[4])), 90)
                if err: return "FAILED " + err
                return ("OK " if r.success else "FAILED ") + r.message
        if c == "locate": return trig("/bunny/locate", 90)
        return "FAILED unknown bunny command; see harness --help"

    # ------------------------------------------------------------------ dtv
    def do_dtv(w, out, cancelled):
        w = w or ["?"]; c = w[0]
        if c == "status":
            st = fresh("dtv", 3.0)
            return "OK " + json.dumps(st) if st else "FAILED no /dtv/status (dtv node running?)"
        if c == "lines":
            m = fresh("lines", 2.0)
            if not m: return "FAILED no frames on /dtv/lines"
            return "OK neck x=%d..%d | ring %s | surface %s | gap %s" % (
                m.x0, m.x1, "row %.1f (strength %.1f, cover %.0f%%)" % (m.ring_row, m.ring_strength, m.ring_cover * 100) if m.ring_found else "not found",
                "rows %.0f..%.0f, lower edge %.1f" % (m.surf_top, m.surf_bot, m.surf_low) if m.surface_found else "not in view",
                "%.1f px (%s the mark)" % (m.gap_px, "below" if m.gap_px > 0 else "above") if m.ring_found and m.surface_found else "--")
        if c == "frame":
            im = last.get("image")
            if im is None: return "FAILED no image on /dtv/image"
            import numpy as np, cv2
            path = os.path.abspath(w[1] if len(w) > 1 else "dtv_frame.png")
            cv2.imwrite(path, np.frombuffer(im.data, np.uint8).reshape(im.height, im.width)); return "OK saved " + path
        if c == "z" and len(w) > 1:
            s = w[1]
            if s in ("rel", "move") and len(w) > 2:
                r, err = call(MoveZ, "/dtv/z_move", MoveZ.Request(mm=float(w[2]), absolute=(s == "move")), 130)
                if err: return "FAILED " + err
                return ("OK z=%.3f mm" % r.z) if r.success else "FAILED " + r.message
            if s in ("zero", "home", "stop"): return trig("/dtv/z_" + {"zero": "set_zero"}.get(s, s), 130)
        if c == "v" and len(w) > 1:
            s = w[1]
            if s in ("clamp", "release", "home"):
                if s != "release": out("V-blocks %s ..." % ("closing" if s == "clamp" else "homing"))
                return trig("/dtv/v_" + s, 200)
            if s == "open" and len(w) > 2:
                r, err = call(MoveV, "/dtv/v_open", MoveV.Request(mm=float(w[2])), 40)
                if err: return "FAILED " + err
                return ("OK " if r.success else "FAILED ") + r.message
        return "FAILED unknown dtv command; see harness --help"

    # ------------------------------------------------------------------ run
    def do_run(w, out, cancelled):
        w = w or ["?"]
        if w[0] in ("dilute-to-volume", "fill-to-mark"):
            pos, kv = opts(w[1:])
            try:
                return "OK " + fill_to_mark.run(n, out, flask_ml=float(kv.get("flask-ml", 25)), tube_ml=float(kv.get("tube-ml", 0.06)),
                                                max_ml=float(kv["max-ml"]) if "max-ml" in kv else None, offset_px=float(kv.get("offset-px", 0)),
                                                watch="watch" in kv, cancelled=cancelled)
            except RuntimeError as e:
                return "FAILED " + str(e)
        return "FAILED unknown workflow; available: dilute-to-volume"

    def handle(line, out, cancelled):
        w = line.split()
        if not w or w[0] in ("--help", "help", "-h"): return "OK\n" + USAGE
        return {"arm": do_arm, "bunny": do_bunny, "dtv": do_dtv, "run": do_run}.get(w[0], lambda *a: "FAILED unknown group; see harness --help")(w[1:], out, cancelled)

    def conn(c):
        with c:
            line = c.recv(1024).decode().strip()

            def out(s):
                try: c.sendall((s + "\n").encode())
                except OSError: pass

            def cancelled():
                try: return c.recv(1, socket.MSG_DONTWAIT) == b""
                except BlockingIOError: return False
                except OSError: return True
            try: r = handle(line, out, cancelled)
            except Exception as e: r = "FAILED error: %s" % e
            out(r)

    if os.path.exists(SOCK): os.unlink(SOCK)
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.bind(SOCK); s.listen(4)
    while True:
        c, _ = s.accept(); threading.Thread(target=conn, args=(c,), daemon=True).start()


def client(line):
    c = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try: c.connect(SOCK)
    except OSError: return None
    c.sendall((line + "\n").encode()); buf = b""; lastline = ""
    try:
        while True:
            chunk = c.recv(4096)
            if not chunk: break
            buf += chunk
            while b"\n" in buf:
                l, buf = buf.split(b"\n", 1); lastline = l.decode(); print(lastline, flush=True)
    except KeyboardInterrupt:
        print("\n(cancelled)"); return "FAILED cancelled"
    finally:
        c.close()
    return lastline


if __name__ == "__main__":
    if sys.argv[1:2] == ["--serve"]:
        serve()
    else:
        r = client(" ".join(sys.argv[1:]))
        if r is None: print("daemon not running"); sys.exit(3)
        sys.exit(1 if r.startswith("FAILED") else 0)
