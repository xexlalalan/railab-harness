#!/usr/bin/env python3
"""/bunny — bridge to the Bunny's harness_server (bunny_side/harness_server.py) over TCP/JSON.

Services (std_srvs/Trigger unless noted): housing_open, housing_close, housing_half, housing_release, tare, abort,
load_begin, load_end, liquid_suck, liquid_stop, gantry_park; weigh_to (WeighTo), liquid_push
(LiquidPush), liquid_run (LiquidRun, continuous, caller re-calls every <1 s), barcode_read (Barcode), gantry_jog (GantryJog), locate (Trigger).
Topics: /bunny/status (String, JSON, 2 Hz), /bunny/weight (Float64).
Long operations block their service call; the node is multi-threaded so status and abort keep working.
"""
import json, socket, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float64
from harness_msgs.srv import WeighTo, LiquidPush, LiquidRun, Barcode, GantryJog


class BunnyLink:
    """One TCP connection per call (the server threads per connection), so calls never queue."""
    def __init__(self, host, port):
        self.host, self.port, self.n = host, port, 0; self.lock = threading.Lock()

    def call(self, cmd, args=None, timeout=10.0):
        with self.lock: self.n += 1; rid = self.n
        s = socket.create_connection((self.host, self.port), timeout=3.0); s.settimeout(timeout)
        try:
            s.sendall((json.dumps({"id": rid, "cmd": cmd, "args": args or {}}) + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = s.recv(65536)
                if not chunk: raise RuntimeError("Bunny closed the connection")
                buf += chunk
        finally:
            s.close()
        r = json.loads(buf.decode())
        if not r.get("ok"): raise RuntimeError(r.get("error", "Bunny error"))
        return r.get("result") or {}


class BunnyNode(Node):
    def __init__(self):
        super().__init__("bunny_node")
        self.declare_parameter("host", "192.168.7.2"); self.declare_parameter("port", 7801)   # Bunny on Wi-Fi (wired 192.168.7.2 gone 2026-09-27)
        self.link = BunnyLink(self.get_parameter("host").value, self.get_parameter("port").value)
        cb = ReentrantCallbackGroup(); self.cb = cb
        self.pub_status = self.create_publisher(String, "/bunny/status", 10)
        self.pub_weight = self.create_publisher(Float64, "/bunny/weight", 10)
        self.create_timer(0.5, self.tick, callback_group=cb)
        for name, long in (("housing_open", 60), ("housing_close", 60), ("housing_half", 60), ("housing_release", 5), ("tare", 60),
                           ("abort", 5), ("load_begin", 5), ("load_end", 5), ("liquid_suck", 10), ("liquid_stop", 5),
                           ("gantry_park", 60), ("locate", 60)):
            self.create_service(Trigger, "/bunny/" + name, self._trigger(name, long), callback_group=cb)
        self.create_service(WeighTo, "/bunny/weigh_to", self.weigh_to, callback_group=cb)
        self.create_service(LiquidPush, "/bunny/liquid_push", self.liquid_push, callback_group=cb)
        self.create_service(LiquidPush, "/bunny/platform", self.platform, callback_group=cb)
        self.create_service(LiquidRun, "/bunny/liquid_run", self.liquid_run, callback_group=cb)
        self.create_service(Barcode, "/bunny/barcode_read", self.barcode, callback_group=cb)
        self.create_service(GantryJog, "/bunny/gantry_jog", self.gantry_jog, callback_group=cb)
        self.online = False

    def _trigger(self, name, timeout):
        def cb(req, res):
            try:
                r = self.link.call(name, timeout=timeout); res.success = True; res.message = json.dumps(r)
            except Exception as e:
                res.success = False; res.message = str(e)
            return res
        return cb

    def tick(self):
        try:
            st = self.link.call("status", timeout=3.0); st["online"] = True
            if not self.online: self.get_logger().info("Bunny online: %s" % st.get("unit")); self.online = True
        except Exception as e:
            st = {"online": False, "error": str(e)}
            if self.online: self.get_logger().warning("Bunny offline: %s" % e); self.online = False
        self.pub_status.publish(String(data=json.dumps(st)))
        if st.get("weight_g") is not None: self.pub_weight.publish(Float64(data=float(st["weight_g"])))

    def weigh_to(self, req, res):
        try:
            r = self.link.call("weigh_to", {"target_g": req.target_g, "material": req.material or "powder",
                                            "powder_id": req.powder_id}, timeout=1800)
            res.outcome = r.get("outcome", ""); res.actual_g = float(r.get("actual_g") or 0.0)
            res.success = res.outcome == "finished"; res.message = json.dumps(r)
        except Exception as e:
            res.success = False; res.outcome = "error"; res.message = str(e)
        return res

    def liquid_push(self, req, res):
        try:
            r = self.link.call("liquid_push", {"steps": req.steps, "rate": req.rate, "wait": req.wait},
                               timeout=60 if req.wait else 5)
            res.success = True; res.steps_done = int(r.get("steps", 0)); res.message = "ok"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def platform(self, req, res):
        """platform lift by req.steps (signed, 0 = read); res.steps_done = resulting position."""
        try:
            r = self.link.call("platform", {"steps": req.steps}, timeout=5 + abs(req.steps) * 0.005)
            res.success = True; res.steps_done = int(r.get("pos", 0)); res.message = "ok"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def liquid_run(self, req, res):
        """Continuous flow: the CALLER must repeat this at least every second, the Bunny stops otherwise."""
        try:
            self.link.call("liquid_run", {"rate": req.rate, "direction": req.direction}, timeout=5)
            res.success = True; res.message = "flowing"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def barcode(self, req, res):
        t = req.timeout_s or 30.0
        try:
            r = self.link.call("barcode", {"timeout_s": t}, timeout=t + 2)
            res.success = True; res.code = r.get("code", ""); res.name = r.get("name", ""); res.message = "ok"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def gantry_jog(self, req, res):
        try:
            self.link.call("gantry_jog", {"dx_mm": req.dx_mm, "dy_mm": req.dy_mm, "dz_up_mm": req.dz_up_mm}, timeout=60)
            res.success = True; res.message = "ok"
        except Exception as e:
            res.success = False; res.message = str(e)
        return res


def main():
    rclpy.init(); n = BunnyNode(); ex = MultiThreadedExecutor(num_threads=6); ex.add_node(n)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
