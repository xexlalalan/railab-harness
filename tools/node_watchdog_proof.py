#!/usr/bin/env python3
"""Offline proof of the nodes' watchdog (tools/node_watchdog.py) and the gateway's nodes.restart / nodes.status: no ROS,
no harness. A fake workspace in a temporary folder: a fake `harness` CLI (the real one's `nodes list` / `nodes restart`
contract: kill the pattern, start detached with /<logs>/<name>.log), fake nodes (a camera that logs "N fps" lines the
way tag_node does, a gateway that accepts on a port, a Bunny node + a fake Bunny server, an arm node, a node that dies
at once, one whose package is not installed) and a fake `ip` saying whether can0 is up. The real Watchdog class runs
with every time scaled down (interval 0.25 s, bad for 0.5 s -> restart, at most one restart per 1.5 s, give up after 3).

    python3 tools/node_watchdog_proof.py
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import node_watchdog as NW  # noqa: E402

FAKE_CAMERA = r'''
import os, sys, time
mode = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_mode")
while True:
    m = open(mode).read().strip() if os.path.exists(mode) else "ok"
    if m != "hang":
        print("[tag_node-1] [INFO] [%.9f] [tag_node]: %s fps" % (time.time(), "0.0" if m == "zero" else "29.8"), flush=True)
    time.sleep(0.3)
'''
FAKE_GATEWAY = r'''
import os, socket, sys, time
port, mode = int(sys.argv[1]), os.path.join(os.path.dirname(os.path.abspath(__file__)), "gateway_mode")
s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(("127.0.0.1", port)); s.listen(8)
s.settimeout(0.2)
while True:
    if os.path.exists(mode) and open(mode).read().strip() == "hang":
        s.close(); time.sleep(1e6)          # alive, but no longer accepting
    try:
        c, _ = s.accept(); c.close()
    except OSError:
        pass
'''
SLEEPER = "import time\nwhile True: time.sleep(1)\n"
BROKEN = "raise SystemExit(1)\n"
CLI_TMPL = r'''#!/usr/bin/env bash
NODES="camera|fake_tag_nod[e]|python3 @T@/fake_tag_node.py
gateway|fake_gatewa[y]|python3 @T@/fake_gateway.py @GP@
bunny|fake_bunny_nod[e]|python3 @T@/fake_bunny_node.py
piper|fake_piper_nod[e]|python3 @T@/fake_piper_node.py
broken|fake_broke[n]|python3 @T@/fake_broken.py
dtv|fake_dtv_nod[e]|ros2 run harness_dtv dtv_node"
case "$1 $2" in
  "nodes list") echo "$NODES"; exit 0 ;;
  "nodes restart") line=$(grep "^$3|" <<<"$NODES") || exit 2
     IFS='|' read -r n p c <<<"$line"
     pkill -f "$p" && echo "$n: stopped" || echo "$n: was not running"; sleep 0.2
     setsid nohup $c >@T@/logs/$n.log 2>&1 </dev/null &
     echo "$n: started"; exit 0 ;;
esac
exit 2
'''
IP_TMPL = r'''#!/usr/bin/env bash
if [ -e @T@/can_up ]; then echo "3: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT"; echo "    can state ERROR-ACTIVE bitrate 1000000 sample-point 0.875"
else echo "3: can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT"; fi
'''


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    T = tempfile.mkdtemp(prefix="watchdog_proof_")
    root = os.path.join(T, "repo")
    os.makedirs(os.path.join(root, "tools"))
    os.makedirs(os.path.join(root, "install"))       # no harness_dtv in it: "dtv" is not installed
    os.makedirs(os.path.join(T, "logs"))
    os.makedirs(os.path.join(T, "bin"))
    gport, bport = free_port(), free_port()
    for name, body in (("fake_tag_node.py", FAKE_CAMERA), ("fake_gateway.py", FAKE_GATEWAY), ("fake_bunny_node.py", SLEEPER),
                       ("fake_piper_node.py", SLEEPER), ("fake_broken.py", BROKEN)):
        with open(os.path.join(T, name), "w") as f:
            f.write(body)
    cli = os.path.join(root, "tools", "harness")
    for path, body in ((cli, CLI_TMPL.replace("@T@", T).replace("@GP@", str(gport))),
                       (os.path.join(T, "bin", "ip"), IP_TMPL.replace("@T@", T))):
        with open(path, "w") as f:
            f.write(body)
        os.chmod(path, 0o755)
    os.environ["PATH"] = os.path.join(T, "bin") + os.pathsep + os.environ["PATH"]
    bunny_server = socket.socket()
    bunny_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bunny_server.bind(("127.0.0.1", bport))
    bunny_server.listen(8)

    def accept_all():                                 # the Bunny app's server accepts every connection
        while True:
            try:
                c, _ = bunny_server.accept()
                c.close()
            except OSError:
                return
    threading.Thread(target=accept_all, daemon=True).start()
    status = os.path.join(T, "status.json")
    w = NW.Watchdog(cli=cli, root=root, status_file=status, log_dir=os.path.join(T, "logs"), interval=0.25, down_grace=0.5,
                    min_restart=1.5, give_up=3, startup_grace=0.8, camera_stale=1.5, camera_zero=2.0, gateway_port=gport,
                    bunny=("127.0.0.1", bport), skip=[])
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            w.step()
            time.sleep(w.interval)
    th = threading.Thread(target=loop, daemon=True)
    t0 = time.time()
    th.start()
    results = []

    def node(name):
        return next((x for x in w.status()["nodes"] if x["name"] == name),
                    {"running": False, "healthy": None, "auto": None, "restarts": 0, "gave_up": False, "why": "", "pid": None})

    def wait(cond, timeout=15.0):
        t = time.time()
        while time.time() - t < timeout:
            if cond():
                return time.time() - t
            time.sleep(0.05)
        return None

    def check(label, ok, detail):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")

    try:
        # 1. from nothing (a reboot): every installed node with auto started; the arm node left down (can0 down)
        dt = wait(lambda: all(node(n)["healthy"] for n in ("camera", "gateway", "bunny")))
        p = node("piper")
        check("start from nothing", dt is not None and not p["running"] and not p["auto"] and node("dtv")["auto"] is False,
              f"camera / gateway / bunny healthy after {dt or -1:.1f} s; piper running={p['running']} auto={p['auto']} "
              f"({p['why']}); dtv auto={node('dtv')['auto']} ({node('dtv')['why']})")

        # 2. the camera killed -> restarted after the grace; killed again at once -> held back to MIN_RESTART_S
        time.sleep(1.6)
        r0, pid0 = node("camera")["restarts"], node("camera")["pid"]
        tk = time.time()
        subprocess.run(["pkill", "-f", "fake_tag_nod[e]"])
        dt = wait(lambda: node("camera")["restarts"] == r0 + 1 and node("camera")["running"])
        check("camera killed -> restarted", dt is not None and node("camera")["pid"] != pid0,
              f"running again {dt or -1:.2f} s after the kill (grace 0.5 s + a pass), restarts {r0} -> {node('camera')['restarts']}")
        t_first = node("camera")["last_restart_at"]
        subprocess.run(["pkill", "-f", "fake_tag_nod[e]"])
        wait(lambda: node("camera")["restarts"] == r0 + 2 and node("camera")["running"])
        gap = node("camera")["last_restart_at"] - t_first
        check("back-off: killed again at once", gap >= 1.5, f"the next restart {gap:.2f} s after the previous one (>= 1.5)")

        # 3. the camera's log goes silent (node hangs) -> restarted; at 0 fps for 2 s -> restarted
        wait(lambda: node("camera")["healthy"] is True)
        time.sleep(1.6)
        r0 = node("camera")["restarts"]
        with open(os.path.join(T, "camera_mode"), "w") as f:
            f.write("hang")
        th0 = time.time()
        wait(lambda: node("camera")["healthy"] is False)
        why = node("camera")["why"]
        dt = wait(lambda: node("camera")["restarts"] == r0 + 1)
        took = time.time() - th0
        with open(os.path.join(T, "camera_mode"), "w") as f:     # the restarted node logs again
            f.write("ok")
        check("camera log silent -> restarted", dt is not None, f"judged unhealthy ({why}), restarted "
              f"{took:.1f} s after the log stopped (stale 1.5 s + grace 0.5 s)")
        wait(lambda: node("camera")["healthy"] is True)
        time.sleep(1.6)
        r0 = node("camera")["restarts"]
        with open(os.path.join(T, "camera_mode"), "w") as f:
            f.write("zero")
        tz = time.time()
        wait(lambda: node("camera")["healthy"] is False)
        why = node("camera")["why"]
        dt = wait(lambda: node("camera")["restarts"] == r0 + 1)
        check("camera at 0 fps -> restarted", dt is not None, f"({why}) restarted {time.time() - tz:.1f} s after the first 0 fps")

        # 4. give up after 3 in a row (the camera stays at 0 fps), the station's restart re-arms it once healthy
        dt = wait(lambda: node("camera")["gave_up"], timeout=30)
        c = node("camera")
        check("give up after 3 in a row", dt is not None and c["restarts"] >= r0 + 3,
              f"gave_up after restarts {r0} -> {c['restarts']}; why: {c['why']}")
        with open(os.path.join(T, "camera_mode"), "w") as f:
            f.write("ok")
        import gateway_server as G                     # the gateway's nodes.restart / nodes.status on the same fake CLI
        G.CLI, G.ROOT, G.RESTART_LOG = cli, root, os.path.join(T, "logs", "nodes_restart.log")
        G.gw = type("GW", (), {"cancel_arm": lambda self: []})()
        pid0 = subprocess.run(["pgrep", "-f", "fake_tag_nod[e]"], capture_output=True, text=True).stdout.split()
        ts = time.time()
        res = G.nodes_restart("camera")
        took = time.time() - ts
        dp = wait(lambda: (lambda p: p and p != pid0)(subprocess.run(["pgrep", "-f", "fake_tag_nod[e]"], capture_output=True,
                                                                      text=True).stdout.split()))
        dt = wait(lambda: node("camera")["healthy"] is True and not node("camera")["gave_up"])
        check("gateway nodes.restart (the station's button)", res == {"ok": True, "restarted": ["camera"]} and took < 0.5
              and dp is not None and dt is not None,
              f"{res} answered in {took * 1000:.0f} ms; a new camera process {dp or -1:.1f} s later (after the 0.5 s "
              f"wait for the reply); healthy again, the give-up cleared")

        # 5. a node that dies at once: 3 restarts, then given up and said so
        dt = wait(lambda: node("broken")["gave_up"], timeout=30)
        b = node("broken")
        check("a node that never stays up", dt is not None and b["restarts"] == 3, f"restarts {b['restarts']}, why: {b['why']}")

        # 6. the gateway stops accepting -> restarted
        time.sleep(1.6)
        r0 = node("gateway")["restarts"]
        with open(os.path.join(T, "gateway_mode"), "w") as f:
            f.write("hang")
        wait(lambda: node("gateway")["healthy"] is False)
        why = node("gateway")["why"]
        os.remove(os.path.join(T, "gateway_mode"))
        dt = wait(lambda: node("gateway")["restarts"] == r0 + 1 and node("gateway")["healthy"] is True)
        check("gateway port closed -> restarted", dt is not None, f"({why}) restarted, accepting again")

        # 7. the Bunny server down -> bunny unhealthy, NOT restarted (one connection per call: a restart cannot help)
        r0 = node("bunny")["restarts"]
        bunny_server.close()
        wait(lambda: node("bunny")["healthy"] is False)
        time.sleep(2.5)
        b = node("bunny")
        check("Bunny server unreachable -> reported only", b["restarts"] == r0 and b["running"], f"restarts {b['restarts']}, why: {b['why']}")

        # 8. the arm node: down + can0 down -> left alone; can0 up -> started; running is never killed for health
        p = node("piper")
        check("arm node with can0 down", not p["running"] and p["restarts"] == 0, f"running={p['running']}, restarts 0: {p['why']}")
        open(os.path.join(T, "can_up"), "w").close()
        dt = wait(lambda: node("piper")["running"])
        check("arm node with can0 up", dt is not None, f"started {dt or -1:.1f} s after can0 came up (restarts {node('piper')['restarts']})")

        # 9. nodes.restart all: one detached shell, the gateway last; nodes.status from the status file
        res = G.nodes_restart("all")
        check("nodes.restart all", res["restarted"][-1] == "gateway" and "dtv" not in res["restarted"], f"{res}")
        NW.STATUS_FILE = status
        v = G.Gateway.nodes_view(type("S", (), {})())
        check("nodes.status from the file", v["watchdog"]["running"] and len(v["nodes"]) == 6,
              f"watchdog {v['watchdog']}, {len(v['nodes'])} nodes: " +
              ", ".join(f"{x['name']} {'up' if x['running'] else 'down'}/{x['healthy']}/auto {x['auto']}" for x in v["nodes"]))
        print("\nthe status file (one node):", json.dumps(node("camera")))
    finally:
        stop.set()
        th.join(2)
        time.sleep(3.0)                               # nodes.restart's detached shell and restart threads finish
        for _ in range(2):
            for p in ("fake_tag_nod[e]", "fake_gatewa[y]", "fake_bunny_nod[e]", "fake_piper_nod[e]", "fake_broke[n]"):
                subprocess.run(["pkill", "-f", p])
            time.sleep(0.5)
        shutil.rmtree(T, ignore_errors=True)
    print(f"\n{sum(results)} / {len(results)} passed in {time.time() - t0:.0f} s")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
