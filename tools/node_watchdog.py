#!/usr/bin/env python3
"""The harness nodes' watchdog (2026-11-16, operator: "需要…一个harness上面的看门狗，如果node失联了，能刷新重连").

Every INTERVAL_S it judges each node of `harness nodes list` (= tools/harness's NODES: the CLI's own list, one place)
and restarts a dead or unhealthy one the way `harness nodes restart <name>` does (it runs exactly that command: the
pattern killed, the node started detached in the workspace's environment, rosenv.sh sourced):

  running   its pgrep pattern matches a process
  healthy   camera: the newest "N fps" line in /tmp/camera.log is under CAMERA_STALE_S old and the camera has not
            sat at 0 fps for CAMERA_ZERO_S (the node's own USB reconnect did not bring it back) · gateway: 127.0.0.1
            :7803 accepts · bunny: the Bunny server BUNNY_HOST:7801 accepts (restarting the node cannot fix it: the
            node opens one connection per call, so an unreachable server is reported, never restarted for) · the
            others: running. A node younger than STARTUP_GRACE_S is not judged yet (healthy null, "starting").
            Any node listening on an ORPHANED Fast DDS port (dds_orphaned, 2026-11-21) = unhealthy, "DDS port orphaned —
            restart <name>": reported, never restarted for.
  restart   down for DOWN_GRACE_S (or unhealthy that long), at most once per MIN_RESTART_S per node, given up after
            GIVE_UP restarts in a row that did not make it healthy (said in its `why`; the station's restart button
            or the node turning healthy re-arms it)
  auto      false = reported, never restarted: the watchdog itself, a node not installed (no install/<package>,
            no tools/<script>), a name in $HARNESS_WATCHDOG_SKIP (comma separated), and THE ARM NODE (piper) while
            can0 is not up at 1 Mbps (bringing can0 up needs sudo — the user's button / a terminal). A start of the
            arm node never enables or moves the arm (it connects, reads the feedback, asks the firmware version and
            sets up the gripper's driver; the e-stop watch only re-sends a hold of the current joints while the
            button is pressed and the arm is powered), so piper IS restarted when its process is gone and can0 is
            up — but never killed for health: a running arm node is left alone.

Status: STATUS_FILE (~/.cache/harness_watchdog.json, written whole every pass) = {at, pid, interval_s, nodes: [{name,
running, healthy, why, restarts, last_restart_at, auto, gave_up, restarting, pid, up_s}]} (times = epoch seconds);
events on stdout (/tmp/watchdog.log under `harness nodes start`). Stdlib only, no ROS, no sudo.

    python3 tools/node_watchdog.py            (a NODES line of tools/harness: `harness nodes start` runs it)
    python3 tools/node_watchdog.py --once     one pass, print the status

At boot (the user's crontab, no sudo; the arm node then waits for can0, which `harness nodes start` brings up with sudo):
    @reboot sleep 30 && cd $HOME/bunny-harness-dev && setsid nohup python3 tools/node_watchdog.py >>/tmp/watchdog.log 2>&1 </dev/null &
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "tools", "harness")
STATUS_FILE = os.path.expanduser("~/.cache/harness_watchdog.json")
LOG_DIR = "/tmp"                      # tools/harness start_bg writes /tmp/<name>.log
INTERVAL_S = 3.0
DOWN_GRACE_S = 6.0                    # a node down / unhealthy this long is restarted (two passes: never mid-restart)
MIN_RESTART_S = 30.0                  # at most one restart per node in this time
GIVE_UP = 5                           # restarts in a row that did not make the node healthy
STARTUP_GRACE_S = 30.0                # a younger node is not judged for health yet
CAMERA_STALE_S = 15.0                 # the camera node logs "N fps" every 5 s
CAMERA_ZERO_S = 30.0
RESTART_TIMEOUT_S = 90.0
GATEWAY_PORT = int(os.environ.get("GATEWAY_PORT", 7803))
BUNNY_HOST, BUNNY_PORT = os.environ.get("BUNNY_HOST", "192.168.7.2"), 7801
CAN_IF = os.environ.get("CAN_IF", "can0")
SELF = "watchdog"
FPS_RE = re.compile(r"\[(\d{9,}\.\d+)\]\s+\[[^\]]*\]:\s+([\d.]+) fps\s*$")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")       # ros2 launch emulate_tty: rcutils may colour the lines
LOCK = threading.Lock()


def log(msg):
    print(time.strftime("%Y-%m-%d %H:%M:%S ") + msg, flush=True)


def node_list(cli=CLI):
    """[(name, pgrep pattern, start command)] from `harness nodes list` (the CLI's NODES)."""
    out = subprocess.run([cli, "nodes", "list"], capture_output=True, text=True, timeout=20, stdin=subprocess.DEVNULL).stdout
    nodes = []
    for line in out.splitlines():
        parts = line.strip().split("|")
        if len(parts) == 3 and parts[0]:
            nodes.append(tuple(p.strip() for p in parts))
    return nodes


def installed(cmd, root=ROOT):
    """'' when the start command's package / script is there, else why not."""
    w = cmd.split()
    if len(w) >= 3 and w[0] == "ros2" and w[1] in ("run", "launch"):
        return "" if os.path.isdir(os.path.join(root, "install", w[2])) else f"not installed (no install/{w[2]})"
    for a in w[1:]:
        if a.endswith(".py"):
            return "" if os.path.exists(os.path.join(root, a)) else f"not installed (no {a})"
    return ""


def pids(pattern):
    r = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return [int(p) for p in r.stdout.split()] if r.returncode == 0 else []


def proc_age(pid):
    """Seconds since process pid started (/proc), or None."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            start = float(f.read().rsplit(")", 1)[1].split()[19]) / os.sysconf("SC_CLK_TCK")
        with open("/proc/uptime") as f:
            return float(f.read().split()[0]) - start
    except (OSError, ValueError, IndexError):
        return None


DDS_LOCK_RE = re.compile(r"fastdds_port(\d+)_el(?: \(deleted\))?$")


def dds_orphaned(pid, proc="/proc"):
    """True when process pid still listens on a Fast DDS shared-memory port that was deleted (and re-created by another
    process's zombie check): nothing started later reaches it (2026-11-21, the arm node; rosenv.sh's comment). Its OWN
    ports = the fastdds_port<N>_el locks it holds open; a deleted port it only writes to is a dead peer's, not this.
    Every node runs on UDPv4 since then, so this should stay False (tools/harness `nodes status` checks the same)."""
    try:
        own = set()
        for fd in os.listdir(f"{proc}/{pid}/fd"):
            try:
                m = DDS_LOCK_RE.search(os.readlink(f"{proc}/{pid}/fd/{fd}"))
            except OSError:
                continue
            if m:
                own.add(m.group(1))
        if not own:
            return False
        with open(f"{proc}/{pid}/maps") as f:
            return any(f"/dev/shm/fastdds_port{n} (deleted)" in line for line in f for n in own)
    except OSError:
        return False


def tcp_ok(host, port, timeout=1.5):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def can_up(iface=CAN_IF):
    """can0 up at 1 Mbps (tools/harness can_up's test, without bringing it up)."""
    r = subprocess.run(["ip", "-details", "link", "show", iface], capture_output=True, text=True)
    return r.returncode == 0 and " state UP " in r.stdout and "bitrate 1000000" in r.stdout


def camera_health(path, now, stale_s=CAMERA_STALE_S, zero_s=CAMERA_ZERO_S):
    """(healthy, why) from the camera node's "N fps" lines (every 5 s, a ROS log line with its own timestamp; a line
    without one is dated by the file's mtime). healthy None = at 0 fps for less than zero_s: not judged yet (the node
    reconnects the camera by itself), neither restarted nor counted as healthy."""
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 65536))
            lines = ANSI_RE.sub("", f.read().decode("utf-8", "replace")).splitlines()
        mtime = os.path.getmtime(path)
    except OSError:
        return False, f"no log {path}"
    fps = []
    for ln in lines:
        m = FPS_RE.search(ln)
        if m:
            fps.append((float(m.group(1)), float(m.group(2))))
        elif ln.rstrip().endswith(" fps"):
            try:
                fps.append((mtime, float(ln.rstrip()[:-4].split()[-1])))
            except ValueError:
                pass
    if not fps:
        return False, "no fps line in its log"
    t_last, f_last = fps[-1]
    if now - t_last > stale_s:
        return False, f"no fps line for {now - t_last:.0f} s (the node hangs?)"
    if f_last > 0:
        return True, f"{f_last:.1f} fps"
    nonzero = [t for t, f in fps if f > 0]
    since = nonzero[-1] if nonzero else fps[0][0]
    if now - since >= zero_s:
        return False, f"0 fps for {now - since:.0f} s (the camera sends no frames: its USB cable?)"
    return None, f"0 fps for {now - since:.0f} s (the node reconnects by itself)"


class Node:
    def __init__(self, name, pattern, cmd):
        self.name, self.pattern, self.cmd = name, pattern, cmd
        self.restarts = 0                 # by the watchdog, since it started
        self.in_row = 0                   # restarts since the node was last healthy
        self.last_restart_at = None
        self.bad_since = None
        self.gave_up = False
        self.restarting = False
        self.st = {"running": False, "healthy": None, "why": "", "auto": False, "pid": None, "up_s": None}

    def view(self):
        return dict(name=self.name, restarts=self.restarts, last_restart_at=self.last_restart_at,
                    gave_up=self.gave_up, restarting=self.restarting, **self.st)


class Watchdog:
    def __init__(self, cli=CLI, root=ROOT, status_file=STATUS_FILE, log_dir=LOG_DIR, interval=INTERVAL_S,
                 down_grace=DOWN_GRACE_S, min_restart=MIN_RESTART_S, give_up=GIVE_UP, startup_grace=STARTUP_GRACE_S,
                 camera_stale=CAMERA_STALE_S, camera_zero=CAMERA_ZERO_S, gateway_port=GATEWAY_PORT,
                 bunny=(BUNNY_HOST, BUNNY_PORT), skip=None):
        self.cli, self.root, self.status_file, self.log_dir = cli, root, status_file, log_dir
        self.interval, self.down_grace, self.min_restart, self.give_up = interval, down_grace, min_restart, give_up
        self.startup_grace, self.camera_stale, self.camera_zero = startup_grace, camera_stale, camera_zero
        self.gateway_port, self.bunny = gateway_port, bunny
        self.skip = set(skip if skip is not None else
                        [s.strip() for s in os.environ.get("HARNESS_WATCHDOG_SKIP", "").split(",") if s.strip()])
        self.nodes, self.listed_at = {}, 0.0
        self.env = dict(os.environ)
        lib = os.path.expanduser("~/lib/libapriltag.so.3")
        if not self.env.get("HARNESS_APRILTAG_LIB") and os.path.exists(lib):
            self.env["HARNESS_APRILTAG_LIB"] = lib      # the camera's AprilTag 3 library (the PC keeps it in ~/lib)

    def refresh_list(self, now):
        if now - self.listed_at < 60 and self.nodes:
            return
        try:
            listed = node_list(self.cli)
        except (OSError, subprocess.SubprocessError) as e:
            log(f"cannot read the node list ({e}); keeping the last one")
            return
        self.listed_at = now
        keep = {}
        for name, pattern, cmd in listed:
            n = self.nodes.get(name) or Node(name, pattern, cmd)
            n.pattern, n.cmd = pattern, cmd
            keep[name] = n
        self.nodes = keep

    def judge(self, n, now):
        """running / healthy / why / auto for node n."""
        ps = pids(n.pattern)
        age = proc_age(ps[0]) if ps else None
        st = {"running": bool(ps), "healthy": None, "why": "", "auto": True, "pid": ps[0] if ps else None,
              "up_s": round(age, 1) if age is not None else None, "restart_unhealthy": True}
        miss = installed(n.cmd, self.root)
        if n.name == SELF:
            st.update(auto=False, healthy=bool(ps), why="this watchdog")
            return st
        if n.name in self.skip:
            st["auto"], st["why"] = False, "skipped ($HARNESS_WATCHDOG_SKIP)"
        elif miss:
            st["auto"], st["why"] = False, miss
        if n.name == "piper":
            st["restart_unhealthy"] = False          # a running arm node is never killed by the watchdog
            if not can_up():
                st["auto"] = False
                st["why"] = (st["why"] + "; " if st["why"] else "") + \
                    f"{CAN_IF} is not up at 1 Mbps (needs sudo: restart from the station or `harness nodes start piper`)"
        if not ps:
            st["healthy"] = False
            st["why"] = "not running" + ("; " + st["why"] if st["why"] else "")
            return st
        if any(dds_orphaned(p) for p in ps):
            # reported, never restarted for: a restart mid-run (the arm: it must be supported) is the user's call
            st.update(healthy=False, restart_unhealthy=False)
            st["why"] = f"DDS port orphaned — restart {n.name}" + ("; " + st["why"] if st["why"] else "")
            return st
        if age is not None and age < self.startup_grace and n.name in ("camera", "gateway"):
            st["why"] = st["why"] or f"starting ({age:.0f} s)"
            return st
        if n.name == "camera":
            ok, why = camera_health(os.path.join(self.log_dir, "camera.log"), now, self.camera_stale, self.camera_zero)
        elif n.name == "gateway":
            ok = tcp_ok("127.0.0.1", self.gateway_port)
            why = f"port {self.gateway_port} accepts" if ok else f"port {self.gateway_port} does not accept"
        elif n.name == "bunny":
            ok = tcp_ok(*self.bunny)
            why = (f"Bunny server {self.bunny[0]}:{self.bunny[1]} reachable" if ok else
                   f"Bunny server {self.bunny[0]}:{self.bunny[1]} not reachable (start the Bunny app; restarting the "
                   "node would not help)")
            st["restart_unhealthy"] = False
        else:
            ok, why = True, "running"
        st["healthy"] = ok
        st["why"] = why + ("; " + st["why"] if st["why"] and st["why"] != why else "")
        return st

    def restart(self, n, why):
        """`harness nodes restart <name>` in a thread (detached start, the workspace's environment)."""
        n.restarting = True
        n.restarts += 1
        n.in_row += 1
        n.last_restart_at = time.time()
        log(f"restarting {n.name} ({why}) - restart {n.in_row} in a row")

        def run():
            try:
                r = subprocess.run([self.cli, "nodes", "restart", n.name], capture_output=True, text=True, env=self.env,
                                   stdin=subprocess.DEVNULL, timeout=RESTART_TIMEOUT_S, cwd=self.root)
                log(f"  {n.name}: " + " / ".join(x for x in (r.stdout + r.stderr).strip().splitlines() if x))
            except (OSError, subprocess.SubprocessError) as e:
                log(f"  {n.name}: restart failed: {e}")
            finally:
                with LOCK:
                    n.restarting = False
        threading.Thread(target=run, daemon=True).start()

    def step(self, now=None):
        now = time.time() if now is None else now
        self.refresh_list(now)
        for n in list(self.nodes.values()):
            with LOCK:
                if n.restarting:
                    continue
            st = self.judge(n, now)
            ru = st.pop("restart_unhealthy")
            n.st = st
            if st["healthy"] is True:
                if n.in_row or n.gave_up:
                    log(f"{n.name}: healthy again ({st['why']})")
                n.in_row, n.gave_up, n.bad_since = 0, False, None
                continue
            bad = (not st["running"]) or (st["healthy"] is False and ru)
            if not bad or not st["auto"]:
                n.bad_since = None
                continue
            n.bad_since = n.bad_since or now
            if n.gave_up:
                st["why"] = f"gave up after {self.give_up} restarts in a row: {st['why']}"
                continue
            if now - n.bad_since < self.down_grace:
                continue
            if n.last_restart_at and now - n.last_restart_at < self.min_restart:
                continue
            if n.in_row >= self.give_up:
                n.gave_up = True
                st["why"] = f"gave up after {self.give_up} restarts in a row: {st['why']}"
                log(f"giving up on {n.name}: {st['why']} - restart it from the station (nodes.restart)")
                continue
            self.restart(n, st["why"])
        self.write(now)

    def status(self, now=None):
        return {"at": round(time.time() if now is None else now, 2), "pid": os.getpid(), "interval_s": self.interval,
                "nodes": [n.view() for n in self.nodes.values()]}

    def write(self, now):
        doc = self.status(now)
        os.makedirs(os.path.dirname(self.status_file), exist_ok=True)
        tmp = self.status_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump(doc, f)
        os.replace(tmp, self.status_file)

    def run(self):
        log(f"watchdog up: every {self.interval:g} s, restart after {self.down_grace:g} s bad, at most once per "
            f"{self.min_restart:g} s, give up after {self.give_up} in a row; status {self.status_file}")
        while True:
            t0 = time.time()
            try:
                self.step()
            except Exception as e:  # noqa: BLE001  the watchdog itself must not die
                log(f"pass failed: {e!r}")
            time.sleep(max(0.2, self.interval - (time.time() - t0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--once", action="store_true", help="one pass (no restart can be due), print the status")
    a = ap.parse_args()
    w = Watchdog()
    if a.once:
        w.step()
        json.dump(w.status(), sys.stdout, indent=1)
        print()
        return
    w.run()


if __name__ == "__main__":
    main()
