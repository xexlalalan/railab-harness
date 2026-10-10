#!/usr/bin/env python3
"""Workflow server: the external planner's way in over the RJ45 (TCP, newline-delimited JSON, port 7802).

One request per line, replies are JSON lines on the same connection; a `run` streams the step's
output and ends with a "finish" event, then the connection closes. One step runs at a time.
"workflow" is optional everywhere: it defaults to the server's workflow (--workflow, default origami).

  {"cmd": "run", "step": 4}            (or with "workflow": "origami")
      -> {"event": "started", "workflow": "origami", "step": 4, "script": "04_citric_acid_4_803g.sh"}
      -> {"event": "output", "line": ">>> harness bunny tare"}          (every line the step prints)
      -> {"event": "finish", "workflow": "origami", "step": 4, "ok": true, "rc": 0, "seconds": 41.2}
      -> {"event": "refused", "reason": "busy", "workflow": "origami", "step": 3}   (while another step runs)
  {"cmd": "status"}  -> {"busy": true, "workflow": "origami", "step": 4, "since_s": 12.0, "last": {...}}
                     -> {"busy": false, "last": {"workflow": ..., "step": ..., "ok": ..., "rc": ..., "seconds": ..., "finished_at": ...}}
  {"cmd": "list"}    -> {"workflow": "origami", "steps": [{"step": 1, "script": "01_housing_open.sh", "title": "open the Bunny housing"}, ...]}
  {"cmd": "stop"}    -> {"ok": true, "stopped": {...}}   SIGINT to the running step (the `harness` call in flight is cancelled;
                                                          a Bunny dispense in progress keeps going until `harness bunny abort`)
Errors: {"error": "..."}.  A workflow is a directory workflows/<dir>/ whose supervisor executable is named <workflow>
(workflows/origami-robot/origami, workflows/neuromorphic/neuromorphic); the step scripts are NN_*.sh next to it.
"""
import glob, json, os, signal, socketserver, subprocess, sys, threading, time

PORT = int(os.environ.get("WORKFLOW_PORT", 7802))
DEFAULT_WORKFLOW = "origami"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKFLOWS = os.path.join(ROOT, "workflows")

lock = threading.Lock()
state = {"busy": None, "proc": None, "last": None}   # busy: {"workflow", "step", "t0"}


def workflow_dir(name):
    hits = [p for p in glob.glob(os.path.join(WORKFLOWS, "*", name)) if os.access(p, os.X_OK)]
    if not hits:
        raise ValueError("unknown workflow '%s' (known: %s)" % (name, ", ".join(sorted(
            os.path.basename(p) for p in glob.glob(os.path.join(WORKFLOWS, "*", "*")) if os.access(p, os.X_OK) and not p.endswith((".sh", ".py"))))))
    return os.path.dirname(hits[0])


def steps_of(d):
    out = []
    for f in sorted(glob.glob(os.path.join(d, "[0-9][0-9]_*.sh"))):
        with open(f) as fh:
            lines = fh.read().splitlines()
        title = lines[1][2:] if len(lines) > 1 and lines[1].startswith("# ") else ""
        out.append({"step": int(os.path.basename(f)[:2]), "script": os.path.basename(f), "title": title})
    return out


class Handler(socketserver.StreamRequestHandler):
    def send(self, obj):
        self.wfile.write((json.dumps(obj) + "\n").encode()); self.wfile.flush()

    def handle(self):
        line = self.rfile.readline()
        if not line:
            return
        try:
            req = json.loads(line.decode())
            cmd = req.get("cmd")
            if cmd == "status":
                self.send(self.status())
            elif cmd == "list":
                wf = str(req.get("workflow") or DEFAULT_WORKFLOW)
                self.send({"workflow": wf, "steps": steps_of(workflow_dir(wf))})
            elif cmd == "run":
                self.run(str(req.get("workflow") or DEFAULT_WORKFLOW), int(req["step"]))
            elif cmd == "stop":
                self.send(self.stop())
            else:
                self.send({"error": "unknown cmd %r (run | status | list | stop)" % cmd})
        except Exception as e:  # noqa: BLE001
            try: self.send({"error": str(e)})
            except OSError: pass

    @staticmethod
    def status():
        with lock:
            b = state["busy"]
            if b:
                return {"busy": True, "workflow": b["workflow"], "step": b["step"], "since_s": round(time.time() - b["t0"], 1), "last": state["last"]}
            return {"busy": False, "last": state["last"]}

    @staticmethod
    def stop():
        with lock:
            b, p = state["busy"], state["proc"]
        if not b or p is None:
            return {"ok": False, "error": "nothing running"}
        os.killpg(p.pid, signal.SIGINT)
        return {"ok": True, "stopped": {"workflow": b["workflow"], "step": b["step"]}}

    def run(self, wf, step):
        d = workflow_dir(wf)
        script = next((s for s in steps_of(d) if s["step"] == step), None)
        if script is None:
            self.send({"error": "no step %d in %s" % (step, wf)}); return
        with lock:
            if state["busy"]:
                self.send({"event": "refused", "reason": "busy", "workflow": state["busy"]["workflow"], "step": state["busy"]["step"]}); return
            env = dict(os.environ, PATH=os.path.expanduser("~/bin") + ":" + os.environ.get("PATH", ""))
            p = subprocess.Popen(["bash", os.path.join(d, script["script"])], cwd=d, env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                 text=True, bufsize=1, start_new_session=True)
            state["busy"] = {"workflow": wf, "step": step, "t0": time.time()}; state["proc"] = p
        t0 = time.time()
        self.send({"event": "started", "workflow": wf, "step": step, "script": script["script"]})
        print("%s run %s %d" % (time.strftime("%H:%M:%S"), wf, step), flush=True)
        try:
            for out in p.stdout:
                out = out.rstrip("\n")
                print("   " + out, flush=True)
                try: self.send({"event": "output", "line": out})
                except OSError: pass              # planner went away: the step still runs to its end
            rc = p.wait()
        finally:
            with lock:
                last = {"workflow": wf, "step": step, "ok": rc == 0, "rc": rc, "seconds": round(time.time() - t0, 1),
                        "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
                state.update(busy=None, proc=None, last=last)
        print("%s finish %s %d rc=%d" % (time.strftime("%H:%M:%S"), wf, step, rc), flush=True)
        try: self.send(dict(last, event="finish"))
        except OSError: pass


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    global DEFAULT_WORKFLOW
    if "--workflow" in sys.argv:
        DEFAULT_WORKFLOW = sys.argv[sys.argv.index("--workflow") + 1]
    workflow_dir(DEFAULT_WORKFLOW)
    srv = Server(("0.0.0.0", PORT), Handler)
    print("workflow server on port %d, default workflow %s, workflows in %s" % (PORT, DEFAULT_WORKFLOW, WORKFLOWS), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
