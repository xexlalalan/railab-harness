# Driving the Bunny Harness from an external orchestrator

For the agent/program on the orchestration PC. The harness PC runs a small **workflow server**; you connect to it over the RJ45 cable, ask it to run a numbered step, and it tells you when the step has finished. Nothing else is needed on your side — no ROS, no SDK, no library.

## Connection

| Item | Value |
|---|---|
| Harness PC address (cable) | `192.168.7.1` — your own cable interface must be in `192.168.7.0/24`, e.g. `192.168.7.3/24` (not `.1`, not `.2`: `.2` is the Bunny) |
| Port | `7802`, plain TCP |
| Framing | one JSON object per line (`\n`-terminated), UTF-8; **one request per connection** |
| Reply | JSON lines on the same connection; the server closes the connection when it is done |

Check it is alive: `echo '{"cmd":"status"}' | nc 192.168.7.1 7802` → `{"busy": false, "last": null}`.

## Requests

### Run a step — `{"cmd":"run","step":N}`

Runs step `N` of the (fixed) workflow and streams its progress. Reply lines, in order:

```json
{"event": "started", "workflow": "origami", "step": 4, "script": "04_citric_acid_4_803g.sh"}
{"event": "output", "line": ">>> harness bunny tare"}
{"event": "output", "line": "OK ..."}
...
{"event": "finish", "workflow": "origami", "step": 4, "ok": true, "rc": 0, "seconds": 41.2, "finished_at": "2026-10-10T12:03:38"}
```

- **`finish` is always the last line** and the connection closes right after it. Read lines until EOF; the step is done when you have seen `finish`.
- `ok: true` = the step completed (every hardware call inside it answered OK). `ok: false` = it stopped at the first failing call; `rc` is the exit code, the preceding `output` lines say which call failed.
- Only one step runs at a time. A `run` while another step is running is refused immediately: `{"event": "refused", "reason": "busy", "workflow": "origami", "step": 3}` — nothing is queued; wait and retry.
- If you disconnect early the step **keeps running to its end**; its result is then visible in `status` → `last`.

### Is the harness busy? — `{"cmd":"status"}`

One line, from any connection, at any time (also while a step is running):

```json
{"busy": true,  "workflow": "origami", "step": 4, "since_s": 12.0, "last": {...}}
{"busy": false, "last": {"workflow": "origami", "step": 4, "ok": true, "rc": 0, "seconds": 41.2, "finished_at": "..."}}
```

`last` is the most recent finished step (`null` after a server start).

### Other

- `{"cmd":"list"}` → `{"workflow": "origami", "steps": [{"step": 1, "script": "...", "title": "..."}, ...]}`
- `{"cmd":"stop"}` → interrupts the running step (`{"ok": true, "stopped": {...}}`); the step then finishes with `ok: false`. A Bunny dispense already in progress continues on the instrument — use this only for the scripts' own flow.
- Anything malformed or unknown → `{"error": "..."}`.

## The steps (workflow `origami`)

| Step | What happens on the harness |
|---|---|
| 1 | open the Bunny housing (windshield) |
| 2 | half-close the housing (10° short of closed) |
| 3 | close the housing |
| 4 | tare the scale, then weigh **4.803 g of "Citric Acid"** powder into the receiver on the scale |
| 5 | tare, then dose **20 g of liquid** by weight into the receiver on the scale |
| 6 | dilute-to-volume: V-blocks clamp the flask, the liquid doser fills to the ring mark under the camera, V-blocks open 10 mm |
| 7 | dose **9 mL of liquid** by volume (open-loop pump steps; the receiver need not be on the scale) |
| 8 | pipette aspirates 1000 µL at 500 µL/s with 1 µL pre-aspirated air |
| 9 | pipette dispenses 1000 µL at 500 µL/s with 100 µL blow-out |
| 10 | pipette ejects its tip |

Steps take no arguments. What must be physically in place before each step (receiver on the scale, flask in the V-blocks, tip on the pipette) is the orchestrator's responsibility — the harness does not check it.

## Minimal client

Python (any OS):

```python
import socket, json

HOST, PORT = "192.168.7.1", 7802

def harness(**req):
    """Send one request, return every reply line as a list of dicts (blocks until the server closes)."""
    s = socket.create_connection((HOST, PORT), timeout=3600)
    s.sendall((json.dumps(req) + "\n").encode())
    msgs = [json.loads(l) for l in s.makefile()]
    s.close()
    return msgs

def run_step(n):
    last = harness(cmd="run", step=n)[-1]
    if last.get("event") == "refused":
        raise RuntimeError("harness busy with step %s" % last["step"])
    return last.get("ok") is True          # finish line

def busy():
    return harness(cmd="status")[0]["busy"]

for n in (1, 3, 4, 2):                     # example sequence
    print("step", n, "ok" if run_step(n) else "FAILED")
```

Shell (Linux/macOS, netcat): `echo '{"cmd":"run","step":4}' | nc 192.168.7.1 7802` — prints the reply lines and returns when the step has finished. `echo '{"cmd":"status"}' | nc 192.168.7.1 7802` for the busy check.

## Timing and errors

- Steps are slow: a weighing step is tens of seconds to a few minutes; dilute-to-volume can be several minutes. Use a long socket timeout (or none) on `run`; `status` answers at once.
- Connection refused = the workflow server is not running on the harness PC (`harness nodes start` there) or the cable/addresses are wrong.
- Treat a `finish` with `ok: false` as "the step did not complete"; re-running the same step is usually safe for 1–3 and 10, but **not** for 4, 5 and 7 (they would dispense again).
