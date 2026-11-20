# Module interface — the generic middle layer, side by side with today's gateway

Added 2026-11-20 on branch `module-interface`; deployed beside the old path on 2026-11-20: the new gateway and
`camera_1` run on the harness PC, the Bunny's and the arm's adapters are being started. Nothing new starts by
itself: no entry in `tools/harness` NODES. The branch adds files only: today's gateway (`tools/gateway_server.py`,
TCP 7803), the nodes, `harness_msgs` and `tools/harness` are untouched.

## What and why

THE SOURCE is the contract in the workflow repo (RAILab Individual Units & Workflow Project):
`module_interface/README.md` — message shapes, the four modes, zones, the error codes, and the decisions at
its end. When this page and the contract disagree, the contract wins. The code here is a port of the same
repo's `harness_skeleton/` (its `railab_harness/` core, `examples/`, `demo.py`).

Today every module is wired into the gateway by hand (about 45 commands in one if-chain, a typed `hello.modules`,
one global job, four names for stop). In the new path every module INSTANCE (`arm_1`, `bunny_1`, `camera_1`, …)
exposes exactly two things — `/<instance>/command` (a request, answered at once: accepted or refused with a code)
and `/<instance>/state` (pushed on change and every second) — and ONE generic gateway forwards the twin's requests
verbatim after four checks (lease, module online, the module's own `describe`, zones). A new module on the bus is
visible to the twin with no gateway change.

## The pieces and their files

| Piece | File | What it is |
|---|---|---|
| the adapter base | `src/railab_harness/railab_harness/module.py` | what every module node is: modes, fault latch, one action at a time, `stop`, values with their sources, the state stream, going safe without the harness's beat (its API: below) |
| the generic door | `…/gateway.py` | lease → online → `describe` check → zones, then forward verbatim; the beat; client silence → `stop_all` (a `stop` of scope "own" to the modules running an action of this layer only) |
| the words | `…/messages.py` | codes, modes, results, sources; `check_request` (the one check, run by gateway and module alike) |
| the picture of the modules | `…/registry.py` | built only from the states the modules push |
| interlocks | `…/zones.py` | zone occupancy, no geometry: a module's `uses` + the twin's `claims` |
| control | `…/lease.py` | one controller, any number of viewers |
| command log | `…/audit.py` | every request and its reply, one JSON line each |
| periods | `…/timing.py` | the beat (1 s, decided), the timeouts (suggested) |
| the transport seam | `…/bus.py` | `Bus` + `LocalBus` (in-process) |
| the bus on ROS 2 | `…/ros_binding.py` | `RosBus`, against `harness_module_msgs` |
| **the twin's door** | `…/tcp_front.py` | JSON lines over TCP on top of `Gateway` (below) |
| the process | `tools/module_gateway.py` | ROS bus + Gateway + the TCP door on port **7804** |
| the messages | `src/harness_module_msgs/` | `srv/Command.srv`, `msg/ModuleState.msg`, `msg/Heartbeat.msg` |
| the adapter nodes | `src/harness_modules/` | `arm_module`, `bunny_module`, `camera_module` (each a `Module` subclass over today's node of that device) |
| the walk-through | `tools/module_demo.py` + `tools/module_sim/` | 15 scenes in one process, two simulated modules, no ROS, no socket listening |

`src/railab_harness/` is an ament_python package (`package.xml`, `setup.py`, `setup.cfg`, `resource/`). The core
imports no ROS at import time; only `ros_binding.py` does, inside `RosBus`. `tools/module_gateway.py` takes the core
from `src/` (as `gateway_server.py` takes `harness_calibration` / `harness_planner`), so a core change needs no
rebuild; the message package needs `colcon build`.

## The adapter base's API

An adapter is a `Module` subclass and uses ONLY these (no reach into the base's private state). Declared
on the class: `TYPE`, `VERSION`, `ACTIONS`, `FIELDS`, `ZONES`.

| Hook (the adapter writes it) | When |
|---|---|
| `do_<action>(args, run)` | the action's body, in its own thread; returning = done; `raise Failed(words)` = failed; `raise Fault(code, reason)` = failed + latched |
| `check_<action>(args)` | optional; before accepting, under the module's lock: `raise Refused(code, reason)` |
| `set_<field>(value)` | optional; before a told value is stored: `raise Refused(…)` |
| `safe_state()` | quick; touches only what THIS node started, nothing when none runs; called on any stop while an action of this node runs, and on a `stop` request of scope "device", close and a latched fault even when none runs |
| `on_stop_request()` | optional; after `safe_state()`, ONLY for a `stop` request of scope "device": what stops the device whoever moves it |
| `check_clear_fault(args)` | optional; only while a fault is latched, under the lock, just before it is cleared: `raise Refused(…)` while the cause persists |

| Call (from the adapter's own threads) | Does |
|---|---|
| `sensed(f, v)` · `told(f, v)` · `estimated(f, v)` · `unknown(f)` | one field and its source; publishes when it changed |
| `sensed_many({f: v, …}, ready=None)` | several sensed fields (a value `"unknown"` gets source unknown) and, unless None, the readiness: ONE state, always published |
| `set_ready(bool)` | the device is there and may take actions |
| `latch_fault(code, reason) -> bool` | a fault seen outside any action: latched, the running action stopped, `safe_state()`; False when one was latched already |
| `value(f)` · `need(f)` · `running()` | read a value; the value or `Refused(UNKNOWN_VALUE)`; the running action's name or None |
| `run.stopped` · `run.sleep(s)` · `run.progress(x)` · `run.ask_stop(reason)` | inside `do_*` |

WHY the stop is split: an adapter runs BESIDE today's path to the same device (the old gateway on 7803).
Going safe by itself (no harness beat, the adapter closed, the gateway's own stop of scope "own") must not touch
the old path; a `stop` the twin asks for (scope "device", the default) must stop the device whoever moves it.
`stop {reason?, scope?}`: scope "device" (default) or "own" (only an action this node runs; nothing when none runs —
a module with no action of its own also does nothing when the harness's beat stops). So: the arm's `safe_state`
cancels only its own `follow_path` goal / `grip_to` grip and never holds a released arm (someone may be guiding it
by hand); its `on_stop_request` cancels every goal on the arm node and holds a released arm. The Bunny's
`safe_state` sends the Bunny's own `stop` (bunny_node's `/bunny/stop`) only while an action of its own runs; its
`on_stop_request` stops the Bunny whoever doses. The Bunny's `abort` stands in only where it has no `stop`: for its
own `weigh_to` / `dispense_volume`, or a stop of scope "device". (SUPERSEDED 2026-11-20 by 6fa8164: the arm's
automatic safe state re-holding an arm it had released itself, and the Bunny's abort as its stop.)

## First steps on the harness PC

Written without ROS (none on the machine this was written on); see "Not tested". The branch adds files only (the
top of this page).

1. Bring the branch onto the harness PC (the lead's way of deploying) and build the three new packages ON
   THEIR OWN — nothing of today's path is rebuilt:
   ```bash
   cd ~/bunny-harness-dev
   source /opt/ros/lyrical/setup.bash
   colcon build --packages-select harness_module_msgs railab_harness harness_modules
   source rosenv.sh                                   # ROS + install/setup.bash + ROS_DOMAIN_ID 36
   ros2 interface show harness_module_msgs/srv/Command
   harness nodes status                               # today's nodes: as before
   ```
2. ONE adapter beside the old path, the camera first (read-only, nothing to move), and look at its state:
   ```bash
   ros2 run harness_modules camera_module --ros-args -p instance:=camera_1
   ros2 topic echo --once /camera_1/state             # mode ready while tag_node sends frames; values.tags
   ros2 topic hz /camera_1/state                      # ~2 Hz, whatever the frame rate
   ```
3. The Bunny's adapter, then the arm's (one at a time, each looked at before the next):
   ```bash
   ros2 run harness_modules bunny_module --ros-args -p instance:=bunny_1
   ros2 topic echo --once /bunny_1/state              # ready; housing, weight_g sensed; platform_mm unknown
   ros2 run harness_modules arm_module --ros-args -p instance:=arm_1 -p harness_root:=$HOME/bunny-harness-dev
   ros2 topic echo --once /arm_1/state                # joints_deg = the old gateway's joints_model_deg; held
   ```
   With no new gateway nobody beats: 3 s after start each adapter refuses every action `NOT_READY` ("no
   harness heartbeat") — by design. Going safe sends nothing to the device while no action of its own runs
   (the old path's dose / motion is not touched); Ctrl-C (close) is the same.
4. The new gateway on 127.0.0.1:7804 (7803 keeps running), and in a second terminal a viewer (no key:
   nothing can move):
   ```bash
   python3 tools/module_gateway.py
   python3 -c 'import json,socket; s=socket.create_connection(("127.0.0.1",7804)); f=s.makefile("rw")
   [f.write(json.dumps(m)+"\n") for m in ({"op":"hello","client":"probe"},{"op":"subscribe"})]; f.flush()
   [print(f.readline()[:160]) for _ in range(6)]'
   ```
5. The first command with the key: the operator decides when. Suggested order: nothing that moves (Bunny
   `set powder_id`, `tare`; `housing_to` the state it already reports = done, nothing sent), then one motion
   with the old path idle.

Other ways to start it: `python3 tools/module_gateway.py --bind 0.0.0.0` (the twin on the workstation),
`--sim` (no ROS, the two simulated modules); `pkill -f tools/module_gateway.py` (SIGTERM: the controller's
link ends → the actions this layer started are stopped; nothing else). The walk-through runs anywhere: `python3 tools/module_demo.py` (~4 s).

- **Port 7804**, `--bind` default **127.0.0.1**: an untested door to hardware is not open to the network by
  itself (reach it on the harness PC or through `ssh -L 7804:127.0.0.1:7804`); `--bind 0.0.0.0` as 7803 binds.
- **The key** is today's gateway key, `~/.config/railab-harness/gateway.key` (`harness gateway key` creates it),
  read at every `hello`, compared in constant time, never printed, logged or answered back. No key file = every
  connection is a viewer (as on 7803).
- **The command log** defaults to `/tmp/module_gateway_commands.jsonl`, beside the process logs `harness nodes
  start` writes to `/tmp/<name>.log`; cleared at a reboot. `--audit PATH` keeps it elsewhere, `--audit ''` = none.
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` is set before rclpy loads, for the reason `gateway_server.py` gives
  (2026-11-19, the orphaned shared-memory port).

## The twin's door: messages (`tcp_front.py`)

Newline-delimited JSON, one object per line, both ways. Every client line has `"op"`. Lines are handled in the
order they arrive (a `set` sent before a `do` reaches the module first); answers leave in that order and carry the
op they answer. One connection may hold control at a time; any number may watch.

| op | example (client ⇒) | answer (⇐) |
|---|---|---|
| `hello` | `{"op": "hello", "client": "railab-wf", "key": "…"}` | `{"op": "hello", "ok": true, "client": "railab-wf@192.168.7.1:51234#3", "access": "control", "harness": "0.1.0", "you": "view", "control": null, "modules": {"bunny_1": {"online": true, "describe": {…}}}}` — `access` = may take control (the key); `you` = holds it now. A wrong key: `access` view + `"warning": "wrong key"`. The client name is fixed by the first hello. |
| `take_control` | `{"op": "take_control"}` | `{"op": "take_control", "ok": true}` · `{"op": "take_control", "ok": false, "code": "NO_LEASE", "reason": "needs the gateway key (hello with the key)"}` |
| `release_control` | `{"op": "release_control"}` | `{"op": "release_control", "ok": true}` (stops nothing, as on 7803) |
| `ping` | `{"op": "ping"}` | `{"op": "ping", "ok": true, "t": 1763633703.12}` |
| `subscribe` | `{"op": "subscribe"}` | `{"op": "subscribe", "ok": true}`, then a state of every module known so far, then every change |
| `unsubscribe` | `{"op": "unsubscribe"}` | `{"op": "unsubscribe", "ok": true}` |
| `request` | `{"op": "request", "module": "bunny_1", "id": 21, "do": "platform_to", "args": {"z_mm": 120}, "claims": []}` — the contract's envelope + `op`; a `set`: `{"op": "request", "module": "bunny_1", "id": 22, "set": {"platform_mm": 87.3}}` | `{"op": "request", "module": "bunny_1", "id": 21, "accepted": true}` · `{"op": "request", "module": "bunny_1", "id": 23, "accepted": false, "code": "OUT_OF_RANGE", "reason": "z_mm = 300 is outside 0 … 109 mm"}` |
| (pushed) | — | `{"op": "state", "module": "bunny_1", "online": true, "state": {"time": …, "mode": "busy", "current": {"id": 21, "do": "platform_to", "progress": 0.4}, …}}` |
| (a bad line) | `this is not JSON` | `{"op": "error", "code": "BAD_ARGS", "reason": "a line must be one JSON object"}` — the link stays |

Rules of the door:

- **Any line counts as alive.** The controlling client sends something (a `ping`) at least every 3 s
  (`timing.CLIENT_SILENT_S`; today's dead-man is 10 s), else the gateway drops the lease and `stop_all` sends a
  `stop` of scope "own" to each module running an action THIS layer started (each from its own thread) — and to
  no other: with none running nothing is sent, so an idle dead-man is harmless. This supersedes the old dead-man
  for the new path (operator 2026-11-20: the newer change wins; PROJECT_SPEC.md "Dead-man" marks it); 7803 keeps
  10 s until it retires. (SUPERSEDED 2026-11-20 by 6fa8164: "stops every module".)
- **The watch follows the action.** A client that gives control up (`release_control`, a hello without the key)
  while an action of this layer runs stays watched until none runs: its silence or its link's end still stops
  them. Taking control hands the watch to the new holder.
- **The controlling client's connection ending** (closed, broken, a line > 1 MiB) = a silent client AT ONCE: the
  lease is dropped and this layer's running actions stopped; their modules' `last.reason` says `client …
  disconnected`. A viewer's ending changes nothing (`Lease.release` says whether the connection held the lease, in
  one step), nor does a released holder's once no action of this layer runs.
- **`stop` from anyone**: a request `{"do": "stop"}` (scope "device", the default) is taken from any connection,
  with or without the key; `"args": {"scope": "own"}` stops only an action that module node runs. A request
  waits for the one before it on the same connection (at most `reply_s` 1 s when a module does not answer): a client
  that wants a stop never queued may keep a second connection for it.
- **The envelope has exactly** `module, id, do | set, args, claims`; another key, or `claims` that is not a list of
  zone names, is refused `BAD_ARGS` at the door and logged with the stray keys' names only. The gateway refuses
  such `claims` `BAD_ARGS` by itself too (for any other front).
- **A gateway exception** on a request answers `accepted: false, code INTERNAL` (and goes to stderr) instead of
  cutting the link — cutting the controller's link would stop this layer's running actions.
- **Pushes never hold up a module.** A client that reads slower than states arrive gets only the newest pending
  state of each module (results stay in `last` until that module's next action ends).
- At most 32 connections; the 33rd gets `{"op": "error", "code": "BUSY", …}` and is closed.
- **Nothing holds up the beat or the bus thread**: a state listener that raises, a malformed `describe` (dropped)
  or a failing beat duty is said once on stderr and the rest goes on.

## Execute or refuse: what the old gateway changed silently

Today's gateway (`gateway_server.py`, `arm.follow_joints` / `arm.gripper`) changes some commands without a
word; the arm's adapter refuses them instead, with the joint, the value and the limit. For the new path these
refusals supersede the old gateway's rules (operator 2026-11-20: the newer change wins; PROJECT_SPEC.md marks the
older ones); 7803 keeps them until it retires:

| Sent | Old gateway (7803) | Arm adapter |
|---|---|---|
| a path starting away from the arm | bridged up to 0.35 rad (20°): the first waypoint replaced by the arm's posture | `OUT_OF_RANGE` beyond `START_TOL_DEG` 1°; within it the path starts exactly where the arm is |
| a waypoint past a joint limit | clamped inside, up to 10° | `OUT_OF_RANGE` outside the controller's range less the arm node's 0.01 rad margin |
| the arm held past a limit | a move back inside prepended, up to 20° | nothing prepended (the path starts where the arm is; every later waypoint must be inside) |
| `speed` / `speeds` | clamped to 0.25 … 2 | `OUT_OF_RANGE`; `speeds` = one per SEGMENT (K − 1 for K waypoints) |
| `corner_deg` | clamped 5 … 45 | not offered: every path is the smooth path |
| a gripper gap above the widest | the arm node clamps it | `OUT_OF_RANGE` (the widest = its gripper calibration now) |
| force 0 | read as the default | the range starts at 0.001 N; leave `force_n` out for the default |

The simulated arm (`tools/module_sim/arm_sim.py`) takes `follow_path` the same way (start gap, speeds per
segment and their range); it has no joint limits and no timed trajectory.

## The Bunny, as found

Through `bunny_node`, the path this branch's adapter uses. The Bunny's own server has had `platform_mm` and `stop`
since 2026-11-20 (below); `bunny_node` passes its `stop` (`/bunny/stop`, 6fa8164), not `platform_mm`.

- **A jog on the Bunny's own screen cannot be seen**: its buttons go straight to the STM32 and `bunny_node`'s
  status carries no position. The adapter notices only a change of the server's step counter (another client, a
  server restart).
- **A platform move or a housing swing stops only with the Bunny's own `stop`** (`/bunny/stop`; a server older
  than 82cc822 answers "unknown cmd stop", and its abort covers dosing and the pump only): then `stop` waits for
  them to end, and the zone stays held until then. The abort stands in only for this node's own `weigh_to` /
  `dispense_volume` or a stop of scope "device" (it latches the Bunny's dose flag; `stop` does not).
  (SUPERSEDED 2026-11-20 by 6fa8164: the abort as the adapter's stop.)
- **The platform height stays unknown after a failed, stopped or late move**, or one during which the Bunny's link
  was lost. No answer within the budget is not a failure: NO_REPLY is latched at once, but the action and its zone
  `platform_travel` stay held until the Bunny answers or its status shows the move over (`platform_moving`, a
  server since 82cc822; a swing: its `busy` word).
- **An EMPTY `WeighTo.powder_id` does not make today's Bunny measure the powder fresh**: it keeps the powder it
  last selected, although `WeighTo.srv`'s comment says otherwise (this affects the old path too). So
  `weigh_to` is refused `UNKNOWN_VALUE` when `powder_id` is unknown while the Bunny reports a current powder.
- **`powder_id` must be digits**: bunny_node treats any other text as a powder NAME.
- **An outage cannot be told from a restart**: it makes every value unknown, the told ones included.
- **`pan_loaded` is kept in the adapter**: the Bunny's `load_begin` / `load_end` handshake is a "do not dose
  now" window, not what stands on the pan.

## What the Bunny's own server needs for the adapter to become thin

- `platform_to {mm}` and `platform_mm` in its status — the steps per mm and the travel living in the Bunny.
- A jog on its screen counted, or reported as unknown.
- A platform stop (the firmware has `stpstop`) and a housing stop.
- "Set current powder" (refuses an unknown id, can clear it) and `pan_loaded` as its own fields.
- A boot id in the status (an outage told from a restart).
- One kept connection that pushes status, instead of a 2 Hz poll and a connection per call.

**The Bunny's side exists since 2026-11-20** (Super Bunny repo, branch dev, commits 82cc822 + b27b5e9; pushed, NOT
deployed to the Pi, not tried on hardware). Its server on 7801 now also takes `platform_set {mm}`,
`platform_to {mm}`, `stop`, `set_powder {powder_id}`, `set_pan_loaded {loaded}` and `subscribe {period_s}`, and its
status carries `boot_id`, `started`, `version`, `platform_mm`, `platform_source` (unknown / told / estimated),
`platform_why`, `platform_moving`, `platform_steps_per_mm`, `platform_travel_mm`, `pan_loaded`. Every old verb is
unchanged, so the old path works as before.

- The height is unknown at every start, after a jog on the screen, a stopped move, a commanded move during which the
  serial link dropped or reconnected (a reconnect with no move running keeps the height), or an estimate that leaves
  the travel. A jog is reported as unknown, not counted (counting needs firmware).
- `stop` ends a dose, the pump, a platform move, and a housing swing in place with the motor still holding.
- The steps per mm (50) and the travel (0 … 109 mm) in the Bunny's `config.py` are INTERIM: the lab has not
  confirmed them.

**Still to do here:** `bunny_module.py` reaches the Bunny through the existing `bunny_node`, which knows only `stop`
of the new verbs (`/bunny/stop`, 6fa8164). Using them means a thin adapter that talks to 7801 itself (kept connection + `subscribe`), or new
services on `bunny_node`. Until then the notes under "The Bunny, as found" stay true for this branch.

## ROS names that differ from the JSON

| JSON (the contract, the TCP door) | ROS (`harness_module_msgs`) | why |
|---|---|---|
| request `do` | `Command.srv` `action` | `do` is a C++ keyword |
| state `module` | `ModuleState.msg` `instance` | `module` is an OMG IDL keyword |

Example: `ros2 service call /bunny_1/command harness_module_msgs/srv/Command "{id: 1, set: '{\"powder_id\": \"9618329212\"}'}"`
(`args` and `set` are JSON text; `""` = empty).

## Not ported yet

- **The twin's own link** (the workflow app's `harness_link.py`) still speaks 7803; nothing on the twin side uses 7804.
- **dtv and pipette** adapters; the **camera**'s ChArUco stream and the cross-module routines today's gateway
  runs in its own process (`teach.fine`, `teach.plan_views` on a client's command; the start-posture precheck,
  computed, no motion); the tag measurement is open ("to be confirmed" in the contract). As the code stands nothing
  on the harness, old path or new, moves the arm by itself to look at tag 3: the twin drives the arm there through
  7803, measures the tag and derives the platform height; `bunny_1`'s `platform_mm` is only told (PROJECT_SPEC.md
  "2026-11-20 — Who looks at the platform's tag 3").
- **The calibrations.** `calibrate_handeye` / `calibrate_absolute` are declared on the arm and refused
  `NOT_READY`. Hand-eye today = the old gateway's `handeye_job` (a subprocess, a target choice tag | board, a
  precheck, installed at rms ≤ 3 mm), and the old gateway keeps the result IN MEMORY — installing it from the
  adapter would leave the old gateway on the old calibration. Absolute = a session of up to five hand-placed
  board positions, then an install and an arm-node restart — not one action. A port needs: `handeye_job` and the
  precheck moved into the arm node, a target argument on the action, the old gateway re-reading `handeye.yaml`
  when it changes.
- **Today's gateway-only commands** with no module behind them: `arm.check_j` (dry run), `arm.jog_pose` /
  `arm.jog_gripper` (live sliders), `arm.gripper_cal`, `nodes.status` / `nodes.restart`, `state` at a chosen rate
  (`subscribe {rate_hz}`; the new door pushes on change and at each module's 1 s beat).
- **Several controlling clients**, a takeover of a stale holder (7803's `HOLDER_STALE_S`): out of scope — a silent
  holder loses the lease after 3 s anyway (and this layer's running actions are stopped).
- **The timeouts** are the contract's suggestions (3 s / 4 s / 3 s), not decided.

## Open

- **An action the DEVICE started on its own** (a housing swing from the Bunny's screen, or one sent through the
  old gateway) is not shown as `current`, so it holds no zone: the gateway may grant an arm path through a
  housing that is swinging. The zone logic is ready for it (a module's own `current` occupies its zones). Since
  2026-11-19 the Bunny's status reports `housing: moving` for any swing by its own process, the GUI's included
  (passed through as the `housing` value), but it has no `busy` flag for it and the adapter does not yet turn it
  into a held zone; the arm adapter reads the arm node's `busy` only to refuse.
- **A lossy transport** (a dropped state) can leave a zone held — the safe side, but it needs a way out
  (from the contract).

## Not tested

Checked by `py_compile` (and a Python 3.10 grammar parse on 3.13) and by direct calls without ROS:
`tools/module_demo.py` (15 / 15 scenes), `tools/module_check_arm.py` (15 / 15 checks: the adapter on a fake arm
node, the real planner on a synthetic arm model), `tools/module_check_bunny.py` (35 / 35 checks: fake bunny_node,
fake tag_node), the door's edge paths on socketpairs. Since 2026-11-20 the new gateway and `camera_1` run on the
harness PC (the Bunny's and the arm's adapters are being started); no result from there is recorded here yet.
NOT tested before that:

- **anything ROS**: `ros_binding.py` (`RosBus`), `RosArmLink`, `RosBunnyLink`, `RosCameraLink` and the three
  `main()`s, their parameters and executors, SIGINT / SIGTERM with rclpy's own handling off; the
  `harness_module_msgs` build; `colcon build` of `railab_harness` / `harness_modules` (`package.xml`, `setup.py`,
  the console scripts never built); `module_gateway.py` on the ROS bus; `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. The
  JSON ↔ message mapping only with stand-in objects.
- **a real listening socket**: `TcpFront.listen` / `serve_forever`, the 32-connection limit, IPv6 `--bind`, a peer
  on another machine, a half-open link (no FIN: only the 3 s silence catches it).
- **the real devices**: the PiPER through the arm node (goals, cancel, hold, the drag-teach release, the gripper,
  the real `calib/kinematics.yaml`), the Bunny through bunny_node and its server (the step counter's meaning was
  read from the code), tag_node's frames at their real rate.
- **Python 3.10 and 3.14** (run on 3.13 only); the key file and the `/tmp` command log on the harness PC.

## Old piece → new piece

For whoever retires the old gateway.

| today (7803) | new path |
|---|---|
| `tools/gateway_server.py`: ~45 commands in one if-chain | `gateway.py`: no command names; a command exists because a module's `describe` lists it |
| `{"id", "cmd", "args"}` → `{"id", "ok", "result" \| "error"}` | `{"op": "request", "module", "id", "do" \| "set", "args", "claims"}` → `{"op": "request", "module", "id", "accepted", "code"?, "reason"?}`; progress and result in the pushed state (`current`, `last`) |
| `hello` {key, client} → `access`, typed `modules` | `hello` {key, client} → `access`, `modules` from each module's own `describe` |
| `KEY_FILE` `~/.config/railab-harness/gateway.key`, `read_key()` | the same file, the same reading (`tcp_front.read_key`) |
| `take_control` / `release_control` (`ctl["conn"]`) | the same ops; `Lease` |
| `subscribe {rate_hz}` → `{"event": "state", …}` at a rate | `subscribe` → `{"op": "state", "module", "online", "state"}` per module, on change |
| `stop` (cancel arm goals, `/bunny/abort`) | `{"do": "stop", "args": {"scope": "device" (default) \| "own"}}` to a module, never refused; `Gateway.stop_all` = scope "own" to the modules running an action of this layer only |
| the dead-man: the controller silent `DEADMAN_S` 10 s while a motion / job runs → cancel arm goals only | the controller silent 3 s, or its link closed → lease dropped, `stop_all` (nothing when no action of this layer runs); the watch follows a running action after `release_control`; a module goes safe by itself 3 s after the harness's beat stops only while an action of its own runs |
| one global `JOB` | one action per module (`current`), no global slot |
| `HOLDER_STALE_S` takeover | none: a silent holder loses the lease (and this layer's running actions are stopped) |
| `hello.modules`, a dict typed in the gateway | each module's `describe`, carried in its state |
| `/bunny/platform` {steps} | `bunny_1`'s `platform_to {z_mm}` (the Bunny's own server: `{mm}`) + the writable field `platform_mm` (`"unknown"` until told) |
| the gateway's time planning / clamping / start-gap recovery for `arm.follow_joints` | inside the arm node behind `follow_path`; out of range is refused, never clamped |
| `node_watchdog.py`'s per-name health; camera health from a log file | `Registry.online` from the age of a module's state (restarting processes stays the watchdog's job) |
| no command log (stdout has only calibration lines) | `audit.py`: every request, its reply, forwarded or not, one JSON line |
| `tools/workflow_server.py` (7802), `workflows/*.sh` | to retire: one door |
