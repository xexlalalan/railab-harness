"""THE BUNNY'S ADAPTER NODE: the weighing Bunny on the module interface, side by side with today's path.

Upward: the contract (docs/module_interface.md; the workflow repo's module_interface/README.md) through the
`Module` base — `/<instance>/command`, `/<instance>/state`, the harness's beat. Downward: a LINK that is a ROS
CLIENT of today's bridge node `harness_instrument/bunny_node.py` (its /bunny/* services and its /bunny/status
JSON topic). The bridge and the Bunny's own server (Weighing Super Bunny code/python/harness_server.py, on the
Pi) keep every check they have; nothing of theirs changes, and the old gateway on TCP 7803 keeps using them.

    ros2 run harness_modules bunny_module --ros-args -p instance:=bunny_1 [-p device_ns:=/bunny] [-p serial:=SB-003]

THE VALUES and where each comes from
  housing      sensed     the Bunny's own words from its BLDC, passed through (HOUSING_WORDS)
  weight_g     sensed     the status's scale reading; "unknown" when the status is stale or the Bunny offline
  platform_mm  told       the twin measures tag 3 with the arm's camera and tells it (`set`)
               estimated  after a platform_to of ours: the told height + our own steps / PLATFORM_STEPS_PER_MM
  pan_loaded   told       kept HERE (see set_pan_loaded)
  powder_id    told       kept HERE and passed down with every weigh_to (see set_powder_id)
               sensed     when the Bunny's own barcode reader scans a bottle (the status's last_scan changes)

THE PLATFORM (the reason this interface exists). The Bunny has no limit switch and takes RELATIVE steps, so it
cannot measure its height; the adapter keeps the height tied to the Bunny server's STEP COUNTER (`platform
{steps: 0}` reads it; it counts only moves made through that server, from 0 at its start):
  - it starts "unknown"; `platform_to` while unknown → UNKNOWN_VALUE; a target outside the travel → OUT_OF_RANGE
    (the describe's range, before anything moves);
  - `set platform_mm` = told, tied to the counter as last read (refused while the counter is not read);
  - `platform_to` sends the step difference; the reply's counter must be the old one + our steps → estimated;
    any other counter, a failed / stopped call, a call answered after its budget, or the Bunny's link lost while it
    ran (offline, `stm32: false`) → "unknown", and it stays unknown (the move's answer never overwrites it);
  - no answer within the budget is NOT a failure: nobody knows whether the platform still moves. A fault NO_REPLY is
    latched at once, but the action — and with it the zone platform_travel — goes on until the Bunny answers, or its
    own status shows the move over (`platform_moving` false: only its server since 82cc822 says so); then it fails
    with the height unknown. The same holds for a housing swing (its status's `busy` no longer the swing's verb);
  - the counter read every COUNTER_POLL_S differs from ours with no move of ours (another client of the
    Bunny's server moved it, or the server restarted and counts from 0 again) → "unknown";
  - the Bunny offline (bunny_node says so, or its status stops), its link to the STM32 firmware down → "unknown",
    and it stays unknown when the Bunny comes back. An outage cannot be told from a restart and every power-on
    starts at "unknown", so an outage makes EVERY value unknown — the told pan_loaded and powder_id too.
  NOT SEEN: a jog on the Bunny's OWN SCREEN (its PLATE LEVEL buttons send `stpup` / `stpstop` straight to the
  STM32 and never touch the server's counter) — today nothing the bridge can read shows it. See THIN below.
  A Bunny server older than 82cc822 cannot interrupt a platform move nor a housing swing (its `abort` stops dosing
  and the pump only): a `stop` during one waits for the Bunny's answer, so the zone is held until the motion really
  ended. Its `stop` (82cc822) ends them in place: the move's call then answers an error, the height is unknown.

STOP: the Bunny's own `stop` verb (its server since 82cc822: a dose, the pump, a platform move, a housing swing; it
never latches the dose flag — `abort` does, and a `tare` after it silently does nothing until the next weigh_to
clears it), through bunny_node's /bunny/stop. Where it is missing (bunny_node or the server too old) the Bunny's
`abort` + liquid stop stands in — ONLY where nothing else can stop what must stop: a dose of ours (weigh_to,
dispense_volume), or a `stop` request of scope "device". Going safe BY ITSELF (no harness beat, close, a latched
fault, the gateway's own stop of scope "own") touches only an action of THIS node and sends nothing at all when none
runs — started beside the old gateway, this node never stops nor aborts the old path's dose by itself. A `stop`
request of scope "device" (the twin's) stops the Bunny whoever doses.

WHAT WOULD MAKE THIS ADAPTER THIN (for whoever changes the Bunny next): `platform_to {z_mm}` / a `platform_mm`
in `status`, the steps per mm and the travel in the Bunny; a jog on its own screen counted, or reported
("platform_mm": "unknown"); a platform stop (the firmware has `stpstop`); `powder_id` and `pan_loaded` as its
own fields (a "set current powder" verb that refuses an id not in its database; the load handshake setting
pan_loaded); a stop for a housing swing; one kept connection that pushes `status` instead of a poll per call.
"""

import json
import threading
import time

from railab_harness import messages as M
from railab_harness.messages import Refused
from railab_harness.module import Failed, Fault, Module
from railab_harness.timing import DEFAULT

# ------------------------------------------------------------------------- THE BUNNY'S OWN KNOWLEDGE, KEPT HERE
# These numbers are the Bunny's, not the harness's. Until the Bunny's server takes millimetres they sit in this
# ONE block; they go away when it does.
#  - steps per mm: the workflow repo's app/railab_station/twin/realflow.py BUNNY_PLATFORM_STEPS_PER_MM = 50.0
#    (operator 2026-11-18: lead-8 screw; the firmware's STPV1 = TMC2208 at 1/2 step, 400 pulses per turn,
#    one `stepper` count = one pulse → 400 / 8 = 50). + steps = up.
#  - travel: 0 … 109 mm as the contract has it (the twin's platform joint spans −53.4 … +55.6 = 109 mm) —
#    TO BE CONFIRMED BY THE LAB (the rail's lower end in the CAD is unconfirmed).
PLATFORM_STEPS_PER_MM = 50.0
PLATFORM_TRAVEL_MM = (0.0, 109.0)

# the Bunny's housing words (its status, 2026-11-19): what its BLDC reports. Any other word reads "unknown".
HOUSING_WORDS = ["closed", "open", "half", "between", "moving", "no_feedback", "untaught"]
HOUSING_VERBS = {"open": "housing_open", "half": "housing_half", "closed": "housing_close"}

# ------------------------------------------------------------------------- the adapter's own periods and budgets
STATUS_STALE_S = 3.0       # bunny_node publishes /bunny/status at 2 Hz: none for this long = the bridge is gone
HOUSING_FRESH_S = 1.5      # "the status already reports the target" only from a status this fresh
COUNTER_POLL_S = 1.0       # the platform counter is read this often (moves that are not ours)
WATCH_S = 0.25             # the watch thread's tick
AWAIT_SLICE_S = 0.05
STOP_GRACE_S = 10.0        # after a stop (the Bunny's stop / abort sent), a dose has this long to answer
READ_S = 6.0               # a counter read (bunny_node gives the Bunny 5 s)
HOUSING_S = 90.0           # the old gateway's budget (bunny_node gives the Bunny 60 s)
TARE_S = 70.0              # bunny_node: 60 s
WEIGH_S = 1830.0           # bunny_node: 1800 s
DISPENSE_S = 630.0         # bunny_node: 600 s
ABORTABLE = ("weigh_to", "dispense_volume")   # what an older Bunny's abort stops (dosing, the pump): the only actions
                                              # an abort stands in for when the Bunny has no `stop`
HOUSING_BUSY = ("housing_open", "housing_close", "housing_half")   # the Bunny's status `busy` during a swing

NO_REPLY = "NO_REPLY"              # fault: a call that never came back — what the Bunny did is not known
HOUSING_FAILED = "HOUSING_FAILED"  # fault: a housing swing the Bunny reports failed, or not where it should be


def platform_budget_s(steps: int) -> float:
    """The old gateway's budget for a platform move (the Bunny waits |steps| x 5 ms, stretched by housing reads)."""
    return 10.0 + abs(steps) * 0.005


# ------------------------------------------------------------------------- the link's words
class LinkError(Exception):
    """A call to the Bunny's bridge that gave no result."""


class NoService(LinkError):
    """Nothing was sent: bunny_node does not offer the service."""


class BunnyError(LinkError):
    """The bridge answered success = false. The words are the Bunny's (or bunny_node's own socket error)."""


class Unanswered(LinkError):
    """No answer within the budget: what the Bunny did is not known."""


class Pending:
    """One call in flight. A link returns it AT ONCE; the adapter waits on it in its worker thread."""

    def __init__(self):
        self._guard = threading.Lock()
        self._done = threading.Event()
        self._value = self._error = None

    @classmethod
    def now(cls, value=None, error=None) -> "Pending":
        pending = cls()
        pending.finish(value, error)
        return pending

    def finish(self, value=None, error=None) -> None:
        with self._guard:
            if self._done.is_set():
                return
            self._value, self._error = value, error
            self._done.set()

    def wait(self, seconds: float) -> bool:
        return self._done.wait(seconds)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value


class BunnyLink:
    """What the adapter needs from today's bunny_node. Every call returns a Pending at once (the adapter decides
    how long to wait); stop() returns at once too. A fake replaces this in tools/module_check_bunny.py."""

    def on_status(self, callback) -> None:
        """callback(dict) on every /bunny/status message (the Bunny's status + bunny_node's "online")."""
        raise NotImplementedError

    def housing(self, verb: str) -> Pending:
        """housing_open | housing_half | housing_close → the Bunny's housing fields after the move (dict)."""
        raise NotImplementedError

    def platform(self, steps: int) -> Pending:
        """Move the platform by signed steps (0 = read only) → the Bunny's counter after it (int)."""
        raise NotImplementedError

    def tare(self) -> Pending:
        """→ {"g": weight after} (dict)."""
        raise NotImplementedError

    def weigh_to(self, target_g: float, powder_id: str) -> Pending:
        """→ {"success", "outcome", "actual_g", "message"}: answered whatever the outcome ("error" included)."""
        raise NotImplementedError

    def dispense_volume(self, volume_ml: float) -> Pending:
        """→ {"success", "steps", "steps_planned", "actual_ml", "message"}."""
        raise NotImplementedError

    def stop(self, abort_if_missing: bool) -> None:
        """The Bunny's own `stop` (its server since 82cc822: ends a dose, the pump, a platform move, a housing swing;
        never latches the dose flag) — sent AT ONCE from a thread of the link's own, so it gets through while another
        call waits. Where bunny_node or the Bunny has no `stop`: abort_if_missing → the Bunny's abort, then its liquid
        stop (the only stop an older Bunny has; it latches the dose flag), else nothing."""
        raise NotImplementedError


# ------------------------------------------------------------------------- the module
class BunnyModule(Module):
    TYPE, VERSION = "weighing_bunny", "adapter-0.1.0"
    ZONES = ["housing_swing", "platform_travel"]
    ACTIONS = {
        "housing_to": {"args": {"target": {"values": ["open", "half", "closed"]}}, "uses": ["housing_swing"]},
        "platform_to": {"args": {"z_mm": {"unit": "mm", "range": list(PLATFORM_TRAVEL_MM)}},
                        "uses": ["platform_travel"]},
        "tare": {"args": {}, "uses": []},
        # WHICH powder is not an argument: it is the told fact powder_id (operator 2026-11-20)
        "weigh_to": {"args": {"target_g": {"unit": "g", "range": [0, 100]}}, "uses": []},
        "dispense_volume": {"args": {"volume_ml": {"unit": "mL", "range": [0, 100]}}, "uses": []},
    }
    FIELDS = {
        "housing": {"values": list(HOUSING_WORDS), "writable": False},
        "weight_g": {"unit": "g", "writable": False},
        "platform_mm": {"unit": "mm", "range": list(PLATFORM_TRAVEL_MM), "writable": True},
        "pan_loaded": {"values": [True, False], "writable": True},
        "powder_id": {"unit": "barcode", "writable": True},
    }

    def __init__(self, name, bus, link: BunnyLink, timing=DEFAULT, serial="", log=None,
                 stale_s=STATUS_STALE_S, poll_s=COUNTER_POLL_S, watch_s=WATCH_S):
        """watch_s None = no watch thread (a check calls watch_once() itself)."""
        super().__init__(name, bus, timing, serial)
        self.link = link
        self.log = log or (lambda text: None)
        self.stale_s, self.poll_s, self.watch_s = stale_s, poll_s, watch_s
        # LOCK ORDER: the base's lock may be held when _mine is taken (the set hooks), never the other way;
        # and nothing that publishes (sensed / unknown / set_ready) runs while _mine is held.
        self._mine = threading.Lock()
        self._started_at = time.monotonic()
        self._status = None          # the newest /bunny/status
        self._status_at = None       # when it arrived (this process's clock)
        self._online = False         # bunny_node says the Bunny answers, and its status is fresh
        self._firmware = False       # the Bunny's link to its STM32 is up
        self._said_ready = False
        self._down_reason = "no /bunny/status yet"
        self._counter = None         # the Bunny server's platform counter as last confirmed by us
        self._lost = 0               # bumped by every _lose(): a move that saw it change ends with the height unknown
        self._moving = False         # a platform_to of ours is in flight
        self._gen = 0                # bumped when a move starts / ends: a read straddling it is not trusted
        self._polled_at = 0.0
        self._scan_seen = None       # the last_scan already seen (a scan is news only when it changes)
        self._scan_primed = False
        self._dose = None            # (run, target_g) of a running weigh_to, for its progress
        self._watch_stop = threading.Event()
        self._watcher = None

    # ------------------------------------------------------------------ life
    def start(self):
        super().start()
        self.link.on_status(self._on_status)   # only now: a state may not leave before the bus serves us
        if self.watch_s:
            self._watcher = threading.Thread(target=self._watch, name=f"{self.name}-watch", daemon=True)
            self._watcher.start()
        return self

    def close(self):
        self._watch_stop.set()
        if self._watcher is not None and self._watcher.is_alive():
            self._watcher.join(2)
        super().close()

    def safe_state(self):
        """The Bunny's stop, at once (never waits: stop must always get through) — ONLY while an action of THIS node
        runs, nothing at all otherwise: the automatic safe state (no harness beat, close, a fault, the gateway's own
        stop) must not stop a dose that another client of the Bunny's server (the old gateway) started. Without the
        Bunny's `stop`, its abort stands in for a dose of ours only (an abort cannot stop a swing, a move or a tare,
        and it would leave the dose flag set)."""
        action = self.running()
        if action is not None:
            self.link.stop(abort_if_missing=action in ABORTABLE)

    def on_stop_request(self):
        """A `stop` request of scope "device" stops the Bunny whoever doses (as the old gateway's stop does): its
        stop, else its abort. A dose of this node's has had that from safe_state already."""
        if self.running() not in ABORTABLE:
            self.link.stop(abort_if_missing=True)

    # ------------------------------------------------------------------ what the Bunny says by itself
    def _on_status(self, status: dict) -> None:
        """Every /bunny/status (2 Hz), from the link's thread."""
        if not isinstance(status, dict):
            return
        online = bool(status.get("online"))
        firmware = online and status.get("stm32") is not False
        with self._mine:
            self._status, self._status_at = status, time.monotonic()
            self._online, self._firmware = online, firmware
            dose = self._dose
            scan_news = None
            if online:
                scan = status.get("last_scan") or None
                code = str(scan.get("code") or "") if isinstance(scan, dict) else ""
                if not self._scan_primed:
                    self._scan_seen, self._scan_primed = code, True   # history from before: not news
                elif code and code != self._scan_seen:
                    self._scan_seen, scan_news = code, code
        if not online:
            self._lose(f"the Bunny is offline ({status.get('error') or 'bunny_node gives no reason'})", restart=True)
            return
        if not firmware:
            self._lose("the Bunny's link to its STM32 firmware is down", restart=False)
        word = status.get("housing")
        if firmware and word in HOUSING_WORDS:
            self.sensed("housing", word)
        else:
            self.unknown("housing")
        weight = status.get("weight_g")
        if isinstance(weight, (int, float)) and not isinstance(weight, bool):
            self.sensed("weight_g", round(float(weight), 4))
            if dose is not None and dose[1] > 0:
                dose[0].progress(float(weight) / dose[1])
        else:
            self.unknown("weight_g")
        if scan_news is not None:
            self.log(f"{self.name}: the Bunny's reader scanned {scan_news}: powder_id is that bottle now")
            self.sensed("powder_id", scan_news)
        if firmware:
            self._say_ready(True, None)

    def _lose(self, reason: str, restart: bool) -> None:
        """The Bunny went away (restart=True: an outage cannot be told from a restart, and every power-on starts
        at "unknown" — every value it holds is lost) or its firmware link dropped (the platform and the housing)."""
        with self._mine:
            self._counter = None
            self._lost += 1          # before the values below go unknown: a move ending meanwhile sees it
            if restart:
                self._online = self._firmware = False
                self._scan_primed = False
        self._say_ready(False, reason)
        lost = ["housing", "platform_mm"] + (["weight_g", "pan_loaded", "powder_id"] if restart else [])
        for field in lost:
            self.unknown(field)

    def _say_ready(self, ready: bool, reason) -> None:
        with self._mine:
            changed = ready != self._said_ready or (not ready and reason != self._down_reason)
            self._said_ready = ready
            if not ready:
                self._down_reason = reason
        if changed:
            self.log(f"{self.name}: " + ("ready" if ready else f"not ready: {reason}"))
            self.set_ready(ready)

    # ------------------------------------------------------------------ the watch: staleness, the counter
    def _watch(self) -> None:
        while not self._watch_stop.wait(self.watch_s):
            try:
                self.watch_once()
            except Exception as error:   # the watch must outlive one bad reading
                self.log(f"{self.name}: watch: {type(error).__name__}: {error}")

    def watch_once(self) -> None:
        now = time.monotonic()
        with self._mine:
            heard = self._status_at if self._status_at is not None else self._started_at
            stale = now - heard > self.stale_s and (self._online or self._status_at is None)
            due = (self._online and self._firmware and not self._moving
                   and now - self._polled_at >= self.poll_s)
            gen = self._gen
        if stale:
            self._lose(f"no /bunny/status for {self.stale_s:.0f} s (bunny_node not running?)", restart=True)
            return
        if due:
            self._read_counter(gen)

    def _read_counter(self, gen: int) -> None:
        pending = self.link.platform(0)
        answered = pending.wait(READ_S)
        with self._mine:
            self._polled_at = time.monotonic()
        try:
            if not answered:
                raise Unanswered(f"no answer within {READ_S:.0f} s")
            position = int(pending.result())
        except LinkError as error:   # nothing learned (not a sign the platform moved): the next read tries again
            self.log(f"{self.name}: reading the platform counter: {error}")
            return
        with self._mine:
            if gen != self._gen or self._moving or not self._online:
                return   # a move of ours began or ended meanwhile: this reading may be from either side of it
            before, self._counter = self._counter, position
        if before is not None and position != before:
            if self.value("platform_mm") != M.UNKNOWN:
                self.log(f"{self.name}: the Bunny's platform counter went {before} → {position} with no move "
                         "of ours (another client, or its server restarted): the height is unknown")
            self.unknown("platform_mm")

    # ------------------------------------------------------------------ actions
    def do_housing_to(self, args, run):
        target = args["target"]
        if self._status_fresh(HOUSING_FRESH_S) and self.value("housing") == target:
            return   # a target, not a motion: already there = done, nothing moves
        try:
            fields = self._call(self.link.housing(HOUSING_VERBS[target]), run, HOUSING_S, "housing_to",
                                interruptible=False, settled=self._housing_over)
        except BunnyError as words:
            self.unknown("housing")   # the status says where it really is within half a second
            if run.stopped:           # the Bunny's own stop ended the swing in place: asked for, not a fault
                self.log(f"{self.name}: the Bunny stopped the swing where it was ({words})")
                run.note("swing stopped part way")
                return
            raise Fault(HOUSING_FAILED, str(words))
        word = fields.get("housing") if isinstance(fields, dict) else None
        if word in HOUSING_WORDS:
            self.sensed("housing", word)
        if run.stopped:
            run.note(f"swing ended {word}")
        elif word != target:
            raise Fault(HOUSING_FAILED, f"swing done but the housing reads {word!r}")

    def check_platform_to(self, args):
        self.need("platform_mm")   # open loop and no limit switch: never move from an unknown height
        with self._mine:
            known = self._counter is not None
        if not known:
            raise Refused(M.UNKNOWN_VALUE, "the Bunny's platform counter is not read yet")

    def do_platform_to(self, args, run):
        start = self.value("platform_mm")
        if start == M.UNKNOWN:
            raise Failed("the platform height became unknown before the move started")
        steps = int(round((float(args["z_mm"]) - start) * PLATFORM_STEPS_PER_MM))
        if steps == 0:
            return   # already there, within half a step
        with self._mine:
            counter, lost = self._counter, self._lost
            self._moving, self._gen = True, self._gen + 1
        position, error, answered, good = None, None, False, False
        try:
            if counter is None:
                raise Failed("the Bunny's platform counter was lost before the move started")
            position = int(self._call(self.link.platform(steps), run, platform_budget_s(steps), "platform_to",
                                      interruptible=False, settled=self._platform_over))
            answered = True
        except (Failed, Fault, LinkError) as caught:
            error = caught
        finally:
            with self._mine:   # in ONE step with the end of the move: a read right after it compares with this
                self._moving, self._gen = False, self._gen + 1
                dropped = self._lost != lost   # offline / the STM32 link down while it ran: the steps may be lost
                # the counter as the Bunny answered it, only for a move it answered with nothing lost meanwhile —
                # never over the None a _lose() set (else the next read anchors afresh)
                self._counter = position if answered and not dropped else None
                good = answered and not dropped and position == counter + steps
            if not good:
                self.unknown("platform_mm")   # failed, stopped part way, answered late, link lost: UNKNOWN
        if error is not None:
            if isinstance(error, BunnyError):
                if run.stopped:   # the Bunny's own stop (its server since 82cc822) ended the move part way
                    self.log(f"{self.name}: the platform stopped part way ({error})")
                    run.note("platform stopped part way: height unknown")
                    return
                raise Failed(M.join("platform move failed, height unknown", error))
            raise error
        if dropped:
            raise Failed("Bunny offline or STM32 link dropped: height unknown")
        if not good:   # someone else moved it too
            raise Failed(f"platform counter {position}, expected {counter + steps}: height unknown")
        self.estimated("platform_mm", round(start + steps / PLATFORM_STEPS_PER_MM, 2))
        with self._mine:
            lost_since = self._lost != lost
        if lost_since:   # lost between the move's end and the line above: _lose's "unknown" must win
            self.unknown("platform_mm")
        if run.stopped:
            run.note("platform move ran to its end")

    def do_tare(self, args, run):
        try:
            reply = self._call(self.link.tare(), run, TARE_S, "tare", interruptible=False)
        except BunnyError as words:
            raise Failed(M.join("tare", words))
        grams = reply.get("g") if isinstance(reply, dict) else None
        if isinstance(grams, (int, float)) and not isinstance(grams, bool):
            self.sensed("weight_g", round(float(grams), 4))

    def check_weigh_to(self, args):
        if args["target_g"] <= 0:
            raise Refused(M.OUT_OF_RANGE, "target_g must be more than 0 g")
        if self.need("pan_loaded") is not True:
            raise Refused(M.NOT_READY, "nothing on the pan: set pan_loaded first")
        if self.value("powder_id") == M.UNKNOWN:
            with self._mine:
                held = (self._status or {}).get("powder_id")
            if held:
                # unknown should mean "the Bunny measures the powder fresh"; today's Bunny would instead dose with
                # the powder it last selected (an empty WeighTo.powder_id does not clear it)
                raise Refused(M.UNKNOWN_VALUE, f"powder_id unknown, the Bunny holds {held}: set powder_id")

    def do_weigh_to(self, args, run):
        target = float(args["target_g"])
        powder = self.value("powder_id")
        powder = "" if powder == M.UNKNOWN else str(powder)   # "" = the Bunny measures the powder fresh
        with self._mine:
            self._dose = (run, target)
        try:
            reply = self._call(self.link.weigh_to(target, powder), run, WEIGH_S, "weigh_to", interruptible=True)
        except BunnyError as words:
            raise Failed(M.join("weigh_to", words))
        finally:
            with self._mine:
                self._dose = None
        outcome = str(reply.get("outcome") or "")
        actual = reply.get("actual_g")
        weighed = (outcome in ("finished", "stopped", "bottle empty")   # "error": bunny_node's 0.0 is no reading
                   and isinstance(actual, (int, float)) and not isinstance(actual, bool))
        if weighed:
            self.sensed("weight_g", round(float(actual), 4))
        said = f"the Bunny: {outcome or 'no outcome'}" + (f", {actual:g} g" if weighed else "")
        message = _plain(reply.get("message"))
        if run.stopped:
            run.note(said)
        elif outcome == "finished":
            return
        elif outcome == "stopped":
            run.ask_stop(f"stopped on the Bunny itself ({said})")   # ends "stopped": nobody here asked for it
        else:
            raise Failed(M.join(said, message) if message else said)   # "bottle empty", "error", anything else

    def check_dispense_volume(self, args):
        if args["volume_ml"] <= 0:
            raise Refused(M.OUT_OF_RANGE, "volume_ml must be more than 0 mL")

    def do_dispense_volume(self, args, run):
        try:
            reply = self._call(self.link.dispense_volume(float(args["volume_ml"])), run, DISPENSE_S,
                               "dispense_volume", interruptible=True)
        except BunnyError as words:
            raise Failed(M.join("dispense_volume", words))
        said = (f"pushed {reply.get('steps', 0)} of {reply.get('steps_planned', 0)} pump steps "
                f"≈ {reply.get('actual_ml', 0)} mL")
        if run.stopped:
            run.note(said)
        elif not reply.get("success"):
            message = _plain(reply.get("message"))
            raise Failed(M.join("dispense_volume", message or said))

    # ------------------------------------------------------------------ told facts
    def set_platform_mm(self, value):
        if self.running() == "platform_to":
            raise Refused(M.BUSY, "the platform is moving: tell its height when it stands")
        with self._mine:
            up, counter = self._online and self._firmware, self._counter
        if not up:
            raise Refused(M.NOT_READY, "Bunny offline or firmware link down: tell the height later")
        if counter is None:   # read every second
            raise Refused(M.NOT_READY, "platform counter not read yet: tell it again")

    def set_pan_loaded(self, value):
        """TODAY'S BUNNY HAS NO pan_loaded: the fact is kept HERE. Its load_begin / load_end handshake is reachable
        (bunny_node's /bunny/load_begin, /bunny/load_end, the status's load_open) but it is a LOADING WINDOW
        ("do not dose now"), not what stands on the pan; the Bunny keeps enforcing it itself (a weigh_to during an
        open load fails with its words). Goes away when the Bunny has the field itself."""
        if self.running() == "weigh_to":
            raise Refused(M.BUSY, "the pan cannot change under a running dose")

    def set_powder_id(self, value):
        """TODAY'S BUNNY HAS NO "set current powder" verb: the told id is kept HERE and passed down with every
        weigh_to (WeighTo.powder_id); the Bunny looks it up then (an id not in its database fails the dose with
        its words). Goes away when the Bunny has the field itself."""
        if not isinstance(value, str) or not value.isdigit():
            raise Refused(M.BAD_ARGS, "powder_id must be the barcode id, digits only")   # other text = a NAME
        if self.running() == "weigh_to":
            raise Refused(M.BUSY, "the powder cannot change under a running dose")

    # ------------------------------------------------------------------ inside
    def _status_fresh(self, seconds: float) -> bool:
        with self._mine:
            return self._online and self._status_at is not None and time.monotonic() - self._status_at <= seconds

    def _call(self, pending: Pending, run, budget_s: float, what: str, interruptible: bool, settled=None):
        """Wait for one call: its result, or the adapter's Failed / Fault; a BunnyError is left to the action.

        interruptible=False (a housing swing, a platform move, a tare: an older Bunny cannot stop them) = a stop does
        not end the wait — the zone stays held until the Bunny says the motion is over.
        settled (a motion that holds a zone): NO ANSWER WITHIN THE BUDGET IS NOT A FAILURE — nobody knows whether the
        Bunny still moves. A fault NO_REPLY is latched at once (the state says so; the safe state sends the Bunny's
        stop where it has one), but the wait — the action, its zone — goes on until the Bunny answers, or settled(t)
        (its own status, received after t) shows the motion over; either way the action then fails."""
        deadline, grace, late = time.monotonic() + budget_s, None, False
        while not pending.wait(AWAIT_SLICE_S):
            now = time.monotonic()
            if interruptible and run.stopped and grace is None:
                grace = now + STOP_GRACE_S
            if settled is not None:
                if now > deadline and not late:
                    late = True
                    self.log(f"{self.name}: {what}: no answer within {budget_s:.0f} s; the action (and its zone) "
                             "waits for its answer or for its status to show the motion over")
                    self.latch_fault(NO_REPLY, f"{what}: no Bunny answer in {budget_s:.0f} s, outcome unknown")
                if late and settled(deadline):
                    raise Failed(f"{what}: no answer in {budget_s:.0f} s, motion over, outcome unknown")
                continue
            if now > deadline or (grace is not None and now > grace):
                self.safe_state()   # nobody knows what the Bunny does now: make it safe
                within = f"{STOP_GRACE_S:.0f} s of the stop" if grace is not None and now > grace else f"{budget_s:.0f} s"
                raise Fault(NO_REPLY, f"{what}: no answer in {within}, stop sent")
        if late:   # whatever it says now (done, an error): the motion's end is not known for certain
            try:
                pending.result()
                said = "it says done"
            except LinkError as words:
                said = f"it says: {words}"
            self.log(f"{self.name}: {what}: the Bunny answered only after the {budget_s:.0f} s budget ({said})")
            raise Failed(f"{what}: answered only after {budget_s:.0f} s, outcome unknown")
        try:
            return pending.result()
        except NoService as absent:
            raise Failed(f"{what}: {absent}")
        except Unanswered as lost:
            self.safe_state()
            raise Fault(NO_REPLY, f"{what}: {lost} (a stop was sent)")

    def _status_after(self, since: float):
        """The newest /bunny/status if the Bunny was online in it and it arrived after `since`, else None."""
        with self._mine:
            status, at, online = self._status, self._status_at, self._online
        return status if online and at is not None and at > since and isinstance(status, dict) else None

    def _platform_over(self, since: float) -> bool:
        """The Bunny's own word that no platform move runs (only its server since 82cc822 says it)."""
        status = self._status_after(since)
        return status is not None and status.get("platform_moving") is False

    def _housing_over(self, since: float) -> bool:
        """The Bunny's own word that no housing swing runs (its `busy` is the swing's verb while one runs)."""
        status = self._status_after(since)
        return status is not None and "busy" in status and status.get("busy") not in HOUSING_BUSY


def _plain(message) -> str:
    """bunny_node's message: an error's words, or the Bunny's reply as JSON (not words: dropped)."""
    text = str(message or "").strip()
    if text.startswith("{"):
        try:
            json.loads(text)
            return ""
        except ValueError:
            pass
    return text


# ------------------------------------------------------------------------- the ROS link (untested: no ROS here)
SERVICE_WAIT_S = 2.0   # a service bunny_node should offer: absent this long = NoService
ABORT_S = 5.0          # bunny_node gives the Bunny 5 s for abort / liquid_stop


class RosBunnyLink(BunnyLink):
    """A ROS client of today's bunny_node: /bunny/<verb> services and the /bunny/status topic (JSON in a String)."""

    def __init__(self, node, ns: str = "/bunny", log=None):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from harness_msgs.srv import DispenseVolume, LiquidPush, WeighTo

        self.node, self.ns = node, "/" + ns.strip("/")
        self.log = log or (lambda text: None)
        self.Trigger, self.LiquidPush, self.WeighTo, self.DispenseVolume = Trigger, LiquidPush, WeighTo, DispenseVolume
        self._group = ReentrantCallbackGroup()
        self._clients = {}
        self._clients_lock = threading.Lock()
        self._stops = []         # stop threads still on their way (close() waits for them)
        self._listener = None
        self._kept = [node.create_subscription(String, f"{self.ns}/status", self._heard, 10,
                                               callback_group=self._group)]
        self._client(Trigger, "stop")   # made now: by the first stop, discovery says whether bunny_node offers it

    def on_status(self, callback):
        self._listener = callback

    def _heard(self, message):
        listener = self._listener
        if listener is None:
            return
        try:
            listener(json.loads(message.data))
        except Exception as error:
            self.log(f"{self.ns}/status: {type(error).__name__}: {error}")

    def _client(self, srv, name):
        with self._clients_lock:
            client = self._clients.get(name)
            if client is None:
                client = self._clients[name] = self.node.create_client(srv, f"{self.ns}/{name}",
                                                                       callback_group=self._group)
            return client

    def _start(self, srv, name, request, convert) -> Pending:
        pending = Pending()
        client = self._client(srv, name)
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            pending.finish(error=NoService(f"{self.ns}/{name} is not offered (bunny_node not running?)"))
            return pending

        def answered(future):
            try:
                pending.finish(value=convert(future.result()))
            except LinkError as error:
                pending.finish(error=error)
            except Exception as error:
                pending.finish(error=BunnyError(f"{name}: {type(error).__name__}: {error}"))

        client.call_async(request).add_done_callback(answered)
        return pending

    def _trigger(self, name) -> Pending:
        def convert(reply):
            if not reply.success:
                raise BunnyError(reply.message or f"{name} failed")
            try:
                out = json.loads(reply.message) if reply.message else {}
            except ValueError:
                out = {}
            return out if isinstance(out, dict) else {}
        return self._start(self.Trigger, name, self.Trigger.Request(), convert)

    def housing(self, verb):
        return self._trigger(verb)

    def platform(self, steps):
        def convert(reply):
            if not reply.success:
                raise BunnyError(reply.message or "platform failed")
            return int(reply.steps_done)   # bunny_node: the Bunny's accumulated position, not the steps of this call
        return self._start(self.LiquidPush, "platform", self.LiquidPush.Request(steps=int(steps), rate=0, wait=False),
                           convert)

    def tare(self):
        return self._trigger("tare")

    def weigh_to(self, target_g, powder_id):
        request = self.WeighTo.Request(target_g=float(target_g), material="powder", powder_id=str(powder_id or ""))
        return self._start(self.WeighTo, "weigh_to", request,
                           lambda r: {"success": bool(r.success), "outcome": r.outcome,
                                      "actual_g": float(r.actual_g), "message": r.message})

    def dispense_volume(self, volume_ml):
        request = self.DispenseVolume.Request(volume_ml=float(volume_ml), density_g_per_ml=0.0, rate=0)
        return self._start(self.DispenseVolume, "dispense_volume", request,
                           lambda r: {"success": bool(r.success), "steps": int(r.steps),
                                      "steps_planned": int(r.steps_planned), "actual_ml": float(r.actual_ml),
                                      "message": r.message})

    def stop(self, abort_if_missing):
        thread = threading.Thread(target=self._stop_now, args=(bool(abort_if_missing),), name="bunny-stop",
                                  daemon=True)
        with self._clients_lock:
            self._stops = [t for t in self._stops if t.is_alive()] + [thread]
        thread.start()

    def close(self, timeout_s: float = 3 * ABORT_S + SERVICE_WAIT_S):
        """Wait for a stop still on its way (the module's close sends one while an action of its runs) before ROS
        goes down."""
        with self._clients_lock:
            pending = list(self._stops)
        for thread in pending:
            thread.join(timeout_s)

    def _stop_now(self, abort_if_missing):
        """The Bunny's `stop` through /bunny/stop. Missing — bunny_node does not offer it, the Bunny's server does not
        know the verb (older than 82cc822: "unknown cmd stop"), or no answer —: the abort when it must stand in."""
        missing = None
        if not self._client(self.Trigger, "stop").service_is_ready():
            missing = f"{self.ns}/stop is not offered (a bunny_node without the Bunny's stop)"
        else:
            pending = self._trigger("stop")
            if not pending.wait(ABORT_S):
                missing = f"{self.ns}/stop: no answer within {ABORT_S:.0f} s"
            else:
                try:
                    pending.result()
                    return
                except LinkError as error:
                    missing = f"{self.ns}/stop: {error}"
        if abort_if_missing:
            self.log(f"{missing}: the Bunny's abort instead (it latches the dose flag until the next weigh_to)")
            self._abort_now()
        else:
            self.log(f"{missing}: nothing sent (an abort cannot stop this, and would latch the dose flag)")

    def _abort_now(self):
        for name in ("abort", "liquid_stop"):
            pending = self._trigger(name)
            if not pending.wait(ABORT_S):
                self.log(f"{self.ns}/{name}: no answer within {ABORT_S:.0f} s")
                continue
            try:
                pending.result()
            except LinkError as error:
                self.log(f"{self.ns}/{name}: {error}")


PARAMETERS = {"instance": "bunny_1", "device_ns": "/bunny", "serial": ""}   # device_ns: today's bunny_node's names


def main(args=None):
    """One node `<instance>_module` for the link and the bus (the arm adapter's way). rclpy's own signal handling is
    off, so on Ctrl-C / SIGTERM the module's close still reaches the Bunny (its stop, if an action of ours runs) while
    ROS runs."""
    import signal

    from railab_harness.ros_binding import RosBus, ros_environment
    ros_environment()   # Fast DDS over UDP only (the old gateway's 7e2e7d5 lesson), before rclpy loads
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    probe = rclpy.create_node("bunny_module_parameters")
    params = {name: probe.declare_parameter(name, default).value for name, default in PARAMETERS.items()}
    probe.destroy_node()
    instance = str(params["instance"])
    node = rclpy.create_node(f"{instance}_module")
    for name, value in params.items():
        node.declare_parameter(name, value)
    log = node.get_logger().info
    link = RosBunnyLink(node, str(params["device_ns"]), log=log)
    bus = RosBus(node=node)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    threading.Thread(target=executor.spin, name="bunny-module-spin", daemon=True).start()
    module = BunnyModule(instance, bus, link, serial=str(params["serial"]), log=log).start()
    log(f"{instance}: the Bunny's adapter on /{instance}/command and /{instance}/state, "
        f"a client of {params['device_ns']}/*")
    leave = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: leave.set())
    while not leave.wait(0.5):
        pass
    try:
        module.close()   # stops a running action of ours (the Bunny's stop) — while ROS still runs; nothing if none
        link.close()     # wait for that stop to go out
    finally:
        bus.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
