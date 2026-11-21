"""The words of the contract, and the one check of a request against a module's own description.

Messages are plain dicts (they are JSON on every wire); this file is the only place their words
are spelled.

  request   {"id", "do": action, "args": {...}}   or   {"id", "set": {field: value}}
  reply     {"id", "accepted": True}   or   {"id", "accepted": False, "code", "reason"}
  state     {"time", "module", "describe", "mode", "fault", "current", "last", "values", "sources"}
  describe  {"identity": {"type", "serial", "version"},
             "actions": {name: {"args": {arg: SPEC}, "uses": [zone]}},
             "fields":  {name: SPEC + {"writable": bool}},
             "zones":   [zone]}
  SPEC      {"unit", "range": [lo, hi]}  a number   |  {"values": [...]}  one of these
            |  {"unit"}  anything (text, a list: only the module can judge it)
            + "optional": True on an argument that may be left out
"""

# why a request is refused
UNKNOWN_ACTION = "UNKNOWN_ACTION"
BAD_ARGS = "BAD_ARGS"
OUT_OF_RANGE = "OUT_OF_RANGE"
BUSY = "BUSY"
FAULT = "FAULT"
NOT_READY = "NOT_READY"
UNKNOWN_VALUE = "UNKNOWN_VALUE"  # a quantity the action needs is unknown (e.g. the platform height)
READ_ONLY = "READ_ONLY"
UNKNOWN_FIELD = "UNKNOWN_FIELD"
INTERLOCK = "INTERLOCK"
NO_LEASE = "NO_LEASE"
OFFLINE = "OFFLINE"

# a module's mode
MODE_NOT_READY = "not_ready"  # no action but stop
MODE_READY = "ready"
MODE_BUSY = "busy"            # only stop and set
MODE_FAULT = "fault"          # only stop and clear_fault; latched

# how an action ended
DONE, STOPPED, FAILED = "done", "stopped", "failed"

# where a value comes from; an unknown value reads "unknown" too
SENSED, TOLD, ESTIMATED, UNKNOWN = "sensed", "told", "estimated", "unknown"

# the two actions every module has
STOP, CLEAR_FAULT = "stop", "clear_fault"
# what a `stop` reaches (its optional arg "scope"):
#   "device" (the default) = the device, whoever moves it: the twin's stop (the old path's motion too);
#   "own"                  = only an action THIS module node runs, nothing at all when it runs none: the harness going
#                            safe by itself (its client silent or gone) — a module runs BESIDE the old path to the same
#                            device, which an automatic stop must never touch (a cancelled goal, a re-held arm, an abort)
STOP_DEVICE, STOP_OWN = "device", "own"
COMMON_ACTIONS = {
    STOP: {"args": {"reason": {"unit": "text", "optional": True},
                    "scope": {"values": [STOP_DEVICE, STOP_OWN], "optional": True}}, "uses": []},
    CLEAR_FAULT: {"args": {}, "uses": []},
}


class Refused(Exception):
    """A request that is not taken, with its code and its reason in words."""

    def __init__(self, code: str, reason: str):
        super().__init__(reason)
        self.code, self.reason = code, reason


# THE WORDS A PERSON READS (operator 2026-11-21: "所有报错信息不能超过10个单词。捡重要的信息说"): a reason says at most
# MAX_WORDS whitespace words; a head and a tail that do not fit together = the tail (the specific words) alone.
MAX_WORDS = 10


def fit(long: str, short: str) -> str:
    """`long` when it says at most MAX_WORDS words, else `short`."""
    return long if len(long.split()) <= MAX_WORDS else short


def join(head: str, tail, sep: str = ": ") -> str:
    """'head: tail' when it says at most MAX_WORDS words, else the tail alone."""
    return fit(f"{head}{sep}{tail}", str(tail))


def accept(rid) -> dict:
    return {"id": rid, "accepted": True}


def refuse(rid, code: str, reason: str) -> dict:
    return {"id": rid, "accepted": False, "code": code, "reason": reason}


def check_value(name: str, value, spec: dict) -> None:
    """One value against its SPEC; raises Refused."""
    if "range" in spec:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise Refused(BAD_ARGS, f"{name} must be a number")
        lo, hi = spec["range"]
        if not lo <= value <= hi:
            raise Refused(OUT_OF_RANGE, f"{name} = {value} is outside {lo} … {hi} {spec.get('unit', '')}".rstrip())
    elif "values" in spec:
        if not any(value == v and type(value) is type(v) for v in spec["values"]):
            raise Refused(BAD_ARGS, f"{name} must be one of {spec['values']}")


def check_request(request: dict, describe: dict) -> None:
    """A request against the module's own description; raises Refused.

    The gateway and the module run the SAME check: the gateway so a wrong request never travels,
    the module because it is the authority on itself and may be reached by another door.
    """
    rid = request.get("id")
    if isinstance(rid, bool) or not isinstance(rid, int):
        raise Refused(BAD_ARGS, "id must be an integer")
    if ("do" in request) == ("set" in request):
        raise Refused(BAD_ARGS, "a request is either a do or a set")
    if "do" in request:
        name = request["do"]
        action = describe["actions"].get(name) if isinstance(name, str) else None
        if action is None:
            raise Refused(UNKNOWN_ACTION, f"no action {name!r}")
        args = request.get("args") or {}
        if not isinstance(args, dict):
            raise Refused(BAD_ARGS, "args must be an object")
        specs = action.get("args", {})
        for arg in args:
            if arg not in specs:
                raise Refused(BAD_ARGS, f"{name} takes no argument {arg!r}")
        for arg, spec in specs.items():
            if arg in args:
                check_value(arg, args[arg], spec)
            elif not spec.get("optional"):
                raise Refused(BAD_ARGS, f"{name} needs {arg}")
    else:
        fields = request["set"]
        if not isinstance(fields, dict) or not fields:
            raise Refused(BAD_ARGS, "set must name at least one field")
        for field, value in fields.items():
            spec = describe["fields"].get(field)
            if spec is None:
                raise Refused(UNKNOWN_FIELD, f"no field {field!r}")
            if not spec.get("writable"):
                raise Refused(READ_ONLY, f"{field} is the module's own to report")
            check_value(field, value, spec)
