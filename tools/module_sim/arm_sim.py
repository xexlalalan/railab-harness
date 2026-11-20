"""An arm module node on a SIMULATED device.

The arm owns no zone of its own here: where its path goes is geometry, and geometry is the twin's.
The twin names the zones a path passes in the request's "claims"; the harness holds them for as
long as `follow_path` is this module's current action.

In a real arm node the timed trajectory (interpolation, speed and acceleration limits, the
controller's joint offsets) lives HERE, behind follow_path — not in the gateway.

follow_path means what the real adapter (src/harness_modules/harness_modules/arm_module.py) means:
the first waypoint is where the arm is (more than START_TOL_DEG away on any joint = OUT_OF_RANGE,
never bridged), `speeds` has one factor per SEGMENT (len(waypoints_deg) − 1, each within
SPEED_MIN … SPEED_MAX, else OUT_OF_RANGE). No joint limits, no timed trajectory: each segment is a
straight line in joint space, SEGMENT_S long at factor 1.
"""

from railab_harness.messages import BAD_ARGS, NOT_READY, OUT_OF_RANGE, Refused
from railab_harness.module import Module
from railab_harness.timing import DEFAULT

SEGMENT_S = 0.6                  # one waypoint to the next at speed 1
START_TOL_DEG = 1.0              # = arm_module.START_TOL_DEG
SPEED_MIN, SPEED_MAX = 0.25, 2.0  # = arm_module.SPEED_MIN / SPEED_MAX


def _number(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


class SimArm:
    def __init__(self, speed: float = 1.0):
        self.speed = speed
        self.joints_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gap_mm = 80.0
        self.halts = 0


class Arm(Module):
    TYPE, VERSION = "piper", "sim-0.1"
    ACTIONS = {
        "follow_path": {"args": {"waypoints_deg": {"unit": "deg: >= 2 postures of 6 joints, the first = where the "
                                                           f"arm is (within {START_TOL_DEG} deg per joint)"},
                                 "speeds": {"unit": "× the teach speed per segment (len(waypoints_deg) − 1 numbers, "
                                                    f"{SPEED_MIN} … {SPEED_MAX}); missing = 1 for every segment",
                                            "optional": True}},
                        "uses": []},
        "grip_to": {"args": {"gap_mm": {"unit": "mm", "range": [0, 80]},
                             "force_n": {"unit": "N", "range": [0, 5], "optional": True}}, "uses": []},
        "hold": {"args": {}, "uses": []},
        "release": {"args": {}, "uses": []},              # drag teach: the user moves the arm by hand
        # The two calibrations are the ARM's own routines (operator 2026-11-20). They need the gripper
        # camera; the arm node reads it INSIDE the middle layer — the twin sees one action and its result.
        "calibrate_handeye": {"args": {}, "uses": []},
        "calibrate_absolute": {"args": {}, "uses": []},
    }
    FIELDS = {
        "joints_deg": {"unit": "deg", "writable": False},
        "gap_mm": {"unit": "mm", "range": [0, 80], "writable": False},
        "held": {"values": [True, False], "writable": False},   # False while released for drag teach
        "handeye_error_mm": {"unit": "mm", "writable": False},  # "unknown" until calibrated
        "absolute_error_mm": {"unit": "mm", "writable": False},
    }

    def __init__(self, name, bus, timing=DEFAULT, device=None, serial="ARM-SIM-1"):
        super().__init__(name, bus, timing, serial)
        self.device = device or SimArm()

    def start(self):
        super().start()
        self.sensed("joints_deg", list(self.device.joints_deg))
        self.sensed("gap_mm", self.device.gap_mm)
        self.sensed("held", True)
        self.set_ready(True)
        return self

    def safe_state(self):
        self.device.halts += 1   # hold position (never go limp)

    def check_follow_path(self, args):
        """What a describe cannot say, the module checks itself — before it accepts."""
        if self.value("held") is not True:
            raise Refused(NOT_READY, "the arm is released for drag teach: send hold first")
        waypoints, speeds = args["waypoints_deg"], args.get("speeds")
        if not isinstance(waypoints, list) or len(waypoints) < 2 or any(
                not isinstance(w, list) or len(w) != 6 or not all(_number(q) for q in w) for w in waypoints):
            raise Refused(BAD_ARGS, "waypoints_deg is at least 2 postures of 6 joint angles (the first = where the "
                                    "arm is)")
        if speeds is not None:
            if not isinstance(speeds, list) or len(speeds) != len(waypoints) - 1 or not all(map(_number, speeds)):
                raise Refused(BAD_ARGS, f"speeds needs {len(waypoints) - 1} numbers, one per segment")
            for k, speed in enumerate(speeds, 1):
                if not SPEED_MIN <= speed <= SPEED_MAX:
                    raise Refused(OUT_OF_RANGE, f"speeds: segment {k} = {speed} is outside {SPEED_MIN} … {SPEED_MAX}")
        gaps = [abs(a - b) for a, b in zip(waypoints[0], self.device.joints_deg)]
        j = max(range(6), key=gaps.__getitem__)
        if gaps[j] > START_TOL_DEG:
            raise Refused(OUT_OF_RANGE, f"waypoint 1, joint {j + 1} is {gaps[j]:.2f} deg from where the arm is, more "
                                        f"than {START_TOL_DEG} deg: plan the path from the arm's current posture")

    def do_follow_path(self, args, run):
        waypoints = args["waypoints_deg"][1:]   # the first is where the arm is: the path starts exactly there
        speeds = args.get("speeds") or [1.0] * len(waypoints)
        steps = 5
        for i, (target, speed) in enumerate(zip(waypoints, speeds)):
            start = list(self.device.joints_deg)
            for step in range(1, steps + 1):
                if not run.sleep(SEGMENT_S / speed / self.device.speed / steps):
                    return   # stays where it is, holding
                self.device.joints_deg = [a + (b - a) * step / steps for a, b in zip(start, target)]
                self.sensed("joints_deg", [round(q, 2) for q in self.device.joints_deg])
                run.progress((i + step / steps) / len(waypoints))

    def do_grip_to(self, args, run):
        if run.sleep(0.3 / self.device.speed):
            self.device.gap_mm = float(args["gap_mm"])
            self.sensed("gap_mm", self.device.gap_mm)

    def do_hold(self, args, run):
        self.sensed("held", True)    # a real node re-arms its position hold here

    def do_release(self, args, run):
        self.sensed("held", False)   # a real node switches to drag teach here

    def do_calibrate_handeye(self, args, run):
        """A real node: look at the board from several postures, solve, install. Result → the state."""
        if run.sleep(0.4 / self.device.speed):
            self.sensed("handeye_error_mm", 1.8)

    def do_calibrate_absolute(self, args, run):
        if run.sleep(0.4 / self.device.speed):
            self.sensed("absolute_error_mm", 0.9)
