"""A Bunny module node on a SIMULATED device — the whole adapter pattern in one file.

Read it top to bottom: the declarations ARE the module's interface (they travel in every state as
`describe`); below them there is one method per action and nothing about the harness.

It shows the three kinds of value a module has:
  sensed     the device measures it                      housing, weight_g
  told       only the outside can know it                platform_mm (no limit switch yet: the twin
                                                         measures tag 3 with the arm's camera and
                                                         tells), pan_loaded, powder_id
  estimated  the module's own dead reckoning             platform_mm after its own platform_to
and that the module knows WHEN it stops knowing: a start, a hand on the jog buttons, an interrupted
open-loop move → "unknown", and the actions that need the value are refused until told again.
"""

from railab_harness.messages import BUSY, NOT_READY, Refused
from railab_harness.module import Fault, Module
from railab_harness.timing import DEFAULT

HOUSING_S = 2.0          # one housing swing
PLATFORM_MM_S = 20.0
DOSE_G_S = 0.5
TARE_S = 0.5


class SimBunny:
    """The "device" (what the Pi + STM32 are): it knows motors, not the contract."""

    def __init__(self, speed: float = 1.0):
        self.speed = speed        # > 1 = faster than life, for the walk-through
        self.z_mm = 37.0          # where the platform REALLY is — the device cannot read this back
        self.housing = "closed"   # read from the housing motor
        self.weight_g = 0.0
        self.jam = False          # make the next housing move stall (to show a fault)
        self.halts = 0            # how often the motors were cut


class Bunny(Module):
    TYPE, VERSION = "weighing_bunny", "sim-0.1"
    ZONES = ["housing_swing", "platform_travel"]
    ACTIONS = {
        "housing_to": {"args": {"target": {"values": ["open", "half", "closed"]}}, "uses": ["housing_swing"]},
        "platform_to": {"args": {"z_mm": {"unit": "mm", "range": [0, 109]}}, "uses": ["platform_travel"]},
        "tare": {"args": {}, "uses": []},
        # WHICH powder is not an argument: it is the fact `powder_id` below (operator 2026-11-20).
        "weigh_to": {"args": {"target_g": {"unit": "g", "range": [0, 100]}}, "uses": []},
    }
    FIELDS = {
        "housing": {"values": ["closed", "open", "half", "between", "moving"], "writable": False},
        "platform_mm": {"unit": "mm", "range": [0, 109], "writable": True},
        "weight_g": {"unit": "g", "writable": False},
        "pan_loaded": {"values": [True, False], "writable": True},
        # The powder's BARCODE id, never its name: two bottles of "NaCl" are two powders (their own
        # dosing profiles). Told by the twin, or sensed when the Bunny's own reader scans the bottle.
        # "unknown" = no dose: the real Bunny keeps the powder it last selected when it is given no id
        # (it does NOT measure fresh), so the real adapter refuses weigh_to UNKNOWN_VALUE then.
        "powder_id": {"unit": "barcode", "writable": True},
    }

    def __init__(self, name, bus, timing=DEFAULT, device=None, serial="SB-SIM-1"):
        super().__init__(name, bus, timing, serial)
        self.device = device or SimBunny()

    def start(self):
        super().start()
        self.sensed("housing", self.device.housing)   # asked from the motor, never remembered
        self.sensed("weight_g", self.device.weight_g)
        # platform_mm, pan_loaded, powder_id stay "unknown": nothing in the device can sense them
        self.set_ready(True)
        return self

    def safe_state(self):
        self.device.halts += 1   # cut the motors; the running action sees run.stopped and returns

    def manual_jog(self, delta_mm: float):
        """What the Bunny's own screen calls when someone jogs the platform by hand."""
        self.device.z_mm += delta_mm
        self.unknown("platform_mm")   # the told height is no longer true, and the device cannot re-read it

    # ---------------------------------------------------------------- actions
    def do_housing_to(self, args, run):
        target = args["target"]
        if self.value("housing") == target:
            return   # a target, not a motion: already there = done
        self.sensed("housing", "moving")
        steps = 8
        for step in range(1, steps + 1):
            if not run.sleep(HOUSING_S / self.device.speed / steps):
                self.sensed("housing", "between")
                return
            if self.device.jam and step == steps // 2:
                self.device.jam = False
                self.sensed("housing", "between")
                raise Fault("HOUSING_STALL", "the housing motor stalled half way")
            run.progress(step / steps)
        self.device.housing = target
        self.sensed("housing", target)

    def check_platform_to(self, args):
        self.need("platform_mm")   # open loop and no end switch: never move from an unknown height

    def do_platform_to(self, args, run):
        start, target = self.need("platform_mm"), args["z_mm"]
        steps = max(1, int(abs(target - start) // 5))
        for step in range(1, steps + 1):
            if not run.sleep(abs(target - start) / PLATFORM_MM_S / self.device.speed / steps):
                self.unknown("platform_mm")   # an interrupted open-loop move: the step count is not trusted
                return
            self.device.z_mm += (target - start) / steps
            self.estimated("platform_mm", round(start + (target - start) * step / steps, 1))
            run.progress(step / steps)

    def do_tare(self, args, run):
        if run.sleep(TARE_S / self.device.speed):
            self.device.weight_g = 0.0
            self.sensed("weight_g", 0.0)

    def check_weigh_to(self, args):
        if self.need("pan_loaded") is not True:
            raise Refused(NOT_READY, "nothing on the pan (set pan_loaded when a container stands there)")

    def do_weigh_to(self, args, run):
        target, dt = args["target_g"], 0.1 / self.device.speed
        while self.device.weight_g < target:
            if not run.sleep(dt):
                return   # the dose so far stays on the pan and in weight_g
            self.device.weight_g = min(target, self.device.weight_g + DOSE_G_S * 0.1)
            self.sensed("weight_g", round(self.device.weight_g, 3))
            run.progress(self.device.weight_g / target)

    # ---------------------------------------------------------------- told facts
    def set_pan_loaded(self, value):
        if self.running() == "weigh_to":
            raise Refused(BUSY, "the pan cannot change under a running dose")
