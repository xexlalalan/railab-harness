"""The harness's picture of the modules, built ONLY from the state messages they push.

There is no list of modules anywhere in the harness: a module exists once its first state has
arrived, it is what its own `describe` says, and it is online while its states keep coming.
A restarted harness therefore knows everything again within one idle beat.
"""

import threading
import time

from .timing import DEFAULT, Timing


class Registry:
    def __init__(self, timing: Timing = DEFAULT):
        self.timing = timing
        self._lock = threading.Lock()
        self._states = {}      # module -> its newest state
        self._heard_at = {}    # module -> when that state arrived (the HARNESS's clock, not the module's)
        self._describes = {}
        self._said_offline = set()

    def update(self, state: dict) -> None:
        module = state["module"]
        with self._lock:
            self._states[module] = state
            self._heard_at[module] = time.monotonic()
            if state.get("describe"):
                self._describes[module] = state["describe"]
            self._said_offline.discard(module)

    def modules(self) -> list:
        with self._lock:
            return sorted(self._states)

    def state(self, module: str):
        with self._lock:
            return self._states.get(module)

    def describe(self, module: str):
        with self._lock:
            return self._describes.get(module)

    def online(self, module: str) -> bool:
        with self._lock:
            return self._online(module)

    def _online(self, module: str) -> bool:
        heard = self._heard_at.get(module)
        return heard is not None and time.monotonic() - heard <= self.timing.offline_s

    def zones(self) -> set:
        """Every zone any module declared, by its full name <module>/<zone>."""
        with self._lock:
            return {f"{module}/{zone}" for module, d in self._describes.items() for zone in d.get("zones", [])}

    def went_offline(self) -> list:
        """The modules that fell silent since the last call (each named once)."""
        with self._lock:
            gone = [m for m in self._states if m not in self._said_offline and not self._online(m)]
            self._said_offline.update(gone)
            return gone

    def snapshot(self) -> dict:
        """{module: {"online", "age_s", "state"}} — a state is passed on as the module sent it."""
        with self._lock:
            now = time.monotonic()
            return {m: {"online": self._online(m), "age_s": round(now - self._heard_at[m], 3), "state": s}
                    for m, s in self._states.items()}
