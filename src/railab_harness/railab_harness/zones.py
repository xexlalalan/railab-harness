"""Interlocks between modules, as ZONE OCCUPANCY — the harness needs no geometry.

A zone is a name for a space, owned by the module whose motion sweeps it: "bunny_1/housing_swing".
  - a module's action says which of its zones it sweeps (describe: "uses");
  - the twin, which has the geometry, says which zones an arm path passes (envelope: "claims").
A zone has one holder at a time. It is held while the holder's action is `current` and freed when
that module's own state shows the action over — never by a timer. A holder that stops talking
keeps its zones: nobody knows where it stopped.
"""

import threading
from typing import Callable


class Zones:
    def __init__(self, online: Callable[[str], bool]):
        self._online = online
        self._lock = threading.Lock()
        self._held = {}   # zone -> {(module, request id): seen running?}

    def acquire(self, holder: tuple, zones: list):
        """All of the zones for (module, request id), or none: returns why not, else None."""
        with self._lock:
            for zone in zones:
                owner = zone.split("/", 1)[0]
                if owner != holder[0] and not self._online(owner):
                    return f"{zone}: {owner} is offline, what it was doing there is unknown"
                others = [h for h in self._held.get(zone, {}) if h != holder]
                if others:
                    return f"{zone} is held by {others[0][0]} (request {others[0][1]})"
            for zone in zones:
                self._held.setdefault(zone, {})[holder] = False
        return None

    def release(self, holder: tuple) -> None:
        """A grant whose request was then not taken by the module."""
        with self._lock:
            self._drop(lambda h, seen: h == holder)

    def on_state(self, state: dict) -> None:
        """A module's state is the only thing that frees its grants."""
        module = state["module"]
        current = state.get("current") or {}
        last_id = (state.get("last") or {}).get("id")
        with self._lock:
            for holders in self._held.values():
                for holder in holders:
                    if holder == (module, current.get("id")):
                        holders[holder] = True
            # over = it was seen running and no longer is, or it is already the module's last result
            self._drop(lambda h, seen: h[0] == module and h[1] != current.get("id") and (seen or h[1] == last_id))
            # a module that moves on its own account (its own screen, a start before the harness
            # came up) occupies its zones all the same
            if current:
                action = (state.get("describe") or {}).get("actions", {}).get(current.get("do"), {})
                for zone in action.get("uses", []):
                    self._held.setdefault(f"{module}/{zone}", {})[(module, current.get("id"))] = True

    def held(self) -> dict:
        with self._lock:
            return {zone: sorted(holders, key=str) for zone, holders in self._held.items()}

    def _drop(self, over) -> None:
        for zone in list(self._held):
            holders = self._held[zone]
            for holder in [h for h, seen in holders.items() if over(h, seen)]:
                del holders[holder]
            if not holders:
                del self._held[zone]
