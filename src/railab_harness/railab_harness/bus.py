"""The transport seam: the only thing the harness and a module node know of each other's wiring.

Three channels, nothing else:
  <module>/command    harness → module   a request, answered at once
  <module>/state      module → harness   pushed
  /harness/heartbeat  harness → modules  a beat

LocalBus carries them inside one process (the walk-through, checks); ros_binding.RosBus carries
them on ROS 2. The core never imports a transport.
"""

import threading
from abc import ABC, abstractmethod
from typing import Callable


class Unreachable(Exception):
    """No reply from a module: it is not there, or it did not answer in time."""


class Bus(ABC):
    # ---- the module's side
    @abstractmethod
    def serve(self, module: str, on_command: Callable[[dict], dict], on_beat: Callable[[], None]) -> None:
        """Offer <module>/command and listen to the harness's beat."""

    @abstractmethod
    def leave(self, module: str) -> None:
        """Stop offering <module>/command."""

    @abstractmethod
    def publish_state(self, state: dict) -> None:
        """Push one state message on <state["module"]>/state."""

    # ---- the harness's side
    @abstractmethod
    def call(self, module: str, request: dict, timeout_s: float) -> dict:
        """Send a request to <module>/command; its reply, or Unreachable."""

    @abstractmethod
    def on_state(self, callback: Callable[[dict], None]) -> Callable[[], None]:
        """Hear every module's state; returns the function that stops listening."""

    @abstractmethod
    def beat(self) -> None:
        """Publish one beat on /harness/heartbeat."""


class LocalBus(Bus):
    """All three channels as direct calls inside one process."""

    def __init__(self):
        self._lock = threading.Lock()
        self._modules = {}   # name -> (on_command, on_beat)
        self._listeners = []

    def serve(self, module, on_command, on_beat):
        with self._lock:
            self._modules[module] = (on_command, on_beat)

    def leave(self, module):
        with self._lock:
            self._modules.pop(module, None)

    def publish_state(self, state):
        with self._lock:
            listeners = list(self._listeners)
        for listener in listeners:
            listener(state)

    def call(self, module, request, timeout_s):
        with self._lock:
            served = self._modules.get(module)
        if served is None:
            raise Unreachable(f"{module} does not answer")
        return served[0](request)

    def on_state(self, callback):
        with self._lock:
            self._listeners.append(callback)

        def stop():
            with self._lock:
                if callback in self._listeners:
                    self._listeners.remove(callback)
        return stop

    def beat(self):
        with self._lock:
            served = list(self._modules.values())
        for _, on_beat in served:
            on_beat()
