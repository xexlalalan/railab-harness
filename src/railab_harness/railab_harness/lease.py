"""The control lease: one client may command, any number may watch.

`stop` needs no lease (the gateway lets it through from anyone). A holder that falls silent loses
the lease — the gateway's watchdog asks `silent_for()` and stops every module first.
"""

import threading
import time


class Lease:
    def __init__(self):
        self._lock = threading.Lock()
        self._holder = None
        self._heard_at = 0.0

    @property
    def holder(self):
        with self._lock:
            return self._holder

    def take(self, client: str) -> bool:
        """True when `client` now holds the lease (it was free, or already this client's)."""
        with self._lock:
            if self._holder not in (None, client):
                return False
            self._holder, self._heard_at = client, time.monotonic()
            return True

    def release(self, client: str) -> bool:
        """Give the lease up; True when `client` held it. WHY: the test and the release in ONE step, so
        a connection that ends can tell whether it was the holder (only the holder's end stops modules)."""
        with self._lock:
            if self._holder is None or self._holder != client:
                return False
            self._holder = None
            return True

    def held_by(self, client: str) -> bool:
        with self._lock:
            return self._holder is not None and self._holder == client

    def touch(self, client: str) -> None:
        """The client is alive (any message from it counts)."""
        with self._lock:
            if self._holder == client:
                self._heard_at = time.monotonic()

    def silent_for(self) -> float:
        with self._lock:
            return 0.0 if self._holder is None else time.monotonic() - self._heard_at

    def drop(self):
        """Take the lease away; returns who held it."""
        with self._lock:
            holder, self._holder = self._holder, None
            return holder
