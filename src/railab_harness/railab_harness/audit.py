"""The command log: every request that reached the door and what was answered, one JSON line each.

It answers "what was the hardware told, by whom, and what did it say" after the fact — the one
question nobody can answer from module logs spread over several machines.
"""

import json
import threading
import time


class Audit:
    def __init__(self, path=None):
        """`path` None = keep no log (a check, a walk-through that does not want one)."""
        self._lock = threading.Lock()
        self._file = open(path, "a", encoding="utf-8") if path else None

    def write(self, client: str, envelope: dict, reply: dict, forwarded: bool) -> None:
        if self._file is None:
            return
        line = json.dumps({"time": round(time.time(), 3), "client": client, "request": envelope,
                           "reply": reply, "forwarded": forwarded}, ensure_ascii=False, default=str)
        with self._lock:
            if not self._file.closed:
                self._file.write(line + "\n")
                self._file.flush()

    def close(self) -> None:
        with self._lock:
            if self._file is not None:
                self._file.close()
