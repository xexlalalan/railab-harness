"""Every period of the harness, in ONE place.

DECIDED (operator 2026-11-20): ONE BEAT PER SECOND, both ways (BEAT_S, IDLE_STATE_S). The
timeouts are still suggestions (three missed beats) that keep the order right: a module must
notice a dead harness (MODULE_SILENT_S) before the harness calls the module dead (OFFLINE_S).
"""

from dataclasses import astuple, dataclass

BEAT_S = 1.0           # the harness publishes /harness/heartbeat this often
MODULE_SILENT_S = 3.0  # a module hearing no beat this long goes to its safe state by itself
IDLE_STATE_S = 1.0     # a module pushes its state at least this often (its heartbeat upward)
OFFLINE_S = 4.0        # a module whose newest state is older than this is offline for the harness
CLIENT_SILENT_S = 3.0  # a controlling client silent this long: drop the lease, stop the actions THIS layer
                       # started (Gateway.stop_all: nothing when none runs, so an idle dead-man is harmless)
REPLY_S = 1.0          # a command's reply is immediate; none within this = the module is unreachable


@dataclass(frozen=True)
class Timing:
    """The periods as one value, so a test or the walk-through can run them scaled down."""

    beat_s: float = BEAT_S
    module_silent_s: float = MODULE_SILENT_S
    idle_state_s: float = IDLE_STATE_S
    offline_s: float = OFFLINE_S
    client_silent_s: float = CLIENT_SILENT_S
    reply_s: float = REPLY_S

    def scaled(self, k: float) -> "Timing":
        return Timing(*(v * k for v in astuple(self)))


DEFAULT = Timing()
