"""The harness (the middle layer): one generic door between the twin and the modules.

module.py     what every module node is (the adapter base)
gateway.py    what the harness is (the generic door)
tcp_front.py  the twin's way in to the gateway (JSON lines over TCP)
the rest      the small parts the two are made of

Module interface 2026-11-20 — see docs/module_interface.md in this repo.
"""

__version__ = "0.1.0"
