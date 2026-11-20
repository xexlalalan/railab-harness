# /harness/heartbeat - published by the harness at a fixed beat (1 s), heard by every module node.
# A module that misses it for the timeout goes to its safe state by itself. Module interface, 2026-11-20.

builtin_interfaces/Time stamp
uint64 seq
