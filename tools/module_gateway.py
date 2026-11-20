#!/usr/bin/env python3
"""The module gateway: the generic middle layer as one process (docs/module_interface.md).

    python3 tools/module_gateway.py [--port 7804] [--bind 127.0.0.1] [--audit PATH]
    python3 tools/module_gateway.py --sim        the two simulated modules instead of ROS (no hardware)

ROS bus (railab_harness.ros_binding: /<instance>/command, /<instance>/state, /harness/heartbeat)
+ Gateway (lease, zones, registry, command log) + the twin's TCP door (railab_harness.tcp_front,
newline-delimited JSON). It runs SIDE BY SIDE with today's gateway (tools/gateway_server.py, TCP
7803), which it does not touch; it is not in `harness nodes` (tools/harness NODES): start it by hand.

--bind defaults to 127.0.0.1: an untested door to hardware is not open to the network by itself.
From the harness PC itself, or through an ssh tunnel, it is reachable as is; for the twin on the
workstation over the cable / Wi-Fi pass --bind 0.0.0.0 (as 7803 binds) once that is wanted.

The control key is today's gateway key (~/.config/railab-harness/gateway.key, `harness gateway key`);
it is read at every hello, never printed. The command log (one JSON line per request and reply)
defaults to /tmp beside the harness's process logs (/tmp/<name>.log of `harness nodes start`):
cleared at a reboot, so it never fills the disk — pass --audit for a lasting copy.
"""

import argparse
import os
import signal
import sys
import threading

# THE ROS LINKS ARE UDP ONLY, for the reason tools/gateway_server.py gives (2026-11-19): Fast DDS's
# shared-memory port of a long-running process can be orphaned after another node's restart, and
# then it hears nothing more. Set before rclpy loads Fast DDS; an explicit value wins.
os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# the core from source, so a change there needs no colcon rebuild (as gateway_server.py takes
# harness_calibration / harness_planner); the message package still comes from the sourced workspace
sys.path[:0] = [os.path.join(ROOT, "src", "railab_harness"), os.path.join(ROOT, "tools")]

from railab_harness.gateway import Gateway          # noqa: E402
from railab_harness.tcp_front import KEY_FILE, TcpFront, read_key   # noqa: E402
from railab_harness.timing import DEFAULT           # noqa: E402

PORT = 7804                                         # today's gateway keeps 7803
BIND = "127.0.0.1"
AUDIT = "/tmp/module_gateway_commands.jsonl"
NODE_NAME = "module_gateway"


def make_bus(sim: bool):
    """(the bus, the simulated modules to close at the end)."""
    if not sim:
        from railab_harness.ros_binding import RosBus   # lazy: rclpy only here
        return RosBus(node_name=NODE_NAME), []   # its own node, spun on its own executor thread
    from railab_harness.bus import LocalBus
    from module_sim.arm_sim import Arm
    from module_sim.bunny_sim import Bunny
    bus = LocalBus()
    return bus, [Bunny("bunny_1", bus, DEFAULT).start(), Arm("arm_1", bus, DEFAULT).start()]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="The module gateway: ROS bus + Gateway + the twin's TCP door.")
    parser.add_argument("--port", type=int, default=PORT, help=f"TCP port of the twin's door (default {PORT})")
    parser.add_argument("--bind", default=BIND, help=f"address to listen on (default {BIND}; 0.0.0.0 = every interface)")
    parser.add_argument("--audit", default=AUDIT, help=f"command log, one JSON line per request (default {AUDIT}; '' = none)")
    parser.add_argument("--sim", action="store_true", help="two simulated modules on an in-process bus instead of ROS")
    args = parser.parse_args(argv)

    if args.audit:
        os.makedirs(os.path.dirname(os.path.abspath(args.audit)), exist_ok=True)
    bus, modules = make_bus(args.sim)
    gateway = Gateway(bus, DEFAULT, args.audit or None).start()
    front = TcpFront(gateway, read_key)
    try:
        front.listen(args.bind, args.port)
    except OSError as error:
        print(f"module gateway: cannot listen on {args.bind}:{args.port}: {error}", file=sys.stderr, flush=True)
        gateway.close()
        for module in modules:
            module.close()
        return 1

    # rclpy takes SIGINT / SIGTERM for its own shutdown, which would leave the door open:
    # ours are set after the bus exists (`pkill` = SIGTERM must end the process)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    server = threading.Thread(target=front.serve_forever, name="module-gateway-accept", daemon=True)
    server.start()
    print(f"module gateway: listening on {args.bind}:{args.port} ({'simulated modules' if args.sim else 'ROS bus'}; "
          f"key {'set' if read_key() else 'MISSING: view-only — harness gateway key'} in {KEY_FILE}; "
          f"command log {args.audit or 'none'})", flush=True)
    while not done.wait(1.0) and server.is_alive():
        pass

    print("module gateway: closing (the controlling client's link ends → every module is stopped)", flush=True)
    front.close()          # ends every connection: the controller's ending stops every module
    server.join(2)
    gateway.close()        # the beat ends: every module node goes safe by itself
    for module in modules:
        module.close()
    close = getattr(bus, "close", None)
    if callable(close):
        close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
