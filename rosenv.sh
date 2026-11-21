# source this in non-interactive shells on the harness PC
source /opt/ros/lyrical/setup.bash
export PYTHONPATH=$HOME/bunny-harness-dev/ros-venv/lib/python3.14/site-packages:${PYTHONPATH:-}
export ROS_DOMAIN_ID=36
# EVERY harness ROS process talks Fast DDS over UDP only (2026-11-21): a long-running process's shared-memory port
# (/dev/shm/fastdds_port<N>) can be deleted and re-created by another process's "zombie port" check; it then listens
# on the deleted segment and no participant started later reaches it (the gateway 2026-11-19, 7e2e7d5; the arm node
# 2026-11-21: "/arm/hold timed out", no /piper in `ros2 node list`). A UDP socket cannot be orphaned that way
# (same-host traffic goes over loopback). Set HERE because every start sources this file: `harness nodes
# start|restart` (all NODES, so the watchdog's and the gateway's restarts too), the CLI daemon, node_ctl.sh, the
# `ros2` daemon a status call spawns. An explicit value wins (as the gateway's and the adapters' own setdefault).
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}
source $HOME/bunny-harness-dev/install/setup.bash
