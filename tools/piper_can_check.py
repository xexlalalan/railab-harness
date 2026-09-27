#!/usr/bin/env python3
"""Read-only PIPER link check over SocketCAN.

Connects with pyAgxArm on the given channel, waits for the firmware reply, and
prints firmware, arm status, joint angles, TCP pose and enable state. It never
enables the arm and never sends a motion command, so it is safe to run at any
time (also while the arm is powered but parked).

Usage:  python tools/piper_can_check.py [--channel can0] [--timeout 10]
Exit code 0 = arm answering, 1 = no firmware reply / SDK error.

Bring the interface up first (WSL2: see PROJECT_SPEC.md, "Development
Environment"):  ip link set can0 type can bitrate 1000000 && ip link set can0 up
"""
import argparse
import sys
import time

from pyAgxArm import AgxArmFactory, PiperFW, create_agx_arm_config


def firmware_enum(software_version: str):
    if software_version >= "S-V1.8-9":
        return PiperFW.V189
    if software_version >= "S-V1.8-8":
        return PiperFW.V188
    if software_version >= "S-V1.8-3":
        return PiperFW.V183
    return PiperFW.DEFAULT


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the firmware reply")
    args = ap.parse_args()

    # Pass 1: default protocol, only to learn the firmware version.
    robot = AgxArmFactory.create_arm(create_agx_arm_config(robot="piper", interface="socketcan", channel=args.channel))
    try:
        robot.connect()
    except Exception as exc:  # interface down, no such device, permission
        print(f"FAIL: cannot open {args.channel}: {exc!r}")
        return 1
    deadline = time.time() + args.timeout
    while robot.get_firmware() is None:
        if time.time() > deadline:
            print(f"FAIL: no firmware reply on {args.channel} within {args.timeout:.0f} s "
                  "(arm unpowered, wrong bitrate, or CAN_H/CAN_L swapped/unplugged)")
            robot.disconnect()
            return 1
        time.sleep(0.25)
    fw = robot.get_firmware()
    robot.disconnect()

    # Pass 2: protocol matching the firmware, read everything once.
    sv = fw["software_version"]
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot="piper", firmeware_version=firmware_enum(sv), interface="socketcan", channel=args.channel))
    robot.connect()
    time.sleep(1.0)
    st = robot.get_arm_status()
    ja = robot.get_joint_angles()
    tcp = robot.get_tcp_pose()
    en = robot.get_joints_enable_status_list()
    robot.disconnect()

    print(f"OK: PIPER answering on {args.channel}")
    print(f"  firmware      : {sv}  hw {fw['hardware_version']}  built {fw['production_date']}  node {fw['node_number']}")
    if st is not None:
        m = st.msg
        print(f"  status        : ctrl_mode={m.ctrl_mode} arm_status={m.arm_status} motion={m.motion_status} "
              f"teach={m.teach_status}  ({st.hz:.0f} Hz)")
        errs = [k for k, v in vars(m.err_status).items() if v]
        print(f"  errors        : {errs if errs else 'none'}")
    if ja is not None:
        print("  joint angles  : " + "  ".join(f"{a:+.3f}" for a in ja.msg) + "  rad")
    if tcp is not None:
        p = tcp.msg
        print(f"  tcp pose      : xyz {p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f} m   rpy {p[3]:+.3f} {p[4]:+.3f} {p[5]:+.3f} rad")
    print(f"  joints enabled: {en}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
