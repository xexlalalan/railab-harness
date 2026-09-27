#!/usr/bin/env python3
"""Enable the PIPER and hold it at its current position (or release it).

    python tools/piper_hold.py            # read pose, enable, hold here, leave enabled
    python tools/piper_hold.py --release  # disable all joints (arm goes limp -- support it!)
    python tools/piper_hold.py --status   # read-only

Hold = enable all six joints, then command a joint-space target equal to the
angles read just before enabling, at a low speed percentage. The script then
watches feedback for a few seconds and DISABLES the arm at once if any joint
drifts more than --max-drift rad from the captured pose, an error bit appears,
or feedback stops. On success the arm stays enabled after the script exits.
"""
import argparse
import sys
import time

from pyAgxArm import AgxArmFactory, PiperFW, create_agx_arm_config


def connect(channel):
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot="piper", firmeware_version=PiperFW.V189, interface="socketcan", channel=channel))
    robot.connect()
    deadline = time.time() + 5
    while robot.get_joint_angles() is None or robot.get_arm_status() is None:
        if time.time() > deadline:
            print("FAIL: no feedback on", channel)
            robot.disconnect()
            sys.exit(1)
        time.sleep(0.1)
    return robot


def errors(robot):
    st = robot.get_arm_status()
    if st is None:
        return ["no status feedback"]
    return [k for k, v in vars(st.msg.err_status).items() if v]


def report(robot, label):
    st = robot.get_arm_status()
    ja = robot.get_joint_angles()
    tcp = robot.get_tcp_pose()
    en = robot.get_joints_enable_status_list()
    print(f"[{label}]")
    if st is not None:
        m = st.msg
        print(f"  ctrl_mode={m.ctrl_mode} arm_status={m.arm_status} motion={m.motion_status} teach={m.teach_status}")
    print(f"  errors        : {errors(robot) or 'none'}")
    print("  joint angles  : " + "  ".join(f"{a:+.4f}" for a in ja.msg) + "  rad")
    p = tcp.msg
    print(f"  tcp pose      : xyz {p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f} m   rpy {p[3]:+.3f} {p[4]:+.3f} {p[5]:+.3f} rad")
    print(f"  joints enabled: {en}")
    return list(ja.msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--release", action="store_true", help="disable all joints and exit")
    ap.add_argument("--status", action="store_true", help="read-only report")
    ap.add_argument("--speed", type=int, default=10, help="speed percent for the hold command (default 10)")
    ap.add_argument("--watch", type=float, default=5.0, help="seconds to supervise after enabling")
    ap.add_argument("--max-drift", type=float, default=0.05, help="rad; disable if any joint moves more than this")
    args = ap.parse_args()

    robot = connect(args.channel)
    try:
        if args.status:
            report(robot, "status")
            return 0

        if args.release:
            report(robot, "before release")
            ok = robot.disable()
            time.sleep(0.3)
            en = robot.get_joints_enable_status_list()
            print(f"disable() -> {ok}; joints enabled now: {en}")
            return 0 if not any(en) else 1

        errs = errors(robot)
        if errs:
            print(f"REFUSED: arm reports errors before enabling: {errs}")
            return 1
        st = robot.get_arm_status().msg
        if str(st.teach_status).startswith("ENABLED") or "TEACH" in str(st.ctrl_mode):
            print(f"REFUSED: arm is in teach/drag mode (ctrl_mode={st.ctrl_mode}, teach={st.teach_status}); "
                  "stop drag-teach on the arm first")
            return 1

        target = report(robot, "captured pose (target for hold)")

        robot.set_speed_percent(args.speed)
        time.sleep(0.1)
        robot.enable()
        t0 = time.time()
        while not all(robot.get_joints_enable_status_list()):
            if time.time() - t0 > 3.0:
                print(f"FAIL: joints did not all enable: {robot.get_joints_enable_status_list()} -- disabling")
                robot.disable()
                return 1
            time.sleep(0.05)
        print(f"  all joints enabled after {time.time() - t0:.2f} s")

        robot.move_j(target)  # zero-length move: sets CAN control mode + explicit hold target
        print(f"  hold target sent (move_j to captured angles, speed {args.speed}%)")

        # Supervise.
        worst = 0.0
        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < args.watch:
            ja = robot.get_joint_angles()
            if ja is None or time.time() - ja.timestamp > 0.5:
                print("FAIL: feedback lost -- disabling")
                robot.disable()
                return 1
            drift = max(abs(a - b) for a, b in zip(ja.msg, target))
            worst = max(worst, drift)
            errs = errors(robot)
            if errs:
                print(f"FAIL: error bits {errs} -- disabling")
                robot.disable()
                return 1
            if drift > args.max_drift:
                print(f"FAIL: joint drift {drift:.4f} rad > {args.max_drift} -- disabling")
                robot.disable()
                return 1
            if time.time() - last_print >= 1.0:
                print(f"  t={time.time() - t0:4.1f}s  max drift {drift:.4f} rad  enabled={all(robot.get_joints_enable_status_list())}")
                last_print = time.time()
            time.sleep(0.05)

        report(robot, "holding")
        print(f"OK: arm enabled and holding; worst drift during watch {worst:.4f} rad.")
        print("Release with:  python tools/piper_hold.py --release   (support the arm -- it goes limp)")
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
