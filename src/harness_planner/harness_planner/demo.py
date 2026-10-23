"""`harness arm demo` (PROJECT_SPEC.md "2026-10-22 — Planner bedrock"): fly a motion file once —
approach from where the arm is, the path once, back to where it started.

Plans with the planner bedrock (trajectory.py) in the harness's corrected arm model, validates
every sample, and — only with yes=True, the operator's explicit request — streams it through
/arm/follow_trajectory. Without yes it is a dry run: plan + report, nothing moves.
Runs on a node that is spun elsewhere (the CLI daemon's multi-threaded executor).
"""
import math
import os
import time

import yaml

from harness_arm.kinematics import PiperKinematics
from harness_planner.trajectory import CartesianPath, Limits, PlanError, plan_path_run

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DEFAULT_MOTION = "motions/twin_demo.json"
DEFAULT_DURATION_S = 24.0                  # operator 2026-10-22: half the twin's 12 s pace for the first runs


def load_model(root="."):
    """(kinematics, limits) from the repo: calib/kinematics.yaml (corrected model) and the arm
    node's own safety parameters in src/harness_arm/config/piper.yaml (tcp offset, floor,
    workspace, IK margin), so the planner checks what the node will check."""
    kin = PiperKinematics.from_file(os.path.join(root, "calib/kinematics.yaml"))
    with open(os.path.join(root, "src/harness_arm/config/piper.yaml")) as f:
        p = yaml.safe_load(f)["piper"]["ros__parameters"]
    lim = Limits(tcp_offset_m=float(p["tcp_offset_m"]), floor_enabled=bool(p.get("floor_enabled", True)),
                 floor_plane=tuple(p["floor_plane"]), floor_margin_m=float(p["floor_margin_m"]),
                 workspace_min=tuple(p["workspace_min"]), workspace_max=tuple(p["workspace_max"]),
                 ik_margin_rad=float(p.get("ik_limit_margin_rad", 0.02)))
    return kin, lim


def describe(tr, out):
    r = tr.report
    seg = r["segments"]
    out(f"plan '{r['path']}': {r['path_length_m'] * 1000:.0f} mm of path, loop {r['loop_s']:.1f} s "
        f"(smooth start/stop {r['ramp_s']:.1f} s); approach {seg['approach']:.1f} s + loop {seg['loop']:.1f} s"
        + (f" + return {seg['return']:.1f} s" if "return" in seg else "") + f" = {tr.duration:.1f} s, "
        f"{r['samples']} samples at {r['rate_hz']:.0f} Hz")
    out("peak joint speeds (deg/s): " + " ".join(f"j{i + 1} {v:.1f}" for i, v in enumerate(r["peak_joint_speed_deg_s"]))
        + f"  | fingertip >= {r['min_floor_clearance_mm']} mm above the floor"
        + f"  | on the curve within {r['max_curve_deviation_mm']} mm / {r['max_curve_deviation_deg']} deg")


def to_msg(tr):
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    jt = JointTrajectory()
    jt.joint_names = list(JOINT_NAMES)
    for t, q in zip(tr.t, tr.q):
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in q]
        sec = int(math.floor(t))
        p.time_from_start = Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)
        jt.points.append(p)
    return jt


def run(node, ac, out, cancelled, joints, arm_status, path_file=DEFAULT_MOTION, duration_s=DEFAULT_DURATION_S,
        yes=False, root="."):
    """joints() -> current 6 joint angles (rad) or None; arm_status() -> latest ArmStatus or None;
    ac = ActionClient for /arm/follow_trajectory. Returns the success text; raises RuntimeError."""
    from harness_msgs.action import FollowTrajectory
    q_now = joints()
    if q_now is None:
        raise RuntimeError("no /joint_states (arm node running? harness nodes status)")
    try:
        kin, lim = load_model(root)
        path = CartesianPath.load(os.path.join(root, path_file) if not os.path.isabs(path_file) else path_file)
        out(f"planning '{path.name or path_file}' ({len(path.p)} samples, {'closed' if path.closed else 'open'}) "
            f"from the current joints, loop {duration_s:.1f} s ...")
        t0 = time.time()
        tr = plan_path_run(kin, path, q_now, duration_s, lim)
    except PlanError as e:
        raise RuntimeError("refused: " + str(e)) from e
    out(f"planned and validated in {time.time() - t0:.1f} s")
    describe(tr, out)
    if not yes:
        return ("dry run: nothing moved. The bench must be CLEAR (the path sweeps +-50 deg in front of the arm, "
                "~0.5 m out, 220-380 mm high); then run it with --yes")

    st = arm_status()
    if st is None or not st.all_enabled:
        raise RuntimeError("the arm is not enabled: harness arm hold first")
    q2 = joints()
    if q2 is None or max(abs(a - b) for a, b in zip(q2, q_now)) > 0.02:
        raise RuntimeError("the arm moved while planning; run the command again")
    if not ac.wait_for_server(timeout_sec=3.0):
        raise RuntimeError("/arm/follow_trajectory not available (arm node too old? rebuild + restart piper)")
    goal = FollowTrajectory.Goal()
    goal.trajectory = to_msg(tr)
    goal.label = "demo " + (path.name or os.path.basename(path_file))
    fb = {}
    gf = ac.send_goal_async(goal, feedback_callback=lambda m: fb.__setitem__("f", m.feedback))
    t0 = time.time()
    while not gf.done():
        if time.time() - t0 > 10:
            raise RuntimeError("goal not accepted in time")
        time.sleep(0.01)
    gh = gf.result()
    if not gh.accepted:
        raise RuntimeError("goal rejected by the arm node (busy, fault or teach mode: harness arm status)")
    out(f"flying ({tr.duration:.0f} s; Ctrl-C stops the arm where it is) ...")
    rf = gh.get_result_async()
    last_print = 0.0
    while not rf.done():
        if cancelled():
            gh.cancel_goal_async()
            out("cancel requested: the arm freezes where it is")
            t1 = time.time()
            while not rf.done() and time.time() - t1 < 5:
                time.sleep(0.05)
            raise RuntimeError("cancelled by the operator")
        f = fb.get("f")
        if f is not None and time.time() - last_print > 3.0:
            last_print = time.time()
            out(f"  {f.progress * 100:5.1f} %  tracking error {math.degrees(f.tracking_error_rad):.2f} deg")
        if time.time() - t0 > tr.duration + 60:
            gh.cancel_goal_async()
            raise RuntimeError("no result long after the trajectory should have ended; cancelled")
        time.sleep(0.05)
    r = rf.result().result
    if not r.success:
        raise RuntimeError(r.message)
    return f"{r.message} (worst tracking error {math.degrees(r.max_tracking_error_rad):.2f} deg)"
