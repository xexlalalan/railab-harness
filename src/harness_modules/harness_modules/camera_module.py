"""THE CAMERA'S ADAPTER NODE: the gripper camera (the RealSense D405 under harness_vision's tag_node) on the
module interface, side by side with today's path. Read-only: no actions of its own (stop / clear_fault come with
the base and have nothing to move).

Downward: a LINK that is a ROS client of the existing tag_node — it only listens to /camera/tags
(harness_msgs/TagArray, one message per frame). Upward, the fields, all sensed:
  tags          what the newest frame saw, one entry per tag: family, id, the tag's pose in the camera's optical
                frame (x right, y down, z forward; position_mm + orientation_xyzw), the size the pose assumed,
                distance, pixels (centre, corners, size), reprojection error, the central-square flag, stereo
                depth — whatever TagArray carries, in mm. "unknown" while frames are not fresh (an empty list
                would say "no tag in view").
  frame_age_s   seconds since the newest frame arrived ("unknown" before the first one). The frame's own time
                = the state's `time` − frame_age_s.
mode: ready while frames are fresh (newest ≤ FRESH_S old), not_ready otherwise.

TAG STREAMS STAY IN THE MIDDLE LAYER (operator 2026-11-20): tag_node publishes every frame (~15–30 Hz). This node
does NOT push a state per frame: it publishes the newest frame's tags every STATE_PERIOD_S, whatever the frame rate.
A consumer that needs EVERY frame (the arm's calibrations, a fine tag measurement) uses the camera's own topics
INSIDE the harness: /camera/tags, /camera/charuco, /camera/image/compressed, /camera/camera_info.

    ros2 run harness_modules camera_module --ros-args -p instance:=camera_1 [-p tags_topic:=/camera/tags]
"""

import threading
import time

from railab_harness import messages as M
from railab_harness.module import Module
from railab_harness.timing import DEFAULT

STATE_PERIOD_S = 0.5   # the slow fixed rate: one state every 0.5 s (2 Hz), never one per frame
FRESH_S = 1.0          # a newest frame older than this = the camera sends no frames (the old gateway's rule)


class CameraLink:
    """What the adapter needs from the camera's node. A fake replaces this in tools/module_check_bunny.py."""

    def newest(self):
        """(frame, age_s) of the newest frame, or (None, None) before the first one. frame = {"frame": the optical
        frame's id, "tags": [tag dicts as described in the module docstring]}."""
        raise NotImplementedError


class CameraModule(Module):
    TYPE, VERSION = "realsense_d405", "adapter-0.1.0"
    ACTIONS = {}
    FIELDS = {
        "tags": {"unit": "a list of {family, id, frame, position_mm, orientation_xyzw, size_mm, distance_mm, "
                         "center_px, corners_px, size_px, reproj_err_px, in_center_square, depth_mm, "
                         "depth_size_mm}", "writable": False},
        "frame_age_s": {"unit": "s", "writable": False},
    }

    def __init__(self, name, bus, link: CameraLink, timing=DEFAULT, serial="", log=None,
                 period_s=STATE_PERIOD_S):
        """period_s None = no publishing thread (a check calls tick_once() itself)."""
        super().__init__(name, bus, timing, serial)
        self.link = link
        self.log = log or (lambda text: None)
        self.period_s = period_s
        self._was_ready = None
        self._tick_stop = threading.Event()
        self._publisher = None

    def start(self):
        super().start()
        if self.period_s:
            self._publisher = threading.Thread(target=self._stream, name=f"{self.name}-states", daemon=True)
            self._publisher.start()
        return self

    def close(self):
        self._tick_stop.set()
        if self._publisher is not None and self._publisher.is_alive():
            self._publisher.join(2)
        super().close()

    def _stream(self):
        while not self._tick_stop.wait(self.period_s):
            try:
                self.tick_once()
            except Exception as error:   # one bad frame must not end the state stream
                self.log(f"{self.name}: {type(error).__name__}: {error}")

    def tick_once(self) -> None:
        """Take the newest frame and publish ONE state with it."""
        frame, age = self.link.newest()
        fresh = age is not None and age <= FRESH_S
        values = {"frame_age_s": M.UNKNOWN if age is None else round(float(age), 2),
                  "tags": list(frame.get("tags") or []) if fresh and frame else M.UNKNOWN}
        if fresh != self._was_ready:
            self.log(f"{self.name}: " + ("frames are fresh: ready" if fresh else
                                          "no fresh frame (" + ("none yet" if age is None else f"newest {age:.1f} s old")
                                          + "): not ready"))
            self._was_ready = fresh
        self.sensed_many(values, ready=fresh)   # every field and the readiness in ONE state


# ------------------------------------------------------------------------- the ROS link (untested: no ROS here)
class RosCameraLink(CameraLink):
    """Listens to tag_node's TagArray; keeps only the newest message and converts it when asked (at the state
    rate, not the frame rate)."""

    def __init__(self, node, topic: str = "/camera/tags"):
        from harness_msgs.msg import TagArray
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._guard = threading.Lock()
        self._newest = None          # (message, arrived on this process's monotonic clock)
        self._converted = (None, None)
        self._kept = [node.create_subscription(TagArray, topic, self._heard, 5,
                                               callback_group=ReentrantCallbackGroup())]

    def _heard(self, message):
        with self._guard:
            self._newest = (message, time.monotonic())

    def newest(self):
        with self._guard:
            newest = self._newest
        if newest is None:
            return None, None
        message, arrived = newest
        if self._converted[0] is not message:
            self._converted = (message, frame_from_msg(message))
        return self._converted[1], time.monotonic() - arrived


def frame_from_msg(message) -> dict:
    """harness_msgs/TagArray → {"frame", "tags"} in mm (the pose's metres × 1000)."""
    frame = message.header.frame_id
    return {"frame": frame, "tags": [_tag(t, frame) for t in message.tags]}


def _tag(t, frame: str) -> dict:
    p, q = t.pose.position, t.pose.orientation
    return {
        "family": str(t.family), "id": int(t.id), "frame": frame,
        "position_mm": [round(p.x * 1000.0, 2), round(p.y * 1000.0, 2), round(p.z * 1000.0, 2)],
        "orientation_xyzw": [round(q.x, 6), round(q.y, 6), round(q.z, 6), round(q.w, 6)],
        "size_mm": round(t.size_m * 1000.0, 2),          # the black-square edge the pose assumed
        "distance_mm": round(t.distance_m * 1000.0, 1),
        "center_px": [round(v, 1) for v in t.center_px],
        "corners_px": [round(v, 1) for v in t.corners_px],
        "size_px": round(t.size_px, 1),
        "reproj_err_px": round(t.reproj_err_px, 3),
        "in_center_square": bool(t.in_center_square),
        "depth_mm": round(t.depth_m * 1000.0, 1),        # 0 = no stereo depth
        "depth_size_mm": round(t.depth_size_m * 1000.0, 2),
    }


PARAMETERS = {"instance": "camera_1", "tags_topic": "/camera/tags", "serial": ""}   # tag_node's topic today


def main(args=None):
    """One node `<instance>_module` for the link and the bus (the arm adapter's way)."""
    import signal

    from railab_harness.ros_binding import RosBus, ros_environment
    # Fast DDS over UDP only, before rclpy loads: the old gateway once heard no camera frame for an hour through an
    # orphaned shared-memory port (7e2e7d5)
    ros_environment()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    probe = rclpy.create_node("camera_module_parameters")
    params = {name: probe.declare_parameter(name, default).value for name, default in PARAMETERS.items()}
    probe.destroy_node()
    instance = str(params["instance"])
    node = rclpy.create_node(f"{instance}_module")
    for name, value in params.items():
        node.declare_parameter(name, value)
    log = node.get_logger().info
    link = RosCameraLink(node, str(params["tags_topic"]))
    bus = RosBus(node=node)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, name="camera-module-spin", daemon=True).start()
    module = CameraModule(instance, bus, link, serial=str(params["serial"]), log=log).start()
    log(f"{instance}: the camera's adapter on {params['tags_topic']}, one state every {STATE_PERIOD_S} s")
    leave = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: leave.set())
    while not leave.wait(0.5):
        pass
    try:
        module.close()
    finally:
        bus.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
