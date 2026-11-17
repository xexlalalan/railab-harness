#!/usr/bin/env python3
"""Gripper camera + AprilTag node (harness_vision).

Owns the RealSense D405. Publishes:
  /camera/tags               harness_msgs/TagArray      every frame (id, pixels, camera-frame pose, "set" flag)
  /camera/image/compressed   sensor_msgs/CompressedImage  at image_rate_hz, with the detection overlay
  /camera/camera_info        sensor_msgs/CameraInfo     with every image
  /camera/charuco            harness_msgs/CharucoDetection  every frame: the ChArUco board (corners found, pose)

Pose: corners are deprojected with the device's own distortion model
(rs2_deproject_pixel_to_point), then solvePnP (IPPE_SQUARE) on normalised rays,
so the RealSense "inverse Brown-Conrady" model is honoured exactly.

"Set" criterion (coarse teach): tag centre inside the central square (side =
center_fraction x image height) AND mean edge length within [min_size_px,
max_size_px] (a standoff gate). The node only reports it; whoever teaches
decides how long it must hold.

Families (2026-10-24): `families` lists the AprilTag families to decode — one
OpenCV detector per family runs on every frame (a detector identifies codes of
its own dictionary only, so a 36h11 print was invisible while the node ran
16h5 alone); every detection carries its family. Black-square sizes per
family and id in `tag_sizes` ("family:id=size_m"); `tag_size_m` otherwise.

tagStandard52h13 / tagStandard41h12 (2026-11-16): decoded by AprilTag 3 (the C
library libapriltag through ctypes, `apriltag_*` parameters) and reported in
the OpenCV families' convention (corner order, tag frame, pixel convention) —
harness_vision/detect.py, where the detection and the report are pure
functions. Without the library the node starts, says so in one line and
decodes the OpenCV families only.

The ChArUco board (2026-11-16, the hand-eye target; camera.yaml `charuco`,
`charuco.dict` "" = off): found on every frame and published on
/camera/charuco whether seen or not (corners found, markers, and when seen
the board pose by solvePnP on the device-normalised corners, like a tag's) —
harness_vision/detect.py Charuco. A harness_msgs built before it has no
CharucoDetection: the node then says so once and publishes tags only.
"""
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from harness_msgs.msg import TagArray, TagDetection
from harness_vision.detect import AT3_INSTALL, Charuco, Detector, measure, parse_tag_sizes, short_family
from harness_vision.geom import pose_msg_from_T

try:                                      # harness_msgs of 2026-11-16 or later
    from harness_msgs.msg import CharucoDetection
except ImportError:
    CharucoDetection = None


class TagNode(Node):
    def __init__(self):
        super().__init__("tag_node")
        prm = self.declare_parameters("", [
            ("width", 1280), ("height", 720), ("fps", 30), ("family", "tag36h11"), ("families", ["tag36h11"]),
            ("tag_size_m", 0.05), ("tag_sizes_ids", [0]), ("tag_sizes_m", [0.0]), ("tag_sizes", [""]),
            ("center_fraction", 0.3333), ("min_size_px", 50), ("max_size_px", 260),
            ("image_rate_hz", 10.0), ("jpeg_quality", 70), ("exposure_us", 0),
            ("error_correction_rate", 0.0),
            ("apriltag_lib", ""), ("apriltag_max_hamming", 1), ("apriltag_threads", 2), ("apriltag_decimate", 2.0),
            ("charuco.dict", "DICT_5X5_100"), ("charuco.squares_x", 9), ("charuco.squares_y", 11),
            ("charuco.square_m", 0.022), ("charuco.marker_m", 0.016), ("charuco.min_corners", 6),
        ])
        self.P = {x.name: x.value for x in prm}
        # black-square edge per (family, id) from tag_sizes "family:id=size_m"; the older tag_sizes_ids / tag_sizes_m
        # pairs count for any family; tag_size_m for everything else
        self.sizes_any = {int(i): float(s) for i, s in zip(self.P["tag_sizes_ids"] or [], self.P["tag_sizes_m"] or []) if s > 0}
        self.sizes = parse_tag_sizes(self.P["tag_sizes"])
        fams = [str(f) for f in (self.P["families"] or []) if str(f).strip()] or [str(self.P["family"])]

        self.pub_tags = self.create_publisher(TagArray, "/camera/tags", 10)
        self.pub_img = self.create_publisher(CompressedImage, "/camera/image/compressed", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", 5)

        # every family on every frame: OpenCV per family, AprilTag 3 once for the tagStandard ones (unknown names raise)
        self.det = Detector(fams, error_correction_rate=float(self.P["error_correction_rate"]),
                            apriltag_lib=str(self.P["apriltag_lib"]) or None,
                            max_hamming=int(self.P["apriltag_max_hamming"]), threads=int(self.P["apriltag_threads"]),
                            decimate=float(self.P["apriltag_decimate"]))
        if self.det.missing:
            self.get_logger().error(f"AprilTag 3 library missing: {', '.join(self.det.missing)} NOT decoded (the other "
                                    f"families are) - install it ({AT3_INSTALL}) and restart the camera node; "
                                    f"{next(iter(self.det.missing.values()))}")
        self.P["family"] = fams[0]
        self.board, self.pub_board = None, None   # the ChArUco board (the hand-eye target)
        if str(self.P["charuco.dict"]).strip():
            self.board = Charuco(str(self.P["charuco.dict"]).strip(), int(self.P["charuco.squares_x"]),
                                 int(self.P["charuco.squares_y"]), float(self.P["charuco.square_m"]),
                                 float(self.P["charuco.marker_m"]), int(self.P["charuco.min_corners"]))
            if CharucoDetection is None:
                self.get_logger().error("ChArUco board: harness_msgs has no CharucoDetection (built before 2026-11-16) - "
                                        "/camera/charuco NOT published: colcon build --symlink-install --packages-select "
                                        "harness_msgs harness_vision, then restart the camera node")
            else:
                self.pub_board = self.create_publisher(CharucoDetection, "/camera/charuco", 10)

        self.pipe = None
        try:
            self._open_camera()
        except Exception as e:  # noqa: BLE001  no camera at start (its cable loose, 2026-10-25): the loop waits for it
            self.get_logger().error(f"camera: {e} - waiting for it (check its USB cable)")
        self.frames = 0
        self.last_img_t = 0.0
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"tag_node up: {self.P['width']}x{self.P['height']}@{self.P['fps']} families "
                               f"{self.det.families}"
                               + (f" (AprilTag 3: {self.det.at3_path}, bits corrected {self.det.at3.hamming})"
                                  if self.det.at3 else "")
                               + f", default size {self.P['tag_size_m']} m, sizes "
                               + ", ".join(f"{f}:{i}={s * 1000:g} mm" for (f, i), s in sorted(self.sizes.items()))
                               + ("".join(f", any:{i}={s * 1000:g} mm" for i, s in sorted(self.sizes_any.items())))
                               + (f"; ChArUco board {self.board.dict_name} {self.board.squares[0]} x {self.board.squares[1]} "
                                  f"x {self.board.square_m * 1000:g} mm (markers {self.board.marker_m * 1000:g} mm, "
                                  f"{self.board.total} corners, pose from {self.board.min_corners})"
                                  + ("" if self.pub_board else " NOT published") if self.board else "; no ChArUco board"))

    # ------------------------------------------------------------------ camera
    def _open_camera(self):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, int(self.P["width"]), int(self.P["height"]), rs.format.bgr8, int(self.P["fps"]))
        cfg.enable_stream(rs.stream.depth, int(self.P["width"]), int(self.P["height"]), rs.format.z16, int(self.P["fps"]))
        prof = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        dev = prof.get_device()
        self.serial = dev.get_info(rs.camera_info.serial_number)
        vs = prof.get_stream(rs.stream.color).as_video_stream_profile()
        self.intr = vs.get_intrinsics()
        if int(self.P["exposure_us"]) > 0:
            for s in dev.query_sensors():
                if s.supports(rs.option.exposure):
                    s.set_option(rs.option.enable_auto_exposure, 0)
                    s.set_option(rs.option.exposure, float(self.P["exposure_us"]))
        i = self.intr
        self.K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]], float)
        self.info = CameraInfo()
        self.info.header.frame_id = "gripper_cam_optical"
        self.info.width, self.info.height = i.width, i.height
        self.info.distortion_model = str(i.model)
        self.info.d = list(map(float, i.coeffs))
        self.info.k = self.K.flatten().tolist()
        self.info.p = [i.fx, 0, i.ppx, 0, 0, i.fy, i.ppy, 0, 0, 0, 1, 0]
        self.get_logger().info(f"D405 {self.serial}: fx={i.fx:.1f} fy={i.fy:.1f} c=({i.ppx:.1f},{i.ppy:.1f}) model={i.model}")

    def _normalised(self, pts):
        """Pixel corners -> normalised image coordinates using the device distortion model."""
        out = []
        for u, v in pts:
            x, y, z = rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], 1.0)
            out.append([x / z, y / z])
        return np.array(out, np.float64)

    # ------------------------------------------------------------------ loop
    def _loop(self):
        W, H = int(self.P["width"]), int(self.P["height"])
        side = float(self.P["center_fraction"]) * H
        sq = (int(W / 2 - side / 2), int(H / 2 - side / 2), int(W / 2 + side / 2), int(H / 2 + side / 2))
        lo, hi = float(self.P["min_size_px"]), float(self.P["max_size_px"])
        failures = 0
        if self.pipe is None:
            self._reconnect()
        while self.running and rclpy.ok():
            try:
                fr = self.pipe.wait_for_frames(2000)
                failures = 0
            except Exception as e:  # noqa: BLE001
                failures += 1
                self.get_logger().error(f"camera: {e}")
                if failures >= 2:                 # not one late frame: the camera is gone (a USB drop) — open it again
                    self._reconnect()
                    failures = 0
                else:
                    time.sleep(0.5)
                continue
            fr = self.align.process(fr)
            cf = fr.get_color_frame()
            df = fr.get_depth_frame()
            if not cf:
                continue
            img = np.asanyarray(cf.get_data())
            dep = np.asanyarray(df.get_data()) if df else None
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            found = self.det.detect(gray)         # (family, corners 4x2 in OpenCV's order, id) over every family
            now = self.get_clock().now().to_msg()

            msg = TagArray()
            msg.header.stamp = now
            msg.header.frame_id = "gripper_cam_optical"
            msg.image_width, msg.image_height = W, H
            msg.center_fraction = float(self.P["center_fraction"])
            for fam, pts, i in found:
                size = float(self.sizes.get((fam, i), self.sizes_any.get(i, self.P["tag_size_m"])))
                r = measure(fam, pts, i, size, self._normalised, self.intr.fx, sq, (lo, hi), dep, self.depth_scale)
                if r is None:
                    continue
                t = TagDetection()
                t.id, t.family, t.size_m = r["id"], r["family"], r["size_m"]
                t.center_px, t.corners_px, t.size_px = r["center_px"], r["corners_px"], r["size_px"]
                pose_msg_from_T(r["T"], t.pose)
                t.distance_m, t.reproj_err_px, t.in_center_square = r["distance_m"], r["reproj_err_px"], r["in_center_square"]
                t.depth_m, t.depth_size_m = r["depth_m"], r["depth_size_m"]
                msg.tags.append(t)
            self.pub_tags.publish(msg)
            self.frames += 1
            bd = None
            if self.board is not None:
                bids, bpts, bmk = self.board.detect(gray)
                bd = self.board.measure(bids, bpts, bmk, self._normalised, self.intr.fx)
                if self.pub_board is not None:
                    self.pub_board.publish(self._board_msg(bd, msg.header))

            tnow = time.time()
            if tnow - self.last_img_t >= 1.0 / float(self.P["image_rate_hz"]):
                self.last_img_t = tnow
                ov = img.copy()
                any_set = any(t.in_center_square for t in msg.tags)
                cv2.rectangle(ov, sq[:2], sq[2:], (0, 200, 0) if any_set else (0, 200, 255), 2)
                if found:
                    cv2.aruco.drawDetectedMarkers(ov, [np.asarray(c, np.float32).reshape(1, 4, 2) for _, c, _ in found],
                                                  np.array([[i] for _, _, i in found], np.int32))
                if bd is not None and bd["corners"]:
                    cv2.aruco.drawDetectedCornersCharuco(ov, np.asarray(bd["corners_px"], np.float32).reshape(-1, 1, 2),
                                                         np.asarray(bd["corner_ids"], np.int32).reshape(-1, 1), (255, 0, 255))
                if bd is not None:
                    cv2.putText(ov, f"board {bd['corners']}/{bd['corners_total']}"
                                + (f" {bd['distance_m'] * 1000:.0f}mm {bd['reproj_err_px']:.2f}px" if bd["seen"] else ""),
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if bd["seen"] else (0, 165, 255), 2)
                for t in msg.tags:
                    fam = short_family(t.family) if len(self.det.families) > 1 else ""   # "36h11 id3" with several families
                    cv2.putText(ov, f"{fam + ' ' if fam else ''}id{t.id} {t.distance_m*1000:.0f}mm{' d' + format(t.depth_m*1000, '.0f') if t.depth_m else ''} {t.size_px:.0f}px{' SET' if t.in_center_square else ''}",
                                (int(t.center_px[0]) + 10, int(t.center_px[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0) if t.in_center_square else (0, 165, 255), 2)
                okj, buf = cv2.imencode(".jpg", ov, [cv2.IMWRITE_JPEG_QUALITY, int(self.P["jpeg_quality"])])
                if okj:
                    im = CompressedImage()
                    im.header = msg.header
                    im.format = "jpeg"
                    im.data = buf.tobytes()
                    self.pub_img.publish(im)
                    self.info.header.stamp = now
                    self.pub_info.publish(self.info)

    def _board_msg(self, bd, header):
        """detect.Charuco.measure's dict -> harness_msgs/CharucoDetection."""
        b = self.board
        m = CharucoDetection()
        m.header = header
        m.seen, m.corners, m.corners_total, m.markers = bd["seen"], bd["corners"], bd["corners_total"], bd["markers"]
        m.dictionary, m.squares_x, m.squares_y = b.dict_name, b.squares[0], b.squares[1]
        m.square_m, m.marker_m = b.square_m, b.marker_m
        if bd["seen"]:
            pose_msg_from_T(bd["T"], m.pose)
            m.distance_m, m.reproj_err_px = bd["distance_m"], bd["reproj_err_px"]
        m.corner_ids, m.corners_px = bd["corner_ids"], bd["corners_px"]
        return m

    def _reconnect(self):
        """The camera stopped delivering ("wait_for_frames cannot be called before start()" after a USB drop: five of
        them in ten minutes on 2026-10-25, one as the arm arrived at a tag — and the node used to stay dead): close
        the pipeline and open it again until it works. Nothing is published meanwhile, so the gateway reports the
        camera offline and the twin says so instead of "tag … not in view"."""
        self.get_logger().error("camera lost: reconnecting until it is back (check its USB cable)")
        while self.running and rclpy.ok():
            try:
                if self.pipe is not None:
                    self.pipe.stop()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(1.0)
            try:
                self._open_camera()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"camera: not back yet ({e})")
                time.sleep(2.0)
                continue
            self.get_logger().info("camera back")
            return

    def _report(self):
        self.get_logger().info(f"{self.frames / 5.0:.1f} fps"), setattr(self, "frames", 0)

    def shutdown(self):
        self.running = False
        try:
            if self.pipe is not None:
                self.pipe.stop()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    n = TagNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
