#!/usr/bin/env python3
"""Gripper camera + AprilTag node (harness_vision).

Owns the RealSense D405. Publishes:
  /camera/tags               harness_msgs/TagArray      every frame (id, pixels, camera-frame pose, "set" flag)
  /camera/image/compressed   sensor_msgs/CompressedImage  at image_rate_hz, with the detection overlay
  /camera/camera_info        sensor_msgs/CameraInfo     with every image

Pose: corners are deprojected with the device's own distortion model
(rs2_deproject_pixel_to_point), then solvePnP (IPPE_SQUARE) on normalised rays,
so the RealSense "inverse Brown-Conrady" model is honoured exactly.

"Set" criterion (coarse teach): tag centre inside the central square (side =
center_fraction x image height) AND mean edge length within [min_size_px,
max_size_px] (a standoff gate). The node only reports it; whoever teaches
decides how long it must hold.
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
from harness_vision.geom import T_from_rvec_tvec, pose_msg_from_T

FAMILIES = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


class TagNode(Node):
    def __init__(self):
        super().__init__("tag_node")
        prm = self.declare_parameters("", [
            ("width", 1280), ("height", 720), ("fps", 30), ("family", "tag36h11"),
            ("tag_size_m", 0.05), ("tag_sizes_ids", [0]), ("tag_sizes_m", [0.0]),
            ("center_fraction", 0.3333), ("min_size_px", 50), ("max_size_px", 260),
            ("image_rate_hz", 10.0), ("jpeg_quality", 70), ("exposure_us", 0),
            ("error_correction_rate", 0.0),
        ])
        self.P = {x.name: x.value for x in prm}
        self.sizes = {int(i): float(s) for i, s in zip(self.P["tag_sizes_ids"] or [], self.P["tag_sizes_m"] or []) if s > 0}

        self.pub_tags = self.create_publisher(TagArray, "/camera/tags", 10)
        self.pub_img = self.create_publisher(CompressedImage, "/camera/image/compressed", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", 5)

        d = cv2.aruco.getPredefinedDictionary(FAMILIES[self.P["family"]])
        dp = cv2.aruco.DetectorParameters()
        dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dp.adaptiveThreshWinSizeMin = 23          # one threshold scale: 5x faster on cluttered 720p scenes
        dp.adaptiveThreshWinSizeMax = 23
        dp.minMarkerPerimeterRate = 0.04          # ignore candidates smaller than ~13 px edge
        dp.errorCorrectionRate = float(self.P["error_correction_rate"])  # 0 = exact code match (16h5 mis-decodes otherwise)
        self.det = cv2.aruco.ArucoDetector(d, dp)

        self._open_camera()
        self.frames = 0
        self.last_img_t = 0.0
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"tag_node up: {self.P['width']}x{self.P['height']}@{self.P['fps']} "
                               f"{self.P['family']} default size {self.P['tag_size_m']} m")

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
        while self.running and rclpy.ok():
            try:
                fr = self.pipe.wait_for_frames(2000)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"camera: {e}")
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
            corners, ids, _ = self.det.detectMarkers(gray)
            now = self.get_clock().now().to_msg()

            msg = TagArray()
            msg.header.stamp = now
            msg.header.frame_id = "gripper_cam_optical"
            msg.image_width, msg.image_height = W, H
            msg.center_fraction = float(self.P["center_fraction"])
            if ids is not None:
                for c, i in zip(corners, ids.flatten()):
                    pts = c.reshape(4, 2)
                    size = float(self.sizes.get(int(i), self.P["tag_size_m"]))
                    h = size / 2.0
                    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
                    ok, rvec, tvec = cv2.solvePnP(obj, self._normalised(pts), np.eye(3), None,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        continue
                    proj, _ = cv2.projectPoints(obj, rvec, tvec, np.eye(3), None)
                    err = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - self._normalised(pts), axis=1))) * self.intr.fx
                    t = TagDetection()
                    t.id, t.family, t.size_m = int(i), self.P["family"], size
                    cx, cy = pts.mean(axis=0)
                    t.center_px = [float(cx), float(cy)]
                    t.corners_px = [float(v) for v in pts.flatten()]
                    edges = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
                    t.size_px = float(np.mean(edges))
                    T = T_from_rvec_tvec(rvec, tvec)
                    pose_msg_from_T(T, t.pose)
                    t.distance_m = float(np.linalg.norm(tvec))
                    t.reproj_err_px = err
                    t.in_center_square = bool(sq[0] <= cx <= sq[2] and sq[1] <= cy <= sq[3] and lo <= t.size_px <= hi)
                    if dep is not None:
                        mask = np.zeros(dep.shape, np.uint8)
                        cv2.fillConvexPoly(mask, pts.astype(np.int32), 1)
                        z = dep[mask == 1].astype(np.float64) * self.depth_scale
                        z = z[(z > 0.05) & (z < 1.5)]
                        if len(z) >= 20:
                            t.depth_m = float(np.median(z))
                            t.depth_size_m = float(size * t.depth_m / t.distance_m)   # foreshortening-free: S_true = S_assumed * depth / PnP range
                    msg.tags.append(t)
            self.pub_tags.publish(msg)
            self.frames += 1

            tnow = time.time()
            if tnow - self.last_img_t >= 1.0 / float(self.P["image_rate_hz"]):
                self.last_img_t = tnow
                ov = img.copy()
                any_set = any(t.in_center_square for t in msg.tags)
                cv2.rectangle(ov, sq[:2], sq[2:], (0, 200, 0) if any_set else (0, 200, 255), 2)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(ov, corners, ids)
                for t in msg.tags:
                    cv2.putText(ov, f"id{t.id} {t.distance_m*1000:.0f}mm{' d' + format(t.depth_m*1000, '.0f') if t.depth_m else ''} {t.size_px:.0f}px{' SET' if t.in_center_square else ''}",
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

    def _report(self):
        self.get_logger().info(f"{self.frames / 5.0:.1f} fps"), setattr(self, "frames", 0)

    def shutdown(self):
        self.running = False
        try:
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
