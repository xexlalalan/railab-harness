#!/usr/bin/env python3
"""Serve /camera/image/compressed as an MJPEG stream: http://<pc>:8080/  (first slice of the web GUI).

    ros2 run harness_vision mjpeg_server [--ros-args -p port:=8080]
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

PAGE = b"""<!doctype html><title>Bunny Harness camera</title>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif">
<div style="padding:8px">gripper camera &mdash; green square = tag SET</div>
<img src="/stream" style="width:100%;max-width:1280px;display:block"></body>"""


class Server(Node):
    def __init__(self):
        super().__init__("mjpeg_server")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.frame = None
        self.cond = threading.Condition()
        self.create_subscription(CompressedImage, "/camera/image/compressed", self._on_img, 5)
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):  # quiet
                pass

            def do_GET(self):
                if self.path != "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                last = None
                try:
                    while True:
                        with node.cond:
                            node.cond.wait(timeout=1.0)
                            f = node.frame
                        if f is None or f is last:
                            continue
                        last = f
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(f))
                        self.wfile.write(f)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f"MJPEG stream on http://0.0.0.0:{self.port}/")

    def _on_img(self, m):
        with self.cond:
            self.frame = bytes(m.data)
            self.cond.notify_all()


def main():
    rclpy.init()
    n = Server()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.httpd.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
